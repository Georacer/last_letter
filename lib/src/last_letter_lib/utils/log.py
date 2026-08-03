#!/usr/bin/env python3
"""Read last_letter_lib data logs (DataTamer MCAP) into Python.

Logging is written from C++ via DataTamer's ``MCAPSink`` (see
``last_letter_lib.enable_logging``). This module is the read side: it decodes
those ``.mcap`` files into numpy arrays / pandas DataFrames and can export CSV.

Why a pure-Python decoder: the ``mcap`` package reads the *container* (schemas,
channels, messages), but the message payload uses DataTamer's own compact
snapshot encoding. This module ports DataTamer's ``BuilSchemaFromText`` and
``ParseSnapshot`` (schema version 4, matching the pinned data_tamer 1.0.3), so
no C++ reader / mcap headers are needed.

Public API
----------
load_log(path)         -> dict[channel_name, pandas.DataFrame]  (index = time [s])
load_log_merged(path)  -> pandas.DataFrame  (all channels joined on time)
to_csv(mcap_path, csv) -> writes the merged frame to `csv`
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from dataclasses import field as dc_field

import pandas as pd

# DataTamer schema version this decoder targets (data_tamer 1.0.3).
_SCHEMA_VERSION = 4

# Numeric BasicTypes from data_tamer_parser.hpp mapped to their little-endian
# struct format + byte size. A custom (nested) type is anything not in here.
_TYPE_FORMAT = {
    "bool": ("<?", 1),
    "char": ("<b", 1),
    "int8": ("<b", 1),
    "uint8": ("<B", 1),
    "int16": ("<h", 2),
    "uint16": ("<H", 2),
    "int32": ("<i", 4),
    "uint32": ("<I", 4),
    "int64": ("<q", 8),
    "uint64": ("<Q", 8),
    "float32": ("<f", 4),
    "float64": ("<d", 8),
}


@dataclass
class _TypeField:
    field_name: str = ""
    type_name: str = ""  # one of _TYPE_NAMES (incl. "other") or a custom type name
    is_other: bool = False
    is_vector: bool = False
    array_size: int = 0  # 0 => dynamic (size prefixes the data)


@dataclass
class _Schema:
    channel_name: str = ""
    fields: list = dc_field(default_factory=list)  # list[_TypeField]
    custom_types: dict = dc_field(default_factory=dict)  # name -> list[_TypeField]


def _parse_schema_text(txt: str) -> _Schema:
    """Port of DataTamerParser::BuilSchemaFromText."""
    schema = _Schema()
    target = schema.fields  # current field list (top-level or a custom type)

    lines = txt.splitlines()
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        i += 1
        if not line:
            continue
        if "==============================" in line:
            # The next line names the custom type: "MSG: <typename>".
            msg_line = lines[i].strip() if i < len(lines) else ""
            i += 1
            if not msg_line.startswith("MSG:"):
                raise ValueError(f"Expected 'MSG:' after separator, got: {msg_line!r}")
            type_name = msg_line[len("MSG:") :].strip()
            target = schema.custom_types.setdefault(type_name, [])
            continue

        # Metadata lines ("### version: 4", "### hash: ...", "### channel_name:
        # ...") contain a space inside the key, so skip past the "### " prefix
        # before locating the key/value separator (mirrors BuilSchemaFromText).
        if line.startswith("### "):
            space = line.find(" ", 4)
        else:
            space = line.find(" ")
        if space == -1:
            raise ValueError(f"Unexpected schema line: {line!r}")
        left = line[:space].strip()
        right = line[space + 1 :].strip()

        if left == "### version:":
            if int(right) != _SCHEMA_VERSION:
                raise ValueError(
                    f"Unsupported DataTamer schema version {right} "
                    f"(expected {_SCHEMA_VERSION})"
                )
            continue
        if left == "### hash:":
            continue
        if left == "### channel_name:":
            schema.channel_name = right
            continue

        # A field line: "<type>[<array>] <name>". `left` is the type (possibly
        # with an array suffix), `right` is the field name.
        tf = _TypeField(field_name=right)
        bracket = left.find("[")
        base_type = left if bracket == -1 else left[:bracket]
        if base_type in _TYPE_FORMAT:
            tf.type_name = base_type
            tf.is_other = False
        else:
            tf.type_name = base_type
            tf.is_other = True
        if bracket != -1:
            tf.is_vector = True
            close = left.find("]", bracket)
            inner = left[bracket + 1 : close]
            tf.array_size = int(inner) if inner else 0
        target.append(tf)

    return schema


def _decode_snapshot(schema: _Schema, mask: bytes, payload: bytes) -> dict:
    """Port of DataTamerParser::ParseSnapshot. Returns {series_name: value}."""
    out: dict = {}
    pos = 0

    def get_bit(index: int) -> bool:
        return bool(mask[index >> 3] & (1 << (index % 8)))

    def read_scalar(type_name: str):
        nonlocal pos
        fmt, size = _TYPE_FORMAT[type_name]
        (val,) = struct.unpack_from(fmt, payload, pos)
        pos += size
        return val

    def read_u32() -> int:
        nonlocal pos
        (val,) = struct.unpack_from("<I", payload, pos)
        pos += 4
        return val

    def parse_field(tf: _TypeField, prefix: str) -> None:
        count = tf.array_size
        if tf.is_vector and tf.array_size == 0:
            count = read_u32()  # dynamic vector: size prefix
        name = tf.field_name if not prefix else f"{prefix}/{tf.field_name}"

        def parse_one(var_name: str) -> None:
            if not tf.is_other:
                out[var_name] = read_scalar(tf.type_name)
            else:
                for sub in schema.custom_types[tf.type_name]:
                    parse_field(sub, var_name)

        if not tf.is_vector:
            parse_one(name)
        else:
            for a in range(count):
                parse_one(f"{name}[{a}]")

    for idx, tf in enumerate(schema.fields):
        if get_bit(idx):
            parse_field(tf, "")
    return out


def _read_channels(path):
    """Yield (channel_name, timestamp_ns, {series: value}) for every message."""
    from mcap.reader import make_reader

    schema_cache: dict = {}  # schema_id -> _Schema
    with open(path, "rb") as f:
        reader = make_reader(f)
        for mcap_schema, channel, message in reader.iter_messages():
            schema = schema_cache.get(mcap_schema.id)
            if schema is None:
                schema = _parse_schema_text(mcap_schema.data.decode("utf-8"))
                schema_cache[mcap_schema.id] = schema

            data = message.data
            (mask_size,) = struct.unpack_from("<I", data, 0)
            mask = data[4 : 4 + mask_size]
            (payload_size,) = struct.unpack_from("<I", data, 4 + mask_size)
            start = 4 + mask_size + 4
            payload = data[start : start + payload_size]

            values = _decode_snapshot(schema, mask, payload)
            yield channel.topic, message.log_time, values


def load_log(path) -> dict:
    """Load an MCAP log into one DataFrame per channel.

    Returns ``{channel_name: DataFrame}`` where each DataFrame is indexed by
    time in seconds (named "time") with one column per flattened series.
    """
    times: dict = {}  # channel -> list[float seconds]
    rows: dict = {}  # channel -> list[dict]
    for channel, ts_ns, values in _read_channels(path):
        times.setdefault(channel, []).append(ts_ns * 1e-9)
        rows.setdefault(channel, []).append(values)

    frames = {}
    for channel, row_list in rows.items():
        df = pd.DataFrame(row_list)
        df.index = pd.Index(times[channel], name="time")
        frames[channel] = df
    return frames


def load_log_merged(path) -> pd.DataFrame:
    """Load an MCAP log into a single wide DataFrame.

    All channels are outer-joined on their timestamp; columns are prefixed with
    the channel name as ``"<channel>/<series>"``.
    """
    frames = load_log(path)
    renamed = []
    for channel, df in frames.items():
        renamed.append(df.add_prefix(f"{channel}/"))
    if not renamed:
        return pd.DataFrame()
    merged = pd.concat(renamed, axis=1)
    merged.sort_index(inplace=True)
    return merged


def to_csv(mcap_path, csv_path) -> None:
    """Export an MCAP log to a single wide CSV (merged on timestamp)."""
    load_log_merged(mcap_path).to_csv(csv_path)
