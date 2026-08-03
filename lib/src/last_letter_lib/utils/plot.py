#!/usr/bin/env python3
"""Quick plotting helpers for last_letter_lib data logs.

Thin convenience wrappers over ``last_letter_lib.utils.log`` for manual
inspection of a recorded ``.mcap``. For anything beyond a quick look, load the
data yourself with ``log.load_log`` / ``log.load_log_merged`` and plot it how
you like.
"""

from __future__ import annotations

import matplotlib.pyplot as plt

from . import log as _log


def plot_channel(path, channel, series=None, ax=None):
    """Plot the series of a single channel versus time.

    Parameters
    ----------
    path : str
        Path to the ``.mcap`` file.
    channel : str
        Channel (log-channel / component name) to plot.
    series : list[str], optional
        Column names to plot; defaults to all columns of the channel.
    ax : matplotlib Axes, optional
        Axes to draw on; a new figure is created if omitted.
    """
    frames = _log.load_log(path)
    if channel not in frames:
        raise KeyError(f"Channel {channel!r} not in log. Available: {list(frames)}")
    df = frames[channel]
    cols = series if series is not None else list(df.columns)

    if ax is None:
        _, ax = plt.subplots()
    for col in cols:
        ax.plot(df.index.to_numpy(), df[col].to_numpy(), label=col)
    ax.set_xlabel("time [s]")
    ax.set_title(channel)
    ax.legend(loc="best", fontsize="small")
    return ax


def plot_series(path, columns, ax=None):
    """Plot specific ``"<channel>/<series>"`` columns from the merged log.

    Parameters
    ----------
    path : str
        Path to the ``.mcap`` file.
    columns : list[str]
        Fully-qualified column names, e.g. ``"airfoil1/airdata/alpha"``.
    ax : matplotlib Axes, optional
        Axes to draw on; a new figure is created if omitted.
    """
    df = _log.load_log_merged(path)
    if ax is None:
        _, ax = plt.subplots()
    for col in columns:
        if col not in df.columns:
            raise KeyError(f"Column {col!r} not in log. Available: {list(df.columns)}")
        ax.plot(df.index.to_numpy(), df[col].to_numpy(), label=col)
    ax.set_xlabel("time [s]")
    ax.legend(loc="best", fontsize="small")
    return ax
