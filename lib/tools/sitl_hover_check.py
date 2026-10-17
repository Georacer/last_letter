#!/usr/bin/env python3
"""Fly a hover check of a copter recipe in ArduPilot SITL.

The script starts SITL with the last_letter bridge, takes off in GUIDED to
10 m, holds for 20 s, lands and disarms. It passes when the altitude stays
within 1 m of the target and the roll and pitch stay within 5 degrees during
the hold, and the aircraft disarms after the landing.

pymavlink is not a dependency of last_letter. Run the script with

    uv run --with pymavlink lib/tools/sitl_hover_check.py <recipe>

The bridge must be on PATH and the recipe under $HOME/last_letter_models,
which `cmake --install` of a Release tree with the bridge on gives you.
The exit code is 0 on a pass and 1 on a fail.
"""

import argparse
import collections
import math
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from pymavlink import mavutil

ARDUPILOT = Path("/home/george/projects_no_dropbox/ardupilot")
CONNECTION = "tcp:127.0.0.1:5760"
PORT = ("127.0.0.1", 5760)
TIME_CAP = 300.0
TARGET_ALT = 10.0
HOLD_TIME = 20.0
ALT_TOLERANCE = 1.0
# The hold starts when the climb comes this close to the target, so the hold
# measures the hover and not the end of the climb.
HOLD_START = 0.25
ANGLE_TOLERANCE = math.radians(5.0)

# The EKF flags that a position estimate needs, from EKF_STATUS_FLAGS.
EKF_READY = (
    mavutil.mavlink.EKF_ATTITUDE
    | mavutil.mavlink.EKF_VELOCITY_HORIZ
    | mavutil.mavlink.EKF_VELOCITY_VERT
    | mavutil.mavlink.EKF_POS_HORIZ_ABS
    | mavutil.mavlink.EKF_POS_VERT_ABS
)


class CheckFailed(Exception):
    """The flight did not do what the check expects."""


class Flight:
    """One SITL flight, with a deadline over the whole run."""

    def __init__(self, deadline: float):
        self.deadline = deadline
        self.statustext = collections.deque(maxlen=20)
        self.master = None
        self.armed = False

    def remaining(self) -> float:
        """Return the time left before the cap, in s."""
        return self.deadline - time.monotonic()

    def connect(self, process: subprocess.Popen) -> None:
        """Connect to SITL. SITL opens its port some seconds after the start.

        The script probes the port first, because pymavlink retries a refused
        TCP connection with no end.
        """
        while True:
            self.check_time(f"waiting for {CONNECTION}")
            if process.poll() is not None:
                raise CheckFailed(
                    f"sim_vehicle.py exited with code {process.returncode}."
                )
            try:
                with socket.create_connection(PORT, timeout=1.0):
                    pass
                break
            except OSError:
                time.sleep(1.0)
        # SITL serves one client on the port, so give it a moment to accept
        # the next one after the probe.
        time.sleep(1.0)
        self.master = mavutil.mavlink_connection(CONNECTION, source_system=255)
        if self.master.wait_heartbeat(timeout=max(self.remaining(), 1.0)) is None:
            raise CheckFailed("No heartbeat from SITL.")
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,
            1,
        )

    def recv(self, types, timeout: float = 1.0):
        """Return the next message of the given types, or None at the timeout.

        Every STATUSTEXT and HEARTBEAT that arrives on the way is recorded.
        """
        end = time.monotonic() + min(timeout, max(self.remaining(), 0.0))
        wanted = set(types)
        while True:
            left = end - time.monotonic()
            if left <= 0:
                return None
            msg = self.master.recv_match(blocking=True, timeout=left)
            if msg is None:
                continue
            kind = msg.get_type()
            if kind == "STATUSTEXT":
                self.statustext.append(msg.text)
            elif kind == "HEARTBEAT" and msg.get_srcComponent() == 1:
                self.armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
            if kind in wanted:
                return msg

    def check_time(self, what: str) -> None:
        """Fail when the cap is over."""
        if self.remaining() <= 0:
            raise CheckFailed(f"The {TIME_CAP:.0f} s cap ran out while {what}.")

    def wait_ready_to_arm(self) -> None:
        """Wait for the EKF and the prearm checks."""
        ekf_ok = False
        prearm_ok = False
        while not (ekf_ok and prearm_ok):
            self.check_time("waiting for the EKF and the prearm checks")
            msg = self.recv(["EKF_STATUS_REPORT", "SYS_STATUS"])
            if msg is None:
                continue
            if msg.get_type() == "EKF_STATUS_REPORT":
                ekf_ok = (msg.flags & EKF_READY) == EKF_READY and not (
                    msg.flags & mavutil.mavlink.EKF_CONST_POS_MODE
                )
            else:
                bit = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
                prearm_ok = bool(msg.onboard_control_sensors_health & bit)

    def set_mode(self, mode: str) -> None:
        """Set a flight mode and wait for the heartbeat to show it."""
        mode_id = self.master.mode_mapping()[mode]
        while True:
            self.check_time(f"setting {mode}")
            self.master.set_mode(mode_id)
            msg = self.recv(["HEARTBEAT"], timeout=2.0)
            if msg is not None and msg.custom_mode == mode_id:
                return

    def arm(self) -> None:
        """Arm, and retry while the autopilot refuses."""
        while not self.armed:
            self.check_time("arming")
            self.master.arducopter_arm()
            self.recv(["HEARTBEAT"], timeout=2.0)

    def take_off(self) -> None:
        """Send the takeoff command and wait to reach the target altitude."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            TARGET_ALT,
        )
        while True:
            self.check_time("climbing")
            msg = self.recv(["GLOBAL_POSITION_INT"])
            if not self.armed:
                raise CheckFailed("The aircraft disarmed during the climb.")
            if msg is not None and msg.relative_alt / 1000.0 > TARGET_ALT - HOLD_START:
                return

    def hold(self) -> dict:
        """Hold the hover and return the worst errors."""
        worst = {"alt": 0.0, "roll": 0.0, "pitch": 0.0, "samples": 0}
        end = time.monotonic() + HOLD_TIME
        while time.monotonic() < end:
            self.check_time("holding")
            msg = self.recv(["GLOBAL_POSITION_INT", "ATTITUDE"])
            if msg is None:
                continue
            if msg.get_type() == "GLOBAL_POSITION_INT":
                error = abs(msg.relative_alt / 1000.0 - TARGET_ALT)
                worst["alt"] = max(worst["alt"], error)
                worst["samples"] += 1
            else:
                worst["roll"] = max(worst["roll"], abs(msg.roll))
                worst["pitch"] = max(worst["pitch"], abs(msg.pitch))
        return worst

    def land(self) -> None:
        """Land and wait for the automatic disarm."""
        self.set_mode("LAND")
        while self.armed:
            self.check_time("landing")
            self.recv(["HEARTBEAT"])


def start_sitl(recipe: str, ardupilot: Path, console_log) -> subprocess.Popen:
    """Start sim_vehicle.py in a process group of its own."""
    env = dict(os.environ)
    # Without a terminal to open, run_in_terminal_window.sh runs ArduPilot in
    # the background and writes its console to /tmp/ArduCopter.log.
    for key in ("DISPLAY", "WAYLAND_DISPLAY", "TMUX", "STY", "SITL_RITW_TERMINAL"):
        env.pop(key, None)
    # sim_vehicle.py needs pexpect from the system Python. Under uv run, the
    # virtualenvs come first on PATH and hide it, so every PATH entry inside a
    # virtualenv goes. A virtualenv holds a pyvenv.cfg next to its bin/.
    env.pop("VIRTUAL_ENV", None)
    env["PATH"] = os.pathsep.join(
        entry
        for entry in env.get("PATH", "").split(os.pathsep)
        if entry and not (Path(entry).parent / "pyvenv.cfg").exists()
    )
    command = [
        "Tools/autotest/sim_vehicle.py",
        "-v",
        "ArduCopter",
        "--model",
        f"last_letter:{recipe}",
        "--no-mavproxy",
        "-w",
        "--no-rebuild",
    ]
    return subprocess.Popen(
        command,
        cwd=ardupilot,
        env=env,
        stdout=console_log,
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        start_new_session=True,
    )


def stop_sitl(process: subprocess.Popen) -> None:
    """Stop the whole process group: sim_vehicle.py, ArduPilot and the bridge."""
    try:
        pgid = os.getpgid(process.pid)
    except ProcessLookupError:
        return
    for sig, wait in ((signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        end = time.monotonic() + wait
        while time.monotonic() < end:
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                process.poll()
                return
            time.sleep(0.2)
    process.poll()


def ardupilot_log(ardupilot: Path) -> str:
    """Return the path of the last dataflash log that ArduPilot wrote."""
    logs = ardupilot / "logs"
    try:
        number = int((logs / "LASTLOG.TXT").read_text().strip())
    except (OSError, ValueError):
        return f"{logs}/ (no LASTLOG.TXT)"
    return str(logs / f"{number:08d}.BIN")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("recipe", help="The recipe, for example ardupilot_copter.")
    parser.add_argument(
        "--ardupilot",
        type=Path,
        default=ARDUPILOT,
        help=f"The ArduPilot checkout with a built arducopter. Default {ARDUPILOT}.",
    )
    args = parser.parse_args()

    start = time.monotonic()
    flight = Flight(start + TIME_CAP)
    console_log = tempfile.NamedTemporaryFile(
        "w", prefix="sitl_hover_check_", suffix=".log", delete=False
    )
    process = start_sitl(args.recipe, args.ardupilot, console_log)
    result = 1
    try:
        flight.connect(process)
        print(f"Connected after {time.monotonic() - start:.1f} s.")
        flight.wait_ready_to_arm()
        print(f"EKF and prearm ready after {time.monotonic() - start:.1f} s.")
        flight.set_mode("GUIDED")
        flight.arm()
        flight.take_off()
        print(
            f"Reached {TARGET_ALT - HOLD_START:.2f} m after "
            f"{time.monotonic() - start:.1f} s, holding {HOLD_TIME:.0f} s."
        )
        worst = flight.hold()
        print(
            f"Hold: worst altitude error {worst['alt']:.3f} m over "
            f"{worst['samples']} samples, worst roll "
            f"{math.degrees(worst['roll']):.2f} deg, worst pitch "
            f"{math.degrees(worst['pitch']):.2f} deg."
        )
        flight.land()
        print(f"Disarmed after {time.monotonic() - start:.1f} s.")
        failures = []
        if worst["samples"] == 0:
            failures.append("no altitude samples during the hold")
        if worst["alt"] > ALT_TOLERANCE:
            failures.append(f"altitude error {worst['alt']:.3f} m")
        if worst["roll"] > ANGLE_TOLERANCE:
            failures.append(f"roll {math.degrees(worst['roll']):.2f} deg")
        if worst["pitch"] > ANGLE_TOLERANCE:
            failures.append(f"pitch {math.degrees(worst['pitch']):.2f} deg")
        if failures:
            raise CheckFailed("Out of tolerance: " + ", ".join(failures) + ".")
        result = 0
    except CheckFailed as error:
        print(f"FAIL: {error}")
    finally:
        stop_sitl(process)
        console_log.close()

    if result == 0:
        print(f"PASS: {args.recipe}")
        os.unlink(console_log.name)
    else:
        print("Last STATUSTEXT lines:")
        for line in flight.statustext:
            print(f"  {line}")
        print(f"ArduPilot log: {ardupilot_log(args.ardupilot)}")
        print("ArduPilot console: /tmp/ArduCopter.log")
        print(f"sim_vehicle.py output: {console_log.name}")
    return result


if __name__ == "__main__":
    sys.exit(main())
