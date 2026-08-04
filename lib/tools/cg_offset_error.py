#!/usr/bin/env python3
"""Quantify the error neglected by CG-referencing the equations of motion.

Background
----------
`last_letter_lib` propagates the linear velocity of the *body-frame origin* O,
while the CG-referenced equations of motion in ``kinematics.cpp`` compute
``F / m``, which is the acceleration of the *centre of gravity*. For a rigid
body the two are related (in body axes) by

    a_cg = a_O + omega_dot x r_cg + omega x (omega x r_cg)

so the term the simulation neglects is

    eps = omega_dot x r_cg  +  omega x (omega x r_cg)
          \\______________/     \\____________________/
            Euler term            centripetal term

* The **Euler term** is linear in the CG offset and in the angular acceleration.
* The **centripetal term** grows with the *square* of the body rate.

Both vanish when ``omega = omega_dot = 0``, which is why a single step from rest
(as in ``test_thruster_1``) is exact. This script lets you probe where that
assumption stops being reasonable.

Usage
-----
    # Single operating point
    ./cg_offset_error.py point --r-cg 0.03 0 0 --omega 0 30 0 --omega-dot 0 5 0 --deg

    # Sweep body rate, holding the CG offset fixed
    ./cg_offset_error.py sweep --r-cg 0.03 0 0 --axis 0 1 0 --deg

    # 2-D table of |error| over (body rate) x (CG offset)
    ./cg_offset_error.py table --deg

    # Body rate at which the error reaches a tolerance
    ./cg_offset_error.py threshold --r-cg 0.03 0 0 --tol-mg 10 --deg

Angular quantities are in rad/s and rad/s^2 unless ``--deg`` is given.
"""

from __future__ import annotations

import argparse

import numpy as np

# Standard gravity, used only to express the error in intuitive "g" units.
G = 9.80665


# --------------------------------------------------------------------------- #
# Core computation
# --------------------------------------------------------------------------- #
def frame_transfer_error(omega, omega_dot, r_cg):
    """Return the acceleration error between the body origin and the CG.

    Parameters
    ----------
    omega : array_like, shape (3,)
        Body angular velocity [rad/s], body axes.
    omega_dot : array_like, shape (3,)
        Body angular acceleration [rad/s^2], body axes.
    r_cg : array_like, shape (3,)
        Position of the CG relative to the body-frame origin [m], body axes.

    Returns
    -------
    dict with keys ``euler``, ``centripetal`` and ``total`` (each a (3,) array
    in m/s^2).
    """
    omega = np.asarray(omega, dtype=float)
    omega_dot = np.asarray(omega_dot, dtype=float)
    r_cg = np.asarray(r_cg, dtype=float)

    euler = np.cross(omega_dot, r_cg)
    centripetal = np.cross(omega, np.cross(omega, r_cg))

    return {
        "euler": euler,
        "centripetal": centripetal,
        "total": euler + centripetal,
    }


def perpendicular_offset(omega, r_cg):
    """Magnitude of the component of ``r_cg`` perpendicular to ``omega``.

    The centripetal term has magnitude ``|omega|^2 * perpendicular_offset``,
    so this is the lever arm that actually drives that term. Returns ``|r_cg|``
    when ``omega`` is zero (the worst case over all rotation axes).
    """
    omega = np.asarray(omega, dtype=float)
    r_cg = np.asarray(r_cg, dtype=float)

    norm = np.linalg.norm(omega)
    if norm == 0.0:
        return float(np.linalg.norm(r_cg))
    axis = omega / norm
    return float(np.linalg.norm(r_cg - np.dot(r_cg, axis) * axis))


def rate_for_tolerance(tol, r_perp):
    """Body rate [rad/s] at which the centripetal term alone reaches ``tol``.

    Solves ``|omega|^2 * r_perp = tol``. Returns ``inf`` if the lever arm is
    zero (the CG lies on the rotation axis, so the term never appears).
    """
    if r_perp <= 0.0:
        return float("inf")
    return float(np.sqrt(tol / r_perp))


# --------------------------------------------------------------------------- #
# Reporting helpers
# --------------------------------------------------------------------------- #
def _vec(v):
    return "[" + ", ".join(f"{c:+9.5f}" for c in v) + "]"


def _mag_line(label, v):
    mag = float(np.linalg.norm(v))
    return (
        f"  {label:<18s} {_vec(v)}  |.| = {mag:9.5f} m/s^2  ({mag / G * 1e3:8.3f} mg)"
    )


def report_point(omega, omega_dot, r_cg, reference=None):
    """Print a full breakdown for one operating point."""
    err = frame_transfer_error(omega, omega_dot, r_cg)
    r_perp = perpendicular_offset(omega, r_cg)

    print("Inputs")
    print(
        f"  r_cg               {_vec(r_cg)} m   |r_cg| = {np.linalg.norm(r_cg):.5f} m"
    )
    print(
        f"  omega              {_vec(omega)} rad/s"
        f"  ({np.rad2deg(np.linalg.norm(omega)):.3f} deg/s)"
    )
    print(
        f"  omega_dot          {_vec(omega_dot)} rad/s^2"
        f"  ({np.rad2deg(np.linalg.norm(omega_dot)):.3f} deg/s^2)"
    )
    print(f"  lever arm _|_ omega  {r_perp:.5f} m")
    print()

    print("Neglected acceleration error")
    print(_mag_line("Euler term", err["euler"]))
    print(_mag_line("centripetal term", err["centripetal"]))
    print(_mag_line("TOTAL", err["total"]))

    total_mag = float(np.linalg.norm(err["total"]))
    if reference is not None and reference > 0:
        print()
        print(
            f"  relative to reference accel {reference:.4f} m/s^2:"
            f" {100.0 * total_mag / reference:.4f} %"
        )


def report_sweep(r_cg, axis, omega_dot, rates):
    """Print error vs. body rate magnitude about a fixed axis."""
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0:
        raise ValueError("--axis must be a non-zero vector")
    axis = axis / norm

    print(f"r_cg = {_vec(r_cg)} m,  rotation axis = {_vec(axis)}")
    print(f"omega_dot = {_vec(omega_dot)} rad/s^2")
    print()
    print(
        f"{'rate [deg/s]':>13s} {'rate [rad/s]':>13s} "
        f"{'Euler':>12s} {'centrip.':>12s} {'total':>12s} {'total':>11s}"
    )
    print(
        f"{'':>13s} {'':>13s} {'[m/s^2]':>12s} {'[m/s^2]':>12s} {'[m/s^2]':>12s} {'[mg]':>11s}"
    )
    print("-" * 78)

    for rate in rates:
        omega = axis * rate
        err = frame_transfer_error(omega, omega_dot, r_cg)
        e_mag = np.linalg.norm(err["euler"])
        c_mag = np.linalg.norm(err["centripetal"])
        t_mag = np.linalg.norm(err["total"])
        print(
            f"{np.rad2deg(rate):13.2f} {rate:13.4f} "
            f"{e_mag:12.6f} {c_mag:12.6f} {t_mag:12.6f} {t_mag / G * 1e3:11.3f}"
        )


def report_table(offsets, rates, axis, omega_dot_mag):
    """Print a 2-D table of total error magnitude in milli-g.

    Rows are body rates, columns are CG offset magnitudes. The CG offset is
    placed perpendicular to the rotation axis (worst case), and the angular
    acceleration is applied about the same axis.
    """
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)

    # Pick an offset direction perpendicular to the rotation axis.
    seed = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(seed, axis)) > 0.9:
        seed = np.array([0.0, 0.0, 1.0])
    offset_dir = np.cross(axis, seed)
    offset_dir /= np.linalg.norm(offset_dir)

    print(
        f"Total |error| in milli-g. Rotation axis {_vec(axis)}, "
        f"CG offset along {_vec(offset_dir)} (perpendicular = worst case)."
    )
    print(f"Angular acceleration = {omega_dot_mag:.4f} rad/s^2 about the same axis.")
    print()

    header = f"{'rate [deg/s]':>13s}" + "".join(f"{o:>11.3f}" for o in offsets)
    print(f"{'':>13s}{'|r_cg| [m]':>{11 * len(offsets)}s}")
    print(header)
    print("-" * len(header))

    for rate in rates:
        omega = axis * rate
        omega_dot = axis * omega_dot_mag
        cells = []
        for off in offsets:
            r_cg = offset_dir * off
            err = frame_transfer_error(omega, omega_dot, r_cg)
            cells.append(np.linalg.norm(err["total"]) / G * 1e3)
        print(f"{np.rad2deg(rate):13.2f}" + "".join(f"{c:>11.3f}" for c in cells))


def report_threshold(r_cg, axis, tolerances_mg):
    """Print the body rate at which the centripetal term reaches each tolerance."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    r_perp = perpendicular_offset(axis, r_cg)

    print(f"r_cg = {_vec(r_cg)} m,  rotation axis = {_vec(axis)}")
    print(f"lever arm perpendicular to axis: {r_perp:.5f} m")
    print()
    print("Body rate at which the centripetal term alone reaches the tolerance:")
    print()
    print(
        f"{'tolerance [mg]':>16s} {'[m/s^2]':>12s} {'rate [rad/s]':>14s} {'rate [deg/s]':>14s}"
    )
    print("-" * 60)

    for tol_mg in tolerances_mg:
        tol = tol_mg * 1e-3 * G
        rate = rate_for_tolerance(tol, r_perp)
        if np.isinf(rate):
            print(f"{tol_mg:16.3f} {tol:12.6f} {'never':>14s} {'never':>14s}")
        else:
            print(f"{tol_mg:16.3f} {tol:12.6f} {rate:14.4f} {np.rad2deg(rate):14.2f}")


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Quantify the acceleration error neglected when the equations of "
            "motion are CG-referenced but linear velocity is tracked at the "
            "body-frame origin."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--deg",
        action="store_true",
        help="interpret angular inputs as deg/s and deg/s^2 instead of rad",
    )
    sub = parser.add_subparsers(dest="mode", required=True)

    p_point = sub.add_parser("point", help="single operating point")
    p_point.add_argument("--r-cg", nargs=3, type=float, default=[0.03, 0.0, 0.0])
    p_point.add_argument("--omega", nargs=3, type=float, default=[0.0, 0.0, 0.0])
    p_point.add_argument("--omega-dot", nargs=3, type=float, default=[0.0, 0.0, 0.0])
    p_point.add_argument(
        "--reference",
        type=float,
        default=None,
        help="reference acceleration [m/s^2] to express the error against",
    )

    p_sweep = sub.add_parser("sweep", help="sweep body rate about a fixed axis")
    p_sweep.add_argument("--r-cg", nargs=3, type=float, default=[0.03, 0.0, 0.0])
    p_sweep.add_argument("--axis", nargs=3, type=float, default=[0.0, 1.0, 0.0])
    p_sweep.add_argument("--omega-dot", nargs=3, type=float, default=[0.0, 0.0, 0.0])
    p_sweep.add_argument("--start", type=float, default=0.0, help="start rate")
    p_sweep.add_argument("--stop", type=float, default=360.0, help="stop rate")
    p_sweep.add_argument("--num", type=int, default=13, help="number of samples")

    p_table = sub.add_parser("table", help="2-D table over rate x CG offset")
    p_table.add_argument(
        "--offsets",
        nargs="+",
        type=float,
        default=[0.01, 0.03, 0.1, 0.3, 1.0],
        help="CG offset magnitudes [m]",
    )
    p_table.add_argument(
        "--rates",
        nargs="+",
        type=float,
        default=[1, 10, 30, 60, 120, 240, 360],
        help="body rate magnitudes",
    )
    p_table.add_argument("--axis", nargs=3, type=float, default=[0.0, 1.0, 0.0])
    p_table.add_argument(
        "--omega-dot-mag",
        type=float,
        default=0.0,
        help="angular acceleration magnitude about the same axis",
    )

    p_thr = sub.add_parser("threshold", help="rate at which a tolerance is reached")
    p_thr.add_argument("--r-cg", nargs=3, type=float, default=[0.03, 0.0, 0.0])
    p_thr.add_argument("--axis", nargs=3, type=float, default=[0.0, 1.0, 0.0])
    p_thr.add_argument(
        "--tol-mg",
        nargs="+",
        type=float,
        default=[0.1, 1.0, 10.0, 100.0],
        help="tolerances in milli-g",
    )

    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)

    def ang(v):
        """Convert an angular quantity to radians if --deg was given."""
        return np.deg2rad(v) if args.deg else np.asarray(v, dtype=float)

    if args.mode == "point":
        report_point(
            ang(args.omega), ang(args.omega_dot), np.asarray(args.r_cg), args.reference
        )
    elif args.mode == "sweep":
        rates = ang(np.linspace(args.start, args.stop, args.num))
        report_sweep(np.asarray(args.r_cg), args.axis, ang(args.omega_dot), rates)
    elif args.mode == "table":
        report_table(
            args.offsets, ang(np.asarray(args.rates)), args.axis, args.omega_dot_mag
        )
    elif args.mode == "threshold":
        report_threshold(np.asarray(args.r_cg), args.axis, args.tol_mg)


if __name__ == "__main__":
    main()
