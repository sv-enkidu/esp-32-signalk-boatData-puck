#!/usr/bin/env python3
"""
qmc_cal_plot.py - visualise and calibrate the puck's QMC5883P compass

Reads a serial log from the puck (firmware v4.10+ with QMC_RAW_DEBUG 1),
fits the magnetometer circle, suggests qmcHard / qmcSoft values, and shows
how well the corrected QMC heading tracks the BNO085 heading ($IIHDG).

USAGE
  python qmc_cal_plot.py capture.txt              analyse a saved log
  python qmc_cal_plot.py capture.txt --save out.png   also save the figure
  python qmc_cal_plot.py --serial COM5 --seconds 60 capture.txt
        record straight from the puck first (needs: pip install pyserial),
        then analyse. On Mac/Linux the port looks like /dev/ttyUSB0 or
        /dev/cu.usbmodem1101. Close the Arduino serial monitor / Signal K
        first - only one program can hold the port.
  python qmc_cal_plot.py capture.txt --hard 0.233 0.143 0.0
        preview what specific offsets would do (instead of the fitted ones)

LOG FORMAT
  Copy-paste from the Arduino serial monitor is fine (timestamps are
  ignored). It needs $PQMCR lines; $IIHDG lines are optional but give the
  BNO comparison. Everything else (GPS, weather...) is skipped.

HOW TO CAPTURE
  - Rotate slowly and steadily: 2-3 full turns over 30-60 s.
  - For the Z offset, also tilt the puck well over (30-90 deg) in several
    directions while turning - otherwise Z is reported as "not determined".
  - Capture where the puck will actually live (final enclosure, cables
    routed as installed; on the boat once mounted).

Requires: pip install numpy matplotlib   (and pyserial for --serial)
"""

import argparse
import math
import re
import sys
import time

import numpy as np
import matplotlib.pyplot as plt

PQMCR = re.compile(r"\$PQMCR,([-\d.]+),([-\d.]+),([-\d.]+)")
IIHDG = re.compile(r"\$IIHDG,([-\d.]+)")
HCHDG = re.compile(r"\$HCHDG,([-\d.]+)")


# ----------------------------------------------------------------- capture
def record_serial(port, baud, seconds, outfile):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial not installed: pip install pyserial")
    print(f"Recording {seconds}s from {port} @ {baud} -> {outfile}")
    print("Start rotating the puck now...")
    end = time.time() + seconds
    n = 0
    with serial.Serial(port, baud, timeout=1) as s, open(outfile, "w") as f:
        while time.time() < end:
            line = s.readline().decode("ascii", errors="ignore")
            if line:
                f.write(line)
                if "$PQMCR" in line:
                    n += 1
                    if n % 50 == 0:
                        print(f"  {n} QMC samples, {end - time.time():.0f}s left")
    print(f"Done: {n} QMC samples")


# ----------------------------------------------------------------- parsing
def parse_log(path):
    """Return arrays mx, my, mz, bno, hc (bno/hc NaN where unavailable).
    Each $PQMCR sample is paired with the next $IIHDG and $HCHDG seen."""
    rows = []
    pending = None
    for line in open(path, errors="ignore"):
        m = PQMCR.search(line)
        if m:
            if pending is not None:
                rows.append(pending)
            pending = [*map(float, m.groups()), np.nan, np.nan]
            continue
        if pending is None:
            continue
        m = HCHDG.search(line)
        if m and np.isnan(pending[4]):
            pending[4] = float(m.group(1))
            continue
        m = IIHDG.search(line)
        if m and np.isnan(pending[3]):
            pending[3] = float(m.group(1))
    if pending is not None:
        rows.append(pending)
    if len(rows) < 20:
        sys.exit(f"Only {len(rows)} $PQMCR samples found - is QMC_RAW_DEBUG 1?")
    return np.array(rows).T


# ----------------------------------------------------------------- fitting
def fit_circle(x, y):
    """Least-squares circle: returns cx, cy, r"""
    A = np.c_[2 * x, 2 * y, np.ones_like(x)]
    s, *_ = np.linalg.lstsq(A, x**2 + y**2, rcond=None)
    cx, cy = s[0], s[1]
    return cx, cy, math.sqrt(max(s[2] + cx**2 + cy**2, 0))


def fit_ellipse(x, y):
    """Axis-aligned ellipse: returns cx, cy, rx, ry (or None if it fails)"""
    # a x^2 + c y^2 + d x + e y + f = 0, solved by SVD (works whether or
    # not the origin lies inside the ellipse)
    A = np.c_[x**2, y**2, x, y, np.ones_like(x)]
    _, _, vt = np.linalg.svd(A, full_matrices=False)
    a, c, d, e, f = vt[-1]
    if a * c <= 0:
        return None
    if a < 0:
        a, c, d, e, f = -a, -c, -d, -e, -f
    cx, cy = -d / (2 * a), -e / (2 * c)
    k = a * cx**2 + c * cy**2 - f
    if k <= 0:
        return None
    return cx, cy, math.sqrt(k / a), math.sqrt(k / c)


def fit_sphere(x, y, z):
    A = np.c_[2 * x, 2 * y, 2 * z, np.ones_like(x)]
    s, *_ = np.linalg.lstsq(A, x**2 + y**2 + z**2, rcond=None)
    cx, cy, cz = s[:3]
    return cx, cy, cz, math.sqrt(max(s[3] + cx**2 + cy**2 + cz**2, 0))


# ----------------------------------------------------------------- helpers
def wrap180(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def heading(x, y):
    return np.degrees(np.arctan2(y, x)) % 360.0


def compare(qmc, bno):
    """Error of qmc vs bno after removing the constant offset.
    Returns (aligned error array, constant offset deg, rms, max)."""
    ok = ~np.isnan(bno)
    diff = wrap180(qmc[ok] - bno[ok])
    off = math.degrees(np.angle(np.mean(np.exp(1j * np.radians(diff)))))
    err = wrap180(diff - off)
    return ok, err, off, float(np.sqrt(np.mean(err**2))), float(np.max(np.abs(err)))


# ----------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logfile")
    ap.add_argument("--serial", help="record from this port first")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=int, default=60)
    ap.add_argument("--hard", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    help="use these hard-iron values instead of the fitted ones")
    ap.add_argument("--soft", type=float, nargs=2, metavar=("X", "Y"),
                    help="use these soft-iron scales (default 1.0 1.0)")
    ap.add_argument("--save", help="save figure to this PNG")
    a = ap.parse_args()

    if a.serial:
        record_serial(a.serial, a.baud, a.seconds, a.logfile)

    mx, my, mz, bno, hc = parse_log(a.logfile)
    n = len(mx)
    has_bno = np.count_nonzero(~np.isnan(bno)) > 10

    # --- fits
    cx, cy, r = fit_circle(mx, my)
    mmx, mmy = (mx.max() + mx.min()) / 2, (my.max() + my.min()) / 2
    ell = fit_ellipse(mx, my)
    zspan = mz.max() - mz.min()
    sph = fit_sphere(mx, my, mz) if zspan > 0.2 else None

    hx, hy = (a.hard[0], a.hard[1]) if a.hard else (cx, cy)
    hz = a.hard[2] if a.hard else (sph[2] if sph else 0.0)
    sx, sy = a.soft if a.soft else (1.0, 1.0)

    cxs = (mx - hx) * sx
    cys = (my - hy) * sy
    raw_hdg = heading(mx, my)
    cor_hdg = heading(cxs, cys)
    radii = np.hypot(mx - cx, my - cy)
    off_mag = math.hypot(cx, cy)

    # angular coverage of the turn (how much of the circle was visited)
    bins = np.unique((cor_hdg // 10).astype(int))
    coverage = len(bins) * 10

    # --- report
    print("\n================ QMC5883P CALIBRATION REPORT ================")
    print(f"Samples: {n}   rotation coverage: {coverage} of 360 deg"
          + ("   <-- do a fuller turn!" if coverage < 300 else ""))
    print(f"Z span: {zspan:.3f} G "
          + ("(enough tilt for a Z fit)" if sph else "(flat turn - Z offset not determined)"))
    print("\nHorizontal circle (least-squares fit):")
    print(f"  centre      X={cx:+.4f}  Y={cy:+.4f} G")
    print(f"  radius      {r:.4f} G  (Earth's horizontal field seen here)")
    print(f"  offset size {off_mag:.4f} G  = {off_mag / r * 100:.0f}% of radius")
    if off_mag > r:
        print("  !! Offset is LARGER than the radius: (0,0) is outside the circle,")
        print("     so the uncorrected heading cannot complete a full turn.")
    elif off_mag > 0.3 * r:
        print("  !  Large offset: uncorrected heading will be badly non-linear.")
    print(f"  radius spread {radii.min():.3f}-{radii.max():.3f} G "
          f"(+/-{(radii.max() - radii.min()) / 2 / r * 100:.0f}%; noise, hand motion, distortion)")
    print(f"\nMin/max centre  X={mmx:+.4f}  Y={mmy:+.4f} G (what the firmware estimates)")
    if ell:
        ecx, ecy, rx, ry = ell
        avg = (rx + ry) / 2
        print(f"Ellipse fit     X={ecx:+.4f}  Y={ecy:+.4f}  rx={rx:.4f} ry={ry:.4f}"
              f"  -> soft X={avg / rx:.3f} Y={avg / ry:.3f}")
        if abs(rx / ry - 1) < 0.08:
            print("                (axes within 8% - soft-iron correction probably not needed)")
    if sph:
        print(f"Sphere fit      X={sph[0]:+.4f}  Y={sph[1]:+.4f}  Z={sph[2]:+.4f}  r={sph[3]:.4f}")

    if has_bno:
        _, e_raw, _, rms_raw, max_raw = compare(raw_hdg, bno)
        ok, e_cor, off_cor, rms_cor, max_cor = compare(cor_hdg, bno)
        print("\nQMC vs BNO heading error (constant offset removed):")
        print(f"  uncorrected  rms {rms_raw:5.1f} deg   max {max_raw:5.1f} deg")
        print(f"  corrected    rms {rms_cor:5.1f} deg   max {max_cor:5.1f} deg")
        print(f"  remaining constant offset {off_cor:+.1f} deg "
              f"(-> offsetQmcYaw = {-off_cor:.1f} to line QMC up with the BNO)")

    print("\n---- paste into the firmware ----")
    print(f"float qmcHardX = {hx:.4f};")
    print(f"float qmcHardY = {hy:.4f};")
    print(f"float qmcHardZ = {hz:.4f};" + ("" if (sph or a.hard) else "   // not measured - tilt during capture"))
    print(f"float qmcSoftX = {sx:.3f};")
    print(f"float qmcSoftY = {sy:.3f};")
    print("float qmcSoftZ = 1.000;")
    print("=============================================================\n")

    # --- plots
    fig = plt.figure(figsize=(15, 5.5))
    fig.suptitle(f"QMC5883P calibration - {a.logfile}", fontsize=12)

    # 1. the circle
    ax = fig.add_subplot(1, 3, 1)
    t = np.linspace(0, 2 * np.pi, 200)
    ax.plot(mx, my, ".", color="tab:red", ms=4, label="raw")
    ax.plot(cx + r * np.cos(t), cy + r * np.sin(t), "--", color="tab:red", lw=0.8)
    ax.plot(cxs, cys, ".", color="tab:blue", ms=4, label="corrected")
    rc = np.mean(np.hypot(cxs, cys))
    ax.plot(rc * np.cos(t), rc * np.sin(t), "--", color="tab:blue", lw=0.8)
    ax.plot(0, 0, "k+", ms=14, mew=2, label="(0,0) - heading measured from here")
    ax.plot(cx, cy, "x", color="tab:red", ms=10, mew=2, label="fitted centre")
    ax.axhline(0, color="grey", lw=0.4)
    ax.axvline(0, color="grey", lw=0.4)
    ax.set_aspect("equal")
    ax.set_xlabel("mx (G)")
    ax.set_ylabel("my (G)")
    ax.set_title("Horizontal field: goal = blue circle centred on +")
    ax.legend(fontsize=7, loc="best")

    # 2. headings over time
    ax = fig.add_subplot(1, 3, 2)
    idx = np.arange(n)
    if has_bno:
        ok, e_cor, off_cor, *_ = compare(cor_hdg, bno)
        ax.plot(idx, bno, "-", color="k", lw=1.2, label="BNO $IIHDG (reference)")
        ax.plot(idx, (cor_hdg - off_cor) % 360, ".", color="tab:blue", ms=3,
                label="QMC corrected (offset-aligned)")
    if np.count_nonzero(~np.isnan(hc)) > 10:
        ax.plot(idx, hc, ".", color="tab:red", ms=3, label="QMC $HCHDG as sent")
    else:
        ax.plot(idx, raw_hdg, ".", color="tab:red", ms=3, label="QMC uncorrected")
    ax.set_ylim(0, 360)
    ax.set_yticks(range(0, 361, 45))
    ax.set_xlabel("sample (10 per second)")
    ax.set_ylabel("heading (deg)")
    ax.set_title("Headings during the capture")
    ax.legend(fontsize=7, loc="best")

    # 3. error vs heading
    ax = fig.add_subplot(1, 3, 3)
    if has_bno:
        okr, e_raw, *_ = compare(raw_hdg, bno)
        ax.plot(bno[okr], e_raw, ".", color="tab:red", ms=4, label="uncorrected")
        ax.plot(bno[ok], e_cor, ".", color="tab:blue", ms=4, label="corrected")
        ax.axhline(0, color="k", lw=0.6)
        ax.set_xlim(0, 360)
        ax.set_xticks(range(0, 361, 45))
        ax.set_xlabel("BNO heading (deg)")
        ax.set_ylabel("QMC - BNO (deg)")
        ax.set_title("Error vs heading: goal = flat line near 0")
        ax.legend(fontsize=7)
    else:
        ax.text(0.5, 0.5, "No $IIHDG lines in log\n(BNO comparison unavailable)",
                ha="center", va="center", transform=ax.transAxes)
        ax.set_axis_off()

    fig.tight_layout()
    if a.save:
        fig.savefig(a.save, dpi=130)
        print(f"Figure saved to {a.save}")
    plt.show()


if __name__ == "__main__":
    main()
