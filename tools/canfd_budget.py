# SPDX-License-Identifier: Apache-2.0
"""CAN-FD per-bus joint-count budget (docs/API.md section 3.5).

Frame model (ISO 11898-1:2015 CAN FD, base 11-bit ID, BRS set), worst-case dynamic
bit stuffing; CRC17 for <= 16 data bytes, CRC21 above; fixed stuff bits in the
stuff-count + CRC field; CRC delimiter, ACK, EOF and 3-bit intermission counted at
the nominal rate. SYNC is a classic CAN frame with a 1-byte counter.
Budget: a schedule may use at most BUDGET of the cycle (rest: SDO, EMCY,
heartbeat, retransmission after an error frame, arbitration jitter).
Run: python tools/canfd_budget.py
"""
import math

DLC = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]
BUDGET = 0.70


def pad(n):
    return min(d for d in DLC if d >= n)


def frame_us(nbytes, fn, fd):
    n = pad(nbytes)
    nom = 17 + 4                      # SOF..BRS, worst stuffing on them
    dp = 1 + 4 + 8 * n                # ESI, DLC, data
    dp += (dp - 1) // 4               # worst dynamic stuffing
    sc = 4 + (17 if n <= 16 else 21)  # stuff count + CRC
    dp += sc + math.ceil(sc / 4)      # fixed stuff bits
    nom += 1 + 1 + 1 + 7 + 3          # CRC delim, ACK, ACK delim, EOF, IFS
    return nom / fn * 1e6 + dp / fd * 1e6


def sync_us(fn):
    bits = 44 + 8                     # classic data frame, 1 data byte
    bits += (34 + 8 - 1) // 4         # worst stuffing
    return bits / fn * 1e6


def cmd_time(n, per_joint_bytes, per_frame, fn, fd):
    """Group command frames: per_frame joints share one frame (dummy-mapped slices)."""
    full, rem = divmod(n, per_frame)
    t = full * frame_us(per_joint_bytes * per_frame, fn, fd)
    if rem:
        t += frame_us(per_joint_bytes * rem, fn, fd)
    return t


SCHEMES = {
    # name: (cmd bytes/joint, joints per cmd frame, feedback bytes/joint, SYNC frame?)
    "S1 CiA402 PDOs as on EtherCAT (Rx 0x1601 28 B, Tx 0x1A00 24 B), SYNC": (28, 1, 24, True),
    "S2 S1 + telemetry 0x1A01 60 B per joint": (28, 1, 24 + 60, True),  # handled below
    "S3 AM compact: group cmd 16 B/joint x4 per 64 B frame, fb 16 B, cmd frame = sync": (16, 4, 16, False),
    "S4 S3 with 32 B estimator-lite feedback": (16, 4, 32, False),
}
RATES = [(1e6, 5e6), (1e6, 2e6), (0.5e6, 2e6)]


def bus_time(n, scheme, fn, fd):
    cb, per, fb, sync = SCHEMES[scheme]
    t = sync_us(fn) if sync else 0.0
    t += cmd_time(n, cb, per, fn, fd)
    if scheme.startswith("S2"):
        t += n * (frame_us(24, fn, fd) + frame_us(60, fn, fd))
    else:
        t += n * frame_us(fb, fn, fd)
    return t


def max_joints(cycle_us, scheme, fn, fd):
    best = 0
    for n in range(1, 64):
        if bus_time(n, scheme, fn, fd) <= BUDGET * cycle_us:
            best = n
    return best


if __name__ == "__main__":
    print("Frame times, worst-case stuffing [us]")
    for fn, fd in RATES:
        row = {b: round(frame_us(b, fn, fd), 1) for b in (8, 16, 24, 32, 64)}
        print(f"  {fn/1e6:g}/{fd/1e6:g} Mbit/s: {row}  SYNC(classic,1B) {sync_us(fn):.1f}")
    print(f"\nMax joints per bus, schedule <= {BUDGET:.0%} of cycle")
    for s in SCHEMES:
        print(" ", s)
        for fn, fd in RATES:
            print(f"     {fn/1e6:g}/{fd/1e6:g} Mbit/s: 1 kHz {max_joints(1000, s, fn, fd):2d} | 500 Hz {max_joints(2000, s, fn, fd):2d}")
