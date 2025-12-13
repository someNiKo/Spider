"""ZeroMQ publisher for servo mode1 test

This script computes sinusoidal angles for all joints (mode1 from
`servo_control_test.py`) and publishes them via a PUB socket as JSON.

Usage (examples):
    python Tests/servo_control_publisher.py --amp 50 --freq 0.2 --base 90 --send-hz 50

Notes:
- By default this script will try to import `Src.Drivers.Transmit.servo_control`
  and use `servo.read_joint_angle()` to discover joint names. If that fails,
  you can pass `--joints` with a comma-separated list of joint names.
"""
from pathlib import Path
import sys
import time
import math
import argparse

FILE = Path(__file__).resolve()
ROOT = FILE.parents[2]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from Src.DDS.publisher import Publisher

def discover_joints():
    try:
        from Src.Drivers.Transmit import servo_control
        sv = servo_control.servo()
        # try read_joint_angle without starting transmitter
        snapshot = sv.read_joint_angle()
        if isinstance(snapshot, dict) and snapshot:
            return list(snapshot.keys())
    except Exception:
        pass
    # fallback: common joint names - adjust if your robot differs
    return [
        "L1_coxa","L1_femur","L1_tibia",
        "L2_coxa","L2_femur","L2_tibia",
        "L3_coxa","L3_femur","L3_tibia",
        "R1_coxa","R1_femur","R1_tibia",
        "R2_coxa","R2_femur","R2_tibia",
        "R3_coxa","R3_femur","R3_tibia",
    ]

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--amp", type=float, default=50.0, help="amplitude (deg)")
    p.add_argument("--freq", type=float, default=0.2, help="frequency (Hz)")
    p.add_argument("--base", type=float, default=90.0, help="base angle (deg)")
    p.add_argument("--send-hz", type=float, default=50.0, help="publish rate")
    p.add_argument("--joints", type=str, default=None, help="comma separated joint names")
    p.add_argument("--bind", type=str, default="tcp://*:5556", help="PUB bind address")
    args = p.parse_args()

    if args.joints:
        joints = [j.strip() for j in args.joints.split(",") if j.strip()]
    else:
        joints = discover_joints()

    publisher = Publisher(
        bind=args.bind,
        topic="servo.angles",
        publish_hz=args.send_hz,
        warmup=0.0,
        add_meta=True,
    )
    publisher.start()

    print(f"Publisher bound to {args.bind}, topic 'servo.angles', publishing to {len(joints)} joints")
    time.sleep(1.0)  # allow subscribers to connect

    try:
        t = 0.0
        dt = 1.0 / float(args.send_hz)
        while True:
            snapshot = {}
            for name in joints:
                snapshot[name] = float(args.base + args.amp * math.sin(2.0 * math.pi * args.freq * t))

            payload = {"angles": snapshot, "time": t}
            publisher.publish_once(payload)

            t += dt
            time.sleep(dt)
    except KeyboardInterrupt:
        print("publisher stopped")
    finally:
        try:
            publisher.stop()
        except Exception:
            pass

if __name__ == "__main__":
    main()
