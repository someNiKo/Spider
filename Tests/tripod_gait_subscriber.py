"""Subscribe to gait controller servo frames, drive hardware, and visualize angles."""

import argparse
import sys
from pathlib import Path
from typing import Dict

import zmq

FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from Src.Drivers.Transmit import servo_control
from Src.Visualization.angle_data_monitor import AngleMonitor


def _coerce_angles(raw: Dict[str, float]) -> Dict[str, float]:
    coerced: Dict[str, float] = {}
    for name, value in raw.items():
        try:
            coerced[name] = float(value)
        except (TypeError, ValueError):
            continue
    return coerced



def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--connect", default="tcp://127.0.0.1:6000", help="PUB socket to connect to")
    parser.add_argument("--timeout", type=float, default=2.0, help="receive timeout in seconds (0 for block)")
    parser.add_argument("--monitor-hz", type=float, default=10.0, help="refresh rate for AngleMonitor UI")
    args = parser.parse_args()

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(args.connect)
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    if args.timeout and args.timeout > 0:
        sub.setsockopt(zmq.RCVTIMEO, int(args.timeout * 1000))

    servo = servo_control.servo()
    servo.start()
    monitor = AngleMonitor(lambda: servo.read_joint_angle(), refresh_hz=args.monitor_hz)

    print(f"Subscribed to {args.connect}. Frames drive the servo and update the monitor. Press Ctrl+C to exit.")

    try:
        while True:
            try:
                payload = sub.recv_json()
            except zmq.Again:
                # continue waiting for frames while keeping monitor alive
                continue

            angles_dict = payload.get("angles", {})
            coerced = _coerce_angles(angles_dict)
            if coerced:
                servo.set_all_angle(coerced)

            seq = payload.get("seq")
            t = payload.get("t")
            preview = {k: round(v, 2) for k, v in list(coerced.items())[:6]}
            if t is not None:
                pass
                #print(f"seq={seq} t={float(t):.3f} -> {preview} ...")
            else:
                pass
                #print(f"seq={seq} -> {preview} ...")

    except KeyboardInterrupt:
        print("Interrupted, shutting down.")
    finally:
        try:
            monitor.stop(timeout=1.0)
        except Exception:
            pass
        try:
            monitor.join(timeout=1.0)
        except Exception:
            pass
        try:
            servo.reset_joint_angle()
        except Exception:
            pass
        try:
            servo.stop()
        except Exception:
            pass
        sub.close(0)
        ctx.term()


if __name__ == "__main__":
    main()
