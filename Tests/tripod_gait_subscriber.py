"""Subscribe to gait controller servo frames, drive hardware, and visualize angles."""

import argparse
import sys
import time
from pathlib import Path
from typing import Dict

FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from Src.Drivers.Transmit import servo_control
from Src.Visualization.angle_data_monitor import AngleMonitor
from Src.DDS.subscriber import Subscriber


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
    parser.add_argument("--topic", default="servo.angles", help="topic name to subscribe")
    parser.add_argument("--timeout", type=float, default=2.0, help="receive timeout in seconds (0 for block)")
    parser.add_argument("--monitor-hz", type=float, default=10.0, help="refresh rate for AngleMonitor UI")
    args = parser.parse_args()

    servo = servo_control.servo()
    servo.start()
    monitor = AngleMonitor(lambda: servo.read_joint_angle(), refresh_hz=args.monitor_hz)

    stop_requested = False

    def _handle(payload: Dict[str, float], topic: str) -> None:
        nonlocal stop_requested
        if not isinstance(payload, dict):
            return

        cmd = payload.get("cmd") or payload.get("command")
        if isinstance(cmd, str) and cmd.lower() in {"interrupt", "stop", "emergency_stop", "e_stop"}:
            stop_requested = True
            return
        if payload.get("interrupt") is True or payload.get("stop") is True:
            stop_requested = True
            return

        angles_dict = payload.get("angles", {})
        coerced = _coerce_angles(angles_dict) if isinstance(angles_dict, dict) else {}
        if coerced:
            servo.set_all_angle(coerced)

    sub = Subscriber(
        connect=args.connect,
        topic=args.topic,
        recv_timeout=args.timeout if args.timeout and args.timeout > 0 else 0.0,
        on_message=_handle,
    )
    sub.start()

    print(
        f"Subscribed to {args.connect} (topic='{args.topic}'). Frames drive the servo and update the monitor. Press Ctrl+C to exit."
    )

    try:
        while not stop_requested:
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("Interrupted, shutting down.")
    finally:
        sub.stop()
        try:
            monitor.stop(timeout=1.0)
            pass
        except Exception:
            pass
        try:
            monitor.join(timeout=1.0)
            pass
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


if __name__ == "__main__":
    main()
