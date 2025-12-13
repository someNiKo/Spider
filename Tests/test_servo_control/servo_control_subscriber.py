"""ZeroMQ subscriber that runs servo control and applies incoming angles.

This script starts a `servo_control.servo()` instance (and calls `start()`),
connects to a PUB socket and receives JSON messages with joint-angle dicts.
It applies received angles with `set_all_angle` if available.

Usage:
    python Tests/servo_control_subscriber.py --connect tcp://127.0.0.1:5556

Note: run this in a separate terminal/process from the publisher.
"""
from pathlib import Path
import sys
import time
import argparse

FILE = Path(__file__).resolve()
ROOT = FILE.parents[2]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from Src.DDS.subscriber import Subscriber


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--connect", type=str, default="tcp://127.0.0.1:5556", help="PUB connect address")
    p.add_argument("--topic", type=str, default="servo.angles", help="topic to subscribe")
    p.add_argument("--timeout", type=float, default=1.0, help="receive timeout (seconds, 0 for blocking)")
    args = p.parse_args()

    from Src.Drivers.Transmit import servo_control

    servo = servo_control.servo()
    servo.start()

    stop_requested = False

    def _handle(payload, topic):
        nonlocal stop_requested
        if not isinstance(payload, dict):
            print("unexpected message format", payload)
            return

        cmd = payload.get("cmd") or payload.get("command")
        if isinstance(cmd, str) and cmd.lower() in {"interrupt", "stop", "emergency_stop", "e_stop"}:
            stop_requested = True
            return
        if payload.get("interrupt") is True or payload.get("stop") is True:
            stop_requested = True
            return

        angles = None
        candidate = payload.get("angles")
        if isinstance(candidate, dict):
            angles = candidate
        elif all(isinstance(v, (int, float)) for v in payload.values()):
            angles = payload

        if angles:
            try:
                servo.set_all_angle(angles)
            except Exception as e:
                print("failed to set angles:", e)

    sub = Subscriber(
        connect=args.connect,
        topic=args.topic,
        recv_timeout=args.timeout if args.timeout and args.timeout > 0 else 0.0,
        on_message=_handle,
    )
    sub.start()

    print(f"Subscriber connected to {args.connect}, topic '{args.topic}', servo started")

    try:
        while not stop_requested:
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("subscriber stopping")
    finally:
        try:
            sub.stop()
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
