"""Interactive servo angle publisher.

Broadcasts joint-angle dictionaries over ZeroMQ while providing a simple
command-line interface to update individual joints, e.g.::

    L1_coxa 32.2

Designed to pair with ``Tests/tripod_gait_subscriber.py`` which expects payloads
containing ``{"seq": int, "t": float, "angles": dict}``.
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, Optional

FILE = Path(__file__).resolve()
ROOT = FILE.parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from Src.Drivers.Transmit import config as cfg
from Src.DDS.publisher import Publisher


@dataclass
class AngleCache:
    """Thread-safe angle cache shared by the publisher and CLI."""

    _angles: Dict[str, float] = field(default_factory=dict)
    _lock: threading.RLock = field(default_factory=threading.RLock, init=False)

    def snapshot(self) -> Dict[str, float]:
        with self._lock:
            return dict(self._angles)

    def update(self, updates: Dict[str, float]) -> None:
        if not updates:
            return
        with self._lock:
            self._angles.update(updates)


def _format_preview(angles: Dict[str, float], limit: int = 6) -> str:
    items = list(angles.items())[:limit]
    return ", ".join(f"{name}:{value:.1f}" for name, value in items)


def _interactive_loop(
    cache: AngleCache,
    stop_event: threading.Event,
    valid_names: Iterable[str],
) -> None:
    valid_lookup = {name for name in valid_names}
    prompt = (
        "Type '<joint> <angle>' to update, 'show' to display current values, "
        "or 'exit' to quit."
    )
    print(prompt)
    while not stop_event.is_set():
        try:
            line = input("> ").strip()
        except EOFError:
            break
        except KeyboardInterrupt:
            print("")
            break

        if not line:
            continue

        lower = line.lower()
        if lower in {"exit", "quit"}:
            stop_event.set()
            break
        if lower in {"show", "print"}:
            snapshot = cache.snapshot()
            print(_format_preview(snapshot, limit=len(snapshot)))
            continue
        if lower in {"help", "?"}:
            print(prompt)
            continue
        if lower == "list":
            print("Available joints:")
            print(" ".join(sorted(valid_lookup)))
            continue

        parts = line.split()
        if len(parts) != 2:
            print("Invalid command. Expected '<joint> <angle>'.")
            continue

        name, value_text = parts
        try:
            angle = float(value_text)
        except ValueError:
            print(f"Invalid angle: '{value_text}'.")
            continue

        angle = max(0.0, min(180.0, angle))

        if valid_lookup and name not in valid_lookup:
            print(f"Unknown joint '{name}'. Use 'list' to inspect available joints.")
            continue

        cache.update({name: angle})
        updated = cache.snapshot()
        confirmed = updated.get(name, angle)
        print(f"{name} -> {confirmed:.2f} deg | {_format_preview(updated)}")


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Servo angle publisher")
    parser.add_argument("--bind", default="tcp://*:6000", help="ZeroMQ bind endpoint")
    parser.add_argument("--pub-hz", type=float, default=20.0, help="publish frequency")
    parser.add_argument("--warmup", type=float, default=0.2, help="publisher warmup")
    parser.add_argument(
        "--no-cli",
        action="store_true",
        help="disable the stdin interface (publisher only)",
    )
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = _build_arg_parser()
    args = parser.parse_args(argv)

    cache = AngleCache(cfg.DEFAULT_JOINT_ANGLE.copy())

    stop_event = threading.Event()

    def _handle_sigint(signum, frame):
        stop_event.set()

    signal.signal(signal.SIGINT, _handle_sigint)

    def _payload_cb():
        if stop_event.is_set():
            return None
        snapshot = cache.snapshot()
        return {"angles": snapshot}

    publisher = Publisher(
        bind=args.bind,
        topic="servo.angles",
        publish_hz=args.pub_hz,
        warmup=args.warmup,
        add_meta=True,
        auto_payload_cb=_payload_cb,
    )
    publisher.start()

    print(
        f"Publishing angles on {args.bind} (topic='servo.angles') at {args.pub_hz:.1f} Hz. Press Ctrl+C to stop."
    )

    try:
        if args.no_cli:
            while not stop_event.is_set():
                time.sleep(0.2)
        else:
            _interactive_loop(cache, stop_event, cfg.SEND_ORDER)
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        stop_event.set()
        try:
            publisher.stop()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
