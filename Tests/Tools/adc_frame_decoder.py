"""Serial ADC frame decoder.

Reads binary frames produced by the STM32 firmware (header 0xAA55,
sequence uint16 little-endian, six uint16 samples) and prints them as
human-readable lines. Designed for quick inspection on Windows.
"""

from __future__ import annotations

import argparse
from collections import deque
import sys
import time
from dataclasses import dataclass
from typing import Deque, Iterable, Optional

from rich.console import Console
from rich.live import Live
from rich.table import Table
import threading
from collections import deque as _deque

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit("pyserial is required: pip install pyserial") from exc

HEADER = 0xAA55
CHANNEL_COUNT = 6
BYTES_PER_FRAME = 2 + 2 + (CHANNEL_COUNT * 2)


@dataclass
class Frame:
    sequence: int
    samples: tuple[int, ...]

    def __str__(self) -> str:
        sample_text = ", ".join(f"ch{i}: {val}" for i, val in enumerate(self.samples))
        return f"seq={self.sequence:05d} | {sample_text}"


def parse_stream(port: serial.Serial) -> Iterable[Frame]:
    buffer = bytearray()
    while True:
        chunk = port.read(1)
        if not chunk:
            continue
        buffer += chunk
        while len(buffer) >= BYTES_PER_FRAME:
            if buffer[0] != (HEADER >> 8) or buffer[1] != (HEADER & 0xFF):
                buffer.pop(0)
                continue
            frame_bytes = bytes(buffer[:BYTES_PER_FRAME])
            buffer = buffer[BYTES_PER_FRAME:]
            sequence = int.from_bytes(frame_bytes[2:4], "little")
            samples = tuple(
                int.from_bytes(frame_bytes[4 + i * 2 : 6 + i * 2], "little")
                for i in range(CHANNEL_COUNT)
            )
            yield Frame(sequence=sequence, samples=samples)


def _build_table(frames: Deque[tuple[int, tuple[int, ...], Optional[float]]]) -> Table:
    table = Table(title="ADC Live Stream", caption="Ctrl+C to exit", expand=True)
    table.add_column("Seq", justify="right", style="cyan", no_wrap=True)
    table.add_column("Rate (Hz)", justify="right", style="magenta")
    for idx in range(CHANNEL_COUNT):
        table.add_column(f"CH{idx}", justify="right")

    if not frames:
        table.add_row("--", "--", *(["--"] * CHANNEL_COUNT))
        return table

    for seq, samples, rate in frames:
        rate_text = f"{rate:.1f}" if rate is not None else "--"
        sample_cells = [str(val) for val in samples]
        table.add_row(f"{seq:05d}", rate_text, *sample_cells)
    return table


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Decode ADC frames from serial port")
    parser.add_argument("port", help="Serial port name, e.g. COM5")
    parser.add_argument(
        "--baud",
        type=int,
        default=460800,
        help="Baud rate (default: 460800)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.5,
        help="Read timeout in seconds",
    )
    parser.add_argument(
        "--rows",
        type=int,
        default=20,
        help="Number of recent frames to keep in the table (default: 20)",
    )
    parser.add_argument(
        "--plot",
        choices=("matplotlib", "ascii", "none"),
        default="none",
        help="Enable live plotting (matplotlib recommended).",
    )
    parser.add_argument(
        "--plot-window",
        type=int,
        default=200,
        help="Number of recent frames to display in plot (default: 200)",
    )
    parser.add_argument(
        "--no-table",
        action="store_true",
        help="Do not show the rich table (useful when plotting)",
    )
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    try:
        port = serial.Serial(args.port, baudrate=args.baud, timeout=args.timeout)
    except serial.SerialException as exc:
        parser.error(f"Unable to open {args.port}: {exc}")

    console = Console()
    console.print(f"Listening on [bold]{args.port}[/] @ {args.baud} baud (Ctrl+C to exit)")

    last_time: Optional[float] = None
    last_seq: Optional[int] = None
    history: Deque[tuple[int, tuple[int, ...], Optional[float]]] = deque(maxlen=max(1, args.rows))

    # If plotting is requested, create a shared deque for plot data and run parse_stream in a background thread.
    plot_frames: Optional[_deque] = None
    reader_thread: Optional[threading.Thread] = None
    stop_event = threading.Event()

    def reader(bg_port: serial.Serial, out_deque: _deque):
        try:
            for frame in parse_stream(bg_port):
                if stop_event.is_set():
                    break
                now = time.perf_counter()
                out_deque.append((frame.sequence, frame.samples, now))
        except Exception:
            pass

    if args.plot != "none":
        plot_frames = _deque(maxlen=max(1, args.plot_window))
        reader_thread = threading.Thread(target=reader, args=(port, plot_frames), daemon=True)
        reader_thread.start()

    try:
        if args.plot == "none":
            with Live(_build_table(history), console=console, refresh_per_second=30) as live:
                try:
                    for frame in parse_stream(port):
                        now = time.perf_counter()
                        rate: Optional[float] = None
                        if last_time is not None and last_seq is not None:
                            dt = now - last_time
                            seq_delta = (frame.sequence - last_seq) & 0xFFFF
                            if dt > 0:
                                rate = seq_delta / dt
                        history.append((frame.sequence, frame.samples, rate))
                        live.update(_build_table(history))
                        last_time = now
                        last_seq = frame.sequence
                except KeyboardInterrupt:
                    console.print("\nStopping serial decoder.")
        else:
            # Plotting mode: use matplotlib to show live plots. Skip the rich Live table unless requested.
            try:
                import matplotlib.pyplot as plt
                from matplotlib.animation import FuncAnimation
            except Exception as exc:  # pragma: no cover
                parser.error(f"matplotlib is required for plotting: {exc}")

            if not args.no_table:
                console.print("Table display disabled when plotting; use --no-table to suppress this message.")

            channels = CHANNEL_COUNT
            fig, ax = plt.subplots(channels, 1, sharex=True, figsize=(8, 2 * channels))
            if channels == 1:
                ax = [ax]
            lines = []
            for ch in range(channels):
                line, = ax[ch].plot([], [], label=f"CH{ch}")
                ax[ch].set_ylabel(f"CH{ch}")
                ax[ch].legend(loc="upper right")
                lines.append(line)
            ax[-1].set_xlabel("Frames")

            def update_plot(frame_idx):
                if not plot_frames:
                    return lines
                data = list(plot_frames)
                if not data:
                    return lines
                seqs = [d[0] for d in data]
                samples = [d[1] for d in data]
                # transpose samples to per-channel lists
                chan_vals = list(zip(*samples))
                x = list(range(len(seqs)))
                for ch in range(channels):
                    y = chan_vals[ch] if ch < len(chan_vals) else [0] * len(x)
                    lines[ch].set_data(x, y)
                    ax[ch].relim()
                    ax[ch].autoscale_view()
                return lines

            ani = FuncAnimation(fig, update_plot, interval=100, blit=False)
            try:
                plt.show()
            except KeyboardInterrupt:
                pass
    except KeyboardInterrupt:
        console.print("\nStopping serial decoder.")
    finally:
        stop_event.set()
        try:
            port.close()
        except Exception:
            pass
        if reader_thread is not None:
            reader_thread.join(timeout=1.0)

    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
