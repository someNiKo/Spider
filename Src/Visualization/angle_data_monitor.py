import time
import threading
from pathlib import Path
import sys
from typing import Callable, Dict, Optional

FILE = Path(__file__).resolve()
ROOT = FILE.parents[2]  # e:\Desktop\spider
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


class AngleMonitor:
    """Monitor that periodically calls `read_function()` to obtain a dict of
    {name: angle} and displays it in a small TUI table.

    Behavior:
    - Starts a daemon thread on construction that reads at `refresh_hz` and
      updates a TUI rendered with `rich` if available, otherwise falls back to
      `curses`, then to simple printing.
    - Provides `stop()` to request thread shutdown and `join()` to wait.
    - Supports context-manager (`with AngleMonitor(...) as m:`) which will stop
      the monitor on exit.
    """

    def __init__(self, read_function: Callable[[], Dict[str, float]], refresh_hz: float = 10.0):
        self.read_function = read_function
        self.refresh_hz = float(refresh_hz) if refresh_hz > 0 else 1.0
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None
        # Start the worker thread immediately
        self.start()

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._worker, daemon=True, name="AngleMonitor")
        self._thread.start()

    def stop(self, timeout: Optional[float] = None) -> None:
        """Request the worker to stop and optionally wait up to `timeout` seconds."""
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout)

    def join(self, timeout: Optional[float] = None) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()

    def _render_table(self, snapshot: Dict[str, float]):
        # Local import to avoid adding hard dependency at module import time
        from rich.table import Table
        table = Table(title="Servo Angles (deg)")
        table.add_column("Idx", justify="right", no_wrap=True)
        table.add_column("Name", justify="left")
        table.add_column("Angle (°)", justify="right")
        names = list(snapshot.keys())
        for i, name in enumerate(names):
            v = snapshot.get(name, float('nan'))
            table.add_row(str(i), name, f"{v:.1f}")
        return table

    def _worker(self) -> None:
        interval = 1.0 / max(1.0, self.refresh_hz)

        # Prefer rich for cross-platform nice table rendering
        try:
            from rich.console import Console
            from rich.live import Live
            console = Console()

            last: Dict[str, float] = {}
            # Use Live correctly: keep the Live instance and call `live.update()`
            with Live(self._render_table(last), console=console, refresh_per_second=self.refresh_hz) as live:
                while not self._stop_evt.is_set():
                    try:
                        snapshot = self.read_function() or {}
                    except Exception:
                        snapshot = {}
                    last = snapshot
                    try:
                        live.update(self._render_table(last))
                    except Exception:
                        # Fallback to console printing if update fails for any reason
                        try:
                            console.clear()
                            console.print(self._render_table(last))
                        except Exception:
                            pass
                    time.sleep(interval)
            return
        except Exception:
            # rich not available, try curses
            try:
                import curses
            except Exception:
                # final fallback: simple print loop
                last = {}
                while not self._stop_evt.is_set():
                    try:
                        snapshot = self.read_function() or {}
                    except Exception:
                        snapshot = {}
                    last = snapshot
                    preview = ", ".join(f"{n}:{last.get(n, float('nan')):.1f}" for n in list(last.keys())[:6])
                    print(f"[AngleMonitor] {preview}")
                    time.sleep(interval)
                return

            def _curses_loop(stdscr):
                curses.curs_set(0)
                stdscr.nodelay(True)
                title = "Servo Angles (deg) — press q to quit viewer"
                last = {}
                while not self._stop_evt.is_set():
                    try:
                        snapshot = self.read_function() or {}
                    except Exception:
                        snapshot = {}
                    last = snapshot

                    stdscr.erase()
                    maxy, maxx = stdscr.getmaxyx()
                    stdscr.addnstr(0, 0, title, maxx - 1)
                    stdscr.addnstr(2, 0, f"{'Idx':>3}  {'Name':<16}  {'Angle(°)':>9}", maxx - 1)
                    stdscr.hline(3, 0, ord('-'), maxx - 1)
                    for i, name in enumerate(list(last.keys())):
                        y = 4 + i
                        if y >= maxy - 1:
                            break
                        val = last.get(name, float('nan'))
                        line = f"{i:>3}  {name:<16}  {val:>9.1f}"
                        stdscr.addnstr(y, 0, line, maxx - 1)
                    stdscr.refresh()
                    ch = stdscr.getch()
                    if ch in (ord('q'), ord('Q')):
                        self._stop_evt.set()
                        break
                    time.sleep(interval)

            curses.wrapper(_curses_loop)
