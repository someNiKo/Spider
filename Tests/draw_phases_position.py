# ...existing code...
import sys
import time
import threading
import argparse
from pathlib import Path
from typing import Dict, Any
from collections import deque

FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from Src.DDS.subscriber import Subscriber
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np
import itertools

class FrameBuffer:
    def __init__(self):
        self.lock = threading.Lock()
        self.positions: Dict[str, Any] = {}
        self.phases = []

    def update(self, positions, phases):
        with self.lock:
            # positions expected: {leg: [x,y,z], ...}
            self.positions = dict(positions) if positions is not None else {}
            self.phases = list(phases) if phases is not None else []

    def snapshot(self):
        with self.lock:
            return dict(self.positions), list(self.phases)

def run(connect: str, topic: str, refresh_hz: float, trail_len: int, window_sec: float = 5.0):
    buf = FrameBuffer()

    count = 3
    # whether axes limits have been initialized (we set them once and do not autoscale every frame)
    _limits_initialized = {"set": False}
    # per-leg phase history and lines
    phase_history: Dict[str, deque] = {}
    phase_lines: Dict[str, Any] = {}
    
    def on_message(payload, topic_received=None):
        if not isinstance(payload, dict):
            return
        # 支持两种字段名：positions 或 feet
        positions = payload.get("positions") or payload.get("feet") or {}
        phases = payload.get("phases") or []

        nonlocal count
        if count == 1:
            buf.update(positions, phases)
            count = 3
        else:
            count -= 1


    sub = Subscriber(connect=connect, topic=topic, on_message=on_message, recv_timeout=0.5)
    sub.start()

    # matplotlib setup
    plt.ion()
    # make left 3D plot larger than the right phase plot using GridSpec
    # Desired: left takes 2/3 width, right 1/3; left subplot approx 4:3 (width:height)
    fig = plt.figure(figsize=(16, 8))
    # width_ratios [2,1] -> left 2/3, right 1/3
    gs = fig.add_gridspec(1, 2, width_ratios=[2, 1], wspace=0.12)
    ax3d = fig.add_subplot(gs[0, 0], projection='3d')
    ax_phase = fig.add_subplot(gs[0, 1])
    # tighten figure margins so subplots fill more of the canvas
    fig.subplots_adjust(left=0.03, right=0.97, top=0.96, bottom=0.06)
    # set 3D box aspect so x and y are equal, and z is stretched a bit (Matplotlib >=3.3)
    try:
        ax3d.set_box_aspect((1.0, 1.0, 0.7))
    except Exception:
        pass
    # try to make the right (phase) subplot visually square (1:1)
    try:
        ax_phase.set_box_aspect((1, 1))
    except Exception:
        try:
            ax_phase.set_aspect('equal', adjustable='box')
        except Exception:
            pass

    ax3d.set_title("Foot positions (3D)")
    ax3d.set_xlabel("X")
    ax3d.set_ylabel("Y")
    ax3d.set_zlabel("Z")
    ax3d.grid(True)

    ax_phase.set_title("Oscillator phases (time window)")
    ax_phase.set_xlabel('Time (s)')
    ax_phase.set_ylabel('Phase (rad)')
    # phase lines per leg will be created dynamically; fix Y range to [0, 2pi]
    ax_phase.set_ylim(0.0, 2.0 * np.pi)

    markers = {}
    trails = {}
    colors = itertools.cycle(plt.cm.tab10.colors)

    def update(_):
        positions, phases = buf.snapshot()
        # ensure leg artists
        for leg, pos in positions.items():
            try:
                x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
            except Exception:
                continue
            if leg not in markers:
                c = next(colors)
                # create a small marker (no label) and a persistent trail line we can use for legend
                m = ax3d.scatter([x], [y], [z], color=c, s=40)
                line = ax3d.plot([], [], [], '-', color=c, linewidth=1, alpha=0.7)[0]
                trails[leg] = {"pts": [], "line": line, "marker": m}
                markers[leg] = m
                # create phase history and line with same color
                phase_history[leg] = deque()
                phase_lines[leg] = ax_phase.plot([], [], '-', color=c, linewidth=1.5, label=leg)[0]
                # update legends using persistent artists (trail lines and phase lines)
                try:
                    ax3d.legend(handles=[trails[k]["line"] for k in sorted(trails.keys())],
                                labels=[k for k in sorted(trails.keys())],
                                loc='upper left', fontsize='small')
                except Exception:
                    pass
                try:
                    ax_phase.legend(handles=[phase_lines[k] for k in sorted(phase_lines.keys())],
                                    labels=[k for k in sorted(phase_lines.keys())],
                                    loc='upper right', fontsize='small')
                except Exception:
                    pass
            # update marker and trail
            entry = trails[leg]
            entry["pts"].append((x, y, z))
            if len(entry["pts"]) > trail_len:
                entry["pts"].pop(0)
            xs = [p[0] for p in entry["pts"]]
            ys = [p[1] for p in entry["pts"]]
            zs = [p[2] for p in entry["pts"]]
            entry["line"].set_data(xs, ys)
            try:
                entry["line"].set_3d_properties(zs)
            except Exception:
                pass
            # update scatter by removing and replotting (matplotlib 3D scatter artists are not easily mutable)
            entry["marker"].remove()
            entry["marker"] = ax3d.scatter([x], [y], [z], color=entry["line"].get_color(), s=40)
            markers[leg] = entry["marker"]

        # set 3D limits only once (do not autoscale every frame)
        all_pts = [p for p in positions.values() if len(p) >= 3]
        if (not _limits_initialized["set"]) and all_pts:
            xs = [float(p[0]) for p in all_pts]
            ys = [float(p[1]) for p in all_pts]
            zs = [float(p[2]) for p in all_pts]
            # dynamic padding based on data span to avoid excessive white space
            dx = max(xs) - min(xs)
            dy = max(ys) - min(ys)
            dz = max(zs) - min(zs)
            # fall back to a small span if data is constant
            if dx == 0:
                dx = 1.0
            if dy == 0:
                dy = 1.0
            if dz == 0:
                dz = 1.0
            pad_x = max(0.2, 0.02 * dx)
            pad_y = max(0.2, 0.02 * dy)
            pad_z = max(0.2, 0.02 * dz)
            ax3d.set_xlim(min(xs) - pad_x, max(xs) + pad_x)
            ax3d.set_ylim(min(ys) - pad_y, max(ys) + pad_y)
            ax3d.set_zlim(-140.0, -120)
            _limits_initialized["set"] = True

        # update phase time-series for each leg
        now = time.time()
        # build mapping leg->phase
        phase_map: Dict[str, float] = {}
        if isinstance(phases, dict):
            phase_map = {k: float(v) for k, v in phases.items()}
        elif isinstance(phases, (list, tuple)):
            if positions and len(phases) == len(positions):
                keys = list(positions.keys())
                phase_map = {k: float(v) for k, v in zip(keys, phases)}
            else:
                phase_map = {f"L{i+1}": float(v) for i, v in enumerate(phases)}

        for leg, ph in phase_map.items():
            if leg not in phase_history:
                # assign a color if new (use colors cycle)
                c = next(colors)
                phase_history[leg] = deque()
                phase_lines[leg] = ax_phase.plot([], [], '-', color=c, linewidth=1.5, label=leg)[0]
                ax_phase.legend(loc='upper right', fontsize='small')
            phase_history[leg].append((now, ph % (2 * np.pi)))
            # purge old samples
            while phase_history[leg] and (now - phase_history[leg][0][0]) > window_sec:
                phase_history[leg].popleft()

        # update phase line data
        for leg, line in phase_lines.items():
            data = phase_history.get(leg, [])
            if data:
                xs = [t for t, _ in data]
                ys = [p for _, p in data]
                line.set_data(xs, ys)
        # update x limits to fixed time window
        ax_phase.set_xlim(now - window_sec, now)

        fig.canvas.draw_idle()
        return []

    interval_ms = int(1000.0 / max(1.0, refresh_hz))
    ani = FuncAnimation(fig, update, interval=interval_ms, blit=False, cache_frame_data=False)
    try:
        plt.show(block=True)
    finally:
        sub.stop()


def main():
    parser = argparse.ArgumentParser(description="订阅 positions+phases 并实时绘图（3D + 相位图）")
    parser.add_argument("--connect", default="tcp://127.0.0.1:6000", help="PUB socket address to connect to")
    parser.add_argument("--topic", default="servo.angles", help="topic to subscribe")
    parser.add_argument("--hz", type=float, default=20.0, help="刷新频率 (Hz)")
    parser.add_argument("--trail", type=int, default=60, help="每条腿轨迹长度 (points)")
    args = parser.parse_args()
    run(connect=args.connect, topic=args.topic, refresh_hz=args.hz, trail_len=args.trail)


if __name__ == "__main__":
    main()