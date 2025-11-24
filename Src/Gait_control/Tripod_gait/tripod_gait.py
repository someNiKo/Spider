import sys
from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[3] 
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))

from typing import Dict, Iterable, List, Optional

import numpy as np

from Src.Gait_control.Tripod_gait import config as cfg
from Src.Gait_control.Tripod_gait import cpg
from Src.Gait_control.Tripod_gait import bezier


class TripodGait:
    """Tripod gait generator driven by coupled oscillators."""

    def __init__(
        self,
        leg_config: Optional[Dict[str, Dict[str, object]]] = None,
        initial_phases: Optional[Iterable[float]] = None,
        z_lift: float = cfg.Z_LIFT,
        z_down: float = cfg.Z_DOWN,
    ) -> None:
        self.leg_config = leg_config if leg_config is not None else cfg.LEG_CONFIG_Forward
        self.z_lift = float(z_lift)
        self.z_down = float(z_down)
        self._phases: List[float] = []
        self._last_time: Optional[float] = None
        self.reset(initial_phases=initial_phases)

    def reset(
        self,
        initial_phases: Optional[Iterable[float]] = None,
        time_reference: Optional[float] = None,
    ) -> None:
        phases = list(initial_phases if initial_phases is not None else cfg.INITIAL_PHASES)
        if len(phases) != 2:
            raise ValueError("Tripod gait expects exactly two coupled oscillators")
        self._phases = [float(p) for p in phases]
        self._last_time = float(time_reference) if time_reference is not None else None

    def phases(self) -> List[float]:
        return list(self._phases)

    def sample(self, time_s: float) -> Dict[str, List[float]]:
        time_s = float(time_s)
        dt = 0.0
        if self._last_time is not None:
            dt = max(0.0, time_s - self._last_time)
        if dt > 0.0:
            derivatives = cpg.coupled_oscillators2(time_s, self._phases)
            self._phases = [self._phases[i] + float(derivatives[i]) * dt for i in range(len(self._phases))]
        self._last_time = time_s
        return self._build_foot_positions()

    def _build_foot_positions(self) -> Dict[str, List[float]]:
        positions: Dict[str, List[float]] = {}
        for leg_key, meta in self.leg_config.items():
            osc_index = int(meta['osc'])
            if osc_index < 0 or osc_index >= len(self._phases):
                raise IndexError(f"Oscillator index {osc_index} out of range for {leg_key}")
            theta = self._phases[osc_index] % (2.0 * np.pi)
            p1 = meta['P1']
            p3 = meta['P3']
            target = bezier.Bezier(p1, p3, theta, self.z_lift, self.z_down)
            robot_leg = self._to_robot_leg_key(leg_key)
            positions[robot_leg] = target.astype(float).tolist()
        return positions

    @staticmethod
    def _to_robot_leg_key(config_key: str) -> str:
        base, side = config_key.split('_', maxsplit=1)
        index = ''.join(ch for ch in base if ch.isdigit())
        if not index:
            raise ValueError(f"Invalid leg key: {config_key}")
        prefix = 'L' if 'left' in side else 'R'
        return f"{prefix}{index}"


# Backwards compatibility with previous lowercase class name.
tripod_gait = TripodGait