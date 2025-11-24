# -*- coding: utf-8 -*-
"""
在 gait_control 的基础上新增“角度偏置（度）”功能：
- 仅在发送与显示时加偏置；内部状态 last_deg 始终存储“未偏置”的角度，避免 IK 回退时重复偏置。
- 偏置优先级：按通道名覆盖 > 按关节类型 > 全局默认。
"""

import time
import math
from typing import Dict, List
from threading import Thread, Event
from queue import Queue, Empty

import numpy as np

import Paras
import cpg
import bezier
from ik import inverse_kinematics

# 复用串口参数与打包函数
import servo_control

# 引入连杆转换：phi(theta)（弧度制）。若不可用则回退为恒等。
try:
    from link import phi as _theta_to_phi_rad  # 输入/输出单位：弧度
except Exception:
    def _theta_to_phi_rad(theta_rad: float) -> float:
        return theta_rad


# 选择步态配置：Forward / Sidle / Turn
LEG_CONFIG = Paras.LEG_CONFIG_Forward

# 发送频率与时间步长
SEND_HZ = servo_control.SEND_HZ
DT = 1.0 / SEND_HZ

# 自定义发送顺序（索引基于 Paras.motor_names）
SEND_ORDER = [
    0, 1, 2,
    6, 7, 8,
    12, 13, 14,
    3, 4, 5,
    9, 10, 11,
    15, 16, 17,
]

# TUI 窗口配置（基于 curses 的表格显示）
TUI_ENABLED = True            # 是否开启 TUI 表格窗口
TUI_HZ = 30.0                 # 刷新频率（帧/秒），建议 20~60 之间

# 角度偏置配置（单位：度）
BIAS_DEFAULT_DEG = 90.0
BIAS_BY_TYPE_DEG = {
    'coxa':  90.0,
    'femur': 90.0,
    'tibia': 90.0,
}
BIAS_BY_NAME_DEG: Dict[str, float] = {
    # 'leg1_left_coxa': 85.0,
}

# 右侧腿关于 90° 对称（v' = 180 - v）开关
RIGHT_SYMMETRY_90_ENABLED = True

# 关节数值来源映射门户（按需交换 coxa/femur/tibia 对应的数值来源）
# 含义：目的关节 -> 数值来源关节
# 例：
#   身份映射（默认）：'coxa': 'coxa', 'femur': 'femur', 'tibia': 'tibia'
#   交换 femur 与 tibia：'coxa': 'coxa', 'femur': 'tibia', 'tibia': 'femur'
JOINT_VALUE_MAP: Dict[str, str] = {
    'coxa':  'coxa',
    'femur': 'femur',
    'tibia': 'tibia',
}


def _format_header_line(names: List[str]) -> str:
    cols = ["time_s"] + names
    return ",".join(cols)


def _format_data_line(t: float, angles_deg_18: List[float]) -> str:
    vals = [f"{t:.3f}"] + [f"{v:.1f}" for v in angles_deg_18]
    return ",".join(vals)


def _joint_type_from_name(name: str) -> str:
    return name.split('_')[-1]


def _bias_deg_for(name: str) -> float:
    jt = _joint_type_from_name(name)
    return BIAS_BY_NAME_DEG.get(name, BIAS_BY_TYPE_DEG.get(jt, BIAS_DEFAULT_DEG))


def apply_joint_value_mapping(base_by_name: Dict[str, float]) -> Dict[str, float]:
    """
    按 JOINT_VALUE_MAP 将每条腿的目标关节数值替换为来源关节的数值，名称不变。
    例如：{'femur': 'tibia'} 表示 femur 槽位使用 tibia 的数值。
    """
    out = dict(base_by_name)
    for leg in LEG_CONFIG.keys():
        for jt in ('coxa', 'femur', 'tibia'):
            src_jt = JOINT_VALUE_MAP.get(jt, jt)
            dst_name = f"{leg}_{jt}"
            src_name = f"{leg}_{src_jt}"
            if src_name in base_by_name:
                out[dst_name] = base_by_name[src_name]
    return out


def apply_right_symmetry_90(angles_deg_18: List[float], ordered_names: List[str], enabled: bool = True) -> List[float]:
    """
    将右侧各腿（名称包含 "_right_"）的角度做关于 90° 的对称：v' = 180 - v。
    只作用于输出角度（已完成映射与偏置的值）。
    """
    if not enabled:
        return angles_deg_18
    out = list(angles_deg_18)
    for i, name in enumerate(ordered_names):
        if "_right_tibia" in name:
            out[i] = 180.0 - out[i]
        elif "_left_tibia" in name:
            out[i] = 180.0 - out[i]
    
    return out


def _tui_worker(names: List[str], q_in: Queue, stop_evt: Event, refresh_hz: float):
    """后台线程：curses TUI 表格显示 angles（度）。"""
    try:
        import curses
    except Exception:
        # 回退：使用 rich 实现跨平台表格（适用于 VSCode/PowerShell）
        try:
            from rich.console import Console
            from rich.table import Table
            from rich.live import Live
        except Exception:
            # 若 rich 不可用，退回到简易打印
            last = {}
            interval = 1.0 / max(1.0, refresh_hz)
            while not stop_evt.is_set():
                drained = 0
                try:
                    while True:
                        t, by = q_in.get_nowait()
                        last = by
                        drained += 1
                        if drained > 8:
                            break
                except Empty:
                    pass
                if last:
                    preview = ", ".join(f"{n}:{last.get(n, float('nan')):.1f}" for n in names[:6])
                    print(f"[TUI] {preview} …")
                time.sleep(interval)
            return

        console = Console()

        def render_table(snapshot: Dict[str, float]):
            table = Table(title="Hexapod Servo Angles (deg)")
            table.add_column("Idx", justify="right", no_wrap=True)
            table.add_column("Name", justify="left")
            table.add_column("Angle (°)", justify="right")
            for i, n in enumerate(names):
                v = snapshot.get(n, float('nan')) if snapshot else float('nan')
                table.add_row(str(i), n, f"{v:.1f}")
            return table

        last = {}
        interval = 1.0 / max(1.0, refresh_hz)
        with Live(render_table(last), console=console, refresh_per_second=refresh_hz) as live:
            while not stop_evt.is_set():
                drained = 0
                try:
                    while True:
                        t, by = q_in.get_nowait()
                        last = by
                        drained += 1
                        if drained > 8:
                            break
                except Empty:
                    pass
                live.update(render_table(last))
                time.sleep(interval)
        return

    def _loop(stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)
        title = "Hexapod Servo Angles (deg) — press q to quit viewer"
        interval = 1.0 / max(1.0, refresh_hz)
        last = {}
        while not stop_evt.is_set():
            # drain latest
            drained = 0
            try:
                while True:
                    t, by = q_in.get_nowait()
                    last = by
                    drained += 1
                    if drained > 8:
                        break
            except Empty:
                pass

            stdscr.erase()
            maxy, maxx = stdscr.getmaxyx()
            stdscr.addnstr(0, 0, title, maxx-1)
            # 表头
            stdscr.addnstr(2, 0, f"{'Idx':>3}  {'Name':<16}  {'Angle(°)':>9}", maxx-1)
            stdscr.hline(3, 0, ord('-'), maxx-1)
            # 内容
            for i, name in enumerate(names):
                y = 4 + i
                if y >= maxy-1:
                    break
                val = last.get(name, float('nan')) if last else float('nan')
                line = f"{i:>3}  {name:<16}  {val:>9.1f}"
                stdscr.addnstr(y, 0, line, maxx-1)
            stdscr.refresh()

            ch = stdscr.getch()
            if ch in (ord('q'), ord('Q')):
                stop_evt.set()
                break
            time.sleep(interval)

    import curses
    curses.wrapper(_loop)


def _start_tui_if_needed():
    if not TUI_ENABLED:
        return None, None, None
    q = Queue(maxsize=1)  # 只保留最新一帧，避免积压造成延迟
    names = [Paras.motor_names[i] for i in SEND_ORDER]
    stop_evt = Event()
    th = Thread(target=_tui_worker, args=(names, q, stop_evt, TUI_HZ), daemon=True)
    th.start()
    return th, q, stop_evt


def rad_to_deg_clamped(rad_val: float) -> float:
    return float(np.degrees(rad_val))


# 安全角度处理与 theta->phi（弧度制）容错转换
def _sanitize_deg(v: float, default: float = 90.0) -> float:
    """将角度（度）中的 NaN/Inf 替换为默认值，并夹紧到 0..180°。"""
    if not np.isfinite(v):
        return default
    return float(min(180.0, max(0.0, v)))


def _safe_theta2phi_deg(theta_deg: float) -> float:
    """theta(度)->phi(度)；使用 link.phi(弧度) 进行转换，越界/异常时返回原值或默认值。"""
    if not np.isfinite(theta_deg):
        return 90.0
    with np.errstate(invalid='ignore', divide='ignore', over='ignore', under='ignore'):
        try:
            phi_rad = _theta_to_phi_rad(np.radians(theta_deg))
        except Exception:
            return theta_deg
    if phi_rad is None or not np.isfinite(phi_rad):
        return theta_deg
    return float(np.degrees(phi_rad))


def build_angles_base_deg(phases: List[float], last_angles_base: Dict[str, float]):
    """
    计算未加偏置的角度（度）。
    返回：
      base_by_name: dict[name] = 度（无偏置）
    """
    base_by_name: Dict[str, float] = {}

    for leg, cfg in LEG_CONFIG.items():
        osc = cfg['osc']
        theta = phases[osc] % (2 * np.pi)

        if isinstance(cfg['P3'], (int, float)):
            target = bezier.ArcBezier(cfg['P1'], cfg['P3'], theta, Paras.Z_LIFT, Paras.Z_DOWN)
        else:
            target = bezier.Bezier(cfg['P1'], cfg['P3'], theta, Paras.Z_LIFT, Paras.Z_DOWN)

        try:
            θ1, θ2, θ3 = inverse_kinematics(target)
            side = leg.split('_')[-1]
            sign = Paras.leg_sign[side]
            raw = {
                'coxa':  Paras.initial_offsets.get(f"{leg}_coxa", 0.0)  + sign * θ1,
                'femur': Paras.initial_offsets.get(f"{leg}_femur", 0.0) + sign * θ2,
                'tibia': Paras.initial_offsets.get(f"{leg}_tibia", 0.0) + sign * θ3,
            }
            for jt, rad_val in raw.items():
                name = f"{leg}_{jt}"
                base_by_name[name] = rad_to_deg_clamped(rad_val)
        except Exception:
            # IK 不可达：沿用上一帧无偏置值
            for jt in ('coxa', 'femur', 'tibia'):
                name = f"{leg}_{jt}"
                base_by_name[name] = last_angles_base.get(name, 90.0)

    return base_by_name


def main():
    # 打开串口（若失败则进入离线发送模式，仅显示/打印）
    ser = None
    try:
        ser = servo_control.serial.Serial(servo_control.PORT, servo_control.BAUD, timeout=0)
        print(f"Opened {servo_control.PORT} @ {servo_control.BAUD} baud. Sending {SEND_HZ:.0f} Hz frames… Ctrl+C to stop.")
    except Exception as e:
        print(f"[WARN] Serial open failed: {e}. Running in display/print-only mode.")

    # CPG 初相
    phases = cpg.initial_phases.copy()
    t = 0.0
    seq = 0

    # 上一帧“无偏置”角度备份
    last_base: Dict[str, float] = {name: 90.0 for name in Paras.motor_names}

    # TUI 表格窗口
    ordered_names = [Paras.motor_names[i] for i in SEND_ORDER]
    tui_thread, tui_q, tui_stop = _start_tui_if_needed()

    try:
        while True:
            # 1) 更新 CPG 相位（欧拉）
            dy = cpg.coupled_oscillators2(t, phases)
            phases = [phases[i] + dy[i] * DT for i in (0, 1)]
            t += DT

            # 2) 计算无偏置角度（度）
            base_by_name = build_angles_base_deg(phases, last_base)
            last_base.update(base_by_name)

            # 3) 应用关节数值映射（交换/重定向来源）
            mapped_by_name = apply_joint_value_mapping(base_by_name)

            # 4) 应用偏置并排序成 18 路
            angles_deg_18 = [mapped_by_name.get(n, 90.0) + _bias_deg_for(n) for n in ordered_names]
            # 4.1) 右侧各腿关于 90° 对称（v' = 180 - v）
            angles_deg_18 = apply_right_symmetry_90(angles_deg_18, ordered_names, enabled=RIGHT_SYMMETRY_90_ENABLED)

            # 4.2) 将所有腿的第 3 关节（tibia 槽位）当前值视作 theta（度），使用 link.phi 进行 theta->phi（弧度），再转回度
            #     注意：此处 tibia 的数值来源由 JOINT_VALUE_MAP 决定，当前配置为 tibia<-coxa
            angles_deg_18 = [
                _safe_theta2phi_deg(v) if _joint_type_from_name(ordered_names[i]) == 'tibia' else v
                for i, v in enumerate(angles_deg_18)
            ]

            # 4.3) 发送前最终安全化：替换 NaN/Inf，夹紧至 0..180°
            angles_deg_18 = [_sanitize_deg(v) for v in angles_deg_18]

            # 5) 组帧并发送
            frame = servo_control.build_frame(seq, angles_deg_18)
            if ser is not None:
                ser.write(frame)
            seq = (seq + 1) & 0xFFFF

            # 6) TUI 表格更新（尽力只保留最新，避免延迟）
            if TUI_ENABLED and tui_q is not None:
                try:
                    # 若已满，丢弃旧的，推入最新
                    if tui_q.full():
                        try:
                            _ = tui_q.get_nowait()
                        except Empty:
                            pass
                    by_name = {ordered_names[i]: angles_deg_18[i] for i in range(len(ordered_names))}
                    tui_q.put_nowait((t, by_name))
                except Exception:
                    pass

            # 8) 固定周期
            time.sleep(DT)

    except KeyboardInterrupt:
        pass
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        # 请求关闭 TUI 线程
        # 注意：该线程是 daemon，主进程结束会自动退出
        # 这里尝试通知一下
        try:
            if TUI_ENABLED and tui_stop is not None:
                tui_stop.set()
        except Exception:
            pass
        print("Serial closed.")


if __name__ == "__main__":
    main()

