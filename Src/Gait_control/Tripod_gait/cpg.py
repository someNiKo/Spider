import numpy as np
import matplotlib.pyplot as plt

# ——— 参数 ———
omega = 0.5   # 基本频率（Hz）
K     = 0.5   # 耦合强度

# 初始相位，可自行修改
initial_phases = [0.0, np.pi]  # φ1(0)=0.0 rad, φ2(0)=1.0 rad

def coupled_oscillators2(t, y):
    """
    两个振荡器的耦合动力学方程。
    y: 长度为 2 的相位向量 [φ1, φ2]
    返回 dy/dt，也是长度为 2 的列表
    """
    target_y = [0.0, np.pi]  # 期望相差 π
    dy = []
    for i in range(2):
        j = 1 - i
        coupling = np.sin(y[j] - y[i] - (target_y[i] - target_y[j]))
        dy_i = 2 * np.pi * omega + K * coupling
        dy.append(dy_i)
    return dy

if __name__ == "__main__":
    # 时间设置
    dt = 0.01      # 步长 (s)
    T  = 10        # 总时长 (s)
    steps = int(T / dt)
    ts = np.linspace(0, T, steps)

    # 状态初始化
    y = initial_phases.copy()
    φ1 = np.zeros(steps)
    φ2 = np.zeros(steps)

    # 仿真循环（欧拉积分）
    for k in range(steps):
        φ1[k], φ2[k] = y
        dy = coupled_oscillators2(ts[k], y)
        y = [y[i] + dy[i] * dt for i in range(2)]

    # 绘图
    plt.figure(figsize=(8,4))
    plt.plot(ts, φ1 % (2*np.pi), label='φ1')
    plt.plot(ts, φ2 % (2*np.pi), label='φ2')
    plt.xlabel('Time (s)')
    plt.ylabel('Phase (rad)')
    plt.title('Two Coupled Oscillators (initial phases = '
              f'{initial_phases[0]:.2f}, {initial_phases[1]:.2f})')
    plt.legend()
    plt.grid(True)
    plt.show()
