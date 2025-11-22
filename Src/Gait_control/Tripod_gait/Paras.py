# Paras.py

import numpy as np
# ——— 连杆长度 (单位同 Webots 世界尺度) ———
L1 = 30.1    # Coxa
L2 = 60.91   # Femur
L3 = 152.32  # Tibia

side = 57.73
# ——— 电机名称列表（请与 .wbt 中的 DEF 保持一致） ———
motor_names = [
    'leg1_left_coxa',   'leg1_left_femur',   'leg1_left_tibia',
    'leg1_right_coxa',  'leg1_right_femur',  'leg1_right_tibia',
    'leg2_left_coxa',   'leg2_left_femur',   'leg2_left_tibia',
    'leg2_right_coxa',  'leg2_right_femur',  'leg2_right_tibia',
    'leg3_left_coxa',   'leg3_left_femur',   'leg3_left_tibia',
    'leg3_right_coxa',  'leg3_right_femur',  'leg3_right_tibia'
]

# ——— 对应每个电机的初始偏移角（rad）——上面读取的数据———
initial_offsets = {
    'leg1_left_coxa':    0.0000,  'leg1_left_femur':   0,  'leg1_left_tibia':  0,
    'leg1_right_coxa':   0.0000,  'leg1_right_femur': 0,  'leg1_right_tibia': 0,
    'leg2_left_coxa':    0.0000,  'leg2_left_femur':   0,  'leg2_left_tibia':  0,
    'leg2_right_coxa':   0.0000,  'leg2_right_femur': 0,  'leg2_right_tibia': 0,
    'leg3_left_coxa':   -0.0000,  'leg3_left_femur':   0,  'leg3_left_tibia':  0,
    'leg3_right_coxa':   0.0000,  'leg3_right_femur': 0,  'leg3_right_tibia': 0
}

# 新增：每类关节的增量
delta_changes = {
    'femur': 0.6,  # femur 关节的增量
    'tibia': 0.3   # tibia 关节的增量
}

Z_LIFT   = 20    # 抬腿高度
Z_DOWN   = 0   # 支撑下压

Y_STEP = 15

X_STEP = 20.0
# ——— CPG 相关 ———
# 初始相位：两个振荡器
initial_phases = [0.0, np.pi]

initial_pos = [110, 0, -89.84]
# ——— 每条腿的轨迹端点及所属振荡器 ———
LEG_CONFIG_Forward = {
    # osc = 0
    'leg2_left':  {
        'osc': 0,
        'P1' : [ initial_pos[0], initial_pos[1] - 2*Y_STEP, initial_pos[2] ],
        'P3' : [ initial_pos[0], initial_pos[1] + 2*Y_STEP, initial_pos[2] ]
    },
    'leg3_right': {
        'osc': 0,
        'P1' : [ initial_pos[0] + Y_STEP*np.sqrt(3), initial_pos[1] - Y_STEP, initial_pos[2] ],
        'P3' : [ initial_pos[0] - Y_STEP*np.sqrt(3), initial_pos[1] + Y_STEP, initial_pos[2] ]
    },
    'leg1_right': {
        'osc': 0,
        'P1' : [ initial_pos[0] - Y_STEP*np.sqrt(3), initial_pos[1] - Y_STEP, initial_pos[2] ],
        'P3' : [ initial_pos[0] + Y_STEP*np.sqrt(3), initial_pos[1] + Y_STEP, initial_pos[2] ]
    },
    # osc = 1
    'leg2_right': {
        'osc': 1,
        'P1' : [ initial_pos[0], initial_pos[1] - 2*Y_STEP, initial_pos[2] ],
        'P3' : [ initial_pos[0], initial_pos[1] + 2*Y_STEP, initial_pos[2] ]
    },
    'leg3_left':  {
        'osc': 1,
        'P1' : [ initial_pos[0] + Y_STEP*np.sqrt(3), initial_pos[1] - Y_STEP, initial_pos[2] ],
        'P3' : [ initial_pos[0] - Y_STEP*np.sqrt(3), initial_pos[1] + Y_STEP, initial_pos[2] ]
    },
    'leg1_left':  {
        'osc': 1,
        'P1' : [ initial_pos[0] - Y_STEP*np.sqrt(3), initial_pos[1] - Y_STEP, initial_pos[2] ],
        'P3' : [ initial_pos[0] + Y_STEP*np.sqrt(3), initial_pos[1] + Y_STEP, initial_pos[2] ]
    },
}

LEG_CONFIG_Sidle = {
    # osc = 0
    'leg2_left':  {
        'osc': 0,
        'P1' : [ initial_pos[0] - 2*X_STEP, initial_pos[1], initial_pos[2] ],
        'P3' : [ initial_pos[0] + 2*X_STEP, initial_pos[1], initial_pos[2] ]
    },
    'leg1_right': {
        'osc': 0,
        'P1' : [ initial_pos[0] + X_STEP, initial_pos[1] - X_STEP*np.sqrt(3), initial_pos[2] ],
        'P3' : [ initial_pos[0] - X_STEP, initial_pos[1] + X_STEP*np.sqrt(3), initial_pos[2] ]
    },
    'leg3_right': {
        'osc': 0,
        'P1' : [ initial_pos[0] + X_STEP, initial_pos[1] + X_STEP*np.sqrt(3), initial_pos[2] ],
        'P3' : [ initial_pos[0] - X_STEP, initial_pos[1] - X_STEP*np.sqrt(3), initial_pos[2] ]
    },
    # osc = 1
    'leg2_right': {
        'osc': 1,
        'P1' : [ initial_pos[0] + 2*X_STEP, initial_pos[1], initial_pos[2] ],
        'P3' : [ initial_pos[0] - 2*X_STEP, initial_pos[1], initial_pos[2] ]
    },
    'leg1_left':  {
        'osc': 1,
        'P1' : [ initial_pos[0] - X_STEP, initial_pos[1] + X_STEP*np.sqrt(3), initial_pos[2] ],
        'P3' : [ initial_pos[0] + X_STEP, initial_pos[1] - X_STEP*np.sqrt(3), initial_pos[2] ]
    },
    'leg3_left':  {
        'osc': 1,
        'P1' : [ initial_pos[0] - X_STEP, initial_pos[1] - X_STEP*np.sqrt(3), initial_pos[2] ],
        'P3' : [ initial_pos[0] + X_STEP, initial_pos[1] + X_STEP*np.sqrt(3), initial_pos[2] ]
    },
}

LEG_CONFIG_Turn = {
    # osc = 0
    'leg2_left':  {
        'osc': 0,
        'P1' : [ initial_pos[0], initial_pos[1], initial_pos[2] ],
        'P3' : 0.2
    },
    'leg1_right': {
        'osc': 0,
        'P1' : [ initial_pos[0], initial_pos[1] - X_STEP*np.sqrt(3), initial_pos[2] ],
        'P3' : -0.2
    },
    'leg3_right': {
        'osc': 0,
        'P1' : [ initial_pos[0], initial_pos[1] + X_STEP*np.sqrt(3), initial_pos[2] ],
        'P3' : -0.2
    },
    # osc = 1
    'leg2_right': {
        'osc': 1,
        'P1' : [ initial_pos[0], initial_pos[1], initial_pos[2] ],
        'P3' : -0.2
    },
    'leg1_left':  {
        'osc': 1,
        'P1' : [ initial_pos[0], initial_pos[1] + X_STEP*np.sqrt(3), initial_pos[2] ],
        'P3' : 0.2
    },
    'leg3_left':  {
        'osc': 1,
        'P1' : [ initial_pos[0], initial_pos[1] - X_STEP*np.sqrt(3), initial_pos[2] ],
        'P3' : 0.2
    },
}
# ——— 左右腿对应的符号 ———
# 用于将 IK 输出的角度转换到 Webots 里：右腿 +，左腿 –
leg_sign = {
    'left':  -1,
    'right': +1
}