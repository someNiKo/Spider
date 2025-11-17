import numpy as np

# ==== serial config ====
PORT = "COM7"          # Windows: "COM3" / "COM7" ...
# PORT = "/dev/ttyUSB0" # Linux
# PORT = "/dev/tty.usbserial-xxxx"  # macOS

BAUD = 460800

# ==== control config ====

CONTROL_FREQUENCE = 200

SEND_ORDER = [
    "L1_coxa", "L1_femur", "L1_tibia",
    "L2_coxa", "L2_femur", "L2_tibia",
    "L3_coxa", "L3_femur", "L3_tibia",
    "R1_coxa", "R1_femur", "R1_tibia",
    "R2_coxa", "R2_femur", "R2_tibia",
    "R3_coxa", "R3_femur", "R3_tibia",    
]

DEFAULT_JOINT_ANGLE = {
    "L1_coxa":90.0, "L1_femur":90.0, "L1_tibia":90.0,
    "L2_coxa":90.0, "L2_femur":90.0, "L2_tibia":90.0,
    "L3_coxa":90.0, "L3_femur":90.0, "L3_tibia":90.0,
    "R1_coxa":90.0, "R1_femur":90.0, "R1_tibia":90.0,
    "R2_coxa":90.0, "R2_femur":90.0, "R2_tibia":90.0,
    "R3_coxa":90.0, "R3_femur":90.0, "R3_tibia":90.0,     
}