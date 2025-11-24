## Overview

This workspace hosts the Python control stack for the ZJU hexapod robot. It provides gait generation, robot kinematics, hardware transport, visualization, and diagnostic tooling that complement the embedded firmware running on the STM32 controller. The codebase is organized to keep locomotion logic, transport drivers, and developer tooling loosely coupled but interoperable through shared data structures and messaging APIs.

## Directory Structure

- `Src/Drivers/Transmit/` — serial transport utilities for sending servo frames. The primary entry point is `servo_control.py`, which manages UART framing, CRC, and thread-safe angle updates using a single-slot queue.
- `Src/Gait_control/` — locomotion algorithms and robot geometry models.
	- `Robot/robot_geometry_model.py` models each leg, performs inverse/forward kinematics, enforces servo limits, and exposes a `Spider_robot` aggregate.
	- `Tripod_gait/` contains gait parameterization (`config.py`), the coupled-oscillator phase model (`cpg.py`), Bezier trajectory helpers (`bezier.py`), and the `TripodGait` generator (`tripod_gait.py`).
	- `Gait_controller/gait_controller.py` integrates gait outputs with the robot model, steps the control loop, and can publish servo frames via ZeroMQ for downstream consumers.
- `Src/Visualization/` — tooling for inspecting telemetry. `angle_data_monitor.py` renders live joint angles using a Rich-based TUI with graceful fallbacks when Rich or curses are unavailable.
- `Tests/` — integration scripts and regression harnesses. Key examples include:
	- `tripod_gait_publisher.py` and `tripod_gait_subscriber.py` for exercising the gait controller over ZeroMQ, visualizing angles, and driving real hardware through `servo_control`.
	- `servo_control_publisher.py` and `servo_control_subscriber.py` for generic sinusoidal testing of the servo transport layer.
	- `test_angle_monitor_mode1.py` demonstrating how to pair the servo sine-wave test with the angle monitor UI.
- `Docs/` — reference material and configuration guides for operators and developers.

## Data Flow

1. A gait implementation (such as `TripodGait`) produces desired foot endpoints from time and gait configuration.
2. `GaitController` maps these endpoints through `Spider_robot`, yielding joint angles and servo setpoints that satisfy mechanical constraints.
3. `servo_control.servo` packages the 18-channel frame, enforces angle limits, and transmits it over UART to the controller board. The driver prioritizes the freshest data via a single-slot queue to minimize latency.
4. Optional ZeroMQ publishers broadcast the same servo payloads to visualization or logging clients. Subscribers (for example, `tripod_gait_subscriber.py`) can render the data with `AngleMonitor` while simultaneously forwarding it to the servo driver.

## Key Design Features

- **Modular gait architecture** — swapping gait definitions or oscillator parameters does not require changes to the controller core.
- **Thread-safe transport** — the servo driver isolates UART handling in a worker thread while exposing safe setters for high-level modules.
- **Pluggable messaging** — ZeroMQ enables remote observation, logging, or simulation without binding tooling to the UART driver.
- **Rich-based telemetry** — the angle monitor offers an informative TUI with automatic fallbacks, making debugging consistent across platforms.

## Getting Started

1. Install Python dependencies listed in `requirements.txt`.
2. Adjust serial settings in `Src/Drivers/Transmit/config.py` to match your hardware (port, baud rate, control frequency, servo order).
3. Launch the gait demo:
	 ```bash
	 python Tests/tripod_gait_publisher.py
	 python Tests/tripod_gait_subscriber.py --connect tcp://127.0.0.1:6000
	 ```
	 The subscriber streams angles to the servo driver and displays them via the angle monitor UI.
4. Use `Tests/test_angle_monitor_mode1.py` for quick sinusoidal sweeps while monitoring joint telemetry.

## Contributing

- Keep module boundaries clear: gait logic belongs under `Src/Gait_control/`, transport drivers under `Src/Drivers/`, and tooling within `Src/Visualization/` or `Tests/`.
- Update or add integration tests whenever transport protocols, gait algorithms, or visualization layers change.
- Document non-trivial additions in this README or in `Docs/` to ease onboarding for future contributors.

This workspace focuses solely on the Python-side architecture. The embedded firmware that runs on the STM32 controller lives in a separate repository and interacts with this codebase via the UART protocol implemented in `servo_control.py`.
