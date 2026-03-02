# Copilot instructions for Team-Gabe-71774

This repository contains VEX V5 Python robot code. The guidance below is focused on quickly making AI coding agents productive in this codebase.

Purpose
- Provide succinct, actionable rules for making changes to drive/auto code, sensors, and deployment-relevant logic.

Big picture
- This is VEX Python code that runs on the robot runtime (`from vex import *`). Key components live in `src/`:
  - [src/cookedautonmis](src/cookedautonmis) — compact driver + simple auton example.
  - [src/bad_auton.py](src/bad_auton.py) — richer autonomous helpers: odometry, inertial turn, `run_path`, `convert_jerry_path`.
  - [src/stupid_testing.py](src/stupid_testing.py) — another autonomous + driver variation; useful for examples of `drive_inches_odom`, `turn_degrees_inertial`.

Repository conventions (discoverable patterns)
- Hardware objects are created at top-level: `Brain()`, `Controller(PRIMARY)`, sensors like `Inertial`, `Rotation`, `Distance`, and DigitalOut pistons.
- Drive and intake use `Motor` + `MotorGroup` patterns. Variables commonly named `left_drive`, `right_drive`, `intake`.
- Driver control: smoothing + deadband constants (`SMOOTHING`, `DEADBAND`) and a `driving()` loop used from `user_control()`.
- Autonomous: command-list pattern `run_path` / `run_smooth_path` where each command is a tuple like `("drive", inches)` or `("turn", degrees)`; expand carefully when adding new actions.
- Competition harness: a `Competition(user_control, auton)` (or `comp = Competition(user_control, autonomous)`) instantiates the entry points — changing those function names will break robot startup.

Important constants and calibrations
- Odometry & turning correction constants are present and tuned per robot: e.g. `ODOMETRY_CORRECTION`, `TURNING_CORRECTION`, `WHEEL_DIAMETER_IN`. Update these when changing gear ratios or wheel hardware.
- Inertial sensor is calibrated on startup in several files; keep calibration logic when refactoring (e.g. `inertial_sensor.calibrate()` and `while inertial_sensor.is_calibrating(): wait(...)`).

Auton command set (examples)
- Existing commands: `drive`, `turn`, `intake`, `score`, `scraper`. See `run_path` implementations in [src/bad_auton.py](src/bad_auton.py).
- To add a new command, follow the existing dispatch pattern inside `run_path` and keep `wait(...)` calls between steps for timing stability.

Editing rules for safe changes
- When changing motor polarity or ports, update all `Motor(...)` constructors and test odometry constants afterward.
- Keep hardware initialization at module top-level so Competition and helper functions can reference shared objects.
- Preserve `Competition(...)` lines at the bottom of each script unless intentionally converting a file into a library module.

Running and testing notes
- These scripts import the VEX runtime (`from vex import *`) and are intended to run on a VEX V5 target. Local execution without the VEX SDK will fail unless a `vex` stub is provided.
- For quick static syntax checks, run `python -m py_compile src/<file>.py` on an environment that provides the `vex` module, or add a minimal local `vex` stub package for static analysis.

Search tips for contributors
- Look for `run_path`, `drive_inches_odom`, `turn_degrees_inertial`, `drive_smooth`, and `turn_smooth` to find autonomous control logic.
- Look for `SMOOTHING`, `DEADBAND`, `MAX_ACCEL`, and `DRIVE_SPEED` for drive tuning parameters.

When to ask for human review
- Changes that affect physical behavior: port changes, gear ratio edits, odometry constants, or inertial calibration logic — require a human to run on robot and verify.

If you need more detail
- Ask for examples of common edits (e.g., adding a new auton sequence or converting `run_path` to accept timestamps). Provide the desired change and the agent will produce a focused patch.

---
Keep edits small, reference the files above, and prefer modifying `run_path`-style logic over in-place duplicated motion code.
