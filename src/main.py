#region VEXcode Generated Robot Configuration
from vex import *
import urandom
import math

# Brain should be defined by default
brain = Brain()

# wait for rotation sensor to fully initialize
wait(30, MSEC)

# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(random))
      
# Set random seed 
initializeRandomSeed()

def play_vexcode_sound(sound_name):
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

wait(200, MSEC)
print("\033[2J")
#endregion VEXcode Generated Robot Configuration

# ---------------------------------------------------------------------------- #
# 	Module:       main.py
# 	Author:       ari
# 	Description:  V5 Arcade Drive with Independent Intakes
# ---------------------------------------------------------------------------- #

# Controller setup
controller = Controller(PRIMARY)

# Pneumatics
piston = DigitalOut(brain.three_wire_port.h)
piston_state = False
button_pressed = False

# Drive motors
left_motor_1 = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
left_motor_2 = Motor(Ports.PORT12, GearSetting.RATIO_18_1, False)
left_motor_3 = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)

# Right side motors
right_motor_1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
right_motor_2 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)
right_motor_3 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)

# Group drive motors
left_drive = MotorGroup(left_motor_1, left_motor_2, left_motor_3)
right_drive = MotorGroup(right_motor_1, right_motor_2, right_motor_3)


intake_motor_1 = Motor(Ports.PORT20, GearSetting.RATIO_18_1, False)
intake_motor_2 = Motor(Ports.PORT6, GearSetting.RATIO_18_1, True)
intake_motor_3 = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)

# Group all intake motors together
intake = MotorGroup(intake_motor_1, intake_motor_2, intake_motor_3)

SMOOTHING = 0.15  # lower = smoother (0.05–0.3 typical)
EXPONENT = 2.4    # exponential response curve (2 = mild, 3 = strong)

# Start at 0 speed for ramping
current_left_speed = 0
current_right_speed = 0

# -----------------------------
# Helper Function: Exponential Joystick Mapping
# -----------------------------
def expo_curve(value):
    """Map joystick input (-100 to 100) to exponential curve."""
    sign = 1 if value >= 0 else -1
    normalized = abs(value) / 100
    curved = (normalized ** EXPONENT) * 100
    return curved * sign

DEADBAND = 5

def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # Autonomous routine here

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")

    while True:
                # --- DRIVE CONTROL ---
        forward = controller.axis3.position()
        turn = controller.axis1.position()

        # Apply deadband
        if abs(forward) < DEADBAND:
            forward = 0
        if abs(turn) < DEADBAND:
            turn = 0

        # Apply exponential response
        forward = expo_curve(forward)
        turn = expo_curve(turn)

        # Calculate target speeds
        target_left = forward + turn
        target_right = forward - turn

        # Clamp to range
        target_left = max(-100, min(100, target_left))
        target_right = max(-100, min(100, target_right))

        # --- SMOOTHING / ACCELERATION CONTROL ---
        current_left_speed += (target_left - current_left_speed) * SMOOTHING
        current_right_speed += (target_right - current_right_speed) * SMOOTHING

        # Spin drive motors
        left_drive.spin(FORWARD, current_left_speed, PERCENT)
        right_drive.spin(FORWARD, current_right_speed, PERCENT)

        if controller.buttonR1.pressing():
            intake.spin(FORWARD, 100, PERCENT)
        elif controller.buttonR2.pressing():
            intake.spin(REVERSE, 100, PERCENT)
        else:
            intake.stop(COAST)

            # Prevent CPU overload
        wait(20, MSEC)

# Competition instance
comp = Competition(user_control, autonomous)

brain.screen.clear_screen()
