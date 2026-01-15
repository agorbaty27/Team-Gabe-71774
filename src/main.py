#region VEXcode Generated Robot Configuration
from vex import *
import urandom
import math

# Brain should be defined by default
brain=Brain()

# Robot configuration code


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
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
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
# Solenoid on Port H (A side = extend, B side = retract)
piston1 = DigitalOut(brain.three_wire_port.f)  # piston on side A
piston2 = DigitalOut(brain.three_wire_port.e)
piston3 = DigitalOut(brain.three_wire_port.d)


# Drive motors
left_motor_1 = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
left_motor_2 = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
left_motor_3 = Motor(Ports.PORT13, GearSetting.RATIO_18_1, True)

# Right side motors
right_motor_1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
right_motor_2 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
right_motor_3 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)

# Group drive motors
left_drive = MotorGroup(left_motor_1, left_motor_2, left_motor_3)
right_drive = MotorGroup(right_motor_1, right_motor_2, right_motor_3)

drivetrain = DriveTrain(left_drive, right_drive)

intake_motor_1 = Motor(Ports.PORT20, GearSetting.RATIO_18_1, True)
intake_motor_3 = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)


intake = MotorGroup(intake_motor_1, intake_motor_3)          

# Odometry sensors
odom_sensor = Rotation(Ports.PORT14)
odom_sensor.reset_position()

WHEEL_DIAMETER_IN = 2.0
WHEEL_CIRCUMFERENCE_IN = math.pi * WHEEL_DIAMETER_IN
TICKS_PER_REV = 360.0  # Rotation sensor gives degrees, 360 per full turn

inertial_sensor = Inertial(Ports.PORT15)
inertial_sensor.calibrate()
brain.screen.print("Calibrating Inertial...")
while inertial_sensor.is_calibrating():
    wait(100, MSEC)
brain.screen.clear_screen()
brain.screen.print("Inertial Ready")

# -----------------------------------------------------------
# BASIC AUTON MOVEMENT HELPERS
# -----------------------------------------------------------
# -----------------------------------------------------------
# ODOM + INERTIAL SIMPLE AUTON HELPERS
# -----------------------------------------------------------

def drive_inches_odom(inches, speed=50):
    """Drive forward/backward using the rotation sensor for distance."""
    target_degrees = (inches / WHEEL_CIRCUMFERENCE_IN) * 360

    odom_sensor.reset_position()
    left_drive.set_velocity(speed, PERCENT)
    right_drive.set_velocity(speed, PERCENT)

    direction = 1 if inches > 0 else -1

    left_drive.spin(FORWARD if direction == 1 else REVERSE)
    right_drive.spin(FORWARD if direction == 1 else REVERSE)

    while abs(odom_sensor.position(DEGREES)) < abs(target_degrees):
        wait(10, MSEC)

    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)


def turn_degrees_inertial(angle, speed=40):
    """Turn the robot using the inertial sensor for accurate heading."""
    inertial_sensor.reset_heading()
    drivetrain.set_turn_velocity(speed, PERCENT)

    if angle > 0:
        drivetrain.turn_for(RIGHT, angle, DEGREES)
    else:
        drivetrain.turn_for(LEFT, -angle, DEGREES)


def run_path(steps):

    for step in steps:
        action = step[0]

        if action == "drive":
            drive_inches_odom(step[1])

        elif action == "turn":
            turn_degrees_inertial(step[1])

        elif action == "intake":
            power = step[1]
            time_sec = step[2]
            intake.spin(FORWARD if power > 0 else REVERSE, abs(power), PERCENT)
            wait(time_sec, SECONDS)

        elif action == "intake_stop":
            intake.stop(COAST)

        wait(50, MSEC)




SMOOTHING = 0.12  # lower = smoother (0.05–0.3 typical)
EXPONENT = 1.5    # exponential response curve (2 = mild, 3 = strong)

# Start at 0 speed for ramping
current_left_speed = 0.0
current_right_speed = 0.0

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

# Odom constants

def driving():
    global current_left_speed, current_right_speed 
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
    turn = expo_curve(turn) * 0.5

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

    wait (10, MSEC)

def intaking():
    # Main intake (R1/R2)
    if controller.buttonR1.pressing():
        intake.spin(FORWARD, 100, PERCENT)
    elif controller.buttonR2.pressing():
        intake.spin(REVERSE, 100, PERCENT)

    elif controller.buttonL2.pressing():
        intake_motor_1.spin(REVERSE, 100, PERCENT)
        intake_motor_3.spin(FORWARD, 100, PERCENT)
      
    elif controller.buttonL1.pressing():
        intake.spin(FORWARD, 100, PERCENT)

    else:
        intake.stop(COAST)


    wait(10, MSEC)

    
def piston():
    if controller.buttonX.pressing():
        piston1.set(True)

    if controller.buttonB.pressing():
        piston1.set(False)

    if controller.buttonUp.pressing():
        piston2.set(False)

    if controller.buttonDown.pressing():
        piston2.set(True)

    if controller.buttonY.pressing():
        piston3.set(False)

    if controller.buttonA.pressing():
        piston3.set(True)

    

def autonomous():
    path = [
        ("drive", 2)
    ]
    run_path(path)


def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")

    while True:
        driving()
        intaking()
        piston()

       


            # Prevent CPU overload
        wait(20, MSEC)

# Competition instance
comp = Competition(user_control, autonomous)

brain.screen.clear_screen()