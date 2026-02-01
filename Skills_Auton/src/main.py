#region VEXcode Generated Robot Configuration
from vex import *
import urandom #type: ignore
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

controller = Controller(PRIMARY)

piston1 = DigitalOut(brain.three_wire_port.f)  
piston2 = DigitalOut(brain.three_wire_port.e)
piston3 = DigitalOut(brain.three_wire_port.d)
piston4 = DigitalOut(brain.three_wire_port.b)

left_motor_1 = Motor(Ports.PORT11, GearSetting.RATIO_6_1, True)
left_motor_2 = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True)
left_motor_3 = Motor(Ports.PORT13, GearSetting.RATIO_6_1, True)

right_motor_1 = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
right_motor_2 = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
right_motor_3 = Motor(Ports.PORT3, GearSetting.RATIO_6_1, False)

left_drive = MotorGroup(left_motor_1, left_motor_2, left_motor_3)
right_drive = MotorGroup(right_motor_1, right_motor_2, right_motor_3)

drivetrain = DriveTrain(left_drive, right_drive)

intake_motor_1 = Motor(Ports.PORT20, GearSetting.RATIO_6_1, True)
intake_motor_3 = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)


intake = MotorGroup(intake_motor_1, intake_motor_3)          

odom_sensor = Rotation(Ports.PORT14)
odom_sensor.reset_position()

WHEEL_DIAMETER_IN = 2.0
WHEEL_CIRCUMFERENCE_IN = math.pi * WHEEL_DIAMETER_IN
TICKS_PER_REV = 360.0  

# Tuning multipliers - adjust these to calibrate odometry and turning
ODOMETRY_CORRECTION = 0.87  # Multiply drive distance by this value
TURNING_CORRECTION = 0.76   # Multiply turn angle by this value

inertial_sensor = Inertial(Ports.PORT15)
inertial_sensor.calibrate()
brain.screen.print("Calibrating Inertial...")
while inertial_sensor.is_calibrating():
    wait(100, MSEC)
brain.screen.clear_screen()
brain.screen.print("Inertial Ready")

def drive_inches_odom(inches, speed=40):
    """Drive forward/backward using the rotation sensor for distance."""
    corrected_inches = inches * ODOMETRY_CORRECTION
    target_degrees = ((corrected_inches / WHEEL_CIRCUMFERENCE_IN) * 360)
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
    wait(10,MSEC)


def turn_degrees_inertial(angle, speed=40):
    """Turn the robot using the inertial sensor for accurate heading."""
    corrected_angle = angle * TURNING_CORRECTION
    inertial_sensor.reset_heading()
    drivetrain.set_turn_velocity(speed, PERCENT)

    if corrected_angle > 0:
        drivetrain.turn_for(RIGHT, corrected_angle, DEGREES)
    else:
        drivetrain.turn_for(LEFT, -corrected_angle, DEGREES)
    wait(10,MSEC)


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
            intake.spin(REVERSE if power > 0 else FORWARD, abs(power), PERCENT)
            wait(time_sec, SECONDS)
            intake.stop(COAST)
        
        elif action == "score":
            power = step[1]
            time_sec = step[2]
            intake.spin(REVERSE if power > 0 else FORWARD, abs(power), PERCENT)
            piston4.set(True)
            wait(time_sec, SECONDS)
            intake.stop(COAST)

        elif action == "scraper":
            state = step[1]
            piston2.set(True if state == "down" else False)

        wait(50, MSEC)




def simplify_path(waypoints, tolerance=1.0):
    """Simplify path by removing intermediate points on lines. Keeps corners."""
    if len(waypoints) <= 2:
        return waypoints
    
    simplified = [waypoints[0]]
    i = 1
    
    while i < len(waypoints):
        current_x, current_y = simplified[-1]
        j = i
        line_points = [waypoints[i]]
        
        # Check if vertical line (x constant within tolerance)
        if abs(waypoints[i][0] - current_x) < tolerance:
            while j < len(waypoints) and abs(waypoints[j][0] - current_x) < tolerance:
                line_points.append(waypoints[j])
                j += 1
            simplified.append(line_points[-1])
            i = j
        
        # Check if horizontal line (y constant within tolerance)
        elif abs(waypoints[i][1] - current_y) < tolerance:
            while j < len(waypoints) and abs(waypoints[j][1] - current_y) < tolerance:
                line_points.append(waypoints[j])
                j += 1
            simplified.append(line_points[-1])
            i = j
        
        else:
            simplified.append(waypoints[i])
            i += 1
    
    return simplified


def convert_jerry_path(path):
    """
    Convert (x, y) path into TURN + DRIVE commands using atan2 for direction.
    """
    commands = []
    current_heading = 0  # degrees

    for i in range(1, len(path)):
        x1, y1 = path[i - 1]
        x2, y2 = path[i]

        dx = x2 - x1
        dy = y2 - y1

        # Ignore zero-length steps
        if dx == 0 and dy == 0:
            continue

        # Calculate target heading using atan2
        target_heading = math.degrees(math.atan2(dy, dx))
        
        # Calculate heading change needed
        heading_diff = target_heading - current_heading
        
        # Normalize angle to -180 to 180 range
        while heading_diff > 180:
            heading_diff -= 360
        while heading_diff < -180:
            heading_diff += 360
        
        # Add turn command if heading changed significantly
        if abs(heading_diff) > 5:
            commands.append(("turn", heading_diff))
            current_heading = target_heading

        # Calculate distance
        distance = math.sqrt(dx**2 + dy**2)
        
        # Add drive command
        if distance > 0.5:
            commands.append(("drive", distance))

    return commands


SMOOTHING = 0.12
DEADBAND = 5

current_left_speed = 0.0
current_right_speed = 0.0


def apply_deadband(value):
    return 0 if abs(value) < DEADBAND else value


def driving():
    global current_left_speed, current_right_speed

    forward = apply_deadband(controller.axis3.position())
    turn = apply_deadband(controller.axis1.position()) * 0.5  

    if forward == 0 and turn == 0:
        current_left_speed = 0
        current_right_speed = 0
        left_drive.stop(COAST)
        right_drive.stop(COAST)
        return


    target_left = forward + turn
    target_right = forward - turn

    target_left = max(-100, min(100, target_left))
    target_right = max(-100, min(100, target_right))


    current_left_speed += (target_left - current_left_speed) * SMOOTHING
    current_right_speed += (target_right - current_right_speed) * SMOOTHING


    left_drive.spin(FORWARD if current_left_speed >= 0 else REVERSE, abs(current_left_speed), PERCENT)
    right_drive.spin(FORWARD if current_right_speed >= 0 else REVERSE, abs(current_right_speed), PERCENT)

    wait(10, MSEC)


def intaking():
    # Main intake (R1/R2)
    if controller.buttonR1.pressing():
        intake.spin(FORWARD, 100, PERCENT)
    elif controller.buttonR2.pressing():
        intake.spin(REVERSE, 100, PERCENT)
    
    elif controller.buttonL2.pressing():
        intake.spin(REVERSE, 100, PERCENT)
        piston4.set(True)
    
    elif controller.buttonL1.pressing():
        intake.spin(FORWARD, 100, PERCENT)

    else:
        intake.stop(COAST)
        piston4.set(False)


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
    wait(10, MSEC)


def autonomous():
    global current_left, current_right
    current_left = 0
    current_right = 0

    path = [
        ("drive", 30),
        ("turn", -90),
        ] 
    
    path_2 = [
        ("drive", -10),
        ("turn", -90),
        ("drive", 10),
        ("turn", 90),
        ("drive", 82),
        ("turn", 90),
        ("drive", 10),
        ("turn", -90),
              ]
    path_3 = [
        ("drive", -10),
        ("turn", -90),
        ("drive", 90), #important distance
        ("turn", -90),
    ]
    
    path_4 = [
        ("drive", -10),
        ("turn", -110), #important turn
        ("drive", 50),  #important distance
    ]


    def matchload_score():
        piston2.set(True)
        wait(1, SECONDS)
        intake.spin(REVERSE, 100, PERCENT)
        drivetrain.drive(FORWARD, 75, PERCENT)
        wait(1.5, SECONDS)
        drivetrain.stop()
        drivetrain.drive(REVERSE, 70, PERCENT)
        wait(1.5, SECONDS)
        piston4.set(True)
        intake.spin(REVERSE, 100, PERCENT)
        wait(1, SECONDS)
        drivetrain.stop()
        wait(0.75, SECONDS)
        intake.stop(COAST)
        piston2.set(False)


    run_path(path)
    matchload_score()
    run_path(path_2)
    matchload_score()
    run_path(path_3)
    matchload_score()
    run_path(path_2)
    matchload_score()
    run_path(path_4)



def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")

    while True:
        driving()
        intaking()
        piston()


       
        wait(20, MSEC)

comp = Competition(user_control, autonomous)

brain.screen.clear_screen()