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

# ===================== SMOOTH AUTON DRIVE SYSTEM ===================== #

# --- Tunables ---
MAX_ACCEL = 4          # % speed change per loop (lower = smoother)
DRIVE_SPEED = 55       # cruise speed
TURN_SPEED = 45
MIN_MOVE_IN = 2.0      # ignore tiny drives
MIN_TURN_DEG = 5.0     # ignore tiny turns

# --- Smooth velocity ramp ---
current_left = 0.0
current_right = 0.0

def ramp_to_speed(left_target, right_target):
    global current_left, current_right

    current_left += max(-MAX_ACCEL, min(MAX_ACCEL, left_target - current_left))
    current_right += max(-MAX_ACCEL, min(MAX_ACCEL, right_target - current_right))

    left_drive.spin(FORWARD if current_left >= 0 else REVERSE,
                    abs(current_left), PERCENT)
    right_drive.spin(FORWARD if current_right >= 0 else REVERSE,
                     abs(current_right), PERCENT)

# --- Smooth distance drive ---
def drive_smooth(inches):
    if abs(inches) < MIN_MOVE_IN:
        return

    direction = 1 if inches > 0 else -1
    target_deg = (abs(inches) / WHEEL_CIRCUMFERENCE_IN) * 360
    odom_sensor.reset_position()

    while abs(odom_sensor.position(DEGREES)) < target_deg:
        ramp_to_speed(direction * DRIVE_SPEED,
                      direction * DRIVE_SPEED)
        wait(20, MSEC)

    left_drive.stop(COAST)
    right_drive.stop(COAST)

# --- Smooth inertial turn ---
def turn_smooth(angle):
    if abs(angle) < MIN_TURN_DEG:
        return

    inertial_sensor.reset_heading()
    direction = 1 if angle > 0 else -1

    while abs(inertial_sensor.rotation()) < abs(angle):
        ramp_to_speed(direction * TURN_SPEED,
                     -direction * TURN_SPEED)
        wait(20, MSEC)

    left_drive.stop(COAST)
    right_drive.stop(COAST)

# --- Run auton command list ---
def run_smooth_path(commands):
    for cmd in commands:
        if cmd[0] == "drive":
            drive_smooth(cmd[1])
        elif cmd[0] == "turn":
            turn_smooth(cmd[1])
        wait(40, MSEC)

def autonomous():
    global current_left, current_right
    current_left = 0
    current_right = 0
    # Paste your jerry.io path below (list of tuples: (x, y))

    path = [
 # 1. Initial long forward path (from -39 to -118)
    ("drive", 30),
       

    # 2. 90° right turn
    ("turn", -90),


] 

    path_2 = [
    # 3. Short forward push
    ("drive", 1),

]

    path_3 = [

    # 4. Reverse back to original line


    # Optional stop
    ("intake_stop"),
]

    run_path(path)
    piston2.set(True)
    wait(2, SECONDS)
    run_path(path_2)
    intake.spin(REVERSE, 100, PERCENT)

    drivetrain.drive(FORWARD, 65, PERCENT)
    wait(1.5, SECONDS)
    drivetrain.stop()
    drivetrain.drive(REVERSE, 25, PERCENT)
    wait(2, SECONDS)
    piston4.set(True)
    intake.spin(REVERSE, 100, PERCENT)
    


    
    test_path = [
    (-49.084, -14.683),
    (-49.27, -46.674),
    (-50.866, -46.75),
    (-62.457, -46.658),
    (-29.045, -47.448),
    (-41.011, -48.096),
    (-43.01, -48.048),
    (-43.953, -46.906),
    (-43.86, -44.908),
    (-43.768, -42.91),
    (-43.676, -40.912),
    (-43.586, -38.914),
    (-43.495, -36.916),
    (-43.406, -34.918),
    (-43.318, -32.92),
    (-43.23, -30.922),
    (-43.143, -28.924),
    (-43.057, -26.926),
    (-42.972, -24.928),
    (-42.888, -22.929),
    (-42.805, -20.931),
    (-42.723, -18.933),
    (-42.641, -16.935),
    (-42.561, -14.936),
    (-42.482, -12.938),
    (-42.404, -10.939),
    (-42.327, -8.941),
    (-42.251, -6.942),
    (-42.177, -4.944),
    (-42.103, -2.945),
    (-42.032, -0.946),
    (-41.961, 1.053),
    (-41.893, 3.051),
    (-41.825, 5.05),
    (-41.76, 7.049),
    (-41.697, 9.048),
    (-41.635, 11.047),
    (-41.577, 13.046),
    (-41.519, 15.046),
    (-41.466, 17.045),
    (-41.414, 19.044),
    (-41.366, 21.044),
    (-41.322, 23.043),
    (-41.28, 25.043),
    (-41.245, 27.042),
    (-41.214, 29.042),
    (-41.187, 31.042),
    (-41.171, 33.042),
    (-41.162, 35.042),
    (-41.164, 37.042),
    (-41.179, 39.042),
    (-41.215, 41.041),
    (-41.282, 43.04),
    (-41.404, 45.036),
    (-41.712, 47.007),
    (-43.542, 47.271),
    (-45.542, 47.219),
    (-47.541, 47.167),
    (-49.54, 47.115),
    (-51.54, 47.063),
    (-53.539, 47.01),
    (-55.538, 46.958),
    (-57.538, 46.906),
    (-59.537, 46.854),
    (-58.742, 47.198),
    (-56.751, 47.379),
    (-54.753, 47.478),
    (-52.754, 47.538),
    (-50.755, 47.574),
    (-48.755, 47.592),
    (-46.755, 47.597),
    (-44.755, 47.591),
    (-42.755, 47.576),
    (-40.755, 47.552),
    (-38.755, 47.52),
    (-36.756, 47.48),
    (-34.756, 47.43),
    (-32.757, 47.369),
    (-30.759, 47.287),
    (-30.716, 47.131),
    (-32.715, 47.125),
    (-34.715, 47.107),
    (-36.714, 47.034),
    (-38.689, 46.753),
    (-38.871, 44.897),
    (-38.92, 42.897),
    (-38.993, 40.899),
    (-39.068, 38.9),
    (-39.134, 36.901),
    (-39.014, 35.048),
    (-37.014, 35.006),
    (-35.014, 34.994),
    (-33.014, 34.989),
    (-31.014, 34.988),
    (-29.014, 34.989),
    (-27.014, 34.993),
    (-25.014, 34.997),
    (-23.014, 35.003),
    (-21.015, 35.009),
    (-19.015, 35.016),
    (-17.015, 35.024),
    (-15.015, 35.032),
    (-13.015, 35.041),
    (-11.015, 35.05),
    (-9.015, 35.059),
    (-7.015, 35.068),
    (-5.015, 35.078),
    (-3.015, 35.088),
    (-1.015, 35.098),
    (0.985, 35.108),
    (2.985, 35.118),
    (4.985, 35.128),
    (6.985, 35.138),
    (8.985, 35.148),
    (10.985, 35.159),
    (12.985, 35.169),
    (14.985, 35.179),
    (16.985, 35.189),
    (18.985, 35.198),
    (20.985, 35.208),
    (22.985, 35.217),
    (24.985, 35.226),
    (26.985, 35.235),
    (28.985, 35.243),
    (30.985, 35.25),
    (32.985, 35.257),
    (34.985, 35.263),
    (36.985, 35.268),
    (38.985, 35.27),
    (40.985, 35.269),
    (42.985, 35.261),
    (44.843, 35.428),
    (45.024, 37.418),
    (45.099, 39.417),
    (45.145, 41.416),
    (45.174, 43.416),
    (45.191, 45.416),
    (45.199, 47.416),
    (47.021, 47.728),
    (49.021, 47.75),
    (51.021, 47.742),
    (53.021, 47.715),
    (55.02, 47.672),
    (57.019, 47.612),
    (58.449, 47.501),
    (56.449, 47.462),
    (54.449, 47.444),
    (52.449, 47.433),
    (50.449, 47.424),
    (48.449, 47.417),
    (46.449, 47.41),
    (44.449, 47.403),
    (42.449, 47.397),
    (40.449, 47.389),
    (38.449, 47.38),
    (36.449, 47.369),
    (34.449, 47.354),
    (32.45, 47.332),
    (30.45, 47.287),
    (28.465, 47.188),
    (29.587, 47.087),
    (31.587, 47.069),
    (33.587, 47.063),
    (35.587, 47.065),
    (37.587, 47.075),
    (39.586, 47.095),
    (41.586, 47.13),
    (42.991, 46.766),
    (42.961, 44.766),
    (42.93, 42.767),
    (42.899, 40.767),
    (42.868, 38.767),
    (42.838, 36.767),
    (42.807, 34.768),
    (42.776, 32.768),
    (42.746, 30.768),
    (42.715, 28.768),
    (42.684, 26.769),
    (42.654, 24.769),
    (42.623, 22.769),
    (42.592, 20.769),
    (42.561, 18.77),
    (42.531, 16.77),
    (42.5, 14.77),
    (42.469, 12.77),
    (42.439, 10.77),
    (42.408, 8.771),
    (42.377, 6.771),
    (42.347, 4.771),
    (42.316, 2.771),
    (42.285, 0.772),
    (42.254, -1.228),
    (42.224, -3.228),
    (42.193, -5.228),
    (42.162, -7.227),
    (42.132, -9.227),
    (42.101, -11.227),
    (42.07, -13.227),
    (42.04, -15.226),
    (42.009, -17.226),
    (41.978, -19.226),
    (41.947, -21.226),
    (41.917, -23.226),
    (41.886, -25.225),
    (41.855, -27.225),
    (41.825, -29.225),
    (41.794, -31.225),
    (41.763, -33.224),
    (41.733, -35.224),
    (41.702, -37.224),
    (41.671, -39.224),
    (41.641, -41.223),
    (41.61, -43.223),
    (41.579, -45.223),
    (41.548, -47.223),
    (43.354, -47.312),
    (45.352, -47.21),
    (47.35, -47.118),
    (49.348, -47.041),
    (51.347, -46.98),
    (53.347, -46.945),
    (55.347, -46.948),
    (57.345, -47.028),
    (58.402, -47.435),
    (56.405, -47.323),
    (54.41, -47.193),
    (52.413, -47.074),
    (50.416, -46.969),
    (48.418, -46.881),
    (46.419, -46.811),
    (44.42, -46.759),
    (42.42, -46.726),
    (40.42, -46.713),
    (38.42, -46.72),
    (36.42, -46.749),
    (34.421, -46.8),
    (32.422, -46.874),
    (30.425, -46.969),
    (28.428, -47.088),
    (27.04, -47.344),
    (29.038, -47.416),
    (31.038, -47.434),
    (33.038, -47.432),
    (35.038, -47.418),
    (37.038, -47.392),
    (39.038, -47.35),
    (41.036, -47.273),
    (41.006, -45.865),
    (40.989, -43.866),
    (41.089, -41.868),
    (41.221, -39.873),
    (41.342, -37.876),
    (41.345, -35.878),
    (39.759, -35.34),
    (37.759, -35.335),
    (35.759, -35.33),
    (33.759, -35.325),
    (31.759, -35.32),
    (29.759, -35.315),
    (27.759, -35.31),
    (25.759, -35.305),
    (23.759, -35.3),
    (21.759, -35.296),
    (19.759, -35.291),
    (17.759, -35.286),
    (15.759, -35.281),
    (13.759, -35.276),
    (11.759, -35.271),
    (9.759, -35.266),
    (7.759, -35.261),
    (5.759, -35.256),
    (3.759, -35.251),
    (1.759, -35.247),
    (-0.241, -35.242),
    (-2.241, -35.237),
    (-4.241, -35.232),
    (-6.241, -35.227),
    (-8.241, -35.222),
    (-10.241, -35.217),
    (-12.241, -35.212),
    (-14.241, -35.207),
    (-16.241, -35.202),
    (-18.241, -35.198),
    (-20.241, -35.193),
    (-22.241, -35.188),
    (-24.241, -35.183),
    (-26.241, -35.178),
    (-28.241, -35.173),
    (-30.241, -35.168),
    (-32.241, -35.163),
    (-34.241, -35.158),
    (-36.241, -35.153),
    (-38.241, -35.148),
    (-40.241, -35.144),
    (-42.241, -35.139),
    (-44.241, -35.134),
    (-46.241, -35.129),
    (-48.241, -35.124),
    (-50.241, -35.119),
    (-52.24, -35.114),
    (-54.24, -35.109),
    (-56.24, -35.104),
    (-58.24, -35.099),
    (-60.24, -35.095),
    (-62.24, -35.09),
    (-63.105, -33.918),
    (-63.087, -31.918),
    (-63.055, -29.918),
    (-63.017, -27.918),
    (-62.974, -25.919),
    (-62.929, -23.919),
    (-62.883, -21.92),
    (-62.835, -19.92),
    (-62.786, -17.921),
    (-62.736, -15.922),
    (-62.685, -13.922),
    (-62.634, -11.923),
    (-62.582, -9.924),
    (-62.531, -7.924),
    (-62.479, -5.925),
    (-62.426, -3.926),
    (-62.375, -1.926),
    (-62.323, 0.073),
    (-62.293, 1.237),
    ]



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