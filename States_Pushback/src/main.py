from vex import *

auton_selection = 0

brain=Brain()
inertial_sensor = Inertial(Ports.PORT15)
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

intake_motor_1 = Motor(Ports.PORT20, GearSetting.RATIO_6_1, False)
intake_motor_3 = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)


intake = MotorGroup(intake_motor_1, intake_motor_3)     

distance = Distance(Ports.PORT17)

odom_sensor = Rotation(Ports.PORT14)
odom_sensor.reset_position()

WHEEL_DIAMETER_IN = 2.0
WHEEL_CIRCUMFERENCE_IN = math.pi * WHEEL_DIAMETER_IN
TICKS_PER_REV = 360.0  
BIG_WHEEL_CIRCUMFERENCE_IN = math.pi * 3.25 * 0.75

# Tuning multipliers - adjust these to calibrate odometry and turning
ODOMETRY_CORRECTION = 1  # Multiply drive distance by this value
TURNING_CORRECTION = 1   # Multiply turn angle by this value

FIELD_ZERO = inertial_sensor.heading(DEGREES)

SMOOTHING = 0.12

current_left_speed = 0
current_right_speed = 0


inertial_sensor = Inertial(Ports.PORT15)
inertial_sensor.calibrate()
brain.screen.print("Calibrating Inertial...")
while inertial_sensor.is_calibrating():
    wait(100, MSEC)
FIELD_ZERO = inertial_sensor.heading(DEGREES)

brain.screen.clear_screen()
brain.screen.print("Inertial Ready")

def turn_pid(target_angle, kP=0.4, kI=0.0001, kD=0.03):
    error = 0
    prev_error = 0
    integral = 0
    at_target_time = 0

    target_angle %= 360  # safety wrap

    while True:
        # Absolute field heading
        current = (inertial_sensor.heading(DEGREES) - FIELD_ZERO) % 360

        # Shortest-path error (-180 to 180)
        error = ((target_angle - current + 180) % 360) - 180

        if abs(error) < 15:
            integral += error
        else:
            integral = 0

        derivative = error - prev_error
        prev_error = error

        power = (error * kP) + (integral * kI) + (derivative * kD)
        power = max(-80, min(80, power))

        left_drive.spin(FORWARD, power, PERCENT)
        right_drive.spin(REVERSE, power, PERCENT)

        if abs(error) < 1:
            at_target_time += 10
        else:
            at_target_time = 0

        if at_target_time > 10:
            break

        wait(3, MSEC)

    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)
    wait(3, MSEC)

def drive_pid_distance(target_distance_in,
                       direction=-1,
                       kP=1.5, kI=0.00, kD=0.2):

    # direction = 1  -> drive forward toward object
    # direction = -1 -> drive backward away from object

    error = 0
    prev_error = 0
    integral = 0

    current_power = 0
    MAX_ACCEL = 2.0

    EXIT_THRESHOLD_IN = 0.5  

    while True:
        # Distance sensor returns MM
        current_distance = distance.object_distance(INCHES)

        error = (current_distance - target_distance_in) * direction

        if abs(error) < 6:
            integral += error
        else:
            integral = 0

        derivative = error - prev_error
        prev_error = error

        target_power = (error * kP) + (integral * kI) + (derivative * kD)

        # Clamp speed
        target_power = max(-80, min(80, target_power))

        # Minimum power so it doesn’t stall
        if abs(target_power) < 2.5:
            target_power = 2.5 if target_power > 0 else -2.5

        # Accel limiting (same as your drive PID)
        if target_power > current_power + MAX_ACCEL:
            current_power += MAX_ACCEL
        elif target_power < current_power - MAX_ACCEL:
            current_power -= MAX_ACCEL
        else:
            current_power = target_power

        left_drive.spin(FORWARD, current_power, PERCENT)
        right_drive.spin(FORWARD, current_power, PERCENT)

        # Exit when close enough
        if abs(current_distance - target_distance_in) < EXIT_THRESHOLD_IN:
            break

        wait(3, MSEC)

    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)
    wait(3, MSEC)

def drive_pid_avg(target_inches, kP=0.08, kI=0.00, kD=0.1):

    corrected_inches = target_inches * ODOMETRY_CORRECTION
    target_degrees = (corrected_inches / BIG_WHEEL_CIRCUMFERENCE_IN) * 360

    EXIT_THRESHOLD_INCHES = 0.5
    error_threshold_degrees = (EXIT_THRESHOLD_INCHES / BIG_WHEEL_CIRCUMFERENCE_IN) * 360

    # Reset all motor encoders
    left_motor_1.reset_position()
    left_motor_2.reset_position()
    left_motor_3.reset_position()
    right_motor_1.reset_position()
    right_motor_2.reset_position()
    right_motor_3.reset_position()

    error = 0
    prev_error = 0
    integral = 0

    current_power = 0
    MAX_ACCEL = 1

    while True:

        # 🔹 Average position of all 6 motors
        avg_position = (
            left_motor_1.position(DEGREES) +
            left_motor_2.position(DEGREES) +
            left_motor_3.position(DEGREES) +
            right_motor_1.position(DEGREES) +
            right_motor_2.position(DEGREES) +
            right_motor_3.position(DEGREES)
        ) / 6.0

        error = target_degrees - avg_position

        if abs(error) < 50:
            integral += error
        else:
            integral = 0

        derivative = error - prev_error
        prev_error = error

        target_power = (error * kP) + (integral * kI) + (derivative * kD)

        # Clamp
        target_power = max(-60, min(60, target_power))

        # Prevent stall
        if abs(target_power) < 2.5:
            target_power = 2.5 if target_power > 0 else -2.5

        # Acceleration limiting
        if target_power > current_power + MAX_ACCEL:
            current_power += MAX_ACCEL
        elif target_power < current_power - MAX_ACCEL:
            current_power -= MAX_ACCEL
        else:
            current_power = target_power

        left_drive.spin(FORWARD, current_power, PERCENT)
        right_drive.spin(FORWARD, current_power, PERCENT)

        if abs(error) < error_threshold_degrees:
            break

        wait(3, MSEC)

    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)
    wait(3, MSEC)



def run_path(steps):

    for step in steps:
        action = step[0]

        if action == "drive":
            drive_pid_avg(step[1])

        elif action == "turn":
            turn_pid(step[1])

        elif action == "distance":
            drive_pid_distance(step[1])

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

        wait(3, MSEC)

def driving():
    global current_left_speed, current_right_speed

    forward = controller.axis3.position()
    turn = controller.axis1.position() * 0.5

   


    target_left = forward + turn
    target_right = forward - turn

    target_left = max(-100, min(100, target_left))
    target_right = max(-100, min(100, target_right))


    current_left_speed += (target_left - current_left_speed) * SMOOTHING
    current_right_speed += (target_right - current_right_speed) * SMOOTHING


    left_drive.spin(FORWARD, (current_left_speed), PERCENT)
    right_drive.spin(FORWARD, (current_right_speed), PERCENT)

    if abs(forward) < 2 and abs(turn) < 2:
        current_left_speed = 0
        current_right_speed = 0
        left_drive.stop(COAST)
        right_drive.stop(COAST)
        return


    else:
        left_drive.spin(FORWARD, current_left_speed, PERCENT)
        right_drive.spin(FORWARD, current_right_speed, PERCENT)
 
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
        intake_motor_1.spin(REVERSE, 100, PERCENT)

    else:
        intake.stop(COAST)
        piston4.set(False)


    wait(5, MSEC)
    
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

def oscillate_drive(power=30, time_ms=120, cycles=3):
    for _ in range(cycles):
        left_drive.spin(FORWARD, power, PERCENT)
        right_drive.spin(FORWARD, power, PERCENT)
        wait(time_ms, MSEC)

        left_drive.spin(REVERSE, power, PERCENT)
        right_drive.spin(REVERSE, power, PERCENT)
        wait(time_ms, MSEC)

    left_drive.stop(COAST)
    right_drive.stop(COAST)

path_none = [
    ("drive", 2), ] 

def no_auton():
    run_path(path_none)

def matchload_only():
    piston2.set(True)
    wait(0.25, SECONDS)
    intake.spin(REVERSE, 100, PERCENT)
    left_drive.spin(FORWARD, 32, PERCENT)
    right_drive.spin(FORWARD , 32, PERCENT)
    wait(1.25, SECONDS)
    left_drive.stop(COAST)
    right_drive.stop(COAST)
    wait(3, SECONDS)
    intake.stop(COAST)

def score_only():
    left_drive.spin(REVERSE, 40, PERCENT)
    right_drive.spin(REVERSE , 40, PERCENT)
    wait(0.9, SECONDS)
    left_drive.stop(COAST)
    right_drive.stop(COAST)
    piston4.set(True)
    intake.spin(FORWARD, 100, PERCENT)
    wait(0.1, SECONDS)
    intake.spin(REVERSE, 100, PERCENT)
    wait(2.5, SECONDS)
    intake.stop(COAST)
    piston4.set(False)

def matchload_score_skills():
    piston2.set(True)
    wait(0.25, SECONDS)
    intake.spin(REVERSE, 100, PERCENT)
    turn_pid(-90)
    left_drive.spin(FORWARD, 32, PERCENT)
    right_drive.spin(FORWARD, 32, PERCENT)
    wait(1.5, SECONDS)
    left_drive.stop(COAST)
    right_drive.stop(COAST)
    wait(3, SECONDS)
    turn_pid(-90)
    left_drive.spin(REVERSE, 40, PERCENT)
    right_drive.spin(REVERSE , 40, PERCENT)
    wait(1.5, SECONDS)
    piston4.set(True)
    intake.spin(FORWARD, 100, PERCENT)
    wait(0.1, SECONDS)
    intake.spin(REVERSE, 100, PERCENT)
    wait(2.5, SECONDS)
    intake.stop(COAST)
    piston4.set(False)
    piston2.set(False)

left_testing = [
    ("distance", 17),
    ("turn", 90)
]

path_a_testing = [
    ("drive", -10),
    ("turn", 0),
    ("distance", 4.5),
    ("turn", 90),
    ("drive", -85),
    ("turn", 0),
    ("distance", 18),
    ("turn", -90),
]   

path_b_testing = [
    ("drive", 10),
    ("turn", 0),
    ("distance", 4.5),
    ("turn", -90),
    ("drive", -80),
    ("turn", 26), 

]   


def skills_auton_testing():
    piston3.set(True)  
    run_path(left_testing)
    matchload_only()
    intake.spin(REVERSE, 100, PERCENT)
    piston2.set(False)
    intake.stop()
    run_path(path_a_testing)
    intake.stop(COAST)
    score_only()
    matchload_score_skills()
    run_path(path_b_testing)
    piston2.set(True)

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")

    while True:
        driving()
        intaking()
        piston()
        wait(5, MSEC)

skills_2_path = [ 
    ("drive", -10),]

def skills_2():
    run_path(skills_2_path)
    piston2.set(True)
    left_drive.spin(FORWARD, 80, PERCENT)
    right_drive.spin(FORWARD , 80, PERCENT)
    wait(2.25, SECONDS)
    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)
    piston2.set(False)


sf_1 = [
    ("drive", 10),
    ("turn", 180),
    ("drive", -90),
    ("distance", 18),
    ("turn", -90),
]

sf_2 = [
    ("drive", -10),
    ("turn", 180),
    ("distance", 4),
    ("turn", -90),
    ("drive", -85),
    ("turn", 180),
    ("distance", 18),
    ("turn", 90)
]

sf_3 = [
    ("drive", 10),
    ("turn", 140)
]

def sf_skills():
    piston3.set(True)  
    run_path(left_testing)
    matchload_only()
    intake.spin(REVERSE, 100, PERCENT)
    piston2.set(False)
    intake.stop()
    run_path(path_a_testing)
    intake.stop(COAST)
    score_only()
    matchload_score_skills()
    run_path(sf_1)
    matchload_only()
    run_path(sf_2)
    matchload_score_skills()
    run_path(sf_3)
    left_drive.spin(FORWARD, 80, PERCENT)
    right_drive.spin(FORWARD , 80, PERCENT)
    wait(1.35, SECONDS)
    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)
    piston2.set(False)
    intake.stop(COAST)

def select_auton():
    global auton_selection
    while True:
        brain.screen.clear_screen()
        brain.screen.set_cursor(1, 1)
        
        if auton_selection == 0:
            brain.screen.print("SELECTED: NONE")
            brain.screen.set_fill_color(Color.RED)
        else:
            brain.screen.print("SELECTED: SF SKILLS")
            brain.screen.set_fill_color(Color.GREEN)
    
        brain.screen.draw_rectangle(10, 40, 200, 100)
        
        if brain.screen.pressing():
       
            auton_selection = 1 if auton_selection == 0 else 0
        
            while brain.screen.pressing():
                wait(10, MSEC)
        
        wait(100, MSEC)
    
def autonomous_task():
    if auton_selection == 1:
        brain.screen.clear_screen()
        brain.screen.print("Running SF Skills...")
        sf_skills() 
    else:
        brain.screen.clear_screen()
        brain.screen.print("Auton Disabled (None)")
        no_auton()


comp = Competition(user_control, autonomous_task)

brain.screen.clear_screen()