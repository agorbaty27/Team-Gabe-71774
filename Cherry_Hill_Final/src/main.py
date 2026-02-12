from vex import *

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

intake_motor_1 = Motor(Ports.PORT20, GearSetting.RATIO_6_1, True)
intake_motor_3 = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)


intake = MotorGroup(intake_motor_1, intake_motor_3)     

distance = Distance(Ports.PORT17)

odom_sensor = Rotation(Ports.PORT14)
odom_sensor.reset_position()

WHEEL_DIAMETER_IN = 2.0
WHEEL_CIRCUMFERENCE_IN = math.pi * WHEEL_DIAMETER_IN
TICKS_PER_REV = 360.0  

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


def drive_pid(target_inches, kP=0.05, kI=0.00, kD=0.01):
    corrected_inches = target_inches * ODOMETRY_CORRECTION
    target_degrees = (corrected_inches / WHEEL_CIRCUMFERENCE_IN) * 360

    EXIT_THRESHOLD_INCHES = 0.1
    error_threshold_degrees = (EXIT_THRESHOLD_INCHES / WHEEL_CIRCUMFERENCE_IN) * 360
    odom_sensor.reset_position()
    
    error = 0
    prev_error = 0
    integral = 0
    derivative = 0

    current_power = 0
    MAX_ACCEL = 0.7   

    while True:
        current_pos = odom_sensor.position(DEGREES)
        error = target_degrees - current_pos

        if abs(error) < 50:
            integral += error
        else:
            integral = 0
            
        derivative = error - prev_error
        prev_error = error
        
        target_power = (error * kP) + (integral * kI) + (derivative * kD)

      
        if target_power > 60: target_power = 60
        if target_power < -60: target_power = -60

        if abs(target_power) < 2.5: target_power = 2.5 if target_power > 0 else -2.5

     
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
        wait(10, MSEC)
        
    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)
    wait(10, MSEC)

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

        if abs(error) < 0.75:
            at_target_time += 10
        else:
            at_target_time = 0

        if at_target_time > 40:
            break

        wait(10, MSEC)

    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)
    wait(10, MSEC)

def drive_pid_distance(target_distance_in,
                       direction=1,
                       kP=1.5, kI=0.00, kD=0.2):

    # direction = 1  -> drive forward toward object
    # direction = -1 -> drive backward away from object

    error = 0
    prev_error = 0
    integral = 0

    current_power = 0
    MAX_ACCEL = 2.0

    EXIT_THRESHOLD_IN = 0.5   # how close is "good enough"

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
        target_power = max(-60, min(60, target_power))

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

        wait(10, MSEC)

    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)
    wait(10, MSEC)


def run_path(steps):

    for step in steps:
        action = step[0]

        if action == "drive":
            drive_pid(step[1])

        elif action == "turn":
            turn_pid(step[1])

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

def oscillate_drive(power=30, time_ms=120, cycles=3):
    for _ in range(cycles):
        left_drive.spin(FORWARD, power, PERCENT)
        right_drive.spin(FORWARD, power, PERCENT)
        wait(time_ms, MSEC)

        left_drive.spin(REVERSE, power*0.75, PERCENT)
        right_drive.spin(REVERSE, power*0.75, PERCENT)
        wait(time_ms, MSEC)

    left_drive.stop(COAST)
    right_drive.stop(COAST)

path_d = [
    ("turn", -90)
]

path_e = [
    ("drive", -10)
]

path_f = [
    ("turn", 90)
]


def matchload_score():
    piston2.set(True)
    wait(0.25, SECONDS)
    intake.spin(REVERSE, 100, PERCENT)
    left_drive.spin(FORWARD, 15, PERCENT)
    right_drive.spin(FORWARD , 15, PERCENT)
    wait(1.0, SECONDS)
    oscillate_drive(power = 30, time_ms=200, cycles=3)
    left_drive.spin(REVERSE, 25, PERCENT)
    right_drive.spin(REVERSE , 25, PERCENT)
    wait(0.4, SECONDS)
    run_path(path_d)
    left_drive.spin(REVERSE, 60, PERCENT)
    right_drive.spin(REVERSE , 60, PERCENT)
    wait(0.7, SECONDS)
    piston4.set(True)
    intake.spin(REVERSE, 100, PERCENT)
    wait(1, SECONDS)
    intake.stop(COAST)
    piston4.set(False)
    piston2.set(False)
    left_drive.spin(FORWARD, 15, PERCENT)
    right_drive.spin(FORWARD , 15, PERCENT)
    wait(0.5, SECONDS)  
    left_drive.spin(REVERSE, 100, PERCENT)
    right_drive.spin(REVERSE , 100, PERCENT)
    wait(0.5, SECONDS)
    left_drive.stop(COAST)
    right_drive.stop(COAST) 

def matchload_score_right():
    piston2.set(True)
    wait(0.25, SECONDS)
    intake.spin(REVERSE, 100, PERCENT)
    left_drive.spin(FORWARD, 15, PERCENT)
    right_drive.spin(FORWARD , 15, PERCENT)
    wait(0.75, SECONDS)
    oscillate_drive(power = 30, time_ms=200, cycles=4)
    intake.stop(COAST)
    run_path(path_f)
    intake.spin(REVERSE, 100, PERCENT)
    left_drive.spin(REVERSE, 60, PERCENT)
    right_drive.spin(REVERSE , 60, PERCENT)
    wait(0.7, SECONDS)
    piston4.set(True)
    intake.spin(REVERSE, 100, PERCENT)
    wait(1.1, SECONDS)
    intake.stop(COAST)
    piston4.set(False)
    piston2.set(False)
    left_drive.stop(COAST)
    right_drive.stop(COAST) 

path_left = [
    ("drive", 31.5),
    ("turn", -90),
    ] 

path_2_left = [
    ("drive", 10.5),
    ("turn", 0),
    ("drive", 10.5),
    ("turn", 90),
    ("drive", 29),
    ]

def left_auton(): 
    run_path(path_left)
    matchload_score()
    run_path(path_2_left)
    piston3.set(True) 

path_none = [
    ("drive", 2), ] 

def no_auton():
    run_path(path_none)

path_right = [
    ("drive", 31.5),
    ("turn", 90),
        ]

path_2_right = [
    ("drive", 10.5),
    ("turn", 180),
    ("drive", 10.5),
    ("turn", -90),
    ("drive", 29),
    ]

path_3_right = [
    ("drive", 20.5),
    ("turn", -135),
    ("drive", 45),
]

def right_auton():
    run_path(path_right)
    matchload_score_right()
    intake.spin(REVERSE, 100, PERCENT)
    run_path(path_3_right)
    intake.spin(FORWARD, 100, PERCENT)
    

path = [
    ("drive", 30.0),
    ("turn", -90),
    ] 

path_6 = [
    ("drive", -23),
    ] 

path_2 = [
    ("drive", 10),
    ("turn", 0),
    ("drive", 11.75),
    ("turn", 90),
    ("drive", 82),
    ("turn", 180),
    ("drive", 11.75),
    ("turn", 90),
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

path_a = [
    ("drive", -10),
    ("turn", 180),]

path_2a = [
    ("turn", -90),
    ("drive", -86),
    ("turn", 180),
]
path_3a = [
    ("turn", 90),
]   

path_1b = [
    ("drive", 10),
    ("turn", 180),
]

path_2b = [
    
    ("turn", 90),
    ("drive", -74),
    ("turn", -135), 
    ("drive", 43),
    ("turn", -180), 
]   

path_c = [
    ("turn", 90)
]

def matchload_only():
    piston2.set(True)
    wait(0.25, SECONDS)
    intake.spin(REVERSE, 100, PERCENT)
    left_drive.spin(FORWARD, 14, PERCENT)
    right_drive.spin(FORWARD , 14, PERCENT)
    wait(1, SECONDS)
    oscillate_drive(power = 30, time_ms=200, cycles=9)
    intake.stop(COAST)

def score_only():
    left_drive.spin(REVERSE, 50, PERCENT)
    right_drive.spin(REVERSE , 50, PERCENT)
    wait(0.9, SECONDS)
    left_drive.stop(COAST)
    right_drive.stop(COAST)
    piston4.set(True)
    intake.spin(FORWARD, 100, PERCENT)
    wait(0.2, SECONDS)
    intake.spin(REVERSE, 100, PERCENT)
    wait(2.5, SECONDS)
    intake.stop(COAST)
    piston4.set(False)

def matchload_score_skills():
    piston2.set(True)
    wait(0.25, SECONDS)
    intake.spin(REVERSE, 100, PERCENT)
    left_drive.spin(FORWARD, 15, PERCENT)
    right_drive.spin(FORWARD , 15, PERCENT)
    wait(1, SECONDS)
    left_drive.spin(FORWARD, 15, PERCENT)
    right_drive.spin(FORWARD , 15, PERCENT)
    run_path(path_c)
    left_drive.spin(FORWARD, 15, PERCENT)
    right_drive.spin(FORWARD , 15, PERCENT)
    wait(1.5, SECONDS)
    oscillate_drive(power = 30, time_ms=215, cycles=9)
    left_drive.stop(COAST)
    right_drive.stop(COAST)
    run_path(path_c)
    left_drive.spin(REVERSE, 40, PERCENT)
    right_drive.spin(REVERSE , 40, PERCENT)
    wait(2.5, SECONDS)
    piston4.set(True)
    intake.spin(FORWARD, 100, PERCENT)
    wait(0.2, SECONDS)
    intake.spin(REVERSE, 100, PERCENT)
    wait(3, SECONDS)
    intake.stop(COAST)
    piston4.set(False)
    piston2.set(False)
    

def skills_auton():
    piston3.set(True)  
    run_path(path)
    matchload_only()
    intake.spin(REVERSE, 100, PERCENT)
    piston2.set(False)
    run_path(path_a)
    intake.stop(COAST)
    drive_pid_distance(5.5,direction=-1,)
    run_path(path_2a)
    drive_pid_distance(17.5,direction= -1,)
    run_path(path_3a)
    score_only()
    matchload_score_skills()
    run_path(path_1b)
    drive_pid_distance(5,direction= -1,)
    run_path(path_2b)
    piston2.set(True)
    left_drive.spin(FORWARD, 80, PERCENT)
    right_drive.spin(FORWARD , 80, PERCENT)
    intake.spin(REVERSE, 100, PERCENT)
    wait(1, SECONDS)
    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)
    piston2.set(False)
    



autons = [
    ("LEFT AUTON", left_auton),
    ("RIGHT AUTON", right_auton),
    ("NO AUTON", no_auton),
    ("SKILLS AUTON", skills_auton)
]

selected = 0
confirmed = False

'''def draw_screen():
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)
    brain.screen.print("Select Autonomous")
    brain.screen.new_line()
    brain.screen.print("----------------")

    for i, (name, _) in enumerate(autons):
        brain.screen.new_line()
        if i == selected:
            brain.screen.print("> " + name)
        else:
            brain.screen.print("  " + name)

# ---- Selection Loop (run BEFORE competition starts) ----



draw_screen()

while not confirmed:
    if controller.buttonRight.pressing():
        selected = (selected + 1) % len(autons)
        draw_screen()
        wait(200, MSEC)

    if controller.buttonLeft.pressing():
        selected = (selected - 1) % len(autons)
        draw_screen()
        wait(200, MSEC)

    if controller.buttonA.pressing():
        confirmed = True
        brain.screen.clear_screen()
        brain.screen.print("Selected:")
        brain.screen.new_line()
        brain.screen.print(autons[selected][0])
        wait(500, MSEC)
'''
wait(20, MSEC)

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")

    while True:
        driving()
        intaking()
        piston()
        wait(5, MSEC)

turn_tuning = [
    ("turn", 40),
    ("turn", 0)
]

def turn_tuning_auton():
    run_path(turn_tuning)



comp = Competition(user_control, skills_auton)

brain.screen.clear_screen()