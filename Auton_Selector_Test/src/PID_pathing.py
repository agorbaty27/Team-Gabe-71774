from main import *
from vex import *
from config import *

def drive_pid(target_inches, kP=0.05, kI=0.0, kD=0.025):
    corrected_inches = target_inches * ODOMETRY_CORRECTION
    target_degrees = (corrected_inches / WHEEL_CIRCUMFERENCE_IN) * 360
    
    # Convert the exit threshold from inches to degrees
    EXIT_THRESHOLD_INCHES = 0.1
    error_threshold_degrees = (EXIT_THRESHOLD_INCHES / WHEEL_CIRCUMFERENCE_IN) * 360
    odom_sensor.reset_position()
    
    error = 0
    prev_error = 0
    integral = 0
    derivative = 0
    
    # 🔹 Acceleration control
    current_power = 0
    MAX_ACCEL = 1   # percent per 10ms loop (tune this)

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

        # Cap target power
        if target_power > 75: target_power = 75
        if target_power < -75: target_power = -75

        if abs(target_power) < 2.5: target_power = 2.5 if target_power > 0 else -2.5

        # 🔹 Slew rate limiting (smooth acceleration)
        if target_power > current_power + MAX_ACCEL:
            current_power += MAX_ACCEL
        elif target_power < current_power - MAX_ACCEL:
            current_power -= MAX_ACCEL
        else:
            current_power = target_power

        left_drive.spin(FORWARD, current_power, PERCENT)
        right_drive.spin(FORWARD, current_power, PERCENT)

        # Exit if the robot is within the target threshold
        if abs(error) < error_threshold_degrees:
            break
        wait(10, MSEC)
        
    left_drive.stop(BRAKE)
    right_drive.stop(BRAKE)
    wait(10, MSEC)

def turn_pid(target_angle, kP=0.4, kI=0.0, kD=0.01):
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

        if abs(error) < 1.0:
            at_target_time += 10
        else:
            at_target_time = 0

        if at_target_time > 120:
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