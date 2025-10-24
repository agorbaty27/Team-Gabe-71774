# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       ari                                                          #
# 	Created:      10/22/2025, 11:33:54 PM                                     #
# 	Description:  V5 Arcade Drive (One Joystick, Correct Direction)            #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

brain = Brain()
controller = Controller(PRIMARY)

# Left side motors (ports 11, 12, 13)
left_motor_1 = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
left_motor_2 = Motor(Ports.PORT12, GearSetting.RATIO_18_1, False)
left_motor_3 = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)

# Right side motors (ports 1, 2, 3)
right_motor_1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
right_motor_2 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)
right_motor_3 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)

# Group the motors for each side
left_drive = MotorGroup(left_motor_1, left_motor_2, left_motor_3)
right_drive = MotorGroup(right_motor_1, right_motor_2, right_motor_3)

intake_1 = Motor(Ports.PORT20, GearSetting.RATIO_18_1, False)
intake_2 = Motor(Ports.PORT6, GearSetting.RATIO_18_1, True)

intake = MotorGroup(intake_1, intake_2)


DEADBAND = 5

def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # place autonomous code here

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")

    while True:
        # Read joystick values
        forward = -controller.axis3.position()  # Inverted so pushing forward drives forward
        turn = controller.axis1.position()       # Left/Right

        # Apply deadband
        if abs(forward) < DEADBAND:
            forward = 0
        if abs(turn) < DEADBAND:
            turn = 0

        # Calculate motor speeds
        left_speed = forward - turn
        right_speed = forward + turn

        # Clamp values to -100% to 100%
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))

        # Spin the motors
        left_drive.spin(FORWARD, left_speed, PERCENT)
        right_drive.spin(FORWARD, right_speed, PERCENT)

        if controller.buttonR1.pressing():
            intake.spin(FORWARD, 100, PERCENT)
        elif controller.buttonR2.pressing():
            intake.spin(REVERSE, 100, PERCENT)
        else:
            intake.stop(COAST)
            
        # Prevent CPU overload
        wait(20, MSEC)

# Create competition instance
comp = Competition(user_control, autonomous)

# Actions to do when the program starts
brain.screen.clear_screen()
