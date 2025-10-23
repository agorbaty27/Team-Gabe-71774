# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       ari                                                          #
# 	Created:      10/22/2025, 11:33:54 PM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

brain = Brain()
controller = Controller(PRIMARY)

# Left side motors (ports 1, 2, 3)
left_motor_1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
left_motor_2 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
left_motor_3 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)

# Right side motors (ports 8, 9, 10)
right_motor_1 = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)
right_motor_2 = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)
right_motor_3 = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)

left_drive = MotorGroup(left_motor_1, left_motor_2, left_motor_3)
right_drive = MotorGroup(right_motor_1, right_motor_2, right_motor_3)

def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # place automonous code here

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    # place driver control in this while loop
    while True:
        # Get joystick positions
        forward = controller.axis3.position()  # Forward/back
        turn = controller.axis1.position()     # Turning

        # Calculate motor speeds
        left_speed = forward + turn
        right_speed = forward - turn

        # Limit speeds to ±100%
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))

        # Spin motors
        left_drive.spin(FORWARD, left_speed, PERCENT)
        right_drive.spin(FORWARD, right_speed, PERCENT)

        # Short delay for smooth control
        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()
brain.screen.draw_circle