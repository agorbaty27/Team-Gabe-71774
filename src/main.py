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
piston = DigitalOut(brain.three_wire_port.h)
piston_state = False   # Starts retracted
button_pressed = False

# Left side motors (ports 11, 12, 13)
left_motors = MotorGroup(Motor(Ports.PORT11, True), Motor(Ports.PORT12, True), Motor(Ports.PORT13, True))
right_motors = MotorGroup(Motor(Ports.PORT1, False), Motor(Ports.PORT2, False), Motor(Ports.PORT3, False))

intake_1 = Motor(Ports.PORT20, GearSetting.RATIO_18_1, False)
intake_2 = Motor(Ports.PORT6, GearSetting.RATIO_18_1, True)
intake_3 = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)

intake = MotorGroup(intake_1, intake_2, intake_3)

expo = 0.6  

# Exponential function
def expo_curve(value):
    return (value / 100) * abs(value / 100) ** expo * 100

DEADBAND = 5

def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # place autonomous code here

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")

    while True:
        # Controller inputs
        forward = controller.axis3.position()  # Left joystick vertical
        turn = controller.axis1.position()     # Right joystick horizontal

        # Apply exponential curve to turning
        turn = expo_curve(turn)

        # Calculate motor speeds for arcade drive
        left_speed = forward + turn
        right_speed = forward - turn

        # Clamp motor speeds to -100 to 100
        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)

        # Set motor velocities
        left_motors.spin(FORWARD, left_speed, PERCENT)
        right_motors.spin(FORWARD, right_speed, PERCENT)

        if controller.buttonR1.pressing():
            intake.spin(FORWARD, 100, PERCENT)
        elif controller.buttonR2.pressing():
            intake.spin(REVERSE, 100, PERCENT)
        else:
            intake.stop(COAST)

        if controller.buttonA.pressing():
            if not button_pressed:
                # Toggle piston state
                piston_state = not piston_state
                piston.set(piston_state)
                button_pressed = True
        else:
            button_pressed = False  # Reset when button released
                
            # Prevent CPU overload
        wait(20, MSEC)

# Create competition instance
comp = Competition(user_control, autonomous)

# Actions to do when the program starts
brain.screen.clear_screen()
