from vex import *
from config import *

def driving():
    global current_left_speed, current_right_speed

    forward = controller.axis3.position()
    turn = controller.axis1.position() * 0.5

    if forward == 0 and turn == 0:
        current_left_speed, current_right_speed = 0, 0
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

