from vex import *
from left_auton import left_auton
from right_auton import *
from skills_auton import *
from no_auton import no_auton
from driver_controls import driving, intaking, piston
from auton_selector import autons, selected, draw_screen

inertial_sensor = Inertial(Ports.PORT15)
inertial_sensor.calibrate()
brain.screen.print("Calibrating Inertial...")
while inertial_sensor.is_calibrating():
    wait(100, MSEC)
FIELD_ZERO = inertial_sensor.heading(DEGREES)

brain.screen.clear_screen()
brain.screen.print("Inertial Ready")

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

wait(20, MSEC)


def autonomous():
    autons[selected][1]()



def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")

    while True:
        driving()
        intaking()
        piston()
       
        wait(10, MSEC)

comp = Competition(user_control, autonomous)

brain.screen.clear_screen()