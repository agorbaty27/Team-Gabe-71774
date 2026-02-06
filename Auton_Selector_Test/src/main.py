from vex import *
from left_auton import left_auton
from right_auton import *
from skills_auton import *
from no_auton import no_auton
from driver_controls import driving, intaking, piston
from auton_selector import autons, selected

inertial_sensor = Inertial(Ports.PORT15)
inertial_sensor.calibrate()
brain.screen.print("Calibrating Inertial...")
while inertial_sensor.is_calibrating():
    wait(100, MSEC)
FIELD_ZERO = inertial_sensor.heading(DEGREES)

brain.screen.clear_screen()
brain.screen.print("Inertial Ready")

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