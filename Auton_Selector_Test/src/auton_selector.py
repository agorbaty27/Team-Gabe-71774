from vex import *
from config import *
from right_auton import right_auton
from left_auton import left_auton
from skills_auton import skills_auton
from no_auton import no_auton


autons = [
    ("LEFT AUTON", left_auton),
    ("RIGHT AUTON", right_auton),
    ("NO AUTON", no_auton),
    ("SKILLS AUTON", skills_auton)
]

selected = 0
confirmed = False

def draw_screen():
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

    wait(20, MSEC)

