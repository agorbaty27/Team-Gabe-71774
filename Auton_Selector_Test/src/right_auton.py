from PID_pathing import *
from auton_actions import * 
from PID_pathing import run_path
from auton_actions import matchload_score  

path_right = [
    ("drive", 33),
    ("turn", 90),
        ]

path_2_right = [
    ("drive", 10),
    ("turn", 180),
    ("drive",-11.75),
    ("turn", 90),
    ("drive", -29),
    ]

def right_auton():
    run_path(path_right)
    matchload_score()
    run_path(path_2_right)
    piston3.set(True) 