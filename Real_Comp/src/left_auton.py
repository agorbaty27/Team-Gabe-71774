from main import *
from PID_pathing import *
from auton_actions import * 
from PID_pathing import run_path
from auton_actions import matchload_score   

path_left = [
    ("drive", 31.5),
    ("turn", -90),
    ] 

path_2_left = [
    ("drive", 10),
    ("turn", 0),
    ("drive", 11.75),
    ("turn", 90),
    ("drive", 29),
    ]

def left_auton(): 
    run_path(path_left)
    matchload_score()
    run_path(path_2_left)
    piston3.set(True) 