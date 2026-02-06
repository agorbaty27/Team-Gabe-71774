from PID_pathing import run_path
from auton_actions import *


path_5 = [
    ("drive", 20),
]

path = [
    ("drive", 31.5),
    ("turn", -90),
    ] 

path_5 = [
    ("drive", 9),
    ] 

path_6 = [
    ("drive", -23),
    ] 

path_2 = [
    ("drive", 10),
    ("turn", 0),
    ("drive", 11.75),
    ("turn", 90),
    ("drive", 82),
    ("turn", 180),
    ("drive", 11.75),
    ("turn", 90),
            ]
path_3 = [
    ("drive", -10),
    ("turn", -90),
    ("drive", 90), #important distance
    ("turn", -90),
]

path_4 = [
    ("drive", -10),
    ("turn", -110), #important turn
    ("drive", 50),  #important distance
]

def skills_auton():
    piston3.set(True)  
    run_path(path)
    matchload_score()
    run_path(path_2)
    matchload_score()
    run_path(path_3)
    matchload_score()
    run_path(path_2)
    matchload_score()
    run_path(path_4)

