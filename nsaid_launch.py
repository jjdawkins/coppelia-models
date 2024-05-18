import subprocess
import os

#!/usr/bin/env python


def start_coppelia_sim():
    # Replace 'path_to_coppelia_sim' with the actual path to your CoppeliaSim executable
    coppelia_sim_path ='/home/ew-admin/CoppeliaSim_Edu_V4_6_0_rev18_Ubuntu22_04/coppeliaSim'
    
    # Replace 'path_to_scene_file' with the actual path to your CoppeliaSim scene file
    scene_file_path = './scenes/rover_estimatiion.ttt'
    
    # Start CoppeliaSim with the specified scene file
    subprocess.Popen([coppelia_sim_path, '-f', scene_file_path])

def start_nsaid_est():
    # Replace 'path_to_nsaid_est' with the actual path to nsaid_est.py
    nsaid_est_path = 'path_to_nsaid_est'
    
    # Start nsaid_est.py
    subprocess.Popen(['python', nsaid_est_path])

if __name__ == '__main__':
    start_coppelia_sim()