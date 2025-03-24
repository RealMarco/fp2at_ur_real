#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 28 20:44:01 2024
@author: marco

Read and write low_dim_obs.pkl file,
an example for post-processing gripper_open according to gripper_joint_positions (finger_positions), (forgot to change the value gripper_open when collecting demos)
"""
import pickle
import pprint  # Pretty printing for better readability
import os  # To work with file paths

# Base directories
base_paths = [
#    "/media/marco/ubuntu_data/RVT3_real_data/insert_round_hole/all_variations/episodes",
    "/media/marco/ubuntu_data/RVT3_real_data2/put_in_drawer/all_variations/episodes"
]

# Loop over both directories and episodes
for base_path in base_paths:
    for K in range(10):  # K ranges from 0 to 9
        # Construct the file path
        pkl_file_path = os.path.join(base_path, f"episode{K}", "low_dim_obs.pkl")

        try:
            # Load the data from the .pkl file
            with open(pkl_file_path, 'rb') as file:
                data = pickle.load(file)

            # Process the data
            print(f"Processing file: {pkl_file_path}")
#            print("Data Type:", type(data)) # <class 'rlbench.demo.Demo'>

            # Access attributes (assuming data has '_observations' attribute and related structure)
            if hasattr(data, '_observations'):
#                print("Number of Observations:", len(data._observations))
                for i in range(len(data._observations)-1):
                    # Read pkl files from folders
                    print("finger distance:", data._observations[i].gripper_joint_positions)
                    print("Gripper Open Status:", data._observations[i].gripper_open)
                    print("Gripper Pose:", data._observations[i].gripper_pose)

                    # ----------------------
                    '''
                    # change the value of gripper_open according to gripper_joint_positions (finger_positions)
                    if data._observations[i].gripper_joint_positions[0]>(0.03-0.0001):
                        data._observations[i].gripper_open=1.0 # open
                    else:
                        data._observations[i].gripper_open=0.0 # close
                    '''
                print("finger distance:", data._observations[-1].gripper_joint_positions)
                print("Gripper Open Status:", data._observations[-1].gripper_open)
                print("Gripper Pose:", data._observations[-1].gripper_pose)
                '''
                # change the last gripper_open to 1 -------------
                data._observations[-1].gripper_open =1.0 # the last action is open
            # Save the modified data back to the same file -----------
            with open(pkl_file_path, 'wb') as file:
                pickle.dump(data, file)
            print(f"Updated and saved file: {pkl_file_path}")
            '''

        except FileNotFoundError:
            print(f"File not found: {pkl_file_path}")
        except Exception as e:
            print(f"Error reading {pkl_file_path}: {e}")





