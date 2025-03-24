#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 14 17:57:40 2023

@author: marco
"""
import cv2
from rlbench.backend import utils 
import numpy as np
import os

'''
# mech-eye depth
img_mech_depth=cv2.imread('/media/marco/ubuntu_data/peract/data/real_data/place_infusion_bottles/all_variations/episodes/episode10/front_depth/0.tiff',cv2.IMREAD_UNCHANGED)
print('img_mech_depth', np.min(img_mech_depth[img_mech_depth>0]), np.max(img_mech_depth[img_mech_depth<4000]))
img_mech_depth[img_mech_depth==0] = img_mech_depth.max()  # assign the missing pixel with farest depth

#mask = np.where(img_mech_depth == 0, 1, 0).astype(np.uint8)
#img_mech_depth = cv2.inpaint(img_mech_depth, mask, 1, cv2.INPAINT_NS) #  fill in pixel value==0, missing value


img_mech_depth = np.clip(img_mech_depth, 500, 2200)
img_mech_depth_norm =  (img_mech_depth- 200)/(img_mech_depth.max() -200)

mech_depth3 = utils.float_array_to_rgb_image(img_mech_depth_norm, 2**24-1)
mech_depth3.save('/media/marco/ubuntu_data/peract/data/real_data/mech_depth3_v2.png')
mech_depth3_np =np.array(mech_depth3)
print('mech_depth3_np', np.min(mech_depth3_np[mech_depth3_np>0]), np.max(mech_depth3_np[mech_depth3_np<255]))
'''


# realsense depth
input_folder= '/media/marco/ubuntu_data/peract/data/real_data/place_infusion_bottles/all_variations/episodes/episode9/front_depth/'
output_folder= '/media/marco/ubuntu_data/peract/data/real_data/'
image_files = os.listdir(input_folder)
for image_file in image_files:
	input_img_path =  os.path.join(input_folder, image_file)
	img_real_depth=cv2.imread(input_img_path,cv2.IMREAD_UNCHANGED)
	
	
	
	mask = np.where(img_real_depth == 0, 1, 0).astype(np.uint8)
	img_real_depth = cv2.inpaint(img_real_depth, mask, 1, cv2.INPAINT_NS) #  fill in pixel value==0, missing value
	
	img_real_depth = np.clip(img_real_depth, 300, 17500)  # for realsense with messy background
	
	print('img_real_depth', np.min(img_real_depth[img_real_depth>0]), np.max(img_real_depth[img_real_depth<2**16-1]))
#	img_real_depth[img_real_depth==0] = img_real_depth.max()# assign the missing pixel with farest depth
	img_real_depth_norm =  (img_real_depth- 0)/(img_real_depth.max())
#	img_real_depth_norm[img_real_depth_norm==0] =1
	
	real_depth3 = utils.float_array_to_rgb_image(img_real_depth_norm, 2**24-1)
	output_path = os.path.join(output_folder, image_file)
#	real_depth3.save(output_path)
	real_depth3_np =np.array(real_depth3)
	real_depth3_cv = cv2.cvtColor(real_depth3_np, cv2.COLOR_RGB2BGR)
	cv2.imwrite(output_path, real_depth3_cv)
	print('real_depth3_np', np.min(real_depth3_np[real_depth3_np>0]), np.max(real_depth3_np[real_depth3_np<255]))

