#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct 20 15:14:57 2023

@author: marco
"""


#!/usr/bin/env python

import numpy as np
import trimesh


scene = trimesh.Scene()

# plane
plane = trimesh.creation.box(extents=[1, 1, 0.01])
plane.visual.face_colors = [0.5, 0.5, 0.5, 0.5]
scene.add_geometry(plane)
scene.add_geometry(trimesh.creation.axis())

# object-1 (box)
box = trimesh.creation.box(extents=[0.3, 0.3, 0.3])
box.visual.face_colors = [0, 1., 0, 0.5]
axis = trimesh.creation.axis(origin_color=[1., 1, 1])
translation = [-0.2, 0, 0.15 + 0.01]  # box offset + plane offset
box.apply_translation(translation)
axis.apply_translation(translation)
rotation = trimesh.transformations.rotation_matrix(
    np.deg2rad(30), [0, 0, 1], point=box.centroid
)
box.apply_transform(rotation)
axis.apply_transform(rotation)
scene.add_geometry(box)
scene.add_geometry(axis)

# object-2 (cylinder)
cylinder = trimesh.creation.cylinder(radius=0.1, height=0.3)
cylinder.visual.face_colors = [0, 0, 1., 0.5]
axis = trimesh.creation.axis(origin_color=[1., 0, 0])
translation = [0.1, -0.2, 0.15 + 0.01]  # cylinder offset + plane offset
cylinder.apply_translation(translation)
axis.apply_translation(translation)
scene.add_geometry(cylinder)
scene.add_geometry(axis)

scene.show()