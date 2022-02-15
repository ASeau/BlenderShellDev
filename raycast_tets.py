import sys
import os
import pygad
import bpy
import math
import cv2 as cv
import numpy as np
import timeit
import glob
import time
start_time = time.time()
from mathutils import Vector
from mathutils.bvhtree import BVHTree

# open the file in the write mode
save_path = "D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/"
os.makedirs(save_path, exist_ok=True)
file_name = os.path.join(save_path, "grids1.npy")
grids = np.load(file_name)
print(grids)

a = bpy.data.objects['Camera']
#b = bpy.data.objects['target']
#obstacle = bpy.data.objects['Obj']

# Get the dependency graph
depsgraph = bpy.context.evaluated_depsgraph_get()
scene = bpy.context.scene

# Make a BVH tree from the obstacle
sel_objs = []
for obj in scene.objects:
    if obj.type == 'MESH':
        sel_objs.append(obj)

bvhtree_list = []
in_shadow = []
#bvhtree_list.append(bvhtree)
for obj in sel_objs:
    bvhtree = BVHTree.FromObject(obj, depsgraph, epsilon=0.0001)
    bvhtree_list.append(bvhtree)
    # Inverted world matrix: to have objects coordinates in obstacle world
    obj_world_inv = obj.matrix_world.inverted()
    # Origin of the raycast
    origin = obj_world_inv @ a.location

    # Direction of the raycast
    for co in grids:
        print("calculating for new_co=",co)
        direction = obj_world_inv @ (Vector(co) - a.location)
        # Raycast result is a tuple of:
        # - location of the hit
        # - its normal
        # - its face index
        # - the distance from the origin
        distance = 100
        result = bvhtree.ray_cast(origin, direction, distance)
        print("co_result=",result)
        if result[0] != None :
            in_shadow.append(co)

for co in in_shadow:
    #bpy.ops.mesh.primitive_cube_add(size=1, location= Vector(co))
    bpy.ops.mesh.primitive_cube_add(size = 1, location= (co[0],co[1],co[2]))
    #bpy.ops.mesh.primitive_cube_add(size = 0.1, location= obj_world @ result[0])
    bpy.context.object.display_type = 'BOUNDS'
    print(Vector(co) ,"Hit")


