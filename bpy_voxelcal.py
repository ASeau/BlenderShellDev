import bpy
import sys
import os
from math import *
from mathutils import Vector
from datetime import datetime, date, timedelta
import ifcopenshell
import ifcopenshell.util






'''
context = bpy.context
scene = context.scene

def CameraCone(cam, scn):

    matrix = cam.matrix_world
    matrix = matrix.inverted()
    sensor_width = cam.data.sensor_width
    lens = cam.data.lens
    resolution_x = scn.render.resolution_x
    resolution_y = scn.render.resolution_y

    w = 0.5* sensor_width / lens
    if resolution_x> resolution_y:
        x = w
        y = w*resolution_y/resolution_x
    else:
        x = w*resolution_x/resolution_y
        y = w

    lr = Vector([x,-y,-1])
    ur = Vector([x,y,-1])
    ll = Vector([-x,-y,-1])
    ul = Vector([-x,y,-1])
    half_plane_normals = [
        lr.cross(ll).normalized(),
        ll.cross(ul).normalized(),
        ul.cross(ur).normalized(),
        ur.cross(lr).normalized()
    ]
    return half_plane_normals

def isVisible(cam_mat, half_plane_normals ,loc, fudge=0):

    loc2 = cam_mat @ loc

    for norm in half_plane_normals:
        z2 = loc2.dot(norm)
        if z2 < -fudge:
            return False

    return True

cam =  bpy.data.objects["CamA"]
half_normals = CameraCone(cam,scene)

voxel_dict = []

for collection in bpy.data.collections:
    if collection.name == 'Voxel':
        for obj in collection.all_objects:
            voxel_dict.append(obj)

def in_cam(voxel_dict,cam,half):
    matrix = cam.matrix_world
    matrix = matrix.inverted()

    #far = cam.data.clip_end

    for obj in voxel_dict:
        bbox_local_center = sum((Vector(b) for b in obj.bound_box), Vector()) / 8
        center = obj.matrix_world @ bbox_local_center
        print(center,type(center))

        result = isVisible(cam_mat=matrix, half_plane_normals=half,loc=center,fudge=0)

        if result == True:
            obj.select_set(True)
            print(obj.name,result)

in_cam(voxel_dict,cam,half_normals)
print("done")
'''
'''
sel_objs = []
for obj in scene.objects:
    if obj.type == 'MESH':
        sel_objs.append(obj)

for obj in sel_objs:  # <<<<<<<<<<
    obj.select_set(True)  # <<<<<<<<<<

bpy.ops.mesh.boundbox_add()
bpy.ops.object.select_all(action='DESELECT')

# BoundingBox_size
bbox_list = []
bbox_verts = []
for o in scene.objects:
    if "Bounding" in o.name:
        bbox_list.append(o)

for box in bbox_list:
    obj = bpy.context.scene.objects.get(str(box.name))
    obj.select_set(True)

    bpy.ops.object.mode_set(mode="EDIT")

    wmtx = obj.matrix_world.copy()

    for v in obj.data.vertices:
        co_world = wmtx @ v.co
        bbox_verts.append(co_world)

    x_list = []
    y_list = []
    z_list = []


    def calculate_bbox_minmax(list):
        for i in range(len(list)):
            x_list.append(list[i][0])
            y_list.append(list[i][1])
            z_list.append(list[i][2])

        max_x = max(x_list)
        max_y = max(y_list)
        max_z = max(z_list)

        min_x = min(x_list)
        min_y = min(y_list)
        min_z = min(z_list)

        max_co = (max_x, max_y, max_z)
        min_co = (min_x, min_y, min_z)

        return max_co, min_co


    # print(calculate_bbox_minmax(bbox_verts))

    max_co, min_co = calculate_bbox_minmax(bbox_verts)
    min_co = [x + 0.5 for x in min_co]
    print(min_co)

    bpy.ops.object.mode_set(mode="OBJECT")
    bpy.ops.mesh.primitive_cube_add(size=1, location=min_co)
'''
'''
'''