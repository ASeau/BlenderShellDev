import bpy
import sys
import os
from math import *
from mathutils import *
import ColorChange as color

##Get cameras in scene

cam_ob = bpy.context.scene.camera

if cam_ob is None:
    print("no scene camera")
elif cam_ob.type == 'CAMERA':
    print("regular scene cam")
else:
    print("%s object as camera" % cam_ob.type)

cam_list = []
cam_dict = {}
ob = bpy.context.object
if ob is not None and ob.type == 'CAMERA':
    cam_list.append(ob.name)
    print(cam_list)

## Get camera properties
for cam in cam_list:
    cam_loc = bpy.data.objects[cam].location
    cam_rot = bpy.data.objects[cam].rotation_euler
    cam_dict.update(zip(cam_list,[cam_loc,cam_rot]))
    print(cam_loc,cam_rot)

##Output_frames

