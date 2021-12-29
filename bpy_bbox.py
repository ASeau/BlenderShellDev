import bpy
import sys
import os
from math import *
from mathutils import *
from datetime import datetime,date,timedelta
import ifcopenshell
import ifcopenshell.util

context = bpy.context
scene = context.scene
viewlayer = context.view_layer

sel_objs = []
for obj in scene.objects:
    taskend_frame = 0
    scene.frame_set(taskend_frame)
    if obj.type == 'MESH' and (obj.hide_viewport == False):
        sel_objs.append(obj)

for obj in sel_objs:  # <<<<<<<<<<
    obj.select_set(False)  # <<<<<<<<<<

while len(sel_objs) >= 1:
    obj1 = sel_objs.pop()
    obj1.select_set(True)
    bpy.ops.mesh.boundbox_add()
    bpy.ops.object.select_all(action='DESELECT')
