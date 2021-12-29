import bpy
import random
import json
import os
import math

for material in bpy.data.materials:
    material.user_clear()
    bpy.data.materials.remove(material)

dict_color = {}

for obj in bpy.context.scene.objects:

    if (obj.type == "MESH"):
        RGB = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        if RGB not in dict_color.values():
            me = obj.data
            mat = bpy.data.materials.new("Mat")

            mat.use_nodes = True
            nodes = mat.node_tree.nodes

            # clear all nodes to start clean
            nodes.clear()

            # create emission node
            node_emission = nodes.new(type='ShaderNodeEmission')
            node_emission.inputs[0].default_value = (RGB[0] / 255, RGB[1] / 255, RGB[2] / 255, 1)  # random mat color RGBA
            node_emission.inputs[1].default_value = 1.0  # strength
            node_emission.location = 0, 0

            # create output node
            node_output = nodes.new(type='ShaderNodeOutputMaterial')
            node_output.location = 400, 0

            # mat.diffuse_color = (RGB[0]/255,RGB[1]/255,RGB[2]/255, 1)
            obj.active_material = mat

            links = mat.node_tree.links
            link = links.new(node_emission.outputs[0], node_output.inputs[0])

            clr = (int(node_emission.inputs[0].default_value[0]*255), int(node_emission.inputs[0].default_value[1]*255),
                   int(node_emission.inputs[0].default_value[2]*255))

            dict_color[obj.name] = clr

print(dict_color)

# encode dict as JSON
data = json.dumps(dict_color, indent=1, ensure_ascii=True)

# set output path and file name (set your own)
save_path = "D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/"
os.makedirs(save_path, exist_ok=True)
file_name = os.path.join(save_path, "color_dict2.json")

# write JSON file
with open(file_name, 'w') as outfile:
    outfile.write(data + '\n')
'''
def to_hex(c):
    if c < 0.0031308:
        srgb = 0.0 if c < 0.0 else c * 12.92
    else:
        srgb = 1.055 * math.pow(c, 1.0 / 2.4) - 0.055

    return hex(max(min(int(srgb * 255 + 0.5), 255), 0))

def rgb_to_hex(rgb_tuple):
    return to_hex([1.0*x for x in rgb_tuple])
    
 mat.shadow_method = 'NONE'
        mat.diffuse_color = (random.randint(0,255)/256,random.randint(0,255)/256,random.randint(0,255)/256, 1)

        if (mat.diffuse_color[0], mat.diffuse_color[1], mat.diffuse_color[2]) not in dict_color:
            obj.active_material = mat
            RGB = (int(mat.diffuse_color[0]*256), int(mat.diffuse_color[1]*256), int(mat.diffuse_color[2]*256))
            dict_color[obj.name] = RGB
            
import bpy
import random

import json
import os

dict_clr = {}

shapes = bpy.data.collections["IfcProject/14400"].all_objects

for obj in shapes:
    RGB = (random.randint(0,255), random.randint(0,255), random.randint(0,255))
    if RGB not in dict_clr.values():
        me = obj.data
        mat = bpy.data.materials.new("Mat")
        
        mat.use_nodes = True
        nodes = mat.node_tree.nodes
        
        # clear all nodes to start clean
        nodes.clear()
        
        # create emission node
        node_emission = nodes.new(type='ShaderNodeEmission')
        node_emission.inputs[0].default_value = (RGB[0]/255,RGB[1]/255,RGB[2]/255, 1)  # random mat color RGBA
        node_emission.inputs[1].default_value = 1.0 # strength
        node_emission.location = 0,0
        
        # create output node
        node_output = nodes.new(type='ShaderNodeOutputMaterial')   
        node_output.location = 400,0
        
        #mat.diffuse_color = (RGB[0]/255,RGB[1]/255,RGB[2]/255, 1)
        obj.active_material = mat
        
        links = mat.node_tree.links
        link = links.new(node_emission.outputs[0], node_output.inputs[0])
    
        clr = (int(node_emission.inputs[0].default_value[0]*255),int(node_emission.inputs[0].default_value[1]*255), int(node_emission.inputs[0].default_value[2]*255))
        dict_clr[obj.name] = clr

# encode dict as JSON
data = json.dumps(dict_clr, indent=1, ensure_ascii=True)

# set output path and file name (set your own)
save_path = r"D:\1.Work\Ph.D\Pointcloud Processing\Blender\Practice Project"
os.makedirs(save_path, exist_ok=True)
file_name = os.path.join(save_path, "clr.json")

# write JSON file
with open(file_name, 'w') as outfile:
    outfile.write(data + '\n')
'''
'''
dict_clr = {}

shapes  = bpy.data.collections.all_objects
for obj in shapes:

    me = obj.data
    mat = bpy.data.materials.new("Mat")
    mat.diffuse_color = random(),random(),random(),1

    if (mat.diffuse_color[0],mat.diffuse_color[1],mat.diffuse_color[2]) not in dict_clr:
        obj.active_material = mat
        RGB = (mat.diffuse_color[0],mat.diffuse_color[1],mat.diffuse_color[2])
        dict_clr[obj.name] = RGB
'''