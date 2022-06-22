import bpy
import sys
import os
import numpy as np
import bmesh
from mathutils import Vector
from mathutils.bvhtree import BVHTree
import open3d as o3d
import datetime

def join_objects_and_undo():
    context = bpy.context
    scene = bpy.context.scene
    depsgraph = bpy.context.evaluated_depsgraph_get()
    '''
    obs = []
    for ob in scene.objects:
        # whatever objects you want to join...
        if ob.type == 'MESH':
            obs.append(ob)
    print(len(obs))
    ctx = bpy.context.copy()

    # one of the objects to join
    ctx['active_object'] = obs[0]
    ctx['selected_editable_objects'] = obs
    bpy.ops.object.join(ctx)
    print(f'{len(obs)} objects joined')
    '''
    # context.view_layer.objects.active = context.scene.objects.get('_ncl1_892Mesh')
    tree = [ ]
    mtx = [ ]
    for obj in scene.objects:
        if obj.type == 'MESH': #and "Bounding" in obj.name :
            bm = bmesh.new()

            bpy.context.view_layer.objects.active = obj
            print(obj.name)
            bvhtree = BVHTree.FromObject(obj, depsgraph)
            mtx.append(obj.matrix_world.inverted())
            '''
            #mesh = obj.to_mesh(depsgraph,)
            bm.from_object(obj,depsgraph)
            bm.transform(obj.matrix_world)

            bvhtree = BVHTree.FromBMesh(bm)
            #bpy.data.meshes.remove(mesh)

            mtx.append(obj.matrix_world.inverted())
            print('name=', obj.name, bpy.context.object.name)
            bpy.ops.object.mode_set(mode='EDIT')
            bm = bmesh.from_edit_mesh(obj.data)
            '''
            tree.append(bvhtree)  # BVHTree.FromBMesh(bm,epsilon=1e5))
    '''
    bvhtree = BVHTree.FromObject(obj, depsgraph, epsilon=0.01)
                tree.append(bvhtree)
    '''
    print('bvhtree obtained', tree)
    # bpy.ops.ed.undo()
    print('action undone')
    # bpy.ops.object.mode_set(mode="OBJECT")
    return tree[ 0 ], mtx[ 0 ]

def filter_voxels(camera, bvhtree, mtx ,distance):
    cam_pts = np.asarray(camera.points)
    print(len(cam_pts))
    context = bpy.context
    scene = context.scene
    filtered_co = []
    in_range = []

    cam_pts = [[int(co[0]),int(co[1]),int(co[2])] for co in cam_pts]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cam_pts)
    '''
    for index, co in enumerate(cam_pts):
        
        
        
        bpy_co = Vector(co)
        local_co = mtx @ bpy_co
        _, _, _, dist = bvhtree.find_nearest(local_co, distance)
        if dist is not None:
            if dist < distance:
                print(f'progress:{index/len(cam_pts)}')
                in_range.append(index)
    '''
    return pcd

if __name__ == "__main__":
    # material_reset()
    now = datetime.time
    print("Starting script, Current Time =", now)
    save_path = "D:/Program Files (x86)/Blender/2.90/scripts/BlenderShellDev/Alpha/plys"
    os.makedirs(save_path, exist_ok=True)
    cam_path = os.path.join(save_path, "downsample_5.ply")
    camera = o3d.io.read_point_cloud(cam_path)

    #for distance in range(5,30,5):
    distance = 1
    bvhtree, mtx = join_objects_and_undo()
    in_range = filter_voxels(camera,bvhtree,mtx,distance)
    #print(len(in_range))
    #cropped_pcd = camera.select_by_index(in_range)
    cam_path = os.path.join(save_path, f'cam_loc.ply')
    o3d.io.write_point_cloud(cam_path, in_range)

    #o3d.visualization.draw_geometries([ cropped_pcd ])