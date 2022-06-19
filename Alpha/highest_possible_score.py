import sys
import os
import pygad
import numpy as np
import time
import math
import open3d as o3d
import json
#import concurrent.futures
from multiprocessing import Pool
import random
import csv
def in360cam_frustum(co, model_pcd, model_kdtree ,scene, ray_cast):

    cam_far = 50
    in_view = []
    queries = []
    ##get in radius points
    cam_co = co
    # frustum generation
    frustum = o3d.geometry.TriangleMesh.create_sphere(radius=2.5)
    frustum.translate(cam_co)
    frustum.paint_uniform_color(np.array([ 1, 0, 0 ]))

    [k, idx, _] = model_kdtree.search_radius_vector_3d(cam_co, cam_far)
    model_pcd = model_pcd.select_by_index(idx)

    ##vector from cam to point and normalize
    for pt in model_pcd.points:
        query = [pt, cam_co - pt]
        query = np.concatenate(query, dtype=np.float32).ravel().tolist()
        queries.append(query)

    rays = o3d.core.Tensor(queries, dtype=o3d.core.Dtype.Float32)

    if ray_cast == 1:
        if len(rays) > 0:
            hits = scene.test_occlusions(rays)
            for index,item in enumerate(hits):
                if item:
                    pass
                else:
                    in_view.append(index)

    cropped_pcd = model_pcd.select_by_index(in_view)
    #coor_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 2])
    #o3d.visualization.draw_geometries([model_pcd, model_mesh, coor_mesh])
    return cropped_pcd, frustum

if __name__ == "__main__":
    # material_reset()
    save_path = "D:/Program Files (x86)/Blender/2.90/scripts/BlenderShellDev/Alpha/"
    os.makedirs(save_path, exist_ok=True)
    distance = 20
    cam_path = os.path.join(save_path, f'plys/new_camlocsvoxel_{distance}.ply')
    camera = o3d.io.read_point_cloud(cam_path)
    # load
    phase_list = [ ]
    kdtree_list = [ ]
    scene_list = [ ]
    model_mesh_list = [ ]
    for p in range(1, 4, 1):
        file_name = os.path.join(save_path, f'plys/pre_filter/phase_{p}.ply')
        model_mesh = o3d.io.read_triangle_mesh(file_name)
        model_mesh_list.append(model_mesh)
        # initializing raycast scene
        scene = o3d.t.geometry.RaycastingScene()
        cube_id = scene.add_triangles(np.asarray(model_mesh.vertices, dtype=np.float32),
                                      np.asarray(model_mesh.triangles, dtype=np.uint32))
        scene_list.append(scene)
        # open filtered voxels
        model = os.path.join(save_path, f'plys/ori_voxels/for_filter/presenting/filter_{p}.ply')
        model_pcd = o3d.io.read_point_cloud(model)
        phase_list.append(model_pcd)
        ##save kdtree for computation
        model_kdtree = o3d.geometry.KDTreeFlann(model_pcd)
        kdtree_list.append(model_kdtree)

    #cam_pts = np.asarray(camera.points)
    for index, mesh in enumerate(model_mesh_list):
        phase_highest = []

        for id, co in enumerate(camera.points):
            print(co)
            print(index, id, len(camera.points))
            in_view_pcd, frustum = in360cam_frustum(co, phase_list[index], kdtree_list[index],
                             scene_list[index], 1 )
            phase_highest.append(in_view_pcd.points)

        print('highest:', len(phase_highest))
        temp_array = np.concatenate(phase_highest, axis=0)
        # print(len(temp_array))
        unique_id = np.unique(temp_array, axis=0)
        print('unique:',len(unique_id))
        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(unique_id)
        #cropped_pcd = phase_list[index].select_by_index(list(unique_id))
        o3d.visualization.draw_geometries([ pcd2, mesh , camera])
        cam_path = os.path.join(save_path, f'highest_voxel_{len(phase_list[index].points)}_{index+1}_{len(pcd2.points)}.ply')
        o3d.io.write_point_cloud(cam_path, pcd2)