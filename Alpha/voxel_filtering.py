import sys
import os
import pygad
import numpy as np
import time
import math
import open3d as o3d
import math

def filter_via_vectorlist(vector_list):
    return

if __name__ == "__main__":
    # File IO
    # open mesh model for raycasting
    # save_path = "/home/adrain/Desktop/bpydev/BlenderShellDev/Alpha/plys/"
    save_path = os.path.dirname(os.path.realpath(sys.argv[ 0 ]))
    os.makedirs(save_path, exist_ok=True)
    phase = 3
    file_name = os.path.join(save_path, f'plys/pre_filter/phase_{phase}.ply')
    model_mesh = o3d.io.read_triangle_mesh(file_name)
    model_mesh.paint_uniform_color([ 1, 0.706, 0 ])
    o3d.visualization.draw_geometries([model_mesh])

    # open filtered voxels
    # save_path1 = "/home/adrain/Desktop/bpydev/BlenderShellDev/Alpha/plys/"
    id = 153624
    model = os.path.join(save_path, f'plys/ori_voxels/for_filter/{phase}voxel_{id}.ply')
    model_pcd = o3d.io.read_point_cloud(model)
    ##save kdtree for computation
    model_kdtree = o3d.geometry.KDTreeFlann(model_pcd)
    ##save points for iteration
    model_pts = model_pcd.points
    model_pts = np.asarray(model_pts)

    # initializing raycast scene
    scene = o3d.t.geometry.RaycastingScene()
    cube_id = scene.add_triangles(np.asarray(model_mesh.vertices, dtype=np.float32),
                                  np.asarray(model_mesh.triangles, dtype=np.uint32))

    ray_cast_range = np.float(50)
    pt = np.asarray([0,0,0])
    '''
    x,y,z,-x,-y,-z
    
    '''
    queries = []
    possible_view = []
    for index, pt in enumerate(model_pcd.points):
        print('progress=',index,pt, index/len(model_pcd.points))
        ground = - 0.1
        if pt[2] >= ground:
            queries = []
            vector_list = np.asarray([ [ pt, pt - [pt[0] + ray_cast_range,pt[1],pt[2]]], [ pt, pt - [pt[0], pt[1] + ray_cast_range,pt[2]]],
                                       [ pt,pt - [pt[0], pt[1], pt[2] + ray_cast_range]], [ pt, pt - [ pt[ 0 ] - ray_cast_range, pt[ 1 ], pt[ 2 ] ] ],
                                     [ pt, pt - [ pt[ 0 ], pt[ 1 ] - ray_cast_range, pt[ 2 ] ] ],
                                    [ pt, pt - [ pt[ 0 ], pt[ 1 ], pt[ 2 ] - ray_cast_range ] ],
                                       [ pt, pt - [ pt[ 0 ] + ray_cast_range, pt[ 1 ] + ray_cast_range, pt[ 2 ] + ray_cast_range ]],
                                       [ pt, pt - [ pt[ 0 ] + ray_cast_range, pt[ 1 ] - ray_cast_range, pt[ 2 ] + ray_cast_range ]],
                                       [ pt, pt - [ pt[ 0 ] - ray_cast_range, pt[ 1 ] - ray_cast_range, pt[ 2 ] + ray_cast_range ]],
                                         [ pt, pt - [ pt[ 0 ] - ray_cast_range, pt[ 1 ] + ray_cast_range, pt[ 2 ] + ray_cast_range ]],
                                       [ pt, pt - [ pt[ 0 ] + ray_cast_range, pt[ 1 ] + ray_cast_range,
                                                    pt[ 2 ] - ray_cast_range ] ],
                                       [ pt, pt - [ pt[ 0 ] + ray_cast_range, pt[ 1 ] - ray_cast_range,
                                                    pt[ 2 ] - ray_cast_range ] ],
                                       [ pt, pt - [ pt[ 0 ] - ray_cast_range, pt[ 1 ] - ray_cast_range,
                                                    pt[ 2 ] - ray_cast_range ] ],
                                       [ pt, pt - [ pt[ 0 ] - ray_cast_range, pt[ 1 ] + ray_cast_range,
                                                    pt[ 2 ] - ray_cast_range ] ]
                                       ])
            #print(vector_list[0],vector_list[1],vector_list[2])
            for vec in vector_list:
                query = np.concatenate(vec, dtype=np.float32).ravel().tolist()
                queries.append(query)

            rays = o3d.core.Tensor(queries, dtype=o3d.core.Dtype.Float32)
            hits = scene.cast_rays(rays)
            #print(hits['t_hit'])
            if any(math.isinf(x.numpy()) is not False for x in hits['t_hit']):
                print('at least one hit is inf', hits['t_hit'])
                possible_view.append(index)
            # if all contain values
            else:
                for hit in hits['t_hit']:
                    if hit.numpy() >= 1:
                        print('out of range caught',hit.numpy()*ray_cast_range)
                        possible_view.append(index)
                        break
                    else:
                        print('in_range caught',hit.numpy()*ray_cast_range)
                        #possible_view.append(index)
    possible_view_set = set(possible_view)
    contains_duplicates = len(possible_view) != len(possible_view_set)
    if contains_duplicates:
        raise Exception('duplicates',len(possible_view),'!=',len(possible_view_set))
    filtered = model_pcd.select_by_index(possible_view)
    #pcd = o3d.geometry.PointCloud()
    #pcd.points = o3d.utility.Vector3dVector(filtered)
    o3d.io.write_point_cloud(f'./plys/ori_voxels/for_filter/presenting/new_filtered_{id}_{phase}.ply', filtered)
    print('done')
    o3d.visualization.draw_geometries([ filtered, model_mesh])



