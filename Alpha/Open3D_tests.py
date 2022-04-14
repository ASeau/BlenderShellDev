import os
import open3d as o3d
import numpy as np
import time
from pprint import pprint
import matplotlib.pyplot as plt
'''
save_path = "D:/Program Files (x86)/Blender/2.90/scripts/BlenderShellDev/Alpha/plys/"
os.makedirs(save_path, exist_ok=True)
file_name = os.path.join(save_path, "renwen_raycasting_test.ply")
model = o3d.io.read_triangle_mesh(file_name)

save_path1 = "D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/Alpha/correct/"
os.makedirs(save_path1, exist_ok=True)
model_pts = os.path.join(save_path1, "filtered.ply")
model_pcd = o3d.io.read_point_cloud(model_pts)
model_pcd.paint_uniform_color([1, 0, 0])

scene = o3d.t.geometry.RaycastingScene()
cube_id = scene.add_triangles(np.asarray(model.vertices,dtype=np.float32),np.asarray(model.triangles,dtype=np.uint32))
#rays = o3d.core.Tensor([[0.5, 0.5, 10, 0, 0, -1], [-1, -1, -1, 0, 0, -1]],
#                      dtype=o3d.core.Dtype.Float32)

rays = scene.create_rays_pinhole(fov_deg=60,
                                 center=[np.sin(np.radians(45)),np.sin(np.radians(45)),np.sin(np.radians(10))],
                                 eye=[0,0,0],
                                 up=[0,0,1],
                                 width_px=int(640/4),
                                 height_px=int(480/4))

ans = scene.cast_rays(rays)
print(ans)
plt.imshow(ans['t_hit'].numpy())

hit = ans['t_hit'].isfinite()
points = rays[hit][:,:3] + rays[hit][:,3:]*ans['t_hit'][hit].reshape((-1,1))
pcd = o3d.t.geometry.PointCloud(points)
# Press Ctrl/Cmd-C in the visualization window to copy the current viewpoint
o3d.visualization.draw_geometries([pcd.to_legacy(),model_pcd],
                                  front=[0.5, 0.86, 0.125],
                                  lookat=[0.23, 0.5, 2],
                                  up=[-0.63, 0.45, -0.63],
                                  zoom=1.0)

'''
def subsample_to_voxel_grid(grid,input_ply,voxel_size):
    v_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(grid, voxel_size=voxel_size)
    bool_list = v_grid.check_if_included(input_ply.points)

    filtered_voxel = []
    for pts in input_ply.points:
        index = v_grid.get_voxel(pts)
        new_point = v_grid.get_voxel_center_coordinate(index)
        filtered_voxel.append(new_point)

    down_sampled_pts = o3d.geometry.PointCloud()
    down_sampled_pts.points = o3d.utility.Vector3dVector(filtered_voxel)
    down_sampled_pts.colors = o3d.utility.Vector3dVector(
        np.random.uniform(0, 1, size=(len(down_sampled_pts.points), 3)))

    return down_sampled_pts

save_path = "D:/Program Files (x86)/Blender/2.90/scripts/BlenderShellDev/Alpha/plys/"
os.makedirs(save_path, exist_ok=True)
complete_grid = os.path.join(save_path, "grids.ply")
pcd = o3d.io.read_point_cloud(complete_grid)

save_path = "D:/Program Files (x86)/Blender/2.90/scripts/BlenderShellDev/Alpha/plys/"
os.makedirs(save_path, exist_ok=True)
file_name = os.path.join(save_path, "renwen_raycasting_test.ply")
mesh_ply = o3d.io.read_triangle_mesh(file_name)

file_name1 = os.path.join(save_path, "filtered.ply")
filtered_ply = o3d.io.read_point_cloud(file_name1)

#pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(len(pcd.points), 3)))

voxel_pt = []
v_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,voxel_size=1)
for voxel in v_grid.get_voxels():
    pt = v_grid.get_voxel_center_coordinate(voxel.grid_index)
    voxel_pt.append(pt)
print('ori=',voxel_pt[:10], np.amin(voxel_pt, axis=0),np.amax(voxel_pt, axis=0))

camera_points_pcd = o3d.geometry.PointCloud()
camera_points_pcd.points = o3d.utility.Vector3dVector(voxel_pt)
camera_points_pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(len(camera_points_pcd.points), 3)))
o3d.visualization.draw_geometries([camera_points_pcd, pcd])

#o3d.visualization.draw_geometries([v_grid])
start_time = time.perf_counter()
print('starting_voxel_generation')
model_voxel = o3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh_ply, voxel_size=1)
voxel_pt = []
for voxel in model_voxel.get_voxels():
    pt = model_voxel.get_voxel_center_coordinate(voxel.grid_index)
    voxel_pt.append(pt)
print('in_voxel=',voxel_pt[:10], np.amin(voxel_pt, axis=0),np.amax(voxel_pt, axis=0))

camera_points_pcd = o3d.geometry.PointCloud()
camera_points_pcd.points = o3d.utility.Vector3dVector(voxel_pt)
camera_points_pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(len(camera_points_pcd.points), 3)))
o3d.io.write_point_cloud(f'D:/Program Files (x86)/Blender/2.90/scripts/BlenderShellDev/Alpha/plys/voxel_fr_triangle.ply', camera_points_pcd)
duration = time.perf_counter() - start_time
print(duration)
o3d.visualization.draw_geometries([camera_points_pcd])#, filtered_ply])

'''
filtered_pts = []
for item,index in enumerate(model_voxel.get_voxels):
    new_pt = model_voxel.get_voxel_center_coordinates(index)
    if v_grid.check_if_included(new_pt):
        filtered_pts.append(new_pt)

camera_points_pcd = o3d.geometry.PointCloud()
camera_points_pcd.points = o3d.utility.Vector3dVector(filtered_pts)
camera_points_pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(len(camera_points_pcd.points), 3)))
o3d.io.write_point_cloud(f'D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/Alpha/site_only_pcd1.ply', camera_points_pcd)
o3d.visualization.draw_geometries([camera_points_pcd])

filtered_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(camera_points_pcd,voxel_size=1)
o3d.visualization.draw_geometries([filtered_grid])

mesh = o3d.io.read_triangle_mesh(file_name)
#o3d.visualization.draw_geometries([mesh])

#mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh])

print('voxelization')
start_time = time.perf_counter()
voxel_grid = o3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh, 1)
duration = time.perf_counter() - start_time
print('done,time=',duration)

o3d.visualization.draw_geometries([voxel_grid])
o3d.io.write_voxel_grid("D:/User/renwen_filtered_voxel.ply",voxel_grid)
# voxel from mesh
# compute 3 camera types
    # compute distance from camera
    # PTZ
        #compute in_view objects
    # 360
        #done
    # laser
        #compute included (using angle)


fply = o3d.io.read_point_cloud(file_name)
fply.paint_uniform_color([1, 0, 0])
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(fply,
                                                            voxel_size=0.5)
o3d.visualization.draw_geometries([voxel_grid])

print("Define parameters used for hidden_point_removal")
diameter = 10
camera = [0, 0, 10]
radius = 10000

print("Get all points that are visible from given view point")
_, pt_map = fply.hidden_point_removal(camera, radius)

print("Visualize result")
pcd = fply.select_by_index(pt_map)
o3d.visualization.draw_geometries([pcd])
xyz_load = np.asarray(pcd.points)

print('xyz_load')
print(xyz_load)
save_path = "D:/User/"
os.makedirs(save_path, exist_ok=True)
file_name1 = os.path.join(save_path, "camera.ply")
o3d.io.write_point_cloud(file_name1, pcd)
'''