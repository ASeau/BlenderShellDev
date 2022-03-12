import os
import open3d as o3d
import numpy as np
import time
from pprint import pprint
import matplotlib.pyplot as plt

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
save_path = "D:/User Data/Documents/Research Ref/Main_research/"
os.makedirs(save_path, exist_ok=True)
file_name = os.path.join(save_path, "subsampled_cloud.ply")
env_pts = o3d.io.read_point_cloud(file_name)

save_path1 = "D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/Alpha/correct/"
os.makedirs(save_path1, exist_ok=True)
model_pts = os.path.join(save_path1, "filtered.ply")
model_pcd = o3d.io.read_point_cloud(model_pts)

camera = [0, 0, 2]
radius = 100 * 100
print("Get all points that are visible from given view point")
_, pt_map = model_pcd.hidden_point_removal(camera, radius)
pcd = model_pcd.select_by_index(pt_map)
o3d.visualization.draw_geometries([pcd])

save_path = "D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/Alpha/"
os.makedirs(save_path, exist_ok=True)
complete_grid = os.path.join(save_path, "grids.ply")
pcd = o3d.io.read_point_cloud(complete_grid)

pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(len(pcd.points), 3)))

v_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,voxel_size=1)
#o3d.visualization.draw_geometries([v_grid])

print(v_grid)
bool_list = v_grid.check_if_included(env_pts.points)
print(bool_list[:10])
pprint(v_grid,indent=2)

filtered_voxel = []
for pts in env_pts.points:
    index = v_grid.get_voxel(pts)
    new_point = v_grid.get_voxel_center_coordinate(index)
    filtered_voxel.append(new_point)

filtered_pcd = o3d.geometry.PointCloud()
filtered_pcd.points = o3d.utility.Vector3dVector(filtered_voxel)
filtered_pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(len(filtered_pcd.points), 3)))
o3d.io.write_point_cloud("D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/Alpha/filtered_pcd.ply", filtered_pcd)

filtered_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(filtered_pcd,voxel_size=1)
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