import os
import open3d as o3d
import numpy as np
import time

def subsample_pcd2grid(camera_pts, voxel_size):
    voxel_size = voxel_size/2
    downpcd = camera_pts.voxel_down_sample(voxel_size= voxel_size)
    #o3d.visualization.draw_geometries([ downpcd ])
    o3d.io.write_point_cloud(
        f'D:/Program Files (x86)/Blender/2.90/scripts/BlenderShellDev/Alpha/plys/downsample_{voxel_size}.ply', downpcd)
save_path = "D:/Program Files (x86)/Blender/2.90/scripts/BlenderShellDev/Alpha/plys/"
os.makedirs(save_path, exist_ok=True)
complete_grid = os.path.join(save_path, "grids.ply")
pcd = o3d.io.read_point_cloud(complete_grid)

save_path = "D:/Program Files (x86)/Blender/2.90/scripts/BlenderShellDev/Alpha/plys/"
os.makedirs(save_path, exist_ok=True)
camera_pts = os.path.join(save_path, "ori_camera.ply")
cam_pcd = o3d.io.read_point_cloud(camera_pts)

for i in range(1, 10, 1):
    subsample_pcd2grid(cam_pcd, i)
print('done')

