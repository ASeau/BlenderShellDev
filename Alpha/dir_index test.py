import sys
import os
import pygad
import numpy as np
import time
import math
import open3d as o3d
import copy

def CameraCone(cam_config):
    fov = 118  # cam_config[0][1]
    resolution = cam_config[ 0 ][ 2 ]
    resolution_x, resolution_y = resolution
    fx = resolution_x / np.tan(fov / 2)
    print('fx=', fx)
    fy = fx * 9 / 16
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(1920, 1080, fx, fx, 1920 / 2, 1080 / 2)

    return intrinsic


def CameraMatrix():
    # translation array
    translate_array = np.array([ 0, 0, 0 ])
    add_row = np.array([ 0, 0, 0, 1 ])
    # Rotation matrix = Rz*Ry*Rx
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=translate_array)

    matrix = mesh.get_rotation_matrix_from_xyz((0, 0, 0))

    cam_matrix = np.c_[ matrix, translate_array ]
    cam_matrix = np.r_[ cam_matrix, [ add_row ] ]
    print("cam_matrix=", cam_matrix)
    return cam_matrix


def frustum_calculation(intrinsic, cam_matrix):
    cam = o3d.camera.PinholeCameraParameters()
    cam.intrinsic = intrinsic
    cam.extrinsic = cam_matrix

    frustum_lineset = o3d.geometry.LineSet.create_camera_visualization(cam.intrinsic, cam.extrinsic, scale=100.)
    frustum_mesh = o3d.geometry.TriangleMesh()
    frustum_mesh.vertices = o3d.utility.Vector3dVector(np.asarray(frustum_lineset.points))
    frustum_mesh.triangles = o3d.utility.Vector3iVector(
        np.array([ [ 0, 1, 2 ], [ 0, 2, 3 ], [ 0, 3, 4 ], [ 0, 1, 4 ], [ 1, 2, 3 ], [ 3, 4, 1 ] ]))
    frustum_mesh.paint_uniform_color([ 1, 0, 0 ])
    return frustum_mesh


def cam_transform(frustum_mesh, cam_co, x_angle, y_angle, z_angle):
    co = (cam_co[ 0 ], cam_co[ 1 ], cam_co[ 2 ])
    neg_co = (-cam_co[ 0 ], -cam_co[ 1 ], -cam_co[ 2 ])
    translate_array = np.array([ cam_co[ 0 ], cam_co[ 1 ], cam_co[ 2 ] ])
    add_row = np.array([ 0, 0, 0, 1 ])
    '''
    x = pitch 
    y = yaw
    z = row
    '''
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=translate_array)
    matrix = mesh.get_rotation_matrix_from_xyz((0, np.radians(y_angle), 0))
    print('ori_matrix',matrix)

    #frustum_mesh.translate(neg_co)
    rot_mat_x = frustum_mesh.get_rotation_matrix_from_xyz((np.radians(x_angle),
                                                         np.radians(0),
                                                         np.radians(0)))

    frustum_mesh.rotate(rot_mat_x, center=co)
    rotated = o3d.geometry.TriangleMesh.create_coordinate_frame(size=20, origin=translate_array)
    rotated.rotate(rot_mat_x,center=co)

    rot_mat_y = frustum_mesh.get_rotation_matrix_from_xyz((np.radians(0),
                                                           np.radians(0),
                                                           np.radians(y_angle)))

    rot_mat_y = np.linalg.inv(rot_mat_y)
    frustum_mesh.rotate(rot_mat_y, center=co)
    #frustum_mesh.translate(co)
    frustum_mesh = rotated + frustum_mesh
    print(rot_mat_x)

    #frustum_mesh.rotate(rot_mat_y,center=[0,0,0])
    #print(rot_mat_y)
    return frustum_mesh

cam_config = [ [ 100., 30, [ 1920, 1080 ] ], [ 100. ] ]
cam_co = np.array([0,0,0])
intrinsic = CameraCone(cam_config)
matrix = CameraMatrix()
frustum_list = []

for x_angle in range(0,360,10):
    frustum_mesh = frustum_calculation(intrinsic, matrix)
    frustum= cam_transform(frustum_mesh, cam_co, 110, x_angle, 0)
    print()
    frustum_list.append(frustum)
print(len(frustum_list))

coor_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[ 0, 0, 0 ])
o3d.visualization.draw_geometries([frustum_list[6],frustum_list[2],frustum_list[4],coor_mesh])
