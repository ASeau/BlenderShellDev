import sys
import os
import pygad
import numpy as np
import time
import math
import open3d as o3d
import copy

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def isVisible(cam_co, half_plane_normals, voxel_co, fudge=0):
    lef = np.dot(voxel_co - cam_co, half_plane_normals[ 0 ])
    bot = np.dot(voxel_co - cam_co, half_plane_normals[ 1 ])
    rig = np.dot(voxel_co - cam_co, half_plane_normals[ 2 ])
    top = np.dot(voxel_co - cam_co, half_plane_normals[ 3 ])
    if lef < - fudge and bot < - fudge and rig < - fudge and top < - fudge:
        return True


def setup_cam(cam_co, x_angle, y_angle, z_angle, cam_config):

    cam_far = cam_config[0][0]

    def quaternion_multiply(quaternion1, quaternion0):
        w0, x0, y0, z0 = quaternion0
        w1, x1, y1, z1 = quaternion1
        return np.array([ -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                          x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                          -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                          x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0 ], dtype=np.float64)

    def get_quaternion_from_euler(roll, pitch, yaw):
        """
        (z,)
        Convert an Euler angle to a quaternion.

        Input
          :param roll: The roll (rotation around x-axis) angle in radians.
          :param pitch: The pitch (rotation around y-axis) angle in radians.
          :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
          :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
            yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
            yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)

        return [qw, qx, qy, qz]

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

    def frustum_calculation(intrinsic, cam_matrix, cam_far):
        cam = o3d.camera.PinholeCameraParameters()
        cam.intrinsic = intrinsic
        cam.extrinsic = cam_matrix

        frustum_lineset = o3d.geometry.LineSet.create_camera_visualization(cam.intrinsic, cam.extrinsic, scale=cam_far)
        frustum_mesh = o3d.geometry.TriangleMesh()
        frustum_mesh.vertices = o3d.utility.Vector3dVector(np.asarray(frustum_lineset.points))
        frustum_mesh.triangles = o3d.utility.Vector3iVector(
            np.array([ [ 0, 1, 2 ], [ 0, 2, 3 ], [ 0, 3, 4 ], [ 0, 1, 4 ], [ 1, 2, 3 ], [ 3, 4, 1 ] ]))
        frustum_mesh.paint_uniform_color([ 1, 0, 0 ])
        return frustum_mesh

    def cam_transform(frustum_mesh, cam_co, x_angle, y_angle, z_angle):
        co = (cam_co[ 0 ], cam_co[ 1 ], cam_co[ 2 ])
        translate_array = np.array([ cam_co[ 0 ], cam_co[ 1 ], cam_co[ 2 ] ])
        add_row = np.array([ 0, 0, 0, 1 ])
        '''
        x = pitch 
        y = yaw
        z = row
        '''
        # frustum_mesh.translate(neg_co)
        frustum_mesh.translate(co)
        rot_mat_x = frustum_mesh.get_rotation_matrix_from_xyz((np.radians(x_angle),
                                                               np.radians(0),
                                                               np.radians(0)))

        frustum_mesh.rotate(rot_mat_x, center=co)
        #rotated = o3d.geometry.TriangleMesh.create_coordinate_frame(size=20, origin=co)
        #rotated.rotate(rot_mat_x, center=co)

        rot_mat_y = frustum_mesh.get_rotation_matrix_from_xyz((np.radians(0),
                                                               np.radians(0),
                                                               np.radians(y_angle)))

        rot_mat_y = np.linalg.inv(rot_mat_y)
        frustum_mesh.rotate(rot_mat_y, center=co)
        # frustum_mesh.translate(co)
        #frustum_mesh = rotated + frustum_mesh
        print(rot_mat_x)

        # frustum_mesh.rotate(rot_mat_y,center=[0,0,0])
        # print(rot_mat_y)
        return frustum_mesh

    def half_plane_cal(frustum_mesh):
        points = np.asarray(frustum_mesh.vertices)
        print('frustum_end_points', points)
        ul = np.array(points[ 1 ] - points[ 0 ])
        ll = np.array(points[ 4 ] - points[ 0 ])
        lr = np.array(points[ 3 ] - points[ 0 ])
        ur = np.array(np.array(points[ 2 ] - points[ 0 ]))

        half_plane_normals = [
            # left
            normalize(np.cross(ul, ll)),
            # bottom
            normalize(np.cross(ll, lr)),
            # right
            normalize(np.cross(lr, ur)),
            # top
            normalize(np.cross(ur, ul))
        ]
        print('half:left,bott,right,top', half_plane_normals)
        return half_plane_normals

    intrinsic = CameraCone(cam_config)
    matrix = CameraMatrix()
    frustum_mesh = frustum_calculation(intrinsic, matrix, cam_far)

    frustum = cam_transform(frustum_mesh, cam_co, x_angle, y_angle, z_angle)
    half_plane_normals = half_plane_cal(frustum)

    frustum_mesh.compute_vertex_normals()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(frustum_mesh.vertices))
    pcd.estimate_normals()

    return half_plane_normals, frustum, pcd #cam_matrix, rot_mat, pcd


def inPTZcam_frustum(model_pcd, model_kdtree, location_index, dir_index, cam_config, ray_cast):
    cam_far = cam_config[ 0 ][ 0 ]
    # test case
    # z = 90
    # tilt =0,90:horizontal,180
    # pan = 0@world_x_axis,90@world_y_axis  # dir_list[dir_index] #CV coordinate: x right, y down, z forward
    cam_co = np.asarray(location_list[ location_index ])
    print('cam_co=', cam_co)
    print('cam_dir=', )
    half_plane_normals, frustum, normal= setup_cam(cam_co,
                                                                         dir_index[ 0 ],
                                                                         dir_index[ 1 ],
                                                                         dir_index[ 2 ],
                                                                         cam_config)

    def frustum_cropping(frustum_mesh, model_pcd):
        points = np.asarray(frustum_mesh.vertices)
        # Convert the corners array to have type float64
        bounding_polygon_y = points.astype("float64")

        # Crop by Y axis
        # Create a SelectionPolygonVolume
        vol_y = o3d.visualization.SelectionPolygonVolume()

        # You need to specify what axis to orient the polygon to.
        # I choose the "Y" axis. I made the max value the maximum Y of
        # the polygon vertices and the min value the minimum Y of the
        # polygon vertices.
        vol_y.orthogonal_axis = "Y"
        vol_y.axis_max = np.max(bounding_polygon_y[ :, 1 ])
        vol_y.axis_min = np.min(bounding_polygon_y[ :, 1 ])

        # Set all the Y values to 0 (they aren't needed since we specified what they
        # should be using just vol.axis_max and vol.axis_min).
        bounding_polygon_y[ :, 1 ] = 0

        # Convert the np.array to a Vector3dVector
        vol_y.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon_y)

        # Crop the point cloud using the Vector3dVector
        cropped_y_pcd = vol_y.crop_point_cloud(model_pcd)
        # bounding_box = cropped_pcd.get_axis_aligned_bounding_box()
        # bounding_box.color = (1, 0, 0)
        return cropped_y_pcd

    in_view = [ ]
    is_visible = [ ]
    queries = [ ]
    index_list = [ ]
    ##get in radius points
    [ k, idx, _ ] = model_kdtree.search_radius_vector_3d(cam_co, cam_far)
    model_pcd = model_pcd.select_by_index(idx)

    cropped_pcd = frustum_cropping(frustum, model_pcd)

    for index, pt in enumerate(model_pcd.points):
        if isVisible(cam_co, half_plane_normals, pt):
            # print(f'{pt}_is_visible')
            is_visible.append(index)

    model_pcd = model_pcd.select_by_index(is_visible)
    print(len(is_visible))
    ##vector from cam to point and normalize
    for index, pt in enumerate(model_pcd.points):
        # if isVisible(cam_matrix, half_plane_normals, pt, fudge=0):
        query = [ pt, cam_co - pt ]
        query = np.concatenate(query, dtype=np.float32).ravel().tolist()
        queries.append(query)
        index_list.append(index)

    print(len(queries))
    print(len(index_list))
    rays = o3d.core.Tensor(queries, dtype=o3d.core.Dtype.Float32)
    # print(rays)

    if ray_cast == 1:
        if len(rays) > 0:
            hits = scene.test_occlusions(rays)
            for index, item in enumerate(hits):
                if item:
                    pass
                else:
                    in_view.append(index_list[ index ])
        else:
            for item, index in enumerate(index_list):
                in_view.append(item)

    cropped_pcd = model_pcd.select_by_index(in_view)
    # frustum = o3d.geometry.LineSet.create_camera_visualization(intrinsic,cam_matrix,1)
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0,0,0])
    #o3d.visualization.draw_geometries([ model_mesh, cropped_pcd, frustum, normal, location_pcd, frame])
    return cropped_pcd, frustum

# GAfunctions
# Preparing camera locations
def in360cam_frustum(model_pcd, model_kdtree ,location_index, cam_config, ray_cast):
    cam_far = cam_config[1][0]
    in_view = []
    queries = []
    ##get in radius points
    cam_co = np.array([0, 0, 2])
    loc = location_list[location_index]
    [k, idx, _] = model_kdtree.search_radius_vector_3d(cam_co, cam_far)
    model_pcd = model_pcd.select_by_index(idx)

    ##vector from cam to point and normalize
    for pt in model_pcd.points:
        query = [pt, cam_co - pt]
        query = np.concatenate(query, dtype=np.float32).ravel().tolist()
        queries.append(query)

    rays = o3d.core.Tensor(queries, dtype=o3d.core.Dtype.Float32)

    if ray_cast == 1:
        #query = [in_range, loc - in_range]
        #query = np.concatenate(query,dtype=np.float32).ravel().tolist()
        #ray = o3d.core.Tensor([query],dtype=o3d.core.Dtype.Float32)
        hits = scene.test_occlusions(rays)

        for index,item in enumerate(hits):
            if item:
                pass
            else:
                in_view.append(index)

    model_pcd = model_pcd.select_by_index(in_view)
    coor_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0, 0, 2])
    o3d.visualization.draw_geometries([model_pcd, model_mesh, coor_mesh])

    return model_pcd

def read_cam_locations(path):
    location_pcd = o3d.io.read_point_cloud(path)
    location_list = np.array(location_pcd.points)
    location_index = [ i for i, _ in enumerate(location_list) ]
    if len(location_list) != len(location_index):
        raise Exception('location length and index length is different')
    return location_pcd, location_list, location_index

def GA_optimiztion(cam_num,cam_config):
    """
    (z,)
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    # camera_types
    # cam_config = [PTZ,360,LiDar]
    # cam_config = [[cam_far,cam_FOV][cam_far][cam_far, cam_FOV]]

    cam_type = 0
    camera_type_size = {'low': 0, 'high': 0}  # len(cam_config)}
    # installable locations
    camera_location_size = {'low': 0, 'high': len(location_index)}
    # installable angles
    rot_x = 0
    rot_x_size = {'low': 5, 'high': 180}
    rot_y = 0
    rot_y_size = {'low': 5, 'high': 360}

    function_inputs = [ ]
    function_inputs_ele = [ cam_type, location_index, rot_x, rot_y ]

    for cam in range(cam_num):
        function_inputs.extend(function_inputs_ele)  # Function inputs

    # print(function_inputs)
    # sol = (installable_list[index], pan, tilt)
    def result_visualization(solution, cam_num):
        print(solution)
        nested_solution = [ solution[ i:i + 4 ] for i in range(0, len(solution), 4) ]
        in_view_list = [ ]
        in_view_frustums = [ ]

        for i in range(cam_num):
            id = nested_solution[ i ][ 0 ]
            index = nested_solution[ i ][ 1 ]
            rot_x = nested_solution[ i ][ 2 ]
            rot_y = nested_solution[ i ][ 3 ]
            rot_list = np.asarray([ rot_x, rot_y, 0 ])
            print('rot_list==',rot_list)

            if id == 0:
                in_view, frustum = inPTZcam_frustum(model_pcd, model_kdtree, index, rot_list, cam_config, 1)
                in_view_list.append(in_view)
                in_view_frustums.append(frustum)

            elif id == 1:
                in_view_360 = in360cam_frustum(model_pcd, model_kdtree, index, cam_config, 1)
                in_view_list.append(in_view_360)

        draw_list = [pcd for pcd in in_view_list]
        draw_list += [frustums for frustums in in_view_frustums]
        draw_list.append(model_mesh)
        draw_list.append(location_pcd)
        o3d.visualization.draw_geometries(draw_list)
        return

    def result_export2blender(solution):

        return

    def fitness_func(solution, solution_idx):
        print(solution)
        total_score = len(np.asarray(model_pcd.points))
        print(total_score)
        nested_solution = [ solution[ i:i + 4 ] for i in range(0, len(solution), 4) ]
        in_view_list = [ ]

        for i in range(cam_num):
            id = nested_solution[ i ][ 0 ]
            index = nested_solution[ i ][ 1 ]
            rot_x = nested_solution[ i ][ 2 ]
            rot_y = nested_solution[ i ][ 3 ]
            rot_list = np.asarray([ rot_x, rot_y, 0 ])
            print('rot_list==',rot_list)

            if id == 0:
                in_view, frustum = inPTZcam_frustum(model_pcd, model_kdtree, index, rot_list, cam_config, 1)
                in_view_list.append(in_view)

            elif id == 1:
                in_view_360 = in360cam_frustum(model_pcd, model_kdtree, index, cam_config, 1)
                in_view_list.append(in_view_360)

        fitness = 0
        # 04/11 note: add color_info into calculation
        for pcd in in_view_list:
            fitness += len(np.asarray(pcd.points))/ total_score
        print('solution & fitness=',solution,'&',fitness)
        print('*' * 150)
        return fitness

    num_generations = 2
    num_parents_mating = 5
    fitness_function = fitness_func
    sol_per_pop = 10
    num_genes = len(function_inputs)
    parent_selection_type = "rank"
    keep_parents = 2
    crossover_type = "single_point"
    crossover_probability = 0.9
    mutation_type = "random"
    mutation_probability = 0.1
    stop_criteria = "saturate_20"
    gene_space = [ ]
    gene = [ camera_type_size, camera_location_size, rot_x_size, rot_y_size]
    for cam in range(cam_num):
        gene_space.extend(gene)

    # initiate GA
    ga_instance = pygad.GA(num_generations=num_generations,
                           num_parents_mating=num_parents_mating,
                           fitness_func=fitness_function,
                           sol_per_pop=sol_per_pop,
                           num_genes=num_genes,
                           parent_selection_type=parent_selection_type,
                           keep_parents=keep_parents,
                           crossover_type=crossover_type,
                           crossover_probability=crossover_probability,
                           mutation_type=mutation_type,
                           mutation_by_replacement=True,
                           mutation_probability=mutation_probability,
                           gene_space=gene_space,
                           allow_duplicate_genes=True,
                           gene_type=int,
                           stop_criteria=stop_criteria)

    # perform mutation and crossover ops
    # Running the GA to optimize the parameters of the function.
    ga_instance.run()
    print("Number of generations passed is {generations_completed}".format(
        generations_completed=ga_instance.generations_completed))
    ga_instance.plot_fitness()

    # Returning the details of the best solution.
    solution, solution_fitness, solution_idx = ga_instance.best_solution(ga_instance.last_generation_fitness)

    result_visualization(solution, cam_num)

    return solution, solution_fitness

# Preparing

if __name__ == "__main__":
    # File IO
    # open mesh model for raycasting
    # save_path = "/home/adrain/Desktop/bpydev/BlenderShellDev/Alpha/plys/"
    save_path = os.path.dirname(os.path.realpath(sys.argv[ 0 ]))
    #save_path = "D:/Program Files (x86)/Blender/2.90/scripts/BlenderShellDev/Alpha/plys/"
    os.makedirs(save_path, exist_ok=True)
    file_name = os.path.join(save_path, "renwen_raycasting_test.ply")
    model_mesh = o3d.io.read_triangle_mesh(file_name)
    # o3d.visualization.draw_geometries([model_mesh])

    # open filtered voxels
    # save_path1 = "/home/adrain/Desktop/bpydev/BlenderShellDev/Alpha/plys/"
    model = os.path.join(save_path, "filtered_test.ply")
    model_pcd = o3d.io.read_point_cloud(model)
    ##save kdtree for computation
    model_kdtree = o3d.geometry.KDTreeFlann(model_pcd)
    ##save points for iteration
    model_pts = model_pcd.points
    model_pts = np.asarray(model_pts)

    # open camera_postions
    # save_path = "/home/adrain/Desktop/bpydev/BlenderShellDev/Alpha/plys/"
    camera = os.path.join(save_path, "camera_points_pcd.ply")
    location_pcd, location_list, location_index = read_cam_locations(camera)

    # initializing raycast scene
    scene = o3d.t.geometry.RaycastingScene()
    cube_id = scene.add_triangles(np.asarray(model_mesh.vertices, dtype=np.float32),
                                  np.asarray(model_mesh.triangles, dtype=np.uint32))

    desired_output = 100  # Function output.
    cam_config = [ [ 50., 30, [ 1920, 1080 ] ], [ 100. ] ]
    for i in range(1,10):
        solution, solution_fitness = GA_optimiztion(cam_num=i, cam_config=cam_config)




'''
# change2quad

quad =[x,y,z,w] np.array 4x1

        quad = np.array([ 1, 0, 0, 0 ])
        quad_x = np.array([ 1, 1, 0, 0 ])
        quad_y = np.array([ 1, 0, 1, 0 ])
        quad_z = np.array([ 1, 0, 0, 1 ])
        rot_mat = frustum_mesh.get_rotation_matrix_from_quaternion(quad_x)
        
        #rot_mat = frustum_mesh.get_rotation_matrix_from_xyz((np.radians(x_angle),
        #                                                     np.radians(y_angle),
        #                                                     np.radians(z_angle)))
        
'''