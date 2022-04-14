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
    lef = np.dot(voxel_co - cam_co, half_plane_normals[0])
    bot = np.dot(voxel_co - cam_co, half_plane_normals[1])
    rig = np.dot(voxel_co - cam_co, half_plane_normals[2])
    top = np.dot(voxel_co - cam_co, half_plane_normals[3])
    if lef < - fudge and bot < - fudge and rig < - fudge and top < - fudge:
        return True

def setup_cam(cam_co,x_angle,y_angle,z_angle,cam_config):
    def CameraCone(cam_config):
        fov = 118 #cam_config[0][1]
        resolution = cam_config[0][2]
        resolution_x, resolution_y = resolution
        fx = resolution_x / np.tan(fov / 2)
        print('fx=',fx)
        fy = fx * 9 / 16
        intrinsic = o3d.camera.PinholeCameraIntrinsic()
        intrinsic.set_intrinsics(1920,1080,fx,fx,1920/2,1080/2)

        return intrinsic

    def CameraMatrix():
        #translation array
        translate_array = np.array([0,0,0])
        add_row = np.array([0,0,0,1])
        #Rotation matrix = Rz*Ry*Rx
        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=translate_array)
        matrix = mesh.get_rotation_matrix_from_xyz((0, 0, 0))
        cam_matrix = np.c_[matrix, translate_array]
        cam_matrix = np.r_[cam_matrix, [add_row]]
        print("cam_matrix=",cam_matrix)
        return cam_matrix

    def frustum_calculation(intrinsic,cam_matrix):
        cam = o3d.camera.PinholeCameraParameters()
        cam.intrinsic = intrinsic
        cam.extrinsic = cam_matrix

        frustum_lineset = o3d.geometry.LineSet.create_camera_visualization(cam.intrinsic, cam.extrinsic, scale=100.)
        frustum_mesh = o3d.geometry.TriangleMesh()
        frustum_mesh.vertices = o3d.utility.Vector3dVector(np.asarray(frustum_lineset.points))
        frustum_mesh.triangles = o3d.utility.Vector3iVector(
            np.array([[0, 1, 2], [0, 2, 3], [0, 3, 4], [0, 1, 4], [1, 2, 3], [3, 4, 1]]))
        frustum_mesh.paint_uniform_color([1, 0, 0])
        return frustum_mesh

    def cam_transform(frustum_mesh,cam_co,x_angle,y_angle,z_angle):
        co = (cam_co[0], cam_co[1], cam_co[2])
        translate_array = np.array([cam_co[0], cam_co[1], cam_co[2]])
        add_row = np.array([0, 0, 0, 1])

        frustum_mesh.translate(co)
        rot_mat = frustum_mesh.get_rotation_matrix_from_xyz((np.radians(x_angle),
                                                        np.radians(y_angle),
                                                        np.radians(z_angle)))
        print(rot_mat)
        frustum_mesh.rotate(rot_mat, center=co)
        cam_matrix = np.c_[rot_mat, translate_array]
        cam_matrix = np.r_[cam_matrix, [add_row]]
        return frustum_mesh, cam_matrix, rot_mat

    def half_plane_cal(frustum_mesh):
        points = np.asarray(frustum_mesh.vertices)
        print('frustum_end_points',points)
        ul = np.array(points[1]-points[0])
        ll = np.array(points[4]-points[0])
        lr = np.array(points[3]-points[0])
        ur = np.array(np.array(points[2]-points[0]))

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
        print('half:left,bott,right,top',half_plane_normals)
        return half_plane_normals

    intrinsic = CameraCone(cam_config)
    matrix = CameraMatrix()
    frustum_mesh = frustum_calculation(intrinsic, matrix)

    frustum, cam_matrix, rot_mat = cam_transform(frustum_mesh,cam_co,x_angle,y_angle,z_angle)
    half_plane_normals = half_plane_cal(frustum)

    frustum_mesh.compute_vertex_normals()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(frustum_mesh.vertices))
    pcd.estimate_normals()

    return half_plane_normals,frustum,cam_matrix,rot_mat,pcd

def inPTZcam_frustum(model_pcd, model_kdtree ,location_index, dir_index, cam_config, ray_cast):
    cam_far = cam_config[0][0]
    #test case
    # z = 90
    # tilt =0,90:horizontal,180
    # pan = 0@world_x_axis,90@world_y_axis
    cam_dir = np.asarray([dir_index[0],dir_index[1],0])#dir_list[dir_index] #CV coordinate: x right, y down, z forward
    cam_co = np.asarray(location_list[location_index])
    print('cam_co=',cam_co)
    half_plane_normals, frustum, cam_matrix, rot_mat, normal = setup_cam(cam_co,
                                                                 cam_dir[0],
                                                                 cam_dir[1],
                                                                 cam_dir[2],
                                                                 cam_config)

    def frustum_cropping(frustum_mesh, model_pcd):
        points = np.asarray(frustum_mesh.vertices)
        # Convert the corners array to have type float64
        bounding_polygon_y = points.astype("float64")

        #Crop by Y axis
        # Create a SelectionPolygonVolume
        vol_y = o3d.visualization.SelectionPolygonVolume()

        # You need to specify what axis to orient the polygon to.
        # I choose the "Y" axis. I made the max value the maximum Y of
        # the polygon vertices and the min value the minimum Y of the
        # polygon vertices.
        vol_y.orthogonal_axis = "Y"
        vol_y.axis_max = np.max(bounding_polygon_y[:, 1])
        vol_y.axis_min = np.min(bounding_polygon_y[:, 1])

        # Set all the Y values to 0 (they aren't needed since we specified what they
        # should be using just vol.axis_max and vol.axis_min).
        bounding_polygon_y[:, 1] = 0

        # Convert the np.array to a Vector3dVector
        vol_y.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon_y)

        # Crop the point cloud using the Vector3dVector
        cropped_y_pcd = vol_y.crop_point_cloud(model_pcd)
        #bounding_box = cropped_pcd.get_axis_aligned_bounding_box()
        #bounding_box.color = (1, 0, 0)
        return cropped_y_pcd

    in_view = []
    is_visible = []
    queries = []
    index_list = []
    ##get in radius points
    [k, idx, _] = model_kdtree.search_radius_vector_3d(cam_co, cam_far)
    model_pcd = model_pcd.select_by_index(idx)

    cropped_pcd = frustum_cropping(frustum, model_pcd)

    for index, pt in enumerate(model_pcd.points):
        if isVisible(cam_co,half_plane_normals,pt):
            #print(f'{pt}_is_visible')
            is_visible.append(index)

    model_pcd = model_pcd.select_by_index(is_visible)
    print(len(is_visible))
    ##vector from cam to point and normalize
    for index, pt in enumerate(model_pcd.points):
        #if isVisible(cam_matrix, half_plane_normals, pt, fudge=0):
        query = [pt, cam_co - pt]
        query = np.concatenate(query, dtype=np.float32).ravel().tolist()
        queries.append(query)
        index_list.append(index)

    print(len(queries))
    print(len(index_list))
    rays = o3d.core.Tensor(queries, dtype=o3d.core.Dtype.Float32)
    #print(rays)

    if ray_cast == 1:
        if len(rays) > 0:
            hits = scene.test_occlusions(rays)
            for index, item in enumerate(hits):
                if item:
                    pass
                else:
                    in_view.append(index_list[index])
        else:
            for item, index in enumerate(index_list):
                in_view.append(item)

    cropped_pcd = model_pcd.select_by_index(in_view)
    #frustum = o3d.geometry.LineSet.create_camera_visualization(intrinsic,cam_matrix,1)
    o3d.visualization.draw_geometries([model_mesh, cropped_pcd, frustum, normal, location_pcd])
    return cropped_pcd

##computes 360 camera voxels, set ray_cast=0 to turn off, 1 for on##
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

def inlidar_frustum(model_pcd, model_kdtree ,location_index, cam_config, ray_cast):
    cam_far = cam_config[2][0]
    lidar_fov = cam_config[2][1]
    fov_vec = np.sin(np.radians(lidar_fov))

    in_view = []
    queries = []
    index_list = []
    ##get in radius points
    loc = location_list[location_index]
    cam_co = np.array([0, 0, 2])
    [k, idx, _] = model_kdtree.search_radius_vector_3d(cam_co, cam_far)
    model_pcd = model_pcd.select_by_index(idx)

    ##vector from cam to point and normalize
    for index, pt in enumerate(model_pcd.points):
        dir_vec = pt - cam_co
        norm = normalize(dir_vec)

        if norm[2] <= fov_vec and norm[2] >= -fov_vec:
            query = [pt, -dir_vec]
            query = np.concatenate(query, dtype=np.float32).ravel().tolist()
            queries.append(query)
            index_list.append(index)

    rays = o3d.core.Tensor(queries, dtype=o3d.core.Dtype.Float32)

    if ray_cast == 1:
        # query = [in_range, loc - in_range]
        # query = np.concatenate(query,dtype=np.float32).ravel().tolist()
        # ray = o3d.core.Tensor([query],dtype=o3d.core.Dtype.Float32)
        hits = scene.test_occlusions(rays)

        for index, item in enumerate(hits):
            if item:
                pass
            else:
                in_view.append(index_list[index])

    model_pcd = model_pcd.select_by_index(in_view)
    coor_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100,origin=[0,0,2])
    o3d.visualization.draw_geometries([model_pcd, model_mesh, coor_mesh])

    return model_pcd

#GAfunctions
#Preparing camera locations
def read_cam_locations(path):
    location_pcd = o3d.io.read_point_cloud(path)
    location_list = np.array(location_pcd.points)
    location_index = [i for i, _ in enumerate(location_list)]
    if len(location_list) != len(location_index):
        raise Exception('location length and index length is different')
    return location_pcd, location_list, location_index
#Preparing

if __name__ == "__main__":
    # File IO
    # open mesh model for raycasting
    #save_path = "/home/adrain/Desktop/bpydev/BlenderShellDev/Alpha/plys/"
    save_path = "D:/Program Files (x86)/Blender/2.90/scripts/BlenderShellDev/Alpha/plys/"
    os.makedirs(save_path, exist_ok=True)
    file_name = os.path.join(save_path, "renwen_raycasting_test.ply")
    model_mesh = o3d.io.read_triangle_mesh(file_name)
    #o3d.visualization.draw_geometries([model_mesh])

    # open filtered voxels
    #save_path1 = "/home/adrain/Desktop/bpydev/BlenderShellDev/Alpha/plys/"
    save_path1 = "D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/Alpha/correct/"
    os.makedirs(save_path1, exist_ok=True)
    model = os.path.join(save_path1, "filtered.ply")
    model_pcd = o3d.io.read_point_cloud(model)
    ##save kdtree for computation
    model_kdtree = o3d.geometry.KDTreeFlann(model_pcd)
    ##save points for iteration
    model_pts = model_pcd.points
    model_pts = np.asarray(model_pts)

    # open camera_postions
    #save_path = "/home/adrain/Desktop/bpydev/BlenderShellDev/Alpha/plys/"
    save_path1 = "D:/Program Files (x86)/Blender/2.90/scripts/BlenderShellDev/Alpha/plys/present/"
    os.makedirs(save_path1, exist_ok=True)
    camera = os.path.join(save_path1, "camera_points_pcd.ply")
    location_pcd, location_list, location_index = read_cam_locations(camera)

    # initializing raycast scene
    scene = o3d.t.geometry.RaycastingScene()
    cube_id = scene.add_triangles(np.asarray(model_mesh.vertices, dtype=np.float32),
                                  np.asarray(model_mesh.triangles, dtype=np.uint32))

    desired_output = 100  # Function output.

    def GA_optimiztion():
        # GA ops
        # camera_types
        # cam_config = [PTZ,360,LiDar]
        # cam_config = [[cam_far,cam_FOV][cam_far][cam_far, cam_FOV]]
        cam_config = [[100., 30, [1920, 1080]], [100.]]
        cam_type = 0
        camera_type_size = {'low': 0, 'high': 0} #len(cam_config)}
        # installable locations
        camera_location_size = {'low': 0 , 'high': len(location_index)}
        # installable angles
        rot_x = 0
        rot_x_size = {'low': 0, 'high': 180}
        rot_y = 0
        rot_y_size = {'low': 0, 'high': 350}

        cam_num = 1
        function_inputs = []
        function_inputs_ele = [cam_type, location_index, rot_x, rot_y]

        for cam in range(cam_num):
            function_inputs.extend(function_inputs_ele)  # Function inputs

        # print(function_inputs)
        # sol = (installable_list[index], pan, tilt)

        def fitness_func(solution, solution_idx):
            print(solution)
            nested_solution = [solution[i:i + 4] for i in range(0, len(solution), 4)]
            in_view_list = []

            for i in range(cam_num):
                id = nested_solution[i][0]
                index = nested_solution[i][1]
                rot_x = nested_solution[i][2]
                rot_y = nested_solution[i][3]
                rot_list = np.asarray([rot_x,rot_y])

                if id == 0:
                    in_view = inPTZcam_frustum(model_pcd, model_kdtree, index, rot_list, cam_config, 1)
                    in_view_list.append(in_view)
                elif id == 1:
                    in_view_360 = in360cam_frustum(model_pcd, model_kdtree, index, cam_config, 1)
                    in_view_list.append(in_view_360)

            fitness = 0
            # 04/11 note: add color_info into calculation
            for pcd in in_view_list:
                fitness = fitness + len(np.asarray(pcd.points))
            print('fitness',fitness)
            return fitness

        num_generations = 100
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
        gene_space = []
        gene = [camera_type_size, camera_location_size, rot_x_size, rot_y_size]
        for cam in range(cam_num):
            gene_space.extend(gene)

        #initiate GA
        ga_instance = pygad.GA(num_generations=num_generations,
                               num_parents_mating=num_parents_mating,
                               fitness_func=fitness_function,
                               sol_per_pop=sol_per_pop,
                               num_genes=num_genes,
                               parent_selection_type=parent_selection_type,
                               keep_parents=keep_parents,
                               crossover_type=crossover_type,
                               crossover_probability = crossover_probability,
                               mutation_type=mutation_type,
                               mutation_by_replacement= True,
                               mutation_probability = mutation_probability,
                               gene_space = gene_space,
                               allow_duplicate_genes = True,
                               gene_type=int,
                               stop_criteria=stop_criteria)

        #perform mutation and crossover ops
        # Running the GA to optimize the parameters of the function.
        ga_instance.run()
        print("Number of generations passed is {generations_completed}".format(generations_completed=ga_instance.generations_completed))
        ga_instance.plot_fitness()

        # Returning the details of the best solution.
        solution, solution_fitness, solution_idx = ga_instance.best_solution(ga_instance.last_generation_fitness)

        return solution, solution_fitness

    solution, solution_fitness = GA_optimiztion()

