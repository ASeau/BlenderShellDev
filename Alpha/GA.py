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
from deap import creator, base, tools, algorithms

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

def isVisible(cam_co, half_plane_normals, voxel_co, fudge=0.0001):
    vec = voxel_co - cam_co
    front = -1 * np.dot(vec, half_plane_normals[ 0 ])
    if front < - fudge:
        lef = np.dot(vec, half_plane_normals[ 1 ])
        bot = np.dot(vec, half_plane_normals[ 2 ])
        rig = np.dot(vec, half_plane_normals[ 3 ])
        top = np.dot(vec, half_plane_normals[ 4 ])
        if lef < - fudge and bot < - fudge and rig < - fudge and top < - fudge:
            return True
    else:
        return False
    '''
    for norm in half_plane_normals:
        z2 = np.dot(vec,norm)
        if z2 < -fudge:
            return True
    return False
    '''

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
        #print('fx=', fx)
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
        #print("cam_matrix=", cam_matrix)
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
        #print(rot_mat_x)

        # frustum_mesh.rotate(rot_mat_y,center=[0,0,0])
        # print(rot_mat_y)
        return frustum_mesh

    def half_plane_cal(frustum_mesh):
        points = np.asarray(frustum_mesh.vertices)
        #print('frustum_end_points', points)
        uf = np.array(points[ 2 ] - points[ 1 ])
        lf = np.array(points[ 4 ] - points[ 1 ])
        ul = np.array(points[ 1 ] - points[ 0 ])
        ll = np.array(points[ 4 ] - points[ 0 ])
        lr = np.array(points[ 3 ] - points[ 0 ])
        ur = np.array(np.array(points[ 2 ] - points[ 0 ]))

        half_plane_normals = [
            # front
            normalize(np.cross(uf, lf)),
            # left
            normalize(np.cross(ul, ll)),
            # bottom
            normalize(np.cross(ll, lr)),
            # right
            normalize(np.cross(lr, ur)),
            # top
            normalize(np.cross(ur, ul))
        ]
        #print('half:left,bott,right,top', half_plane_normals)
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


def inPTZcam_frustum(model_pcd, model_kdtree, scene, location_list,
                     location_index, dir_index, cam_config, ray_cast):
    cam_far = cam_config[ 0 ][ 0 ]
    # test case
    # z = 90
    # tilt =0,90:horizontal,180
    # pan = 0@world_x_axis,90@world_y_axis  # dir_list[dir_index] #CV coordinate: x right, y down, z forward
    cam_co = np.asarray(location_list[ location_index ])
    #print('cam_co=', cam_co)
    #print('cam_dir=', )
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
    #print(len(is_visible))
    ##vector from cam to point and normalize
    for index, pt in enumerate(model_pcd.points):
        # if isVisible(cam_matrix, half_plane_normals, pt, fudge=0):
        query = [ pt, cam_co - pt ]
        query = np.concatenate(query, dtype=np.float32).ravel().tolist()
        queries.append(query)
        index_list.append(index)

    #print(len(queries))
    #print(len(index_list))
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
            for index, item in enumerate(index_list):
                in_view.append(index_list[ index ])

    if ray_cast == 1:
        cropped_pcd = model_pcd.select_by_index(in_view)
    else:
        cropped_pcd = model_pcd
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

def read_cam_locations(path, write_file):
    location_pcd = o3d.io.read_point_cloud(path)
    location_list = np.array(location_pcd.points)
    location_index = [ i for i, _ in enumerate(location_list) ]

    if write_file == 1:
        loc_index_dict = {}
        print(location_list)
        for index, co in enumerate(location_index):
            tmp =  dict(zip([str(location_list[index])],[index]))
            loc_index_dict.update(tmp)
        with open('file_name.json', 'w', encoding='utf-8') as f:
            json.dump(loc_index_dict, f, ensure_ascii=False, indent=4)
        print('json created')

    if len(location_list) != len(location_index):
        raise Exception('location length and index length is different')

    return location_pcd, location_list, location_index

# print(function_inputs)
# sol = (installable_list[index], pan, tilt)


def GA_optimiztion(cam_num,cam_config,location_list, location_index,
                   phase_list, kdtree_list, scene_list, model_mesh_list):

    start_time = time.perf_counter()

    num_generations = 100 #65
    num_parents_mating = 95 #
    sol_per_pop = 100 #250
    parent_selection_type = "tournament"
    keep_parents = 5
    '''
    num_generations = 1
    num_parents_mating = 2
    sol_per_pop = 2
    parent_selection_type = "rank"
    keep_parents = 1
    '''
    crossover_type = "uniform"
    crossover_probability = 0.9 #0.9
    mutation_type = "random" #"random"
    # [probabilty if high,probability if low]
    mutation_probability = 0.1 #[ 0.8, 0.2 ] #0.05
    score_discount = 1.0
    saturate_gen = 100
    stop_criteria = [ f'reach_{score_discount * 100}', f'saturate_{saturate_gen}' ]
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
    rot_x_size = range(45,135,15)#{'low': 5, 'high': 90}
    rot_y = 0
    rot_y_size = range(30,360,30)#{'low': 5, 'high': 360}

    function_inputs = [ ]
    function_inputs_ele = [ cam_type, location_index, rot_x, rot_y ]

    for cam in range(cam_num):
        function_inputs.extend(function_inputs_ele)  # Function inputs

    initial = [ 0, 7, 90, 225, 0, 93, 90, 180, 0, 150, 90, 0, 0, 223, 90, 0 ]
    #flatlist = [ element for sublist in initial for element in sublist ]
    population = [ initial ] * sol_per_pop
    def result_export2blender(solution):

        return

    def result_visualization(solution, cam_num):
        print('starting_visualization')
        print('solution==[cam_type,index,x,y]', solution)
        print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))
        print('solution==[cam_type,index,x,y]', solution)
        total_score = len(np.asarray(model_pcd.points))
        print('expected_score is', total_score)

        nested_solution = [ solution[ i:i + 4 ] for i in range(0, len(solution), 4) ]
        in_view_list = [ ]
        in_view_frustums = [ ]
        fitness_list = [ ]
        for i in range(cam_num):
            id = nested_solution[ i ][ 0 ]
            index = nested_solution[ i ][ 1 ]
            rot_x = nested_solution[ i ][ 2 ]
            rot_y = nested_solution[ i ][ 3 ]
            rot_list = np.asarray([ rot_x, rot_y, 0 ])
            print('rot_list==[x,y,z]', rot_list)

            for countt, phase in enumerate(phase_list):
                if id == 0:
                    in_view, frustum = inPTZcam_frustum(phase, kdtree_list[countt], scene_list[countt],
                                                        location_list, index, rot_list, cam_config,
                                                        1)
                    score = len(np.asarray(in_view.points))
                    total = len(np.asarray(phase.points))
                    fitness_list.append(score/total)
                    in_view_list.append(in_view)

                    in_view_frustums.append(frustum)

                elif id == 1:
                    in_view_360 = in360cam_frustum(model_pcd, model_kdtree, index, cam_config, 1)
                    in_view_list.append(in_view_360)

        draw_list = [ pcd for pcd in in_view_list ]

        fitness = 0
        for index, item in enumerate(fitness_list):
            print(f'phase{index}_score:', item)

        all_fitness = np.sum(fitness_list)/len(phase_list)
        print('all_phases = fitness_solution:', all_fitness)

        draw_list += [frustums for frustums in in_view_frustums]
        draw_list.append(model_mesh_list[-1])
        draw_list.append(location_pcd)
        o3d.visualization.draw_geometries(draw_list)

        # camera_points_pcd = o3d.geometry.PointCloud()
        # camera_points_pcd.points = o3d.utility.Vector3dVector(draw_list)
        # o3d.io.write_point_cloud(f'./{solution}_{cam_num}.ply',camera_points_pcd)
        return

    def fitness_func(solution, solution_idx):
        phase_fitness = [ ]
        for count, phase in enumerate(phase_list):
            in_view_list = [ ]
            print('starting phase:',count)
            print('solution==[cam_type,index,x,y]',solution)
            total_score = len(np.asarray(phase.points))
            print('expected_score is',total_score)
            nested_solution = [ solution[ i:i + 4 ] for i in range(0, len(solution), 4) ]

            # Introduce multiprocessing
            for i in range(cam_num):
                id = nested_solution[ i ][ 0 ]
                index = nested_solution[ i ][ 1 ]
                rot_x = nested_solution[ i ][ 2 ]
                rot_y = nested_solution[ i ][ 3 ]
                rot_list = np.asarray([ rot_x, rot_y, 0 ])

                if id == 0:
                    in_view, frustum = inPTZcam_frustum(phase, kdtree_list[count], scene_list[count],
                                                        location_list, index, rot_list, cam_config, 1)
                    in_view_list.append(len(np.asarray(in_view.points)))

            phase_fitness.append(100 * sum(in_view_list)/total_score)
            print('solution & fitness=', solution, '&', phase_fitness[count], '%', int((phase_fitness[count] / 100) * total_score))
        fitness = sum(phase_fitness)/len(phase_list)
        print('all_phases, solution & fitness=', solution, '&', fitness)
        #fitness = int(fitness)
        #print('rounded_fitness', fitness)
        '''
        # 04/11 note: add color_info into calculation
        for index, pcd in enumerate(in_view_list):
            camera_percentage = len(np.asarray(pcd.points))/ total_score
            if camera_percentage >= 0.01:
                fitness = fitness + len(np.asarray(pcd.points))/ total_score
            else:
                break
            print(index, 'individual solution & fitness=', solution, '&', len(np.asarray(pcd.points)), '&', fitness)
        '''
        #fitness = 100 * fitness/len(in_view_list)
        #print('averaged_fitness',fitness)
        duration = time.perf_counter() - start_time
        print('time used so far:',duration)
        print('*' * 150)

        return fitness

    fitness_function = fitness_func
    num_genes = len(function_inputs)

    gene_space = [ ]
    gene = [ camera_type_size, camera_location_size, rot_x_size, rot_y_size]
    print('gene_setup:', gene)
    for cam in range(cam_num):
        gene_space.extend(gene)

    # initiate GA
    ga_instance = pygad.GA(num_generations=num_generations,
                           num_parents_mating=num_parents_mating,
                           fitness_func=fitness_function,
                           initial_population= population,
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
                           allow_duplicate_genes=False,
                           gene_type=int,
                           stop_criteria=stop_criteria,
                           save_best_solutions= True)


    # perform mutation and crossover ops
    # Running the GA to optimize the parameters of the function.
    ga_instance.run()
    print("Number of generations passed is {generations_completed}".format(
        generations_completed=ga_instance.generations_completed))
    ga_instance.plot_fitness()

    # Returning the details of the best solution.
    best = np.sort(ga_instance.best_solutions_fitness)
    print('best_sol=',best)
    print(ga_instance.last_generation_fitness)
    #entrance,office,fence,renwen
    #[  0 7 150 180   0  93  150 0   0 150  150 180   0 223 150 0]
    solution, solution_fitness, solution_idx = ga_instance.best_solution(best)

    #solution = [  0, 7, 90, 225,   0,  93,  90, 180,   0, 150,  90, 0,   0, 223, 90, 0]

    result_visualization(solution,cam_num)

    return solution, solution_fitness

# DEAP Implementation
def DEAP(cam_num,cam_config):

    creator.create("FitnessMax", base.Fitness, weights=(1.0,))
    creator.create("Individual", list, fitness=creator.FitnessMax)
    toolbox = base.Toolbox()
    # Structure initializers
    # gene
    toolbox.register("attr_bool", random.randint, 0, 1)
    # gene space
    toolbox.register("individual", tools.initRepeat, creator.Individual,
                     toolbox.attr_bool, 100)
    #
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    def fitness_func(solution, solution_idx):
        print('solution==[cam_type,index,x,y]',solution)
        total_score = len(np.asarray(model_pcd.points))
        print('expected_score is',total_score)
        nested_solution = [ solution[ i:i + 4 ] for i in range(0, len(solution), 4) ]
        in_view_list = [ ]

        camera_params = []
        # Introduce multiprocessing
        for i in range(cam_num):
            id = nested_solution[ i ][ 0 ]
            index = nested_solution[ i ][ 1 ]
            rot_x = nested_solution[ i ][ 2 ]
            rot_y = nested_solution[ i ][ 3 ]
            rot_list = np.asarray([ rot_x, rot_y, 0 ])

            if id == 0:
                in_view, frustum = inPTZcam_frustum(model_pcd, model_kdtree, location_list, index, rot_list, cam_config, 1)
                in_view_list.append(in_view)

        fitness = 0
        # 04/11 note: add color_info into calculation
        for pcd in in_view_list:
            fitness = fitness + len(np.asarray(pcd.points))/ total_score
            print('solution & fitness=',solution,'&',len(np.asarray(pcd.points)),'&',fitness)
        #duration = time.perf_counter() - start_time
        print('time used so far:',duration)
        print('*' * 150)

        return fitness
    toolbox.register("evaluate", fitness_func)
    toolbox.register("mate", tools.cxTwoPoint)
    toolbox.register("mutate", tools.mutFlipBit, indpb=0.05)
    toolbox.register("select", tools.selTournament, tournsize=3)

    def run():
        pop = toolbox.population(n=300)

        # Evaluate the entire population
        fitnesses = list(map(toolbox.evaluate, pop))
        for ind, fit in zip(pop, fitnesses):
            ind.fitness.values = fit

        # CXPB  is the probability with which two individuals
        #       are crossed
        # MUTPB is the probability for mutating an individual
        CXPB, MUTPB = 0.5, 0.2

        # Extracting all the fitnesses of
        fits = [ ind.fitness.values[ 0 ] for ind in pop ]
        # Variable keeping track of the number of generations
        g = 0

        # Begin the evolution
        while max(fits) < 100 and g < 1000:
            # A new generation
            g = g + 1
            print("-- Generation %i --" % g)
            # Select the next generation individuals
            offspring = toolbox.select(pop, len(pop))
            # Clone the selected individuals
            offspring = list(map(toolbox.clone, offspring))

        # Apply crossover and mutation on the offspring
        for child1, child2 in zip(offspring[ ::2 ], offspring[ 1::2 ]):
            if random.random() < CXPB:
                toolbox.mate(child1, child2)
                del child1.fitness.values
                del child2.fitness.values

        for mutant in offspring:
            if random.random() < MUTPB:
                toolbox.mutate(mutant)
                del mutant.fitness.values

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ ind for ind in offspring if not ind.fitness.valid ]
        fitnesses = map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

        pop[ : ] = offspring

        # Gather all the fitnesses in one list and print the stats
        fits = [ ind.fitness.values[ 0 ] for ind in pop ]

        length = len(pop)
        mean = sum(fits) / length
        sum2 = sum(x * x for x in fits)
        std = abs(sum2 / length - mean ** 2) ** 0.5

        print("  Min %s" % min(fits))
        print("  Max %s" % max(fits))
        print("  Avg %s" % mean)
        print("  Std %s" % std)

# Preparing
if __name__ == "__main__":

    total_start_time = time.perf_counter()
    # File IO
    # open mesh model for raycasting
    # save_path = "/home/adrain/Desktop/bpydev/BlenderShellDev/Alpha/plys/"
    save_path = os.path.dirname(os.path.realpath(sys.argv[ 0 ]))
    #save_path = "D:/Program Files (x86)/Blender/2.90/scripts/BlenderShellDev/Alpha/plys/"
    os.makedirs(save_path, exist_ok=True)
    phase_list = []
    kdtree_list = []
    scene_list = []
    model_mesh_list = []
    for p in range(1,4,1):
        file_name = os.path.join(save_path, f'plys/pre_filter/phase_{p}.ply')
        model_mesh = o3d.io.read_triangle_mesh(file_name)
        o3d.visualization.draw_geometries([model_mesh])
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
        '''
        ##save points for iteration
        model_pts = model_pcd.points
        model_pts = np.asarray(model_pts)
        '''
    print('phase_count=',len(model_mesh_list),len(phase_list))
    # open camera_postions
    # save_path = "/home/adrain/Desktop/bpydev/BlenderShellDev/Alpha/plys/"
    camera = os.path.join(save_path, "plys/downsample_5.ply")
    location_pcd, location_list, location_index = read_cam_locations(camera, 0)

    desired_output = 100  # Function output.
    cam_config = [ [ 50., 30, [ 1920, 1080 ] ], [ 100. ] ]
    #for i in range(4,6):
    solution, solution_fitness = GA_optimiztion(cam_num=4, cam_config=cam_config, location_list=location_list,
                                                location_index=location_index, phase_list=phase_list,
                                                kdtree_list=kdtree_list,scene_list=scene_list,
                                                model_mesh_list= model_mesh_list)
    duration = time.perf_counter() - total_start_time
    print(f'time_for{1}cams_op:',duration)
    '''
    for i in range(4,6):
        solution, solution_fitness = GA_optimiztion(cam_num=1, cam_config=cam_config)
        duration = time.perf_counter() - total_start_time
        # print(f'time_for{i}cams_op:',duration)
    '''
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