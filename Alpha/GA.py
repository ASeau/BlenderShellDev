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

def isVisible(cam_matrix, half_plane_normals, voxel_co, fudge=0):
    #voxel_co = normalize(voxel_co)
    #making coordinates homogeneous
    hvoxel_co = np.append(voxel_co,[1.])
    #print(hvoxel_co)
    cam_inv_matrix = np.linalg.inv(cam_matrix)
    transformed_co = cam_inv_matrix @ hvoxel_co
    transformed_co = transformed_co[:-1]
    #print(transformed_co)
    #check for in_view coordinates
    for norm in half_plane_normals:
        z2 = np.dot(transformed_co, norm)
        if z2 < -fudge:
            return False
        else:
            return True

def setup_cam(cam_co,x_angle,y_angle,z_angle,cam_config):
    def CameraCone(cam_config):
        fov = 118 #cam_config[0][1]
        resolution = cam_config[0][2]
        resolution_x, resolution_y = resolution
        fx = resolution_x / np.tan(fov / 2)
        fy = fx * 9 / 16
        intrinsic = o3d.camera.PinholeCameraIntrinsic()
        intrinsic.set_intrinsics(1920,1080,fx,fx,1920/2,1080/2)

        return intrinsic

    ##Convert to quaternion based method
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
        frustum_mesh.rotate(rot_mat, center=co)
        cam_matrix = np.c_[rot_mat, translate_array]
        cam_matrix = np.r_[cam_matrix, [add_row]]
        return frustum_mesh, cam_matrix, rot_mat

    def half_plane_cal(frustum_mesh):
        points = np.asarray(frustum_mesh.vertices)
        print(points[1])
        x, y, z = points[1]
        lr = np.array([z-2, -y, x])
        ur = np.array([z-2, y, x])
        ll = np.array([z-2, -y, x])
        ul = np.array([z-2, y, x])

        half_plane_normals = [
            normalize(np.cross(lr, ll)),
            normalize(np.cross(ll, ul)),
            normalize(np.cross(ul, ur)),
            normalize(np.cross(ur, lr))
        ]
        print(half_plane_normals)
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
    cam_dir = np.array([90,90,0])#dir_list[dir_index] #CV coordinate: x right, y down, z forward
    cam_co = np.array([0, 0, 2])

    half_plane_normals, frustum, cam_matrix, rot_mat, normal = setup_cam(cam_co,
                                                                 cam_dir[0],
                                                                 cam_dir[1],
                                                                 cam_dir[2],
                                                                 cam_config)

    def frustum_cropping(frustum_mesh,model_pcd):
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
        #bounding_polygon_y[:, 1] = 0

        # Convert the np.array to a Vector3dVector
        vol_y.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon_y)

        # Crop the point cloud using the Vector3dVector
        cropped_y_pcd = vol_y.crop_point_cloud(model_pcd)
        #bounding_box = cropped_pcd.get_axis_aligned_bounding_box()
        #bounding_box.color = (1, 0, 0)
        '''
        # Crop by X axis
        bounding_polygon_x = points.astype("float64")
        # Create a SelectionPolygonVolume
        vol_x = o3d.visualization.SelectionPolygonVolume()

        # You need to specify what axis to orient the polygon to.
        # I choose the "Y" axis. I made the max value the maximum Y of
        # the polygon vertices and the min value the minimum Y of the
        # polygon vertices.
        vol_x.orthogonal_axis = "X"
        vol_x.axis_max = np.max(bounding_polygon_x[:, 1])
        vol_x.axis_min = np.min(bounding_polygon_x[:, 1])

        # Set all the Y values to 0 (they aren't needed since we specified what they
        # should be using just vol.axis_max and vol.axis_min).
        bounding_polygon_x[:, 1] = 0

        # Convert the np.array to a Vector3dVector
        vol_x.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon_x)

        # Crop the point cloud using the Vector3dVector
        cropped_pcd = vol_x.crop_point_cloud(cropped_y_pcd)
        # bounding_box = cropped_pcd.get_axis_aligned_bounding_box()
        # bounding_box.color = (1, 0, 0)
        '''
        return cropped_y_pcd

    '''
    print('cam_matrix=',cam_matrix)
    coor_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=cam_co)
    coor_mesh.rotate(rot_mat,center=cam_co)
    lineset = o3d.geometry.LineSet()
    points = []
    for half in half_plane_normals:
        points.append(cam_co)
        points.append(half)

    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(np.asarray(points))
    pcd1.paint_uniform_color([0,1,0])

    lineset.points = o3d.utility.Vector3dVector(points)
    lineset.lines = o3d.utility.Vector2iVector([[0,1],[0,2],[0,3],[0,4],[1,2],[2,3],[3,4],[4,1]])
    colors = [[1, 0, 0] for i in range(len(np.asarray(lineset.lines)))]
    lineset.colors = o3d.utility.Vector3dVector(colors)
    '''
    in_view = []
    queries = []
    index_list = []
    ##get in radius points
    [k, idx, _] = model_kdtree.search_radius_vector_3d(cam_co, cam_far)
    model_pcd = model_pcd.select_by_index(idx)
    cropped_pcd= frustum_cropping(frustum, model_pcd)

    ##vector from cam to point and normalize
    for index, pt in enumerate(cropped_pcd.points):
        #if isVisible(cam_matrix, half_plane_normals, pt, fudge=0):
        query = [pt, cam_co - pt]
        query = np.concatenate(query, dtype=np.float32).ravel().tolist()
        queries.append(query)
        index_list.append(index)

    rays = o3d.core.Tensor(queries, dtype=o3d.core.Dtype.Float32)
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

    cropped_pcd = cropped_pcd.select_by_index(in_view)
    #frustum = o3d.geometry.LineSet.create_camera_visualization(intrinsic,cam_matrix,1)

    o3d.visualization.draw_geometries([model_mesh, cropped_pcd, frustum, normal])
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

if __name__ == "__main__":
    # open mesh model for raycasting
    save_path = "D:/Program Files (x86)/Blender/2.90/scripts/BlenderShellDev/Alpha/plys/"
    os.makedirs(save_path, exist_ok=True)
    file_name = os.path.join(save_path, "renwen_raycasting_test.ply")
    model_mesh = o3d.io.read_triangle_mesh(file_name)
    #o3d.visualization.draw_geometries([model_mesh])
    # open filtered voxels
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
    '''
    save_path = "D:/Program Files (x86)/Blender/2.90/scripts/BlenderShellDev/Alpha/plys/"
    os.makedirs(save_path, exist_ok=True)
    file_name = os.path.join(save_path, "renwen_raycasting_test.ply")
    model = o3d.io.read_triangle_mesh(file_name)
    '''
    # initializing raycast scene
    scene = o3d.t.geometry.RaycastingScene()
    cube_id = scene.add_triangles(np.asarray(model_mesh.vertices, dtype=np.float32),
                                  np.asarray(model_mesh.triangles, dtype=np.uint32))
    ##cam_config = [PTZ,360,LiDar]
    ##cam_config = [[cam_far,cam_FOV][cam_far][cam_far, cam_FOV]]
    cam_config = [[100.,30,[1920,1080]], [100.], [100.,20]]

    def dir_index():
        x_size = []
        y_size = []
        z_size = []
        dir_size = []

        for x in range(0, 180, 10):
            x_size.append(x)

        for y in range(0):
            y_size.append(y)

        for z in range(0,360,10):
            z_size.append(z)

        nx = np.array(x_size, dtype=np.float64)
        ny = np.array(y_size, dtype=np.float64)
        nz = np.array(z_size, dtype=np.float64)
        dir_list = np.vstack(np.meshgrid(nx, ny, nz)).reshape(3, -1).T
        for i in range(len(dir_list)):
            dir_size.append(i)

        return dir_list, dir_size
    def location_index():
        x_size = []
        y_size = []
        z_size = []
        co_size = []

        for x in range(0, 600, 10):
            # if x >= 2500 or x <= :
            x_size.append(x)

        for y in range(0, 400, 10):
            # if y >= 500 or y >= 1500:
            y_size.append(y)

        for z in range(2):
            # if y >= 500 or y >= 1500:
            z_size.append(z)

        nx = np.array(x_size,dtype=np.float64)
        ny = np.array(y_size,dtype=np.float64)
        nz = np.array(z_size,dtype=np.float64)
        location_list = np.vstack(np.meshgrid(nx, ny, nz)).reshape(3, -1).T
        for i in range(len(location_list)):
            co_size.append(i)
        return location_list, co_size
    location_list, co_size = location_index()
    location_index = 0
    start_time = time.perf_counter()
    #in_view, idx = in360cam_frustum(model_pcd, model_kdtree,location_index, cam_config, 1)
    inPTZcam_frustum(model_pcd, model_kdtree ,location_index, dir_index, cam_config, 1)
    #in360cam_frustum(model_pcd,model_kdtree,location_index,cam_config,1)
    #inlidar_frustum(model_pcd,model_kdtree,location_index,cam_config,1)
    duration = time.perf_counter() - start_time
    #print(idx,duration,len(in_view),len(in_view)/idx)
    '''
    rays = scene.create_rays_pinhole(fov_deg=118,
                                     center=[np.sin(np.radians(90)), np.sin(np.radians(0)), np.sin(np.radians(90))],
                                     eye=[0, 0, 2],
                                     up=[0, 0, 1],
                                     width_px=int(640 / 4),
                                     height_px=int(480 / 4))
    ans = scene.cast_rays(rays)
    print(ans)

    hit = ans['t_hit'].isfinite()
    points = rays[hit][:,:3] + rays[hit][:,3:]*ans['t_hit'][hit].reshape((-1,1))
    pcd = o3d.t.geometry.PointCloud(points)
    o3d.visualization.draw_geometries([pcd.to_legacy()],
                                      front=[0.5, 0.86, 0.125],
                                      lookat=[0.23, 0.5, 2],
                                      up=[-0.63, 0.45, -0.63],
                                      zoom=1.0)
'''
'''
# prepare fitness
x_size = []
y_size = []
z_size = []

for x in range(0, bound_x, 1):
    # if x >= 2500 or x <= :
    x_size.append(x)

for y in range(0, bound_y, 1):
    # if y >= 500 or y >= 1500:
    y_size.append(y)

for y in range(0, bound_z, 1):
    #if y >= 500 or y >= 1500:
    z_size.append(y)

nx = np.array(x_size)
ny = np.array(y_size)
nz = np.array(z_size)

#print(nx,ny,nz)
cam_loc = np.vstack(np.meshgrid(nx, ny, nz)).reshape(3, -1).T
print(cam_loc, type(cam_loc))
print(f'grid of {cam_loc.size} generated')

rot_x = 0
rot_x_size = {'low': 90, 'high': 180}
rot_y = 0
rot_y_size = {'low': 0, 'high': 0}
rot_z = 0
rot_z_size = {'low': 90, 'high': 270}

cam_num = 1
function_inputs = []

function_inputs_ele = [x_pos, y_pos, z_pos, rot_x, rot_y, rot_z]

for cam in range(cam_num):
    function_inputs.extend(function_inputs_ele)  # Function inputs
# print(function_inputs)

desired_output = 100  # Function output.
expected_num = 999

def GA_optimiztion(co_list,):
    # sol = (installable_list[index], pan, tilt)
    def fitness_func(solution, solution_idx):
        # pass to blender
        context = bpy.context
        scene = context.scene
        scene.frame_set(235)

        cam_list = []
        nested_solution = [solution[i:i + 3] for i in range(0, len(solution), 3)]
        #defining list for whole generation
        gen_fitness = []

        for i in range(len(cam_list)):
            cam = cam_list[i]
            bpy.context.scene.camera = cam
            #print('Active camera set to ' + cam.name)
            cam_loc = cam.location
            cam_rot = cam.rotation_euler
            cam_roty = cam_rot[2]
            pi = math.pi
            rad = (pi / 180.0)
            #print(solution,type(solution))

            cam_x = nested_solution[i][0]
            cam_y = nested_solution[i][1]
            cam_z = nested_solution[i][2]
            cam_rot_x = nested_solution[i][3]
            cam_rot_y = nested_solution[i][4]
            cam_rot_z = nested_solution[i][5]

            config = list([cam_rot_x, cam_rot_y, cam_rot_z, cam_x, cam_y, cam_z])
            #
            setupCamera(scene=scene, c=config)
            # voxel_cal
            context = bpy.context
            scene = context.scene
            #
            half_normals = CameraCone(cam, scene)
            girds = grids
            #
            inview_co = in_cam(girds, cam, half_normals)
            fitness = len(inview_co)
            print("cam,fitness",cam.name,fitness)
            #
            gen_fitness.append(fitness)
        #using cam average
        fitness = sum(gen_fitness)/len(gen_fitness)
        return fitness

    num_generations = 100
    num_parents_mating = 500

    fitness_function = fitness_func

    sol_per_pop = 1000
    num_genes = len(function_inputs)

    parent_selection_type = "rank"
    keep_parents = 250

    crossover_type = "single_point"
    crossover_probability = 0.9

    mutation_type = "random"
    mutation_probability = 0.1

    stop_criteria = "reach_1.0", "saturate_20"
    gene_space = [x_size,y_size,z_size,rot_x_size,rot_y_size,rot_z_size] #x_size,y_size,z_size,rot_x_size,rot_y_size,rot_z_size,x_size,y_size,z_size,rot_x_size,rot_y_size,rot_z_size]
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

    #initiate pop
    ga_instance.initialize_population(low=0,high=360,mutation_by_replacement=True, allow_duplicate_genes = True, gene_type=int)
    bpy_pop = ga_instance.population
    print("init_pop=",bpy_pop)

    #perform mutation and crossover ops
    # Running the GA to optimize the parameters of the function.
    ga_instance.run()
    print("Number of generations passed is {generations_completed}".format(generations_completed=ga_instance.generations_completed))
    ga_instance.plot_fitness()

    # Returning the details of the best solution.
    solution, solution_fitness, solution_idx = ga_instance.best_solution(ga_instance.last_generation_fitness)

best_sol = solution

cam_list = []

for obj in bpy.data.objects:
    if obj.type != "CAMERA":
        continue
    # print('Camera with name "' + obj.name + '" found')
    cam_list.append(obj)
    print(cam_list)

context = bpy.context
scene = context.scene

for i in range(len(cam_list)):
    cam = cam_list[i]
    bpy.context.scene.camera = cam
    print('Active camera set to ' + cam.name)

    cam_loc = cam.location
    cam_rot = cam.rotation_euler
    cam_roty = cam_rot[2]
    pi = math.pi
    rad = (pi / 180.0)
    #print(solution,type(solution))

    cam_x = best_sol[0]
    cam_y = best_sol[1]
    cam_z = best_sol[2]
    cam_rot_x = best_sol[3]
    cam_rot_y = best_sol[4]
    cam_rot_z = best_sol[5]

    config = list([cam_rot_x, cam_rot_y, cam_rot_z, cam_x, cam_y, cam_z])
    print(config)
    #
    setupCamera(scene=scene, c=config)

    #
    half_normals = CameraCone(cam, scene)
    girds = grids
    #
    # inPTZview_co = inPTZcam_frustum(girds, cam, half_normals)
    # in360view_co = in360cam_frustum(girds,cam,15)
    inlidar_frustum(girds, cam, 30, 5)




if __name__ == "__main__":
    print("Parameters of the best solution : {solution}".format(solution=solution))
    print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))
    print("Index of the best solution : {solution_idx}".format(solution_idx=solution_idx))

    print("--- %s seconds ---" % (time.time() - start_time))

'''