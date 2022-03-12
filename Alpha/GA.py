import sys
import os
import pygad
import numpy as np
import time
import math
import open3d as o3d

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

def CameraCone(cam_config):
    fov = cam_config[0][1]
    resolution = cam_config[0][2]
    resolution_x, resolution_y = resolution
    fx = resolution_x / np.tan(fov / 2)
    fy = fx*9/16
    s=0
    cam_intrinsic = np.array([[fx,s,resolution_x/2],
                              [0,fy,resolution_x/2],
                              [0,0,1]])
    w = fx/resolution_x
    if resolution_x > resolution_y:
        x = w
        y = w * resolution_y / resolution_x
    else:
        x = w * resolution_x / resolution_y
        y = w

    lr = np.array([x, -y, -1])
    ur = np.array([x, y, -1])
    ll = np.array([-x, -y, -1])
    ul = np.array([-x, y, -1])

    half_plane_normals = [
        normalize(np.cross(lr, ll)),
        normalize(np.cross(ll, ul)),
        normalize(np.cross(ul, ur)),
        normalize(np.cross(ur, lr))
    ]

    return half_plane_normals

def isVisible(dir, half_plane_normals, fudge=0):
    dir = normalize(dir)
    for norm in half_plane_normals:
        z2 = dir.dot(norm)
        if z2 < -fudge:
            return False
    return True

def inPTZcam_frustum(model_pcd, model_kdtree ,location_index, cam_config, ray_cast, half_plane_normals):
    cam_far = cam_config[0][0]
    in_view = []
    queries = []
    ##get in radius points
    loc = location_list[location_index]
    [k, idx, _] = model_kdtree.search_radius_vector_3d(loc, cam_far)
    model_pcd = model_pcd.select_by_index(idx)

    ##vector from cam to point and normalize
    for pt in model_pcd.points:
        dir = pt - loc
        if isVisible(dir, half_plane_normals, fudge=0):
            query = [pt, loc - pt]
            query = np.concatenate(query, dtype=np.float32).ravel().tolist()
            queries.append(query)
            print(queries)

    rays = o3d.core.Tensor(queries, dtype=o3d.core.Dtype.Float32)

    if ray_cast == 1:
        hits = scene.test_occlusions(rays)

        for index, item in enumerate(hits):
            if item:
                pass
            else:
                in_view.append(index)

    model_pcd = model_pcd.select_by_index(in_view)
    o3d.visualization.draw_geometries([model_pcd])
    return model_pcd

##computes 360 camera voxels, set ray_cast=0 to turn off, 1 for on##
def in360cam_frustum(model_pcd, model_kdtree ,location_index, cam_config, ray_cast):
    cam_far = cam_config[1][0]
    in_view = []
    queries = []
    ##get in radius points
    loc = location_list[location_index]
    [k, idx, _] = model_kdtree.search_radius_vector_3d(loc, cam_far)
    model_pcd = model_pcd.select_by_index(idx)

    ##vector from cam to point and normalize
    for pt in model_pcd.points:
        query = [pt, loc - pt]
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

    return model_pcd

def inlidar_frustum(model_pcd, model_kdtree ,location_index, cam_config, ray_cast):
    cam_far = cam_config[2][0]
    lidar_fov = cam_config[2][1]
    fov_vec = np.sin(np.radians(lidar_fov))

    in_view = []
    queries = []
    ##get in radius points
    loc = location_list[location_index]
    [k, idx, _] = model_kdtree.search_radius_vector_3d(loc, cam_far)
    model_pcd = model_pcd.select_by_index(idx)

    ##vector from cam to point and normalize
    for pt in model_pcd.points:
        dir_vec = pt - loc
        norm = normalize(dir_vec)

        if norm[2] <= fov_vec and norm[2] >= -fov_vec:
            query = [pt, -dir_vec]
            query = np.concatenate(query, dtype=np.float32).ravel().tolist()
            queries.append(query)

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
                in_view.append(index)

    model_pcd = model_pcd.select_by_index(in_view)

    return model_pcd

if __name__ == "__main__":
    # open mesh model for raycasting
    save_path = "D:/Program Files (x86)/Blender/2.90/scripts/BlenderShellDev/Alpha/plys/"
    os.makedirs(save_path, exist_ok=True)
    file_name = os.path.join(save_path, "renwen_raycasting_test.ply")
    model_mesh = o3d.io.read_triangle_mesh(file_name)

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
    cam_config = [[100,118,[1920,1080]], [100.], [100.]]

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

    print(location_list,len(location_list))
    location_index = 0
    start_time = time.perf_counter()
    #in_view, idx = in360cam_frustum(model_pcd, model_kdtree,location_index, cam_config, 1)
    half_plane_normals = CameraCone(cam_config)
    inPTZcam_frustum(model_pcd, model_kdtree ,location_index, cam_config, 1, half_plane_normals)
    duration = time.perf_counter() - start_time
    #print(idx,duration,len(in_view),len(in_view)/idx)


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

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def inPTZcam_frustum(voxel_co, cam, half):
    in_view = []
    matrix = cam.matrix_world
    matrix = matrix.inverted()
    # far = cam.data.clip_end
    for co in voxel_co:
        co = Vector(co)
        result = isVisible(cam_mat=matrix, half_plane_normals=half, loc=co, fudge=0)
        if result == True:
            in_view.append(co)
    return in_view






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