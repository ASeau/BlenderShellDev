import sys
import os
'''
path = os.path.abspath("C:/Users/User/anaconda3/envs/bpydev/Lib/")
path2 = os.path.abspath("C:/Users/User/anaconda3/envs/bpydev/DLLs/")
path3 = os.path.abspath("C:/Users/User/anaconda3/envs/bpydev/Lib/site-packages")
if path not in sys.path:
    sys.path.append(path)
    print('appended')
if path2 not in sys.path:
    sys.path.append(path2)
    print('appended')
if path3 not in sys.path:
    sys.path.append(path3)
    print('appended')
'''
import pygad
import bpy
import math
import cv2 as cv
import numpy as np
import timeit
import glob
import time
start_time = time.time()
from mathutils import Vector
from mathutils.bvhtree import BVHTree

# open the file in the write mode
save_path = "D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/"
os.makedirs(save_path, exist_ok=True)
file_name = os.path.join(save_path, "grids.npy")
grids = np.load(file_name)
print(grids)

##REF from https://pygad.readthedocs.io/en/latest/README_pygad_ReadTheDocs.html#examples
#prepare fitness
x_pos = 0
x_size = {'low': -42, 'high': 18}
y_pos = 0
y_size = {'low': 27, 'high': 27}
z_pos = 0
z_size = {'low': 2, 'high': 2}

rot_x = 0
rot_x_size = {'low': 90, 'high': 180}
rot_y = 0
rot_y_size = {'low': 0, 'high': 0}
rot_z = 0
rot_z_size = {'low': 90, 'high': 270}

cam_num = 1
function_inputs = []

function_inputs_ele = [x_pos,y_pos,z_pos,rot_x,rot_y,rot_z]

for cam in range(cam_num):
    function_inputs.extend(function_inputs_ele)# Function inputs
#print(function_inputs)

desired_output = 100 # Function output.
expected_num = 999

'''
def setupCamera(scene, c):
    pi = math.pi
    scene.camera.rotation_euler = [0, 0, 0]
    scene.camera.rotation_euler[0] = c[0] * (pi / 180.0)
    scene.camera.rotation_euler[1] = c[1] * (pi / 180.0)
    scene.camera.rotation_euler[2] = c[2] * (pi / 180.0)

    scene.camera.location.x = c[3]
    scene.camera.location.y = c[4]
    scene.camera.location.z = c[5]

    return
'''


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

def CameraCone(cam, scn):
    matrix = cam.matrix_world
    matrix = matrix.inverted()
    sensor_width = cam.data.sensor_width
    lens = cam.data.lens
    resolution_x = scn.render.resolution_x
    resolution_y = scn.render.resolution_y

    w = 0.5 * sensor_width / lens
    if resolution_x > resolution_y:
        x = w
        y = w * resolution_y / resolution_x
    else:
        x = w * resolution_x / resolution_y
        y = w

    lr = Vector([x, -y, -1])
    ur = Vector([x, y, -1])
    ll = Vector([-x, -y, -1])
    ul = Vector([-x, y, -1])
    half_plane_normals = [
        lr.cross(ll).normalized(),
        ll.cross(ul).normalized(),
        ul.cross(ur).normalized(),
        ur.cross(lr).normalized()
    ]
    return half_plane_normals


def isVisible(cam_mat, half_plane_normals, loc, fudge=0):
    loc2 = cam_mat @ loc
    for norm in half_plane_normals:
        z2 = loc2.dot(norm)
        if z2 < -fudge:
            return False
    return True

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

def in360cam_frustum(voxel_co, cam, cam_far):
    in_view = []
    for co in voxel_co:
        co = Vector(co)
        loc = cam.location
        dst = np.sqrt((co[0] - loc.x) ** 2 + (co[1] - loc.y) ** 2 + (co[1] - loc.z) ** 2)
        if dst <= cam_far:
            in_view.append(co)
            bpy.ops.mesh.primitive_cube_add(size=5, location=co)
            bpy.context.object.display_type = 'BOUNDS'
            # my_coll.objects.link(obj)
    # print(type(in_view), len(in_view))
    return in_view

def inlidar_frustum(voxel_co, cam, cam_far, lidar_fov):
    in_view = []
    fov_vec = np.sin(np.radians(lidar_fov))

    bpy.context.context.objects.active = cam
    bpy.ops.object.mode_set(mode='OBJECT')
    print(bpy.context.mode)

    #cam_BVHT = BVHTree.FromObject(cam, bpy.context.evaluated_depsgraph_get())

    for co in voxel_co:
        co = Vector(co)
        loc = cam.location
        dst = np.sqrt((co[0] - loc.x) ** 2 + (co[1] - loc.y) ** 2 + (co[1] - loc.z) ** 2)

        v3 = loc - co
        norm = normalize(v3)

        if dst <= cam_far:
            if norm[2] <= fov_vec and norm[2] >= -fov_vec:
                local_cam = cam.matrix_world.inverted() #@ cam.location
                local_voxel = cam.matrix_world.inverted() #@ co
                dir = local_voxel - local_cam
                print(local_cam, dir)
                #location, normal, index, dist = cam_BVHT.ray_cast(local_cam, dir, dst)
                #(location, normal, index) = cam.ray_cast(local_cam, local_voxel)
                #print(location, normal, index, dist)
                #print(-fov_vec,"<=",norm[2],"<=",fov_vec)

                #if (location == None):
                #    in_view.append(co)
                #    bpy.ops.mesh.primitive_cube_add(size=2, location=location)
                #    bpy.context.object.display_type = 'BOUNDS'
                #    #my_coll.objects.link(obj)

    # print(type(in_view), len(in_view))
    return in_view

'''
def fitness_func(solution, solution_idx):
    # pass to blender
    context = bpy.context
    scene = context.scene
    scene.frame_set(235)

    cam_list = []
    gen_fitness = []

    for obj in bpy.data.objects:
        if obj.type != "CAMERA":
            continue
        #print('Camera with name "' + obj.name + '" found')
        cam_list.append(obj)
        print(cam_list)

    nested_solution = [solution[i:i + 6] for i in range(0, len(solution), 6)]
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

#output = numpy.sum(solution*function_inputs)

#GA parameters
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
'''
'''
fitness = fitness_func(img_rgb=img_rgb,solution_idx=0)
print(fitness)
'''
'''
#perform mutation and crossover ops
# Running the GA to optimize the parameters of the function.
ga_instance.run()
print("Number of generations passed is {generations_completed}".format(generations_completed=ga_instance.generations_completed))
ga_instance.plot_fitness()

# Returning the details of the best solution.
solution, solution_fitness, solution_idx = ga_instance.best_solution(ga_instance.last_generation_fitness)

best_sol = solution
'''
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
    '''
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
    '''
    #
    half_normals = CameraCone(cam, scene)
    girds = grids
    #
    #inPTZview_co = inPTZcam_frustum(girds, cam, half_normals)
    #in360view_co = in360cam_frustum(girds,cam,15)
    inlidar_frustum(girds,cam,30,5)
'''
print("Parameters of the best solution : {solution}".format(solution=solution))
print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))
print("Index of the best solution : {solution_idx}".format(solution_idx=solution_idx))

print("--- %s seconds ---" % (time.time() - start_time))
'''
#evalute fitness
#perform mutation
'''
matched_num = len(inview_co)
        coverage = int(100 * (matched_num / expected_num))
        print("coverage,i,sol_idx=",coverage,i,solution_idx)
        output = coverage
        print("output,desired_output=",output,desired_output)
        fitness = np.abs(output/desired_output)/1.0
        print("cam,fitness",cam.name,fitness)
        gen_fitness.append(fitness)

save_path = "D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/Renders/GA_test"
        os.makedirs(save_path, exist_ok=True)
        bpy.context.scene.render.image_settings.file_format = 'PNG'
        filename = cam.name + "_" + str(solution_idx) + "_" + str(i) + ".png"
        bpy.context.scene.render.filepath = os.path.join(save_path,(filename))
        read_path = bpy.context.scene.render.filepath
        bpy.ops.render.render(use_viewport=True, write_still=True)
        print("saved as", cam.name, read_path)

        ##Color analysis
        path = read_path
        img = cv.imread(path)
        img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        ######
        rgb_count = []
        all_rgb_codes = img.reshape(-1, img.shape[-1])
        unique_rgb, counts = np.unique(all_rgb_codes, return_counts=True, axis=0)
        unique_rgb = unique_rgb.tolist()
#store and rank
# encode dict as JSON
data = json.dumps(bpy_pop, indent=1, ensure_ascii=True)

# set output path and file name (set your own)
save_path = "D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/"
os.makedirs(save_path, exist_ok=True)
file_name = os.path.join(save_path, "bpy_pop.json")

# write JSON file
with open(file_name, 'w') as outfile:
    outfile.write(data + '\n')
    
dir = os.path.dirname(bpy.data.filepath)
if not dir in sys.path:
    sys.path.append(dir)
    print(sys.path)

path = os.path.abspath("C:/Users/User/anaconda3/envs/BlenderShellDev/Lib/site-packages")
if path not in sys.path:
    sys.path.append(path)
    print('appended')
    
path = os.path.abspath("C:/Users/User/anaconda3/envs/BlenderShellDev/Lib/site-packages")
if path not in sys.path:
    sys.path.append(path)
    print('appended')
    

    # calculate objective function
    cv_img = []
    for img in sorted(
            glob.glob("D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/Renders/GA_test/*.png")):
        n = cv.imread(img)
        cv_img.append(n)

    img_rgb = []
    for img in cv_img:
        cvt = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        img_rgb.append(cvt)

    coverage_list = []
    
    for img in img_rgb:
        rgb_count = []
        all_rgb_codes = img.reshape(-1, img.shape[-1])
        unique_rgb, counts = np.unique(all_rgb_codes, return_counts=True, axis=0)
        unique_rgb = unique_rgb.tolist()
        # print(list(counts),sol_idx)
        matched_num = len(unique_rgb)
        print(matched_num)
        coverage = int(100 * (matched_num / expected_num))
        coverage_list.append(coverage)
'''
print("done")
