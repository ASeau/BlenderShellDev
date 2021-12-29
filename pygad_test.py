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

##REF from https://pygad.readthedocs.io/en/latest/README_pygad_ReadTheDocs.html#examples
#prepare fitness
x_pos = 0
x_size = {'low': -20, 'high': 0}
y_pos = 0
y_size = {'low': 0, 'high': 20}
z_pos = 0
z_size = {'low': 5, 'high': 5}
pan = 0
pan_size = {'low': 0, 'high': 360}
tilt = 0
tilt_size = {'low': 0, 'high': 90}

function_inputs = [x_pos,y_pos,z_pos,pan,tilt] # Function inputs.
desired_output = 100 # Function output.
expected_num = 100

def on_start(ga_instance):
    print("on_start()")

def on_fitness(ga_instance, population_fitness):
    print("on_fitness()")

def on_parents(ga_instance, selected_parents):
    print("on_parents()")

def on_crossover(ga_instance, offspring_crossover):
    print("on_crossover()")

def on_mutation(ga_instance, offspring_mutation):
    print("on_mutation()")

def on_generation(ga_instance):
    print("on_generation()")

def on_stop(ga_instance, last_population_fitness):
    print("on_stop()")

counter = 0
def fitness_func(solution, solution_idx):
    # pass to blender
    context = bpy.context
    scene = context.scene
    viewlayer = context.view_layer

    def setupCamera(scene, c):
        pi = math.pi

        scene.camera.rotation_euler[0] = c[0] * (pi / 180.0)
        scene.camera.rotation_euler[1] = c[1] * (pi / 180.0)
        scene.camera.rotation_euler[2] = c[2] * (pi / 180.0)

        scene.camera.location.x = c[3]
        scene.camera.location.y = c[4]
        scene.camera.location.z = c[5]

        return

    cam = scene.camera
    cam_loc = cam.location
    cam_rot = cam.rotation_euler
    cam_roty = cam_rot[2]
    pi = math.pi
    rad = (pi / 180.0)
    # test_list = (1,1,1,1,1)
    #print(solution,type(solution))

    cam_x = solution[0]
    cam_y = solution[1]
    cam_z = solution[2]
    cam_pan = solution[3]
    cam_tilt = solution[4]

    config = list([cam_tilt, 0, cam_pan, cam_x, cam_y, cam_z])
    setupCamera(scene=scene, c=config)
    print(config, cam.rotation_euler,solution_idx)

    save_path = "D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/Renders/GA_test"
    os.makedirs(save_path, exist_ok=True)
    bpy.context.scene.render.image_settings.file_format = 'PNG'
    bpy.context.scene.render.filepath = bpy.context.scene.render.filepath = os.path.join(save_path,("GA%0.2d.png"%solution_idx))
    bpy.ops.render.render(use_viewport=False, write_still=True)
    '''
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
    '''
    path = os.path.join(save_path,("GA%0.2d.png"%solution_idx))
    img = cv.imread(path)
    img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    coverage_list = []

    rgb_count = []
    all_rgb_codes = img.reshape(-1, img.shape[-1])
    unique_rgb, counts = np.unique(all_rgb_codes, return_counts=True, axis=0)
    unique_rgb = unique_rgb.tolist()
    # print(list(counts),sol_idx)
    matched_num = len(unique_rgb)
    coverage = int(100 * (matched_num / expected_num))
    print(coverage,solution_idx)
    coverage_list.append(coverage)

    output = np.sum(coverage_list)
    print("out",output,desired_output)
    fitness = np.abs(output/desired_output)/1.0
    print("fit",fitness)
    return fitness

#output = numpy.sum(solution*function_inputs)

#GA parameters
num_generations = 10
num_parents_mating = 2

fitness_function = fitness_func

sol_per_pop = 100
num_genes = len(function_inputs)

parent_selection_type = "rank"
keep_parents = 2

crossover_type = "single_point"
crossover_probability = 0.9

mutation_type = "random"
mutation_probability = 0.05

stop_criteria = "reach_1.0", "saturate_20"
gene_space = [x_size,y_size,z_size,pan_size,tilt_size]
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
'''
ga_instance.initialize_population(low=0,high=360,mutation_by_replacement=True, allow_duplicate_genes = True, gene_type=int)
bpy_pop = ga_instance.population
print(bpy_pop)

fitness = fitness_func(img_rgb=img_rgb,solution_idx=0)
print(fitness)
'''
#perform mutation and crossover ops
# Running the GA to optimize the parameters of the function.
ga_instance.run()
print("Number of generations passed is {generations_completed}".format(generations_completed=ga_instance.generations_completed))
ga_instance.plot_fitness()

# Returning the details of the best solution.
solution, solution_fitness, solution_idx = ga_instance.best_solution(ga_instance.last_generation_fitness)
print("Parameters of the best solution : {solution}".format(solution=solution))
print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))
print("Index of the best solution : {solution_idx}".format(solution_idx=solution_idx))

print("--- %s seconds ---" % (time.time() - start_time))

'''
".pic%0.2d.jpg" % i
#evalute fitness

#perform mutation

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
'''
'''