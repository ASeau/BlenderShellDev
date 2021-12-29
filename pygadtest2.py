import pygad
#import bpy
import math
import cv2 as cv
import numpy as np
import timeit
import glob
import time
from decimal import Decimal
start_time = time.time()

##REF from https://pygad.readthedocs.io/en/latest/README_pygad_ReadTheDocs.html#examples
#prepare fitness
a = np.float32(0.00)
a_size = {'low': np.float32(-148259.00), 'high': np.float32(9999.00)}
b = np.float32(0.00)
b_size = {'low': np.float32(0.00), 'high': np.float32(5121.70)}
c = np.float32(0.00)
c_size = {'low': np.float32(-2.98), 'high':np.float32(48.07)}
d = np.float32(0.00)
d_size = {'low': np.float32(-162.21), 'high': np.float32(85.27)}

function_inputs = [a,b,c,d], # Function inputs.
desired_output = np.float32(1)  # Function output.

counter = 0
def fitness_func(solution, solution_idx):
    print(solution)
    output = np.sum(solution * function_inputs)
    print("out",output,solution_idx)
    fitness = 1.0 / (np.abs(output - desired_output) + 0.000001)
    print('fit',fitness,solution_idx)
    return fitness

#output = numpy.sum(solution*function_inputs)

#GA parameters
num_generations = 6
num_parents_mating = 2

fitness_function = fitness_func

sol_per_pop = 10
num_genes = 4

parent_selection_type = "rank"
keep_parents = 2

crossover_type = "single_point"
crossover_probability = 0.9

mutation_type = "random"
mutation_probability = 0.05

stop_criteria = "saturate_20"
gene_space = [a_size,b_size,c_size,d_size]

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
                       gene_type = np.float16,
                       stop_criteria=stop_criteria)
#initiate pop
#perform mutation and crossover ops
# Running the GA to optimize the parameters of the function.
ga_instance.run()
print("Number of generations passed is {generations_completed}".format(generations_completed=ga_instance.generations_completed))
ga_instance.plot_fitness()
'''
# Returning the details of the best solution.
solution, solution_fitness, solution_idx = ga_instance.best_solution(ga_instance.last_generation_fitness)
print("Parameters of the best solution : {solution}".format(solution=solution))
print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))
print("Index of the best solution : {solution_idx}".format(solution_idx=solution_idx))

print("--- %s seconds ---" % (time.time() - start_time))
'''