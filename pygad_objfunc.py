import sys
import os
from math import *
from mathutils import *
import cv2 as cv
import numpy as np
import random
import json
import matplotlib.colors as colors
np.set_printoptions(threshold=sys.maxsize)

expected_num = 100


#Reads rendered output and produces the objective function
json_path = os.path.abspath("D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/GAtest/color_dict.json")
with open(json_path) as json_file:
    data = json.load(json_file)

path = os.path.abspath("D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/GAtest/test.png")
img = cv.imread(path)
img_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)

rgb_count = []
all_rgb_codes = img_rgb.reshape(-1, img_rgb.shape[-1])
unique_rgb, counts = np.unique(all_rgb_codes, return_counts=True, axis=0)

# detected_dict = dict(zip(unique_rgb.tolist(),list(counts)))
# print(detected_dict)
unique_rgb = unique_rgb.tolist()
print(unique_rgb)
print(list(counts))

matched= []
for u in unique_rgb:
    for key, values in data.items():
        if tuple(values) == tuple(u):
            matched.append(key)
            print("matched", key, u, values)

matched_num = len(matched)
print(matched_num)

#returns objective function (coverage  = 1*100)
coverage = 100*(matched_num/expected_num)

#write into GA file for further optimization
