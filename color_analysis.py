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

json_path = os.path.abspath("D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/color_dict2.json")
with open(json_path) as json_file:
    data = json.load(json_file)


path = os.path.abspath("D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/Renders/test18.png")
img = cv.imread(path)
img_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)

rgb_count = []
all_rgb_codes = img_rgb.reshape(-1, img_rgb.shape[-1])
unique_rgb, counts = np.unique(all_rgb_codes, return_counts= True, axis=0)

#detected_dict = dict(zip(unique_rgb.tolist(),list(counts)))
#print(detected_dict)
unique_rgb = unique_rgb.tolist()
print(unique_rgb)
print(list(counts))

for u in unique_rgb:
    for key, values in data.items():
        if tuple(values) == tuple(u):
            print("matched",key,u,values)




#print(img_rgb[0,0],img_rgb[0,img_rgb.shape[1]-1],img_rgb[img_rgb.shape[0]-1,0],img_rgb[img_rgb.shape[0]-1,img_rgb.shape[1]-1])
#print(img_rgb.shape,unique_rgbs)
'''
def rgb_to_hex(rgb_tuple):
    return colors.rgb2hex([1.0*x/255 for x in rgb_tuple])

for e in unique_rgb:
    hex_code = rgb_to_hex(e)

clr_dict = {}
for i in range(img_rgb.shape[0]):
    for j in range(img_rgb.shape[1]):
        for rgb in unique_rgbs:
            if (img_rgb[i,j].all() == rgb.all()):
                rgb = (random.randint(0,255),random.randint(0,255),random.randint(0,255))
                if rgb not in clr_dict.values:
                    img_copy[i,j] = rgb
                    clr_dict[i,j] = rgb
                    
Z = img_rgb.reshape((-1,3))
# convert to np.float32
Z = np.float32(Z)
# define criteria, number of clusters(K) and apply kmeans()
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 10, 1.0)
K = 4
ret,label,center=cv.kmeans(Z,K,None,criteria,10,cv.KMEANS_RANDOM_CENTERS)

# Now convert back into uint8, and make original image
center = np.uint8(center)
print(center)
res = center[label.flatten()]
res2 = res.reshape((img.shape))

res2_codes = res2.reshape(-1, res2.shape[-1])
res2_rgbs = np.unique(res2_codes, axis=0)
print(res2_rgbs)

cv.imshow('res2',res2)
cv.waitKey(0)
cv.destroyAllWindows()

#img_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)


#Apply edge clearing
'''






