import bpy
import sys
import os
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree
import open3d as o3d

# reset materials
def material_reset():
    context = bpy.context
    scene = context.scene
    mat = bpy.data.materials['Cam0']
    for obj in scene.objects:
        if obj.type == 'MESH':
            # Assign it to object
            if obj.data.materials:
                # assign to 1st material slot
                obj.data.materials[0] = mat
            else:
                # no slots
                obj.data.materials.append(mat)

#material_reset()

#Calculate voxel points
def voxelcoor_cal(voxel_size):
    context = bpy.context
    scene = context.scene
    viewlayer = context.view_layer

    sel_objs = []
    for obj in scene.objects:
        if obj.type == 'MESH':
            sel_objs.append(obj)

    for obj in sel_objs:  # <<<<<<<<<<
        obj.select_set(True)  # <<<<<<<<<<

    bpy.ops.mesh.boundbox_add()
    bpy.ops.object.select_all(action='DESELECT')

    # BoundingBox_size
    bbox_list = []
    bbox_verts = []

    for o in scene.objects:
        if "Bounding" in o.name:
            bbox_list.append(o)

    for box in bbox_list:
        obj = bpy.context.scene.objects.get(str(box.name))
        obj.select_set(True)

        bpy.ops.object.mode_set(mode="EDIT")

        wmtx = obj.matrix_world.copy()

        for v in obj.data.vertices:
            co_world = wmtx @ v.co
            bbox_verts.append(co_world)

    x_list = []
    y_list = []
    z_list = []

    def calculate_bbox_minmax(list):
        for i in range(len(list)):
            x_list.append(list[i][0])
            y_list.append(list[i][1])
            z_list.append(list[i][2])

        max_x = max(x_list)
        max_y = max(y_list)
        max_z = max(z_list)

        min_x = min(x_list)
        min_y = min(y_list)
        min_z = min(z_list)

        max_co = (max_x, max_y, max_z)
        min_co = (min_x, min_y, min_z)

        return max_co, min_co

    max_co, min_co = calculate_bbox_minmax(bbox_verts)
    print(max_co, min_co)
    max_co = [int(x + 5) for x in max_co]
    min_co = [int(x - 5) for x in min_co]
    print(max_co, min_co)
    voxel_x = []
    voxel_y = []
    voxel_z = []

    x_length = abs(max_co[0]-min_co[0])
    y_length = abs(max_co[1]-min_co[1])
    z_length = abs(max_co[2]-min_co[2])
    print(x_length,y_length,z_length)

    step = voxel_size/2
    print(step)

    x = int(x_length/step)
    y = int(y_length/step)
    z = int(z_length/step)
    print(x,y,z)

    co = list(min_co)
    for i in range(x):
        x_co = int(co[0]) + step * i
        #print(x_co)
        voxel_x.append(x_co)
    #print(voxel_x)

    for j in range(y):
        y_co = int(co[1]) + step * j
        voxel_y.append(y_co)

    for k in range(z):
        z_co = int(co[2]) + step * k
        voxel_z.append(z_co)

    nx = np.array(voxel_x)
    ny = np.array(voxel_y)
    nz = np.array(voxel_z)

    #print(nx,ny,nz)
    grids = np.vstack(np.meshgrid(nx, ny, nz)).reshape(3, -1).T
    print(grids, type(grids))
    print(f'grid of {grids.size} generated')

    bpy.ops.object.mode_set(mode="OBJECT")

    for box in bbox_list:
        bpy.data.objects[str(box.name)].select_set(True)
    bpy.ops.object.delete()
    return grids

def join_objects_and_undo():
    scene = bpy.context.scene
    depsgraph = bpy.context.evaluated_depsgraph_get()

    obs = []
    for ob in scene.objects:
        # whatever objects you want to join...
        if ob.type == 'MESH':
            obs.append(ob)

    ctx = bpy.context.copy()

    # one of the objects to join
    ctx['active_object'] = obs[0]
    ctx['selected_editable_objects'] = obs
    joined = bpy.ops.object.join(ctx)
    print(f'{len(obs)} objects joined')

    tree = []
    for obj in scene.objects:
        if obj.type == 'MESH':
            bvhtree = BVHTree.FromObject(obj, depsgraph, epsilon=0.0001)
            tree.append(bvhtree)

    print('bvhtree obtained')
    #bpy.ops.ed.undo()
    print('action undone')
    return tree[0]

def filter_voxels(grids, bvhtree, reso):
    context = bpy.context
    scene = context.scene

    filtered_co = []
    #filtered_y = []
    #filtered_z = []

    for index, co in enumerate(grids):
        #lcoation, normal, index, dist =
        result = bvhtree.find_nearest(co, reso / 2)
        print(result)
        if result[0]:
            filtered_co.append(co)

    print(f'filtered_grid of {grids.size} generated')

    fnx = np.array(filtered_co)
    #fny = np.array(filtered_y)
    #fnz = np.array(filtered_z)

    # print(nx,ny,nz)
    fgrids = np.vstack(fnx)#.reshape(3, -1).T
    '''
    for index, co in enumerate(fgrids):
        me = bpy.data.meshes.new(f'line{co}')
        xyz4, xyz5, xyz6 = co
        me.from_pydata([(xyz4, xyz5, xyz6)], [], [])
        me.update()
        print(f'mesh created',index,'/',len(fgrids))
        new_object = bpy.data.objects.new(f'line', me)
        bpy.data.collections['new_collection'].objects.link(new_object)
    '''
    print(fgrids, type(fgrids))
    print(f'grid of {fgrids.size} generated')

    return fgrids

if __name__ == "__main__":
    # material_reset()
    reso = 2
    grids = voxelcoor_cal(2)
    save_path = "D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/Alpha/"
    os.makedirs(save_path, exist_ok=True)
    file_name = os.path.join(save_path, "grids.npy")
    np.save(file_name, grids)

    bvhtree = join_objects_and_undo()
    fgrids = filter_voxels(grids,bvhtree,reso)

    # open the file in the write mode
    save_path = "D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/Alpha/"
    os.makedirs(save_path, exist_ok=True)
    file_name = os.path.join(save_path, "filtered_grids.npy")
    np.save(file_name, fgrids)

    # Pass numpy array to Open3D.o3d.geometry.PointCloud and visualize
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(grids)
    o3d.io.write_point_cloud("D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/Alpha/grids.ply", pcd)
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(fgrids)
    o3d.io.write_point_cloud("D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/Alpha/filtered.ply", pcd2)
    print('done')
'''

girds = voxelcoor_cal(1, max_co=max_co, min_co=min_co)
print("shape=",girds,len(girds),type(girds))

# open the file in the write mode
save_path = "D:/User Data/Documents/Research Ref/Main_research/BlenderShellDev/"
os.makedirs(save_path, exist_ok=True)
file_name = os.path.join(save_path, "grids.csv")
f = open(file_name, 'w')

# create the csv writer
writer = csv.writer(f)

# write a row to the csv file
writer.writerow(girds)

# close the file
f.close()

    voxel_size = 100  # cm
    bpy.ops.object.modifier_add(type='REMESH')
    bpy.context.object.modifiers['Remesh'].mode = 'VOXEL'
    bpy.context.object.modifiers['Remesh'].voxel_size = int(voxel_size / 100)
    bpy.ops.object.modifier_apply(modifier='Remesh')

    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_mode(type="VERT")
    bpy.ops.mesh.select_all(action='DESELECT')
    # bpy.ops.object.mode_set(mode="EDIT")
    wmtx = obj.matrix_world.copy()
    print('wmtx', wmtx)
    for v in obj.data.vertices:
        v.select_set(True)
        #co_world = wmtx @ v.co
        #bpy.ops.mesh.primitive_cube_add(size=0.5, location= co_world)

    #bpy.data.objects[str(box.name)].select_set(True)




##asubdivide the bbox into set resolution
##bbox.select_set(True)
bpy.ops.object.mode_set(mode="EDIT")
bpy.ops.mesh.subdivide(number_cuts= 2)
bpy.ops.object.mode_set(mode="OBJECT")



while len(sel_objs) >= 1:
    obj1 = sel_objs.pop()
    obj1.select_set(True)

    bpy.ops.object.select_all(action='DESELECT')
'''