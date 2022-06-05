import bpy
import sys
import os
import numpy as np
import bmesh
from mathutils import Vector
from mathutils.bvhtree import BVHTree
import open3d as o3d
import datetime

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

    step = voxel_size
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
    print(grids[:10], type(grids))
    print(f'grid of {len(grids)} generated')

    bpy.ops.object.mode_set(mode="OBJECT")

    for box in bbox_list:
        bpy.data.objects[str(box.name)].select_set(True)
    bpy.ops.object.delete()
    return grids

def join_objects_and_undo():
    context = bpy.context
    scene = bpy.context.scene
    depsgraph = bpy.context.evaluated_depsgraph_get()
    '''
    obs = []
    for ob in scene.objects:
        # whatever objects you want to join...
        if ob.type == 'MESH':
            obs.append(ob)
    print(len(obs))
    ctx = bpy.context.copy()

    # one of the objects to join
    ctx['active_object'] = obs[0]
    ctx['selected_editable_objects'] = obs
    bpy.ops.object.join(ctx)
    print(f'{len(obs)} objects joined')
    '''
    #context.view_layer.objects.active = context.scene.objects.get('_ncl1_892Mesh')
    tree = []
    mtx = []
    for obj in scene.objects:
        if obj.type == 'MESH':
            bm = bmesh.new()

            bpy.context.view_layer.objects.active = obj
            print(obj.name)
            bvhtree = BVHTree.FromObject(obj, depsgraph)
            mtx.append(obj.matrix_world.inverted())
            '''
            #mesh = obj.to_mesh(depsgraph,)
            bm.from_object(obj,depsgraph)
            bm.transform(obj.matrix_world)

            bvhtree = BVHTree.FromBMesh(bm)
            #bpy.data.meshes.remove(mesh)
            
            mtx.append(obj.matrix_world.inverted())
            print('name=', obj.name, bpy.context.object.name)
            bpy.ops.object.mode_set(mode='EDIT')
            bm = bmesh.from_edit_mesh(obj.data)
            '''
            tree.append(bvhtree)#BVHTree.FromBMesh(bm,epsilon=1e5))
    '''
    bvhtree = BVHTree.FromObject(obj, depsgraph, epsilon=0.01)
                tree.append(bvhtree)
    '''
    print('bvhtree obtained', tree)
    #bpy.ops.ed.undo()
    print('action undone')
    #bpy.ops.object.mode_set(mode="OBJECT")
    return tree[0], mtx[0]

def filter_voxels(grids, mtx ,bvhtree, reso):
    context = bpy.context
    scene = context.scene
    filtered_co = []
    for index, co in enumerate(grids):
        #print('mtx',mtx)
        ray_begin = Vector(co)

        range = 3
        z = co[2]
        if z >= 0:
            z = z - range
        elif z < 0:
            z = -z - range

        ray_end = Vector((co[0],co[1],z))

        ray_dir = ray_end - ray_begin
        #print(ray_end,ray_begin)
        ray_dir.normalize()
        local_co = mtx @ ray_begin
        #print(index,"/",len(grids))
        #lcoation, normal, index, dist =
        #donw_vec = mtx @ Vector(np.array([0,0,-1]))
        loc,nor,idx,dist = bvhtree.ray_cast(local_co,ray_dir,3)
        #pos, norm, idx, d = bvhtree.find_nearest(local_co, reso)
        #print(loc, '@', co)
        #print(result[0])
        if loc is not None:
            filtered_co.append(co)
            #print(result)
    print('len(filter):',len(filtered_co))
    #print(f'grid of {grids.size} inputed')

    fnx = np.array(filtered_co)
    #fny = np.array(filtered_y)
    #fnz = np.array(filtered_z)

    # print(nx,ny,nz)
    fgrids = np.vstack(fnx)#.reshape(3, -1).T

    print(fgrids, type(fgrids))
    print(f'filtered_grid of {fgrids.size} generated')

    return fgrids

if __name__ == "__main__":
    # material_reset()
    now = datetime.time
    print("Starting script, Current Time =", now)

    reso = 1
    grids = voxelcoor_cal(reso)
    '''
    save_path = "./plys/ori_voxels/"
    os.makedirs(save_path, exist_ok=True)
    file_name = os.path.join(save_path, f'grids{len(grids)}.npy')
    np.save(file_name, grids)

    # Pass numpy array to Open3D.o3d.geometry.PointCloud and visualize
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(grids)
    o3d.io.write_point_cloud(f'D:/Program Files (x86)/Blender/2.90/scripts/BlenderShellDev/Alpha/plys/ori_voxels/grid_{len(grids)}.ply', pcd)
    print('done')
    '''
    # get filtered grids from open3d mesh2voxel
    bvhtree_1, mtx = join_objects_and_undo()

    f_grids = filter_voxels(grids, mtx, bvhtree_1, 1)
    '''
    # open the file in the write mode
    save_path = "./plys/ori_voxels/"
    os.makedirs(save_path, exist_ok=True)
    file_name = os.path.join(save_path, f'voxel_{f_grids.size}.npy')
    np.save(file_name, f_grids)
    '''
    phase = 2
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(f_grids)
    o3d.io.write_point_cloud(f'D:/Program Files (x86)/Blender/2.90/scripts/BlenderShellDev/Alpha/plys/ori_voxels/for_filter/{phase}voxel_{f_grids.size}.ply',
                             pcd2)

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