import sys
import os
import pygad
import numpy as np
import time
import math
import open3d as o3d
import json
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

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

        frustum_lineset = o3d.geometry.LineSet.create_camera_visualization(cam.intrinsic, cam.extrinsic, scale=5)
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
                     location_index, dir_list ,dir_index, cam_config, ray_cast):
    cam_far = cam_config[ 0 ][ 0 ]
    # test case
    # z = 90
    # tilt =0,90:horizontal,180
    # pan = 0@world_x_axis,90@world_y_axis  # dir_list[dir_index] #CV coordinate: x right, y down, z forward
    cam_co = np.asarray(location_list[ location_index ])
    dir = dir_list[dir_index]
    #print('cam_co=', cam_co)
    #print('cam_dir=', )
    half_plane_normals, frustum, normal = setup_cam(cam_co,
                                                   dir[ 0 ],
                                                   dir[ 1 ],
                                                   dir[ 2 ],
                                                                         cam_config)

    in_view = [ ]
    is_visible = [ ]
    queries = [ ]
    index_list = [ ]
    ##get in radius points
    [ k, idx, _ ] = model_kdtree.search_radius_vector_3d(cam_co, cam_far)
    model_pcd = model_pcd.select_by_index(idx)

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

def read_cam_locations(path, write_file):
    location_pcd = o3d.io.read_point_cloud(path)
    location_list = np.array(location_pcd.points)
    location_index = [ i for i, _ in enumerate(location_list) ]

    if write_file == 1:
        loc_index_dict = {}
        #print(location_list)
        for index, co in enumerate(location_index):
            tmp =  dict(zip([str(location_list[index])],[index]))
            loc_index_dict.update(tmp)
        with open('file_name.json', 'w', encoding='utf-8') as f:
            json.dump(loc_index_dict, f, ensure_ascii=False, indent=4)
        print('json created')

    if len(location_list) != len(location_index):
        raise Exception('location length and index length is different')

    return location_pcd, location_list, location_index

def result_visualization(solution, cam_num):
    # installable angles
    rot_x = 0
    rot_x_size = range(45, 140, 15)  # {'low': 5, 'high': 90}
    rot_y = 0
    rot_y_size = range(30, 370, 30)  # {'low': 5, 'high': 360}

    dir_list_size = [ ]
    for y in rot_y_size:
        for x in rot_x_size:
            dir_list_size.append([ x, y, 0 ])

    dir_index = [ i for i, _ in enumerate(dir_list_size) ]

    print('starting_visualization')
    print('solution==[cam_type,index,x,y]', solution)
    total_score = len(np.asarray(model_pcd.points))
    print('expected_score is', total_score)

    nested_solution = [ solution[ i:i + 3 ] for i in range(0, len(solution), 3) ]
    cam_cost = [ ]
    # Introduce multiprocessing
    for i in range(cam_num):
        id = nested_solution[ i ][ 0 ]
        cam_cost.append(id)

    network_cost = [ ]
    for cam in cam_cost:
        if cam == 0:
            unit_cost = cam_config[ cam ][ 1 ]
            network_cost.append(unit_cost)
        elif cam == 1:
            unit_cost = cam_config[ cam ][ 1 ]
            network_cost.append(unit_cost)
        elif cam == 2:
            unit_cost = cam_config[ cam ][ 1 ]
            network_cost.append(unit_cost)

    print('network_cost:', sum(network_cost))
    in_view_list = [ ]
    in_view_pts = [ ]
    in_view_frustums = [ ]
    fitness_list = [ ]
    phase_fitness = [ ]
    for countt, phase in enumerate(phase_list):
        for i in range(cam_num):
            id = nested_solution[ i ][ 0 ]
            loc_index = nested_solution[ i ][ 1 ]
            rot_index = nested_solution[ i ][ 2 ]

            if id == 0:
                in_view, frustum = inPTZcam_frustum(phase, kdtree_list[ countt ], scene_list[ countt ],
                                                    location_list, loc_index, dir_list_size, rot_index, cam_config,
                                                    1)

                in_view_list.append(in_view)
                in_view_pts.append(np.asarray(in_view.points))
                in_view_frustums.append([frustum,location_list[loc_index]])

        temp_array = np.concatenate(in_view_pts, axis=0)
        # print(len(temp_array))
        unique_co = np.unique(temp_array, axis=0)
        # print(len(unique_co))
        phase_fitness.append(100 * len(unique_co) / total_score)

    draw_list = []
    #draw_list = [ pcd for pcd in in_view_list ]
    fitness = 0
    for index, item in enumerate(phase_fitness):
        print(f'phase{index}_score:', item)

    all_fitness = np.sum(phase_fitness) / len(phase_list)
    print('all_phases = fitness_solution:', all_fitness)

    draw_list += [ frustums[0] for frustums in in_view_frustums ]
    draw_list.append(model_mesh_list[ -1 ])
    draw_list.append(location_pcd)
    draw_list.append(dense_cam_pcd)
    draw_list.append(site_pcd)
    '''
    app = gui.Application.instance
    app.initialize()

    w = app.create_window("Open3D - 3D Text", 1024, 768)
    widget3d = gui.SceneWidget()
    widget3d.scene = rendering.Open3DScene(w.renderer)
    mat = rendering.Material()
    mat.shader = "defaultUnlit"

    mat.point_size = 5 * w.scaling
    for pcd in in_view_list:
        widget3d.scene.add_geometry("Points", pcd)
    
    #widget3d.scene.add_geometry("Points", points, mat)
    #widget3d.scene.add_geometry("Points", points, mat)

    #for idx in range(0, len(points.points)):
    #    widget3d.add_3d_label(points.points[ idx ], "{}".format(idx))

    bbox = widget3d.scene.bounding_box
    widget3d.setup_camera(60.0, bbox, bbox.get_center())
    w.add_child(widget3d)

    app.run()
    
    for frustums in in_view_frustums:
        label = o3d.visualization.gui.Label3D
        label.scale = 5.0
        label.color = o3d.visualization.gui.Color(r=0.,g=0.,b=0.,a=1.)
        label(str(frustums.vertices[0]),frustums.vertices[0])
    '''
    #o3d.visualization.draw_geometries(draw_list)

    # camera_points_pcd = o3d.geometry.PointCloud()
    # camera_points_pcd.points = o3d.utility.Vector3dVector(draw_list)
    # o3d.io.write_point_cloud(f'./{solution}_{cam_num}.ply',camera_points_pcd)
    return in_view_frustums, in_view_list, all_fitness, draw_list

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
    for p in range(0,7,1):
        #triangle_meshes
        file_name = os.path.join(save_path, f'plys/ori_voxels/for_filter/mesh/phase_{p}.ply')
        model_mesh = o3d.io.read_triangle_mesh(file_name)
        #o3d.visualization.draw_geometries([model_mesh])
        model_mesh_list.append(model_mesh)
        # initializing raycast scene
        scene = o3d.t.geometry.RaycastingScene()
        cube_id = scene.add_triangles(np.asarray(model_mesh.vertices, dtype=np.float32),
                                      np.asarray(model_mesh.triangles, dtype=np.uint32))
        scene_list.append(scene)
        # open filtered voxels
        model = os.path.join(save_path, f'plys/ori_voxels/for_filter/presenting/{p}_filtered_vox.ply')
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

    #print('phase_count=',len(model_mesh_list),len(phase_list))
    # open camera_postions
    # save_path = "/home/adrain/Desktop/bpydev/BlenderShellDev/Alpha/plys/"
    #use new filtered
    #camera = os.path.join(save_path, "plys/downsample_5.ply")
    distance = 20
    camera = os.path.join(save_path, f'plys/ori_voxels/for_filter/cam_loc.ply')
    location_pcd, location_list, location_index = read_cam_locations(camera, 0)
    dense_cam = os.path.join(save_path, f'plys/ori_camera.ply')
    site = os.path.join(save_path, f'plys/site_only.ply')
    dense_cam_pcd = o3d.io.read_point_cloud(dense_cam)
    site_pcd = o3d.io.read_point_cloud(site)
    '''
        num_generations = 1
        num_parents_mating = 1
        sol_per_pop = 1
        keep_parents = 1
    '''
    test_set = [ 1, 2, 2, 1 ]
    run_set = [ 1000, 90, 100, 10 ]
    GA_settings = test_set

    desired_output = 100  # Function output.
    # cam_config =[[range., cost, resolution]]
    cam_config = [ [ 50., 18000, [ 1920, 1080 ] ], [ 50., 2400] ]
    ori_budget = 2800
    #for i in range(4,6):
    solution  = [ 0, 6, 45, 0, 90, 59, 0, 147, 80, 0, 220, 12, 0, 20, 52, 0, 91, 31, 0, 148, 73, 0, 221, 11, 0, 8, 24, 0, 215, 72]
    cam_num = int(len(solution)/3)
    in_view_frustums, in_view_list, all_fitness, draw_list  = result_visualization(solution=solution,cam_num=cam_num)
    '''
    voxels_pts = [np.asarray(pcd.points) for pcd in in_view_list]
    f_voxel = []
    voxels = o3d.geometry.Voxel()
    for pcd in voxels_pts:
        for idx in pcd:
            voxels.grid_index = idx
            f_voxel.append(voxels)

    voxel_grid = o3d.geometry.VoxelGrid()
    voxel_grid.voxels = f_voxel
    o3d.visualization.draw_geometries(voxel_grid)
    o3d.io.write_voxel_grid(voxel_path, voxel_grid)
    '''
    voxels_pts = [ pcd.points for pcd in in_view_list ]
    voxels = [ food for sublist in voxels_pts for food in sublist ]

    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(voxels)
    pt_num = len(pcd2.points)
    voxel_kdtree = o3d.geometry.KDTreeFlann(pcd2)
    detection = [5.,10.,20.,50.]
    np_colors = np.asarray([[ 1, 0, 0 ],[ 0, 1, 0 ],[ 0, 0, 1 ],[1,1,1]],dtype=np.float64)
    # [ 1, 0, 0 ],[ 0, 1, 0 ],[ 0, 0, 1 ]
    in_range = [[] for _ in range(len(detection))]
    for frustum in in_view_frustums:
        cam_co = frustum[1]
        for i in range(len(detection)):
            [ _, idx, _ ] = voxel_kdtree.search_radius_vector_3d(cam_co, detection[i])
            in_range[i].append(idx)
    pcds = []
    for j in range(len(in_range)):
        k = np.asarray(in_range[j])
        #print(k)
        temp_array = np.concatenate(k, axis=0)
        unique_indexes = np.unique(temp_array, axis=0)
        colored = pcd2.select_by_index(unique_indexes)
        #model_pcd.colors = o3d.utility.Vector3dVector(np_colors[0])
        colored.paint_uniform_color(np_colors[j])
        pcds.append(colored)

    draw_list += [pcd for pcd in pcds]
    o3d.visualization.draw_geometries(draw_list)
    #voxel_path = os.path.join(save_path, f'plys/results/{cam_num}_{all_fitness}.ply')
    #o3d.io.write_point_cloud(voxel_path, pcd2)

    #o3d.visualization.draw_geometries([pcds])

    duration = time.perf_counter() - total_start_time
