#%%
import open3d as o3d
import time
import numpy as np
import opt_einsum.backends
import matplotlib.pyplot as plt
from pymeshfix import _meshfix
import mcubes
import pyvista as pv
import pymeshfix as mf
from pyvista import examples

def show_pc(pc):
    if isinstance(pc, list):
        o3d.visualization.draw_geometries(pc)
    else:
        o3d.visualization.draw_geometries([pc])

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

def show_point_cloud_clusters(pcd):
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(pcd.cluster_dbscan(eps=3, min_points=100, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([pcd])

def show_largest_mesh_cluster(mesh):
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        triangle_clusters, cluster_n_triangles, cluster_area = (
            mesh.cluster_connected_triangles())
    triangle_clusters = np.asarray(triangle_clusters)
    cluster_n_triangles = np.asarray(cluster_n_triangles)
    cluster_area = np.asarray(cluster_area)

    print("Show largest cluster")
    mesh_1 = copy.deepcopy(mesh)
    largest_cluster_idx = cluster_n_triangles.argmax()
    triangles_to_remove = triangle_clusters != largest_cluster_idx
    mesh_1.remove_triangles_by_mask(triangles_to_remove)
    o3d.visualization.draw_geometries([mesh_1])

def close_mesh_holes(mesh, **kwargs):

    filename = './mesh_cleaned.ply'
    save = False
    for key, value in kwargs.items():
        if key in ['keep', 'save']:
            save = value
        if key in ['name', 'filename']:
            filename = value
    
    v = np.asarray(mesh.vertices)
    f = np.asarray(mesh.triangles)

    # Clean mesh
    tin = _meshfix.PyTMesh()
    tin.load_array(v, f)

    tin.join_closest_components()
    tin.fill_small_boundaries()

    #tin.clean(max_iters=10, inner_loops=3)

    vclean, fclean = tin.return_arrays()
    meshfix = mf.MeshFix(vclean, fclean)

    meshfix.save(filename)
    mclean = o3d.io.read_triangle_mesh(filename)

    if not save:
        os.remove(filename)
    
    return mclean

def clean_point_cloud(pcd, filter_aggressiveness=0.2, **kwargs):
    
    filename = './test_data/pcd.ply'
    save = True
    
    for key, value in kwargs.items():
        if key in ['filename', 'name', 'output']:
            filename = value
        if key in ['save']:
            save = value
    
    g = filter_aggressiveness*10
    if g < 1:
        g = 1
    pcd = pcd_orig
    pcd = pcd.voxel_down_sample(voxel_size=max(0.1, g*0.05))
    pcd, ind = pcd.remove_radius_outlier(nb_points=int((1+0.5*g + 0.5*g**2)), radius=2)
    pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=int((1+0.5*(g+2) + 0.5*(g+2)**2)), std_ratio=(2-0.1*g))

    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(pcd.cluster_dbscan(eps=2, min_points=int((1+0.5*(g+2) + 0.5*(g+2)**1)), print_progress=True))
    ind = []
    for i in range(len(labels)):
        if labels[i] >= 0:
            ind.append(i)
    pcd = pcd.select_by_index(ind)
    
    pcd.estimate_normals()

    if save:
        o3d.io.write_point_cloud(filename, pcd)
        print('Saving point cloud to file: %s' % filename)
    
    return pcd

def create_mesh(pcd, method='ball_pivoting', **kwargs):
    
    filename = './test_data/mesh.ply'
    save = True

    for key, value in kwargs.items():
        if key in ['filename', 'name', 'output']:
            filename = value
        if key in ['save']:
            save = value
    
    if not pcd.has_normals():
        print('Estimating point cloud normals...')
        pcd.estimate_normals()
    print('Converting point cloud from mesh using method: %s' % method)
    if method == 'poisson':
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=10, linear_fit=False)
        vertices_to_remove = densities < np.quantile(densities, 0.1)
        mesh.remove_vertices_by_mask(vertices_to_remove)
        bbox = pcd.get_axis_aligned_bounding_box()
        mesh = mesh.crop(bbox)
    elif method == 'ball_pivoting':
        distances = pcd.compute_nearest_neighbor_distance()
        avg_dist = np.mean(distances)
        dist_pct = np.percentile(distances, 80)
        radius = dist_pct * 1
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector([radius, radius*2]))
    elif method == 'mcubes':
        pass

    print('Cleaning up mesh...')
    mesh = clean_mesh(mesh)

    if save:
        o3d.io.write_triangle_mesh(filename, mesh)
        print('Saved point cloud to file: %s' % filename)
    
    return mesh

def mesh_to_pointcloud(mesh, num_points=-1):
    
    if num_points == -1:
        num_points = np.asarray(mesh.triangles).shape[0] * 3
        num_points = num_points * 2 # Oversample the mesh
    pcd = mesh.sample_points_poisson_disk(num_points)
    print('Resampled mesh to point cloud with %d points' % num_points)

    return pcd

def clean_mesh(mesh):
    mesh.remove_non_manifold_edges()
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh = mesh.merge_close_vertices(eps=1.0)
    mesh = mesh.subdivide_midpoint(number_of_iterations=2)
    mesh = mesh.filter_smooth_taubin(number_of_iterations=50)
    mesh = mesh.simplify_vertex_clustering(voxel_size=0.2)
    mesh = mesh.filter_smooth_taubin(number_of_iterations=100)

    return mesh
    
def save_mesh(mesh, filename='./test_data/mesh.ply'):
    o3d.io.write_triangle_mesh(filename, mesh)

def save_pcd(pcd, filename='/test_data/pcd.ply'):
    o3d.io.write_point_cloud(filename, pcd)

# %%
pcd = o3d.io.read_point_cloud('./test_data/apple.ply', format='ply')

pcd = clean_point_cloud(pcd, filter_aggressiveness=0.1)
show_pc(pcd)

mesh = create_mesh(pcd, method='ball_pivoting')
show_pc(mesh)

# %%
mclean = close_mesh_holes(mesh)
mclean = clean_mesh(mclean)
save_mesh(mclean, './test_data/out.ply')

# %%
meshfix = pymeshfix.MeshFix(v, f)
meshfix.plot()

meshfix.repair()
meshfix.save('./test_data/mesh_cleaned.ply')
