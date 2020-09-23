"""
Downsample all the points to around 100 for path planner

Cora Sept 21, 2020
"""
import numpy as np
import open3d as o3d
print(o3d.__version__)

from scipy.spatial.transform import Rotation as R
if __name__ == '__main__':
    pcd = o3d.io.read_point_cloud("../../../data/stl2.ply")
    print(np.asarray(pcd.points))
    # o3d.visualization.draw_geometries([pcd])

    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))
    dense_normals_outward = np.asarray(pcd.normals)
    dense_normals_inward = dense_normals_outward.copy()
    dense_normals_inward[:,-1] = 2*np.pi - dense_normals_outward[:,-1]

    downpcd = pcd.voxel_down_sample(voxel_size=0.02)
    downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))

    normals_sparse_outward = np.asarray(downpcd.normals)
    normals_sparse_inward = normals_sparse_outward.copy()
    normals_sparse_inward[:,-1] *= 2*np.pi - normals_sparse_outward[:,-1]

    position = np.asarray(downpcd.points)
    sparse_posAndNormals_outward = np.concatenate((position, normals_sparse_outward), axis=1)
    sparse_posAndNormals_inward = np.concatenate((position, normals_sparse_inward), axis=1)
    np.save("../../../data/normals_sparse_outward.npy", normals_sparse_outward)
    np.save("../../../data/normals_sparse_inward.npy", normals_sparse_inward)

    dense_posAndNormals_outward = np.concatenate((np.asarray(pcd.points), dense_normals_outward), axis=1)
    np.save("../../../data/normals_dense_outward.npy", dense_posAndNormals_outward)

    o3d.visualization.draw_geometries([downpcd])
    


