"""
Downsample all the points to around 100 for path planner

Cora Sept 21, 2020
"""
import numpy as np
import open3d as o3d

print(o3d.__version__)

from scipy.spatial.transform import Rotation as R
if __name__ == '__main__':
    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud("../../../data/stl2.ply")
    print(pcd)
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd])

    print("Compute the normal of the point cloud")
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))
    dense_normals_outward = np.asarray(pcd.normals)
    dense_normals_outward[:,-1] *= -1

    print("Downsample the point cloud with a voxel of 0.02")
    downpcd = pcd.voxel_down_sample(voxel_size=0.02)

    print("Recompute the normal of the downsampled point cloud")
    downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))

    # get the vector point into the liver instead of point out
    normals_outward = np.asarray(downpcd.normals)

    normals_inward = normals_outward.copy()
    normals_outward[:,-1] *= -1

    position = np.asarray(downpcd.points)
    posAndNormals_outward = np.concatenate((position, normals_outward), axis=1)
    posAndNormals_inward = np.concatenate((position, normals_inward), axis=1)
    np.save("../../../data/liverGrid_outward_normals.npy", posAndNormals_outward)
    np.save("../../../data/liverGrid_inward_normals.npy", posAndNormals_inward)

    dense_posAndNormals_outward = np.concatenate((np.asarray(pcd.points), dense_normals_outward), axis=1)
    np.save("../../../data/liverGrid_dense_outward_normals.npy", dense_posAndNormals_outward)

    print(np.load("../../../data/liverGrid_outward_normals.npy", allow_pickle=True).shape)
    print(np.load("../../../data/liverGrid_outward_normals.npy", allow_pickle=True).shape)
    o3d.visualization.draw_geometries([downpcd])
    


