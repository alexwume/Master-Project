"""
Downsample all the points to around 100 for path planner

Cora Sept 21, 2020
"""
import numpy as np
import open3d as o3d

from scipy.spatial.transform import Rotation as R
if __name__ == '__main__':
    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud("../../../data/stl2.ply")
    print(pcd)
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd])

    print("Downsample the point cloud with a voxel of 0.05")
    downpcd = pcd.voxel_down_sample(voxel_size=0.02)

    print("Recompute the normal of the downsampled point cloud")
    downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))

    # get the vector point into the liver instead of point out
    r_euler_outward = np.asarray(downpcd.normals)
    r_euler_inward = r_euler_outward.copy()
    r_euler_outward[:,-1] *= -1
    r_inward = R.from_euler('xyz', r_euler_inward, degrees=False)
    r_outward = R.from_euler('xyz', r_euler_outward, degrees=False)

    # get the reverse vector point into the liver
    inward_normal_quat = r_inward.as_quat()
    outward_normal_quat = r_outward.as_quat()
    position = np.asarray(downpcd.points)
    posAndQuat_inward = np.concatenate((position, inward_normal_quat), axis=1)
    posAndQuat_outward = np.concatenate((position, outward_normal_quat), axis=1)
    posAndEuler_outward = np.concatenate((position, r_euler_outward), axis=1)
    posAndEuler_inward = np.concatenate((position, r_euler_inward), axis=1)
    np.save("../../../data/liverGrid_inward_quat.npy", posAndQuat_inward)
    np.save("../../../data/liverGrid_outward_quat.npy", posAndQuat_outward)
    np.save("../../../data/liverGrid_outward_euler.npy", posAndEuler_outward)
    np.save("../../../data/liverGrid_inward_euler.npy", posAndEuler_inward)

    print(np.load("../../../data/liverGrid_outward_euler.npy", allow_pickle=True).shape)
    print(np.load("../../../data/liverGrid_outward_quat.npy", allow_pickle=True).shape)
    o3d.visualization.draw_geometries([downpcd])



