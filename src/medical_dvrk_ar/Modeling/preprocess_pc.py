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
    r_euler = np.asarray(downpcd.normals)
    r_euler_rev = r_euler.copy()
    r_euler_rev[:,-1] *= -1
    r = R.from_euler('xyz', r_euler_rev, degrees=False)


    # get the reverse vector point into the liver
    normal_quat = r.as_quat()
    position = np.asarray(downpcd.points)
    posAndQuat = np.concatenate((position, normal_quat), axis=1)
    np.save("../../../data/liverGrid_inv_norm.npy", posAndQuat)
    print(np.load("../../../data/liverGrid_inv_norm.npy", allow_pickle=True).shape)
    o3d.visualization.draw_geometries([downpcd])



