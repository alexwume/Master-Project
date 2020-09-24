"""
Downsample all the points to around 100 for path planner

Cora Sept 21, 2020
"""
import numpy as np
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R

def LookatRotation(pos_x,pos_y,pos_z):

    eulerAngles = np.array([0.0,0.0,0.0])

    # AngleX = arccos(sqrt((x ^ 2 + z ^ 2) / (x ^ 2 + y ^ 2 + z ^ 2)))
    eulerAngles[0] = np.arccos(np.sqrt((pos_x * pos_x + pos_z * pos_z) / (pos_x * pos_x + pos_y * pos_y + pos_z * pos_z)))
    if (pos_y > 0): eulerAngles[0] = np.pi*2 - eulerAngles[0]

    # AngleY = arctan(x / z)
    eulerAngles[1] = np.arctan2(pos_x, pos_z)
    if (eulerAngles[1] < 0): eulerAngles[1] += np.pi;
    if (pos_x < 0): eulerAngles[1] += np.pi;

    # AngleZ = 0
    eulerAngles[2] = 0
    return eulerAngles


from scipy.spatial.transform import Rotation as R
if __name__ == '__main__':
    pcd = o3d.io.read_point_cloud("../../../data/stl2.ply")
    print(np.asarray(pcd.points))
    # o3d.visualization.draw_geometries([pcd])

    # get the dense version of the downsample point cloud
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))
    o3d.visualization.draw_geometries([pcd])
    dense_normals_outward = np.asarray(pcd.normals)
    dense_normals_inward = dense_normals_outward.copy()
    dense_normals_inward[:,-1] = 2*np.pi - dense_normals_outward[:,-1]

    # get the sparse version of the downsample point cloud
    downpcd = pcd.voxel_down_sample(voxel_size=0.02)
    downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))
    o3d.visualization.draw_geometries([downpcd])

    # get the orientation from the normal vector
    normals_sparse_outward = np.asarray(downpcd.normals)
    euler_sparse_outward = np.ones(normals_sparse_outward.shape)
    for row_idx in np.arange(normals_sparse_outward.shape[0]):
        temp = normals_sparse_outward[row_idx].copy()
        print("temp", temp)
        euler_sparse_outward[row_idx] = LookatRotation(temp[0],temp[1],temp[2])
        print("normals_sparse_outward", normals_sparse_outward[row_idx])

    position = np.asarray(downpcd.points)
    sparse_posAndNormals_outward = np.concatenate((position, normals_sparse_outward), axis=1)
    sparse_posAndNormals_outward_euler = np.concatenate((position, euler_sparse_outward), axis=1)
    np.save("../../../data/normals_sparse_outward.npy", sparse_posAndNormals_outward)
    np.save("../../../data/euler_sparse_outward.npy", sparse_posAndNormals_outward_euler)

    a = np.load("../../../data/euler_sparse_outward.npy").shape
    print(a)


    # dense_posAndNormals_outward = np.concatenate((np.asarray(pcd.points), dense_normals_outward), axis=1)
    # np.save("../../../data/normals_dense_outward.npy", dense_posAndNormals_outward)

    


