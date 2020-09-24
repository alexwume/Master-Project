"""
Downsample all the points to around 100 for path planner

Cora Sept 21, 2020
"""
import numpy as np
import open3d as o3d
<<<<<<< HEAD
=======
import numpy as np
from scipy.spatial.transform import Rotation as R
>>>>>>> cora

from scipy.spatial.transform import Rotation as R
if __name__ == '__main__':
    pcd = o3d.io.read_point_cloud("../../../data/stl2.ply")
    print(np.asarray(pcd.points))
    # o3d.visualization.draw_geometries([pcd])

    # get the sparse version of the downsample point cloud
    downpcd = pcd.voxel_down_sample(voxel_size=0.005)
    downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
    o3d.visualization.draw_geometries([downpcd])

    # get the orientation from the normal vector
    normals_sparse_outward = np.asarray(downpcd.normals)

    position = np.asarray(downpcd.points)
    sparse_posAndNormals_outward = np.concatenate((position, normals_sparse_outward), axis=1)
    np.save("../../../data/normals_sparse_outward.npy", sparse_posAndNormals_outward)

    a = np.load("../../../data/normals_sparse_outward.npy").shape
    print(a)


    # dense_posAndNormals_outward = np.concatenate((np.asarray(pcd.points), dense_normals_outward), axis=1)
    # np.save("../../../data/normals_dense_outward.npy", dense_posAndNormals_outward)

    


