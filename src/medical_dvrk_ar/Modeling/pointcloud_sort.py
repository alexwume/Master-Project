import numpy as np

def pointcloud_sort(input_path, result_path):
	values = np.load(input_path)
	point_num = values.shape[0]
	dtype = [('x',float),('y',float),('z',float),('raw',float),('pitch',float),('yaw',float)]
	pointcloud = values.view(dtype)
	pointcloud = pointcloud.reshape((point_num,))
	pointcloud = np.sort(pointcloud, order=["x","y"])
	pointcloud = np.array(pointcloud.tolist())
	np.save(result_path, pointcloud)

if __name__ == "__main__":
    file_path = "/home/chang/catkin_ws/src/Medical-DVRK-AR/data/"
    file_name = "liverGrid_norm.npy"
    sorted_file_name = "sorted_liverGrid_norm.npy"
    pointcloud_sort(file_path+file_name, file_path+sorted_file_name)