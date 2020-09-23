import numpy as np

def pointcloud_filter(pointcloud, max_angle):
	# only filter out the points with normals within the max_angle cone of z positive direction
	# to ensure the robot arm can reach the point
	pointcloud_reachable = []
	for point in pointcloud:
		# x,-z, y
		angle = np.arccos(np.dot(point[3:],[0,-1,0]))
		if angle <= max_angle * np.pi / 180:
			pointcloud_reachable.append(point)
	pointcloud_reachable = np.array(pointcloud_reachable)
	print("Remaining points:",pointcloud_reachable.shape[0])
	return pointcloud_reachable


def pointcloud_sort(pointcloud):
	point_num = pointcloud.shape[0]
	dtype = [('x',float),('y',float),('z',float),('normal_x',float),('normal_y',float),('normal_z',float)]
	pointcloud = pointcloud.view(dtype)
	pointcloud = pointcloud.reshape((point_num,))
	pointcloud = np.sort(pointcloud, order=["x","y"])
	pointcloud = np.array(pointcloud.tolist())
	#to switch from zigzag to corner turn
	line_startpoint_idx = []
	for i in range(1,pointcloud.shape[0]-1):
		last_dis = np.linalg.norm(pointcloud[i]-pointcloud[i-1])
		next_dis = np.linalg.norm(pointcloud[i+1]-pointcloud[i])
		if next_dis > 2 * last_dis: # if the moving distance to next point is too large, it's probably the end of the line
			line_startpoint_idx.append(i+1)
	
	print("Line number:",len(line_startpoint_idx))
	# reverse the order in a line
	for i in range(len(line_startpoint_idx)-1):
		pointcloud[line_startpoint_idx[i]:line_startpoint_idx[i+1]] = pointcloud[line_startpoint_idx[i]:line_startpoint_idx[i+1]][::-1]

	pointcloud[line_startpoint_idx[-1]:] = pointcloud[line_startpoint_idx[-1]:][::-1]
	return pointcloud


if __name__ == "__main__":
	# parameter to adjust
    file_path = "/home/chang/catkin_ws/src/Medical-DVRK-AR/data/"
    file_name = "liverGrid_dense_outward_normals.npy"
    sorted_file_name = "liverGrid_dense_outward_normals_cleanForPlanning.npy"
    max_angle = 60 # change  the param within [0,90)

    pointcloud = np.load(file_path+file_name)
    pointcloud = pointcloud_sort(pointcloud_filter(pointcloud,max_angle))
    np.save(file_path+sorted_file_name, pointcloud)


