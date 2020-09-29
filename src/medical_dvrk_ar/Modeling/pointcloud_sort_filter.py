
Skip to content
Pulls
Issues
Marketplace
Explore
@KoalalikeTree
biorobotics /
Medical-DVRK-AR

21
0

    1

Code
Issues
Pull requests
Actions
Projects
Wiki
Insights

More
Medical-DVRK-AR/src/medical_dvrk_ar/Modeling/pointcloud_sort_filter.py /
@KoalalikeTree
KoalalikeTree no message
Latest commit d4f2fd5 4 days ago
History
3 contributors
@KoalalikeTree
@koooooalaliketree
@ChangShiRaine
191 lines (160 sloc) 7.87 KB
"""
Data preprocessing to Get the n by 6 key points pose for path planning,
Every row will contain [x,y,z,x',y',z'] where x' y' z' is the normal vector.
All rows will be in the order for the robot to execute.
"""

import numpy as np
import open3d as o3d

class filter_pointcloud_for_path_planner():
	def __init__(self, path_to_raw_data):
		"""
		Param: path_to_raw_data: string,path to the ply file which is exported from the blender stl file
			   max_angle_filtering: int, angle in degree, the threshold for filtering the norm vector
		"""

		# the initial point cloud read from the ply file, this one does not has norm in its property
		self.raw_pcl_without_norm = o3d.io.read_point_cloud(path_to_raw_data)


		# the point cloud with the normal vector
		self.raw_pcl_with_raw_norm = []

		# the filtered point cloud whose normal vector is all reachable by the robot
		self.raw_pcl_with_filtered_norm = []

		# the points is ordered for the robot to execute, copy() just work as a placeholder
		self.sorted_pcl_with_filtered_norm = []

	def downsample_raw_pcl_get_normal_vector(self, dsample_voxelSize=0.005, norm_est_radius=0.02, visualize=False):
		"""
		Param:
			1. voxelSize: float,the larger this number is, the sparse the output point cloud will be
			2. norm_est_radius：float,the surface area which the norm is get from, the bigger it is, the more "uniform" normal vector will be
			3. visualize: Bool, if ture, there will be a pop-up window for visualize the points, press "n" to visualize the normal vector
		Output:
			self.sparse_pcl_with_norm : N X 6 nparray, float, N indicates the number of the downsampled point,
									    6 is [x,y,z,u,v,w] where[x,y,z] is the position of the points, [u,v,w] is the normal vector of that point
		"""

		# get the sparse version of the downsample point cloud
		downpcd = self.raw_pcl_without_norm.voxel_down_sample(voxel_size=dsample_voxelSize)
		# get the normal vector of the downsample point cloud
		downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=norm_est_radius, max_nn=30))
		if visualize:
			o3d.visualization.draw_geometries([downpcd])

		# get the np array version of the normal
		normals_outward = np.asarray(downpcd.normals)
		# get the np array version of the points
		position = np.asarray(downpcd.points)
		# np.save('xyz_for_stiffness_est.npy', position)
		# this is for the robot frame, the z will shift by 0.2
		position[2,:] -=0.2
		# combine the position and norm

		return self.raw_pcl_with_raw_norm


	def filter_vector_with_angle_threshold(self, max_angle_filtering):
		"""only filter out the points with normals within the max_angle cone of z positive direction
		to ensure the robot arm can reach the point
		Param: max_angle_filtering: int, degree, will delete points that is bigger than this angle
		"""
		for point in self.raw_pcl_with_raw_norm:
			# x,-z, y
			angle = np.arccos(np.dot(point[3:],[0,0,1]))
			if (angle <= max_angle_filtering * np.pi / 180):
				self.raw_pcl_with_filtered_norm.append(point)
		self.raw_pcl_with_filtered_norm = np.array(self.raw_pcl_with_filtered_norm)
		print("Remaining points:",self.raw_pcl_with_filtered_norm.shape[0])


	def sorted_poinst_with_xy_position(self):
		"""
		this function turn the disordered point cloud into a sorted one
		which can directly be fed into the path planner in dvrk
		"""
		point_num = self.raw_pcl_with_filtered_norm.shape[0]
		dtype = [('x',float),('y',float),('z',float),('normal_x',float),('normal_y',float),('normal_z',float)]
		self.sorted_pcl_with_filtered_norm = self.raw_pcl_with_filtered_norm.view(dtype)
		self.sorted_pcl_with_filtered_norm = self.sorted_pcl_with_filtered_norm.reshape((point_num,))
		self.sorted_pcl_with_filtered_norm = np.sort(self.sorted_pcl_with_filtered_norm, order=["x","y"])
		self.sorted_pcl_with_filtered_norm = np.array(self.sorted_pcl_with_filtered_norm.tolist())
		#to switch from zigzag to corner turn
		line_startpoint_idx = []
		for i in range(1,self.sorted_pcl_with_filtered_norm.shape[0]-1):
			last_dis = np.linalg.norm(self.sorted_pcl_with_filtered_norm[i]-self.sorted_pcl_with_filtered_norm[i-1])
			next_dis = np.linalg.norm(self.sorted_pcl_with_filtered_norm[i+1]-self.sorted_pcl_with_filtered_norm[i])
			if next_dis > 2 * last_dis: # if the moving distance to next point is too large, it's probably the end of the line
				line_startpoint_idx.append(i+1)

		print("Line number:",len(line_startpoint_idx))
		# reverse the order in a line
		for i in range(len(line_startpoint_idx)-1):
			self.sorted_pcl_with_filtered_norm[line_startpoint_idx[i]:line_startpoint_idx[i+1]] = self.sorted_pcl_with_filtered_norm[line_startpoint_idx[i]:line_startpoint_idx[i+1]][::-1]

		self.sorted_pcl_with_filtered_norm[line_startpoint_idx[-1]:] = self.sorted_pcl_with_filtered_norm[line_startpoint_idx[-1]:][::-1]

		return self.sorted_pcl_with_filtered_norm
	
	# def savefile(self, fileType='quat',path):
	# 	"""
	# 	Param:
	# 		fileType: string, 'quat','euler'
	# 		'quat': save a 7 by N npy file, the first three colomns are position, last four is quat angle
	# 		'euler': save a 6 by N npy file, the first three colomns are position, last four is quat angle
	# 	"""
	# 	save_quat = np.empty((0,7))
	# 	save_euler = np.empty((0,6))
	# 	for i in range(self.sorted_pcl_with_filtered_norm[0]):
	# 	# negative because it's pointing inward to the liver
	# 		norm_x = -self.sorted_pcl_with_filtered_norm[i,3]
	# 		norm_y = -self.sorted_pcl_with_filtered_norm[i,4]
	# 		norm_z = -self.sorted_pcl_with_filtered_norm[i,5]
	#
	# 		x_euler, z_euler = self.norm2euler(norm_x,norm_y,norm_z)
	# 		if fileType == 'euler':
	# 			save_euler = np.concatenate((save_euler, np.array([[
	# 							self.sorted_pcl_with_filtered_norm[i,0],
	# 							self.sorted_pcl_with_filtered_norm[i,1],
	# 							self.sorted_pcl_with_filtered_norm[i,2],
	# 							norm_x,
	# 							norm_y,
	# 							norm_z
	# 						]])), axis=0)
	# 			np.save('path', save_euler)
	# 		if fileType == 'quat':
	# 			quat_r = R.from_euler('zxy',[x_euler, 0, z_euler],degrees=False)
	# 			save_euler = np.concatenate((save_euler, np.array([[
	# 							self.sorted_pcl_with_filtered_norm[i,0],
	# 							self.sorted_pcl_with_filtered_norm[i,1],
	# 							self.sorted_pcl_with_filtered_norm[i,2],
	# 							quat_r.as_quat()[0],
	# 							quat_r.as_quat()[1],
	# 							quat_r.as_quat()[2],
	# 							quat_r.as_quat()[3]
	# 						]])), axis=0)
	# 			np.save('path', save_quat)
		
	
	def norm2euler(self, x, y, z):
		x_euler = 0
		z_euler = 0
		length = np.sqrt(x ** 2 + y ** 2)
		if ((x > 0) and (y > 0) and (z > 0)):
			x_euler = np.arctan(y / x)
			z_euler = -np.arctan(z / length)
		if ((x < 0) and (y < 0) and (z < 0)):
			x_euler = np.pi + np.arctan(-y / -x)
			z_euler = np.arctan(z / length)

		if ((x < 0) and (y > 0) and (z < 0)):
			x_euler = np.pi - np.arctan(y / -x)
			z_euler = np.arctan(z / length)
		if ((x > 0) and (y < 0) and (z > 0)):
			x_euler = -np.arctan(-y / x)
			z_euler = -np.arctan(z / length)

		if ((x > 0) and (y > 0) and (z < 0)):
			x_euler = np.arctan(y / x)
			z_euler = -np.arctan(z / length)
		if ((x < 0) and (y < 0) and (z > 0)):
			x_euler = np.pi + np.arctan(-y / -x)
			z_euler = np.arctan(z / length)

		if ((x < 0) and (y > 0) and (z > 0)):
			x_euler = np.pi / 2 + np.arctan(-x / y)
			z_euler = np.arctan(z / length)
		if ((x > 0) and (y < 0) and (z < 0)):
			x_euler = -np.arctan(-y / x)
			z_euler = np.arctan(-z / length)

		return x_euler, z_euler


if __name__ == "__main__":
	raw_data_path = "../../../data/stl2.ply"
	# parameter to adjust
	max_angle = 60  # change  the param within [0,90)
	file_path = "D:/CMU2020FALL/HapticSurgery/Medical-DVRK-AR/data/"
	sorted_file_name = "dense_outward_normals_cleanForPlanning_60.npy"

	my_filter = filter_pointcloud_for_path_planner(raw_data_path)
	my_filter.downsample_raw_pcl_get_normal_vector()
	my_filter.filter_vector_with_angle_threshold(max_angle)
	sorted_pcl_with_filtered_norm = my_filter.sorted_poinst_with_xy_position()

	np.save(file_path+sorted_file_name, sorted_pcl_with_filtered_norm)
