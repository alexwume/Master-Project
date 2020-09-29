#system
import rospy
import argparse

#data processing
import numpy as np
import math
import struct

#vector visualizer
import rospy
from geometry_msgs.msg import Pose, PoseArray


# point cloud
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2

from std_msgs.msg import Header

# rotation
from scipy.spatial.transform import Rotation as R



class liverGrid:
    def __init__(self):
        #ros node
        rospy.init_node('liverGrid', anonymous=True)

        # publisher
        self.pub1 = rospy.Publisher("liverGrid", PointCloud2, queue_size=2)
        self.pub2 = rospy.Publisher("liverGridNorm", PoseArray, queue_size=2)
        self.rate = rospy.Rate(10)

        # point cloud2
        self.point_cloud = []
        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),PointField('y', 4, PointField.FLOAT32, 1),PointField('z', 8, PointField.FLOAT32, 1), PointField('rgba', 12, PointField.UINT32, 1)]
        self.header = Header()
        self.header.frame_id = "PSM1_psm_base_link" # the 3dpcl is in a new frame
        self.pc2 = point_cloud2.create_cloud(self.header, self.fields, self.point_cloud)
        self.point_nparray = np.array([])

        self.normal_vectors = PoseArray()
        self.normal_vectors.header = self.header
        
        

    def convert_array_to_pointcloud2(self):
        """
        param: 3 by N array
        return: pointcloud2
        """
        for i in range(self.point_nparray.shape[0]):
            color = np.int(np.floor(np.float(i)/self.point_nparray.shape[0] * 255))
            rgb = self.compressRGBA(color, color, color)
            self.point_cloud.append([self.point_nparray[i,0], self.point_nparray[i,1], self.point_nparray[i,2], rgb])

        self.pc2 = point_cloud2.create_cloud(self.header, self.fields, self.point_cloud)

    def publish_pointcloud(self):
        while not rospy.is_shutdown():
            self.pc2.header.stamp = rospy.Time.now()
            self.pub1.publish(self.pc2)
            self.normal_vectors.header.stamp = rospy.Time.now()
            self.pub2.publish(self.normal_vectors)
            self.rate.sleep()

    def compressRGBA(self,r,g,b,a=255):
        return struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]

    def readArrayfromFile(self, path):
        """
        path: string, path of the npy file
        scale: float,scale the python file
        rotateAxis: string,'x' / 'y' / 'z'
        rotateDegree: rotation in degree, 0-360 
        """

        self.point_nparray  = np.load(path) # N by 3 matrix
        self.point_nparray = np.transpose(self.point_nparray) # 3 by N matrix
        
        # self.point_nparray = np.array([
        #     [0,0,0,3,1,10],
        #     [0,0,0,-3,-1,-10],
        #     [0,0,0,3,-1,10],
        #     [0,0,0,-3,1,-10],
        #     [0,0,0,-3,-1,10],
        #     [0,0,0,3,1,-10],
        #     [0,0,0,-3,1,10],
        #     [0,0,0,3,-1,-10],
        # ]).T
        for i in range(self.point_nparray.shape[1]):
            normal_vector = Pose()
            normal_vector.position.x = self.point_nparray[0,i]
            normal_vector.position.y = self.point_nparray[1,i]
            normal_vector.position.z = self.point_nparray[2,i]
            normal_vector.position.x = self.point_nparray[0,i]
            normal_vector.position.y = self.point_nparray[1,i]
            normal_vector.position.z = self.point_nparray[2,i]
            
            norm_x=-self.point_nparray[3,i]
            norm_y=-self.point_nparray[4,i]
            norm_z=-self.point_nparray[5,i]

            x_euler,y_euler,z_euler = self.norm2euler_xyz(norm_x,norm_y,norm_z)
            r_euler = R.from_euler('xyz',[x_euler,y_euler,z_euler],degrees=False)
            r_quat = r_euler.as_quat()

            normal_vector.orientation.x = r_quat[0]
            normal_vector.orientation.y = r_quat[1]
            normal_vector.orientation.z = r_quat[2]
            normal_vector.orientation.w = r_quat[3]

            self.normal_vectors.poses.append(normal_vector)

        # self.save_norm_all_down() 
        self.point_nparray = self.point_nparray[0:3,:].T
    
    def save_norm_all_down(self):
        save_numpy = np.empty((0,7))
        
        for i in range(self.point_nparray.shape[1]):
            normal_vector = Pose()

            normal_vector.position.x = self.point_nparray[0,i]
            normal_vector.position.y = self.point_nparray[1,i]
            normal_vector.position.z = self.point_nparray[2,i]
            
            x_euler,y_euler,z_euler = math.pi,0,0
            r_euler = R.from_euler('xyz',[x_euler,y_euler,z_euler],degrees=False)
            r_quat = r_euler.as_quat()

            save_numpy_row = np.zeros((1,7))

            save_numpy_row[0,0] = self.point_nparray[0,i]
            save_numpy_row[0,1] = self.point_nparray[1,i]
            save_numpy_row[0,2] = self.point_nparray[2,i]
            save_numpy_row[0,3] = r_quat[0]
            save_numpy_row[0,4] = r_quat[1]
            save_numpy_row[0,5] = r_quat[2]
            save_numpy_row[0,6] = r_quat[3]        

            print(save_numpy.shape, save_numpy_row.shape)
            save_numpy = np.concatenate((save_numpy, save_numpy_row),axis=0)

        np.save('../../../data/Alex_norm_all_downward.npy',save_numpy)
        print(save_numpy.shape) 

    def norm2euler_xyz(self,x,y,z):
        x_euler=0
        y_euler=0
        length = np.sqrt(x**2+y**2+z**2)
        length_cos = np.sqrt(y**2+z**2)
        if((x>0)and(y>0)and(z>0)):
            x_euler = -np.arctan(y/z)
            y_euler = np.arccos(length_cos/length)
        if((x<0)and(y<0)and(z<0)):
            x_euler = np.pi + np.arctan(y/-z)
            y_euler = np.arccos(length_cos/length)

        if((x>0)and(y<0)and(z>0)):
            x_euler = np.arctan(-y/z)
            y_euler = np.arccos(length_cos/length)
        if((x<0)and(y>0)and(z<0)):
            x_euler = np.pi+np.arctan(y/-z)
            y_euler = np.arccos(length_cos/length)

        if((x<0)and(y<0)and(z>0)):
            x_euler = np.arctan(-y/z)
            y_euler = -np.arccos(length_cos/length)
        if((x>0)and(y>0)and(z<0)):
            x_euler = np.pi+np.arctan(y/-z)
            y_euler = -np.arccos(length_cos/length)

        if((x<0)and(y>0)and(z>0)):
            x_euler = -np.arctan(y/z)
            y_euler = -np.arccos(length_cos/length)
        if((x>0)and(y<0)and(z<0)):
            x_euler = np.pi-np.arctan(-y/-z)
            y_euler = -np.arccos(length_cos/length)

        return x_euler,y_euler,0

    def norm2euler(self, x, y, z):
        x_euler=0
        z_euler=0
        length = np.sqrt(x**2+y**2)
        if((x>0)and(y>0)and(z>0)):
            x_euler = np.arctan(y/x)
            z_euler = -np.arctan(z/length)
        # if((x<0)and(y<0)and(z<0)):
        #     x_euler = np.pi+np.arctan(-y/-x)
        #     z_euler = np.arctan(z/length)
        
        # if((x<0)and(y>0)and(z<0)):
        #     x_euler = np.pi-np.arctan(y/-x)
        #     z_euler = np.arctan(z/length)
        # if((x>0)and(y<0)and(z>0)):
        #     x_euler = -np.arctan(-y/x)
        #     z_euler = -np.arctan(z/length)
        
        # if((x>0)and(y>0)and(z<0)):
        #     x_euler = np.arctan(y/x)
        #     z_euler = -np.arctan(z/length)
        # if((x<0)and(y<0)and(z>0)):
        #     x_euler = np.pi+np.arctan(-y/-x)
        #     z_euler = np.arctan(z/length)

        # if((x<0)and(y>0)and(z>0)):
        #     x_euler = np.pi/2+np.arctan(-x/y)
        #     z_euler = np.arctan(z/length)
        # if((x>0)and(y<0)and(z<0)):
        #     x_euler = -np.arctan(-y/x)
        #     z_euler = np.arctan(-z/length)

        return x_euler, z_euler

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--path',help='the path to the position&quat npy file')
    args = parser.parse_args()

    liverGrid = liverGrid()
    a = liverGrid.readArrayfromFile(args.path)
    liverGrid.convert_array_to_pointcloud2()
    liverGrid.publish_pointcloud()
