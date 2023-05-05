import numpy as np
import rospy

from nav_msgs.msg import Odometry

def rot_x (theta):
   return np.array([[1, 0 , 0],[0, np.cos(theta) , -np.sin(theta)],[0, np.sin(theta) , np.cos(theta)]])

def rot_y (theta):
   return np.array([[np.cos(theta), 0 , np.sin(theta)],[0, 1 , 0],[-np.sin(theta), 0 , np.cos(theta)]])

def rot_z (theta):
   return np.array([[np.cos(theta),- np.sin(theta), 0],[np.sin(theta), np.cos(theta),0],[0,0,1]])

def map_to_ros_frame (coord):
   return np.matmul(coord,np.matmul(rot_z(np.pi/2),rot_y(np.pi)))

def ros_to_map_frame (coord):
   return np.matmul(coord,np.matmul(rot_z(-np.pi/2),rot_x(np.pi)))

if __name__ == '__main__':
    #    rospy.init_node('ros_to_gps_node')
    #    sub = rospy.Subscriber("/lio_sam/mapping/odometry", Odometry, queue_size=0)
    zoomlevel = 19
    latitude = 13.02631 * np.pi/180
    #    resolution = 156543.03 * np.cos(latitude) / (2 ^ zoomlevel)
    resolution = 0.299 * np.cos(latitude) # According to https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Resolution_and_Scale
    # distance = resolution * pixels
    distance = 40
    pixels = (distance) / resolution

    print(pixels)

    start = np.array([1 ,1, 0])
    end = (np.matmul(start,np.matmul(rot_z(np.pi/2),rot_y(np.pi))))

    print(ros_to_map_frame(end))