#!usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import requests
import math
import cv2
import rospy
import tf2_ros
import sys

from io import BytesIO
from PIL import Image
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import TransformStamped,Vector3,Quaternion

def scale(A, B, k):     # fill A with B scaled by k
    Y = A.shape[0]
    X = A.shape[1]
    for y in range(0, k):
        for x in range(0, k):
            A[y:Y:k, x:X:k] = B


def numpy_to_occupancy_grid(arr, info=None):
        if not len(arr.shape) == 2:
                raise TypeError('Array must be 2D')
        if not arr.dtype == np.int8:
                raise TypeError('Array must be of int8s')

        grid = OccupancyGrid()
        if isinstance(arr, np.ma.MaskedArray):
                # We assume that the masked value are already -1, for speed
                arr = arr.data
        grid.data = arr.ravel()
        grid.info = info or MapMetaData()
        grid.info.height = arr.shape[0]
        grid.info.width = arr.shape[1]

        return grid

def compute_dist_constant(zoom_level = 19,latitude = 13.02631):
    latitude_in_rad = latitude * np.pi / 180
    dconst = 40075016.686/(2**(zoom_level+8))* np.cos(latitude_in_rad) # According to https://wiki.openstreetmap.org/wiki/Zoom_levels
    return dconst

def tile_meter_to_pixel(meter):
    resolution = 0.2908986275853943 # According to https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Resolution_and_Scale
    # resolution = 0.3 # According to https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Resolution_and_Scale

    pixel = math.floor(meter / resolution)
    return pixel

def tile_meter_to_pixel_2(meter):
    return meter


def move_origin(px,py,orx,ory):
    return px + orx, py + ory

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

def hddeg2num(lat_deg, lon_deg, zoom):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = (lon_deg + 180.0) / 360.0 * n
    ytile = (1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n
    return (xtile, ytile)

def deg2num(lat_deg, lon_deg, zoom):
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
    return (xtile, ytile)
  
def num2deg(xtile, ytile, zoom):
    n = 2.0 ** zoom
    lon_deg = xtile / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
    lat_deg = math.degrees(lat_rad)
    return (lat_deg, lon_deg)  
    
def getImageCluster(lat_deg, lon_deg, delta_lat,  delta_long, zoom):
    # smurl = r"http://a.tile.openstreetmap.org/{0}/{1}/{2}.png"
    smurl = r"http://localhost:8080/wmts/gm_layer/gm_grid/{0}/{1}/{2}.png"
    xact, yact = hddeg2num(lat_deg, lon_deg, zoom)
    xmin, ymax = deg2num(lat_deg - delta_lat, lon_deg - delta_long, zoom)
    xcur, ycur = deg2num(lat_deg, lon_deg, zoom)
    xmax, ymin = deg2num(lat_deg + delta_lat, lon_deg + delta_long, zoom)

    xmin = xcur - 2
    ymin = ycur - 2

    xmax = xcur + 2
    ymax = ycur + 2 


    origin_x, origin_y =  int((xact%1)*256) , int((yact%1)*256) 

    print((ycur-ymin)*256)
    origin_x = ((xcur-xmin)*256) + origin_x
    origin_y = 256*(ymax-ymin+1) - (((ycur-ymin)*256) + origin_y)

    Cluster = Image.new('RGB',((xmax-xmin+ 1)*256,(ymax-ymin+ 1)*256) ) 
    for xtile in range(xmin, xmax+ 1):
        for ytile in range(ymin,  ymax+ 1):
            try:
                imgurl=smurl.format(zoom, xtile, ytile)
                # print("Opening: " + imgurl)
                imgstr = requests.get(imgurl)
                tile = Image.open(BytesIO(imgstr.content))
                Cluster.paste(tile, box=((xtile-xmin)*256 ,  (ytile-ymin)*256))
            except Exception as e: 
                print(e)
                print("Couldn't download image")
                tile = None
    plt.imshow(Cluster)
    plt.show()
    return Cluster, origin_x, origin_y

def createImageMask(img):
    color1 = np.asarray([0, 0, 255])   # LL
    color2 = np.asarray([255, 255, 255])   # UL
    SCALE_CONST = (2048/img.shape[0]) 
    mask = cv2.inRange(img, color1, color2)
    mask = cv2.resize(mask.astype(np.uint8), dsize=(2048, 2048), interpolation=cv2.INTER_LINEAR)
    mask = cv2.medianBlur(mask,11)
    # mask = cv2.medianBlur(mask,81)
    processed_mask = process_mask(mask)
    return processed_mask, SCALE_CONST

def process_mask(masked):
    # np.set_printoptions(threshold=sys.maxsize)
    processed_mask = np.flip(masked,0)
    processed_mask = 100 * (processed_mask<200).astype(np.int8)
    return processed_mask.astype(np.int8)


class GPS_PF_ROS:


    def __init__(self) -> None:
        # Aerospace Bridge Corner 13.02631, 77.56317
        # init_gps_lat =  13.02631 #13.0244987 #13.02435  #13.0244987 
        # init_gps_long = 77.56317 #77.564222  #77.56332  #77.5647412 

        # Aerospace Plane Corner 13.02596, 77.5639
        # init_gps_lat  = 13.02596
        # init_gps_long = 77.5639
        
        # # RV Civil 
        init_gps_lat  = 12.92441 
        init_gps_long = 77.49918
        
        a, self.cx, self.cy = getImageCluster(init_gps_lat,init_gps_long, 0.0015,  0.0015, 19)
        self.mask = np.zeros((5110,5110),dtype=np.int8)

        self.mask, SCALE_CONST = createImageMask(np.asarray(a))

        # scale(self.mask,mini_mask,10)
        
        # self.sub = rospy.Subscriber('/lio_sam/mapping/odometry',Odometry,self.odom_cb)
        self.pub = rospy.Publisher('map',OccupancyGrid,queue_size=1)

        self.mapinfo = MapMetaData()
        self.mapinfo.resolution =  0.2908986275853943/SCALE_CONST                    # 0.3 from dist const and 1/4 from rescaling
        self.mapinfo.origin.position.x = -int(self.cx* 0.2908986275853943)           # 0.3 is based on compute dist constant function According to https://wiki.openstreetmap.org/wiki/Zoom_levels
        self.mapinfo.origin.position.y = -int(self.cy* 0.2908986275853943) # 0.3 is based on compute dist constant function According to https://wiki.openstreetmap.org/wiki/Zoom_levels
        
        self.loop_rate = rospy.Rate(1)

        self.br = tf2_ros.TransformBroadcaster()

        self.trans = Vector3()
        self.rot = Quaternion()

        self.pub_maps()
     
    
    def pub_maps(self):
        while not rospy.is_shutdown():
            message = numpy_to_occupancy_grid(self.mask,self.mapinfo)
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = 'map'
            self.pub.publish(message)
            self.loop_rate.sleep()
    
if __name__ == '__main__':
    rospy.init_node('tilemap_pf',anonymous=True)
    gps_pf = GPS_PF_ROS()
    