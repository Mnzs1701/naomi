#!usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import requests
import math
import cv2
import rospy

from io import BytesIO
from PIL import Image
from nav_msgs.msg import Odometry

def compute_dist_constant(zoom_level = 19,latitude = 13.02631):
    latitude_in_rad = latitude * np.pi / 180
    dconst = 40075016.686/(2**(zoom_level+8))* np.cos(latitude_in_rad) # According to https://wiki.openstreetmap.org/wiki/Zoom_levels
    return dconst

def tile_meter_to_pixel(meter):
    # resolution = 0.2908986275853943 # According to https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Resolution_and_Scale
    resolution = 0.3 # According to https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Resolution_and_Scale

    pixel = math.floor(meter / resolution)
    return pixel

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
    xmin, ymax = deg2num(lat_deg, lon_deg, zoom)
    xmax, ymin = deg2num(lat_deg + delta_lat, lon_deg + delta_long, zoom)

    origin_x, origin_y = int((xact%1)*256) ,255+ int((yact%1)*256) 

    Cluster = Image.new('RGB',((xmax-xmin+1)*256-1,(ymax-ymin+1)*256-1) ) 
    for xtile in range(xmin, xmax+1):
        for ytile in range(ymin,  ymax+1):
            try:
                print(xtile,ytile)
                imgurl=smurl.format(zoom, xtile, ytile)
                print("Opening: " + imgurl)
                imgstr = requests.get(imgurl)
                tile = Image.open(BytesIO(imgstr.content))
                Cluster.paste(tile, box=((xtile-xmin)*256 ,  (ytile-ymin)*255))
            except Exception as e: 
                print(e)
                print("Couldn't download image")
                tile = None

    return Cluster, origin_x, origin_y

def createImageMask(img):
    color1 = np.asarray([0, 0, 255])   # LL
    color2 = np.asarray([255, 255, 255])   # UL

    mask = cv2.inRange(img, color1, color2)
    print(mask)
    return cv2.cvtColor(mask, cv2.COLOR_BGR2RGB)

def odom_cb(data):
    global cx
    global cy

    ros_x = data.pose.pose.position.x
    ros_y = data.pose.pose.position.y

    map_x,map_y = ros_x, -ros_y #ros_to_map_frame(np.array([ros_x,ros_y,0]))
    px_x, px_y = tile_meter_to_pixel(map_x),tile_meter_to_pixel(map_y) 
    moved_px, moved_py = move_origin(px_x ,px_y,cx,cy)
    mask[moved_py-2:moved_py+2,moved_px-2:moved_px + 2] = (0,0,255)
    plt.imshow(mask)
    plt.pause(0.05)



if __name__ == '__main__':
    global cx
    global cy

    rospy.init_node('tilemap_pf',anonymous=True)
    # Aerospace Bridge Corner 13.02631, 77.56317
    init_gps_lat = 13.02631
    init_gps_long = 77.56317

    # Aerospace Plane Corner 13.02596, 77.5639
    # init_gps_lat = 13.02596
    # init_gps_long = 77.5639

    a, cx, cy = getImageCluster(init_gps_lat,init_gps_long, 0.0005,  0.0005, 19)
    fig = plt.figure()
    fig.patch.set_facecolor('white')
    mask = createImageMask(np.asarray(a))
    sub = rospy.Subscriber('/lio_sam/mapping/odometry',Odometry,odom_cb)
    print(cx,cy)
    mask[cy:cy+10,cx:cx+10] = (255,0,0)
    mask[cy:cy+10,cx:cx+10] = (0,255,0)
    plt.figure(figsize = (256,256))

    plt.imshow(mask)
    # plt.imshow(a)
    plt.show()
    