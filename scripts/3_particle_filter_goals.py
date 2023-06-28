#!/usr/bin/python3

import urllib3
import json

import rospy
import math
import actionlib
import tf
# import matplotlib.pyplot as plt

from geographiclib.geodesic import Geodesic
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from pyroutelib3 import Router # Import the router
from visualization_msgs.msg import Marker


def translate_xy(origin,point):
    ox, oy = origin
    px, py = point

    px = px + ox
    py = py + oy

    return px,py


def rotate_xy(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

def DMS_to_decimal_format(lat,long):
  # Check for degrees, minutes, seconds format and convert to decimal
  if ',' in lat:
    degrees, minutes, seconds = lat.split(',')
    degrees, minutes, seconds = float(degrees), float(minutes), float(seconds)
    if lat[0] == '-': # check for negative sign
      minutes = -minutes
      seconds = -seconds
    lat = degrees + minutes/60 + seconds/3600
  if ',' in long:
    degrees, minutes, seconds = long.split(',')
    degrees, minutes, seconds = float(degrees), float(minutes), float(seconds)
    if long[0] == '-': # check for negative sign
      minutes = -minutes
      seconds = -seconds
    long = degrees + minutes/60 + seconds/3600

  lat = float(lat)
  long = float(long)
  rospy.loginfo('Given GPS goal: lat %s, long %s.' % (lat, long))
  return lat, long


def get_compass_heading():
  rospy.loginfo("Waiting for a message to initialize the origin compass direction..")
  # compass_from_imu = rospy.wait_for_message('imu/compass_heading', Float32)
  compass_from_imu = 0
  rospy.loginfo('Received origin: compass %s' % (compass_from_imu))
  return compass_from_imu

def get_origin_lat_long():
  # Get the lat long coordinates of our map frame's origin which must be publshed on topic /local_xy_origin. We use this to calculate our goal within the map frame.
  rospy.loginfo("Waiting for a message to initialize the origin GPS location...")
  # origin_pose = rospy.wait_for_message('gnss', NavSatFix)
  # origin_lat = origin_pose.latitude #origin_pose.pose.position.y
  # origin_long = origin_pose.longitude  #origin_pose.pose.position.x
  origin_lat = 13.02631  #13.0270060 #origin_pose.pose.position.y
  origin_long = 77.56318 #77.5634649  #origin_pose.pose.position.x
  rospy.loginfo('Received origin: lat %s, long %s.' % (origin_lat, origin_long))
  return origin_lat, origin_long

def get_origin_xy():
  # Get the lat long coordinates of our map frame's origin which must be publshed on topic /local_xy_origin. We use this to calculate our goal within the map frame.
  rospy.loginfo("Waiting for a message to initialize the origin map location...")
  origin_pose = rospy.wait_for_message('lio_sam/mapping/odometry', Odometry)
  x = origin_pose.pose.pose.position.x
  y = origin_pose.pose.pose.position.y
  rospy.loginfo('Received origin: x %s, y %s.' % (x, y))
  return x, y

def rotate_xy(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

def calc_goal(origin_lat, origin_long, goal_lat, goal_long):
  # Calculate distance and azimuth between GPS points
  geod = Geodesic.WGS84  # define the WGS84 ellipsoid
  g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
  hypotenuse = distance = g['s12'] # access distance
  rospy.loginfo("The distance from the origin to the goal is {:.3f} m.".format(distance))
  azimuth = g['azi1']
  rospy.loginfo("The azimuth from the origin to the goal is {:.3f} degrees.".format(azimuth))

  # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
  # Convert azimuth to radians
  azimuth = math.radians(azimuth)
  x = adjacent = math.cos(azimuth) * hypotenuse
  y = opposite = math.sin(azimuth) * hypotenuse
  rospy.loginfo("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))

  return x, y

class GpsGoal():
  def __init__(self):
    rospy.init_node('gps_goal')

    rospy.loginfo("Connecting to move_base...")
    self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.move_base.wait_for_server()
    rospy.loginfo("Connected.")

    self.originlat, self.originlong = get_origin_lat_long()
    self.originx, self.originy = get_origin_xy()
    self.originheading =  get_compass_heading()

    self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

    rospy.Subscriber('gps_goal_pose', PoseStamped, self.gps_goal_pose_callback)
    rospy.Subscriber('gnss', NavSatFix, self.gps_goal_fix_callback)
    rospy.Subscriber('lio_sam/mapping/odometry', Odometry, self.odometry_callback)

    # Start a pyroute3 instance
    self.router = Router("car","IISc_Map.osm") # Vehicle type and map 

    # Start a tf listener
    self.tf_listener_ = tf.TransformListener()



  def do_gps_goal(self, goal_lat=13.02629989, goal_long=77.56317598, z=0, yaw=0, roll=0, pitch=0):
    # Calculate goal x and y in the frame_id given the frame's origin GPS and a goal GPS location

    start = self.router.findNode(self.originlat, self.originlong) 
    end = self.router.findNode(goal_lat, goal_long)
    self.planning_status, self.route = self.router.doRoute(start, end)

    if self.planning_status == 'success':
        planned_waypoints = []

        rospy.loginfo("Planned route successfully...Starting Execution...")
        routeLatLons = list(map(self.router.nodeLatLon, self.route)) # Get actual route coordinates
        last_point = (self.originlat,self.originlong) #routeLatLons[0]
        for point in routeLatLons: 
            map_x , map_y = calc_goal(last_point[0],last_point[1],point[0],point[1])
            rospy.logwarn("Going to position %d %d on base_link frame",map_x,map_y)

            self.tf_listener_.waitForTransform("base_link", "dynamic_z_map", rospy.Time(), rospy.Duration(4.0))
            t = self.tf_listener_.getLatestCommonTime("base_link", "dynamic_z_map")
            # position, quaternion = self.tf_listener_.lookupTransform("base_link", "map", t)

            p1 = PointStamped()
            p1.header.frame_id = "base_link"
            p1.point.x = map_x
            p1.point.y = -map_y
            p1.point.z = 0
            p_in_odom = self.tf_listener_.transformPoint("dynamic_z_map", p1)
            planned_waypoints.append((p_in_odom.point.x,p_in_odom.point.y))
            rospy.logwarn("Going to position %d %d on map frame",p_in_odom.point.x,p_in_odom.point.y)
        
        for (p_x,p_y) in planned_waypoints:
            self.publish_goal(p_x, p_y, z=z, yaw=yaw, roll=roll, pitch=pitch)


  def gps_goal_pose_callback(self, data):
    lat = data.pose.position.y
    long = data.pose.position.x
    z = data.pose.position.z
    euler = tf.transformations.euler_from_quaternion(data.pose.orientation)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    self.do_gps_goal(lat, long, z=z, yaw=yaw, roll=roll, pitch=pitch)

  def gps_goal_fix_callback(self, data):
    self.currentlat = data.latitude
    self.currentlong = data.longitude

  def odometry_callback(self, data):
    self.currrentx = data.pose.pose.position.x
    self.currrenty = data.pose.pose.position.y

  def publish_marker(self, x, y, z, quat, frame):
    marker = Marker()

    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 1
    marker.id = 0

    # Set the scale of the marker
    marker.scale.x = 2.0
    marker.scale.y = 2.0
    marker.scale.z = 2.0

    # Set the color
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Set the pose of the marker
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]

    self.marker_pub.publish(marker)

  def publish_goal(self, x=0, y=0, z=0, yaw=0, roll=0, pitch=0, frame="odom"):
    # Create move_base goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z =  z
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    rospy.loginfo('Executing move_base goal to position (x,y) %s, %s, with %s degrees yaw.' %
            (x, y, yaw))
    rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")

    self.publish_marker(x,y,z,quaternion,frame)
    
    # Send goal
    self.move_base.send_goal(goal)
    rospy.loginfo('Inital goal status: %s' % GoalStatus.to_string(self.move_base.get_state()))
    status = self.move_base.get_goal_status_text()
    if status:
      rospy.loginfo(status)
    self.move_base.wait_for_result()
    rospy.loginfo('Final goal status: %s' % GoalStatus.to_string(self.move_base.get_state()))
    status = self.move_base.get_goal_status_text()
    if status:
      rospy.loginfo(status)


def ros_main():

    input("Place the robot facing north exactly")
    print("Waiting for Goal")

    # http = urllib3.PoolManager()
    # r = http.request('GET', 'https://api.thingspeak.com/channels/1949145/feeds.json?api_key=YLR6N30DHH1P6PI5&results=2')
    # data_recv = json.loads(r.data)
    # entry_id = (data_recv['channel']['last_entry_id'])
    # feeds = data_recv['feeds']
    # for element in feeds:
    #     if element['entry_id'] == entry_id:
    #         app_longitude = float(element['field1'])
    #         app_latitude = float(element['field2'])
    app_latitude = float(13.02448)
    app_longitude = float(77.56417)
        
    gpsGoal = GpsGoal();
    gpsGoal.do_gps_goal(app_latitude,app_longitude)
    rospy.spin()


if __name__ == '__main__':
    ros_main()
    