#! /usr/bin/python

from time import sleep
import rospy
from mavros_msgs.msg import Waypoint, WaypointList
from mavros_msgs.srv import *
from sensor_msgs.msg import NavSatFix

MAV_GLOBAL_FRAME = 3
MAV_CMD_WAYPOINT = 16
MAV_CMD_RTL = 20
MAV_CMD_LAND = 21
MAV_CMD_TAKEOFF = 22

lat = 0.0
lon = 0.0

def push_mission(waypoints):
    rospy.wait_for_service('mavros/mission/clear')
    try:
        service = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
        service()
    except rospy.ServiceException, e:
        print('Service call failed: {0}'.format(e))
    rospy.wait_for_service('mavros/mission/push')
    try:
        service = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
        service(0, waypoints)
    except rospy.ServiceException, e:
        print('Service call failed: {0}'.format(e))

def set_mode(mode):
    rospy.wait_for_service('mavros/set_mode')
    try:
        service = rospy.ServiceProxy('mavros/set_mode', SetMode)
        service(0, mode)
    except rospy.ServiceException, e:
        print('Service call failed: {0}'.format(e))

def arming():
    rospy.wait_for_service('mavros/cmd/arming')
    try:
        service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        service(True)
    except rospy.ServiceException, e:
        print('Service call failed: {0}'.format(e))

def mission_create ():
    param = rospy.get_param('/waypoints')
    print 'Waypoints from parameter server: '
    print param

    wl = WaypointList()
    wp = Waypoint()
    wp.frame = MAV_GLOBAL_FRAME
    wp.command = MAV_CMD_TAKEOFF  # takeoff
    wp.is_current = True
    wp.autocontinue = True
    wp.param1 = param[0]['alt']  # takeoff altitude
    wp.param2 = 0
    wp.param3 = 0
    wp.param4 = 0
    wp.x_lat = lat
    wp.y_long = lon
    wp.z_alt = param[0]['alt']
    wl.waypoints.append(wp)

    for index in range(len(param)):
        wp = Waypoint()
        wp.frame = MAV_GLOBAL_FRAME
        wp.command = MAV_CMD_WAYPOINT  # simple point
        wp.is_current = False
        wp.autocontinue = True
        wp.param1 = 0  # takeoff altitude
        wp.param2 = 0
        wp.param3 = 0
        wp.param4 = 0

        wp.x_lat = param[index]['lat']
        wp.y_long = param[index]['lon']
        wp.z_alt = param[index]['alt']
        wl.waypoints.append(wp)

    wp = Waypoint()
    wp.frame = 2 
    wp.command = MAV_CMD_RTL
    wp.is_current = False
    wp.autocontinue = True
    wl.waypoints.append(wp)

    push_mission(wl.waypoints)

def mission_start ():
    # Set manual mode
    set_mode('ACRO')
    # Enable motors 
    arming()
    # Set autopilot mode
    # set_mode('AUTO.TAKEOFF')
    sleep(3)
    set_mode('AUTO.MISSION')

def gps_cb(data): 
    global lat
    global lon

    lat = data.latitude
    lon = data.longitude
    # print "{0:.2f} {1:.2f}".format(lat, lon)

if __name__ == '__main__':
    rospy.init_node('de_px4_mission')
    print 'Waiting mavros for services...'
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_cb)
    sleep(2)
    mission_create()
    print('Mission created')
    mission_start()
    print('Flight!')

    rospy.spin()
