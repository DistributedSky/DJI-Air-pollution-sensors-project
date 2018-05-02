#! /usr/bin/python

from time import sleep
import rospy
import roslib
import dji_sdk.srv
import dji_sdk.msg
from dji_sdk.srv import DroneTaskControlRequest
from std_msgs.msg import UInt8

# status_start = False

# def callback(data):
#     status_int = data.data 
    # if data.data == 3 and status_start == False:
        # status_start = True
        # print (status_start)
    # print('Status {0}'.format(data.data))
    # rospy.sleep(2)

def waypoints_creat ():
    param = rospy.get_param('/de_airsense_mission/waypoints')
    print 'Waypoints from parameter server: '
    print param
    newWaypointList = []
    for index in range(len(param)):
        newWaypointList.append(dji_sdk.msg.MissionWaypoint( latitude = param[index]['lat'], 
                                                            longitude = param[index]['lon'], 
                                                            altitude = param[index]['alt'], 
                                                            damping_distance = 2, 
                                                            target_yaw = 0, 
                                                            has_action = 0, 
                                                            target_gimbal_pitch = 0, 
                                                            turn_mode = 0, 
                                                            action_time_limit = 10))
    return newWaypointList

def mission_start():
    try:
        auth = rospy.ServiceProxy('dji_sdk/sdk_control_authority', dji_sdk.srv.SDKControlAuthority)
        resp = auth(1)
        print 'Service. Sdk control authority:'
        print resp.result

    except rospy.ServiceException, e:
        print e

    # sleep(1)
       
    try:
        mission = rospy.ServiceProxy('dji_sdk/mission_waypoint_upload', dji_sdk.srv.MissionWpUpload)
        mission_msg = dji_sdk.msg.MissionWaypointTask()
        mission_msg.velocity_range     = 10;
        mission_msg.idle_velocity      = 5;
        mission_msg.action_on_finish   = dji_sdk.msg.MissionWaypointTask.FINISH_RETURN_TO_HOME;
        mission_msg.mission_exec_times = 1;
        mission_msg.yaw_mode           = dji_sdk.msg.MissionWaypointTask.YAW_MODE_AUTO;
        mission_msg.trace_mode         = dji_sdk.msg.MissionWaypointTask.TRACE_POINT;
        mission_msg.action_on_rc_lost  = dji_sdk.msg.MissionWaypointTask.ACTION_AUTO;
        mission_msg.gimbal_pitch_mode  = dji_sdk.msg.MissionWaypointTask.GIMBAL_PITCH_FREE;

        # newWaypointList = [
        #         dji_sdk.msg.MissionWaypoint(latitude = 60.008217, longitude = 30.320849, altitude = 8, damping_distance = 0, target_yaw = 0, has_action = 0, target_gimbal_pitch = 0, turn_mode = 0, action_time_limit = 1),
        #         dji_sdk.msg.MissionWaypoint(latitude = 60.007914, longitude = 30.320765, altitude = 8, damping_distance = 0, target_yaw = 0, has_action = 0, target_gimbal_pitch = 0, turn_mode = 0, action_time_limit = 1)
        #         ]
        mission_msg.mission_waypoint = waypoints_creat()

        # print mission_msg
        resp = mission(mission_msg)
        print 'Service. Mission waypoint upload:'
        print resp.result

    except rospy.ServiceException, e:
        print e
    
    # try:
    #     arm = rospy.ServiceProxy('dji_sdk/drone_arm_control', dji_sdk.srv.DroneArmControl)
    #     resp = arm(1)
    #     print 'drone_arm_control'
    #     print resp.result

    # except rospy.ServiceException, e:
    #     print e
    
    # try:
    #     task = rospy.ServiceProxy('dji_sdk/drone_task_control', dji_sdk.srv.DroneTaskControl)
    #     resp = task(DroneTaskControlRequest.TASK_TAKEOFF)
    #     print 'drone_task_control'
    #     print resp.result

    # except rospy.ServiceException, e:
    #     print e
 
    try:
        start = rospy.ServiceProxy('dji_sdk/mission_waypoint_action', dji_sdk.srv.MissionWpAction)
        resp = start(0)
        print 'Service. Mission waypoint action:'
        print resp.result

    except rospy.ServiceException, e:
        print e


if __name__ == '__main__':
    rospy.init_node('de_airsense_mission')
    print 'Waiting dji_sdk for services...'
    # rospy.Subscriber('dji_sdk/flight_status', UInt8, callback)
    rospy.wait_for_service('dji_sdk/sdk_control_authority')
    # rospy.wait_for_service('dji_sdk/drone_arm_control')
    # rospy.wait_for_service('dji_sdk/drone_task_control')
    rospy.wait_for_service('dji_sdk/mission_waypoint_action')
    rospy.wait_for_service('dji_sdk/mission_waypoint_upload')
    
    mission_start()

    rospy.spin()

