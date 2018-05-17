#! /usr/bin/python

from time import sleep
import rospy
import dji_sdk.srv
import dji_sdk.msg
from dji_sdk.srv import DroneTaskControlRequest

def waypoints_create ():
    param = rospy.get_param('/de_airsense_mission/waypoints')
    print 'Waypoints from parameter server: '
    print param
    newWaypointList = []
    # cmd_list = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # cmd_parameter = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    for index in range(len(param)):
        cmd_parameter = [param[index]['delay'] * 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        newWaypointList.append(dji_sdk.msg.MissionWaypoint( latitude = param[index]['lat'], 
                                                            longitude = param[index]['lon'], 
                                                            altitude = param[index]['alt'], 
                                                            damping_distance = 2, 
                                                            target_yaw = 0, 
                                                            has_action = 1, 
                                                            target_gimbal_pitch = 0, 
                                                            turn_mode = 0, 
                                                            action_time_limit = 64000,
                                                            waypoint_action = dji_sdk.msg.MissionWaypointAction(
                                                                action_repeat = 10,
                                                                command_parameter = cmd_parameter)))
    return newWaypointList

def mission_start():
    try:
        auth = rospy.ServiceProxy('dji_sdk/sdk_control_authority', dji_sdk.srv.SDKControlAuthority)
        resp = auth(1)
        print 'Service. Sdk control authority:'
        print resp.result

    except rospy.ServiceException, e:
        print e
       
    try:
        mission = rospy.ServiceProxy('dji_sdk/mission_waypoint_upload', dji_sdk.srv.MissionWpUpload)
        mission_msg = dji_sdk.msg.MissionWaypointTask()
        mission_msg.velocity_range     = 10;
        mission_msg.idle_velocity      = 10;
        mission_msg.action_on_finish   = dji_sdk.msg.MissionWaypointTask.FINISH_RETURN_TO_HOME;
        mission_msg.mission_exec_times = 1;
        mission_msg.yaw_mode           = dji_sdk.msg.MissionWaypointTask.YAW_MODE_AUTO;
        mission_msg.trace_mode         = dji_sdk.msg.MissionWaypointTask.TRACE_POINT;
        mission_msg.action_on_rc_lost  = dji_sdk.msg.MissionWaypointTask.ACTION_AUTO;
        mission_msg.gimbal_pitch_mode  = dji_sdk.msg.MissionWaypointTask.GIMBAL_PITCH_FREE;

        mission_msg.mission_waypoint = waypoints_create()

        print mission_msg
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
    rospy.wait_for_service('dji_sdk/sdk_control_authority')
    # rospy.wait_for_service('dji_sdk/drone_arm_control')
    # rospy.wait_for_service('dji_sdk/drone_task_control')
    rospy.wait_for_service('dji_sdk/mission_waypoint_action')
    rospy.wait_for_service('dji_sdk/mission_waypoint_upload')
    
    mission_start()

    rospy.spin()

