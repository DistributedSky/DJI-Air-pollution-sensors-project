from time import sleep
import rospy
import roslib
import dji_sdk.srv
import dji_sdk.msg
from dji_sdk.srv import DroneTaskControlRequest

#rospy.init_node('mission_loader', anonymous=True)
#rospy.wait_for_service('dji_sdk/sdk_control_authority')
#initialize = rospy.ServiceProxy("dji_sdk/sdk_control_authority", dji_sdk.srv.SDKControlAuthority)

if __name__ == "__main__":
    rospy.wait_for_service('dji_sdk/sdk_control_authority')
    rospy.wait_for_service('dji_sdk/drone_arm_control')
    rospy.wait_for_service('dji_sdk/drone_task_control')
    rospy.wait_for_service('dji_sdk/mission_waypoint_upload')
    rospy.wait_for_service('dji_sdk/mission_waypoint_action')
    try:
        auth = rospy.ServiceProxy("dji_sdk/sdk_control_authority", dji_sdk.srv.SDKControlAuthority)
        resp = auth(1)
        print resp.result

    except rospy.ServiceException, e:
        print e

    sleep(1)
       
    try:
        mission = rospy.ServiceProxy("dji_sdk/mission_waypoint_upload", dji_sdk.srv.MissionWpUpload)
        mission_msg = dji_sdk.msg.MissionWaypointTask()
        mission_msg.velocity_range = 1
        print mission_msg
        resp = mission(mission_msg)
        print resp.result

    except rospy.ServiceException, e:
        print e
    
    sleep(1)
    
    try:
        arm = rospy.ServiceProxy("dji_sdk/drone_arm_control", dji_sdk.srv.DroneArmControl)
        resp = arm(1)
        print resp.result

    except rospy.ServiceException, e:
        print e
    
    sleep(1)
    
    try:
        task = rospy.ServiceProxy("dji_sdk/drone_task_control", dji_sdk.srv.DroneTaskControl)
        resp = task(DroneTaskControlRequest.TASK_TAKEOFF)
        print resp.result

    except rospy.ServiceException, e:
        print e

    sleep(5)       
 
    try:
        start = rospy.ServiceProxy("dji_sdk/mission_waypoint_action", dji_sdk.srv.MissionWpAction)
        resp = start(0)
        print resp.result

    except rospy.ServiceException, e:
        print e


