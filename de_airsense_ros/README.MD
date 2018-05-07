## de_airsense_ros package

#### Installation
pip install pyserial ipfsapi

#### Usage
Start DJI SDK ROS serial communication:<br>
`roslaunch de_airsense_ros matrice100_sdk.launch`

For start mission:<br>
`roslaunch de_airsense_ros mission.launch`

Mission waypoints file defined by argument m. For example: <br>
`roslaunch de_airsense_ros mission.launch m:=test_mission`

Missions folder is `missions` in YAML format.

For collect data from Waspmote and Matrice 100 and save it on storage and IPFS, run script: <br>
`rosrun de_airsense_ros de_airsense_waspmote_ipfs.py`

Script automatically collect data when drone in flight and then after landing save it to file and send to IPFS.
