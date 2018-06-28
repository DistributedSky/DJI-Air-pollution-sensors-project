# Automatic Environmental Inspection Service

### Introduction
This product helps you measure air pollution level with the help of an aerial drone. Drone Employee software allows you to connect to the drone remotely and launch an autonomous inspection mission. The drone will automatically collect data from electrochemical gas sensors and send them to a distributed IPFS file system.

### Hardware
- Dji Matrice 100
- Libelium Waspmote
- Libelium Gases PRO board
- Libelium GPS board
- Calibrated gas sensors: CO, CO2, NO, SO2, CH4
- Raspberry Pi 3
- Huawei E3372
- Plastic case and DC voltage regulators

General [block diagram](../master/Functional_scheme_v.1.1.pdf) and equipment interfaces.

### Software
We use Robot Operating System (ROS) framework to manage the drone flight and data collection process. It operates on a single-board Raspberry Pi 3. Interaction with the drone happens via DJI Onboard SDK. To connect to the drone remotely we use SSH through static IPv4 from service provider or IPv6 cjdns peer-to-peer network.

[Instruction](../master/Ubuntu_image/README.MD) on how to create an image for Raspberry Pi.</br>
[Instruction](../master/de_airsense_ros/README.MD) on how to use ROS nodes.</br>
[Instruction](../master/Libelium_Waspmote_API/README.MD) on how to use Libelium Waspmote.

**General structure of ROS nodes:**

![](../master/rosgraph.png)

**de_airsense_mission** - a node that launches a drone for a mission and receives the necessary telemetry data from it. Uses DJI SDK ROS services to download and run the mission. Missions are stored in yaml format files.

**de_airsense_waspmote_ipfs** -  a node for data collection and recording with Waspmote during the flight. Also collects data on the coordinates and time of the measuring point. Data recording begins when the drone takes off. When the drone is landed the data is written to a file and sent to IPFS.
