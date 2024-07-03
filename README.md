# neobotix_beerpong
This is our main repo for the Neobotix MMO500 platform with an UR5 robot placing beerpong cups on a table. The whole repo was developed by Leo Schäfer, Maurice Droll, Andreas Schmitt, Robin Wolf and Mathias Fuhrer as part of the project work of the robogistics lecture at Karlsruhe University of Applied Sciences by Prof. Dr.-Ing. Christian Wurll.

## Project overview
![beerpong_neobotix](images/beerpong_neobotix.png)

In this project a ROS2 implementation for the Neobotix MMO500 and the UR5 was set up. The ROS packages are all in the [ros2-packages](https://github.com/mathias31415/neobotix_beerpong/tree/main/ros2-packages) folder. A table position can be entered via a web page. The Neobotix platform then moves to this position. The UR-5 then moves over the table with its end effector and captures an image with an Intel Realsense D415. Using an Aruco code on the table, the robot determines the positions for the Beerpong cup pyramid. The UR5 then picks the cups from the cup dispenser and places them in the calculated positions on the table. The Zimmer GEP5010IO-00-A gripper is used here. The CAD files for the cup dispenser, the gripper jaws and the camera holder are stored in the [cad](https://github.com/mathias31415/neobotix_beerpong/tree/main/cad) folder. Detailed documentation (in German) can be found here TODO.

## Video
This [YouTube video](https://www.youtube.com/watch?v=gvZ-DCJvOs4) shows the results of our project:

[![Video abspielen](images/VideoPreview.png)](https://www.youtube.com/watch?v=gvZ-DCJvOs4)

## Software
All ros2 packages of this project are in the folder [ros2-packages](https://github.com/mathias31415/neobotix_beerpong/tree/main/ros2-packages).

When everything was running on the neobotix platform PC, all CPUs were fully utilized. This led to problems. We therefore moved parts of the program to other laptops. The laptops are connected either in a shared wifi Network or via ssh as shown in the following graphic.

![system_overview](images/system_overview.png)

### [docker_website_beerpong](https://github.com/mathias31415/neobotix_beerpong/tree/main/ros2-packages/docker_website_beerpong)
- Hosting the Website
- Python web framework Flask

### [neobotix_coordinator](https://github.com/mathias31415/neobotix_beerpong/tree/main/ros2-packages/neobotix_coordinator)
- Behavior Tree
- High-level control flow
- Includes clients

### [neobotix_mmo500_driver](https://github.com/mathias31415/neobotix_beerpong/tree/main/ros2-packages/neobotix_mmo500_driver)
- Movement of the Neobotix Platform
- Hardware communication: Motors and Lidar scanner
- Initiating navigation and SLAM mapping

### [realsense_driver](https://github.com/mathias31415/neobotix_beerpong/tree/main/ros2-packages/realsense_driver)
- Processing of Aruco Markers
- ROS2 driver for the camera and server for Aruco detection

### [ur_ros2_driver_humble](https://github.com/mathias31415/neobotix_beerpong/tree/main/ros2-packages/ur_ros2_driver_humble)
- Control of the UR5
- Path planning pipeline and IO server
- PTP, LIN, Joint-Space-PTP in MoveIt via MoveGroup interface


## How-To Use
### PC Neobotix
Plug in the display-port cable to a monitor and connect keyboard and mouse to the neobotix.
Clone/ download the ros2-packages `neobotix_mmo500_driver` and `realsense_driver` and launch them with the following commands:
```
git clone https://github.com/mathias31415/neobotix_beerpong.git
```
build the docker images:
```
cd neobotix_beerpong/ros2-packages/neobotix_mmo500_driver
./build_docker.sh

cd neobotix_beerpong/ros2-packages/realsense_driver
./build_docker.sh
```
start the neobotix driver package:
```
cd neobotix_beerpong/ros2-packages/neobotix_mmo500_driver
./start_docker.sh
```
all needed nodes will be launched in autostarty now you should be able to drive the neobotix with the logitech controller

### Laptop on the Neobotix
This laptop is placed on the Neobotix platform and connected to it via an Ethernet cable to the UR-Control. Additionally, the laptop is on the same Wi-Fi network as the other laptops. Clone/ download the ros2-package `ur_ros2_driver_humble` and launch it with the following commands:
```
git clone neobotix_beerpong/ros2-packages/ur_ros2_driver_humble
```
build and run the container:
```
cd neobotix_beerpong/ros2-packages/ur_ros2_driver_humble
./build_ur.sh
./start_ur.sh
```
all nodes will bei started in autostart and rviz should launch.
Select and run the UR-Cap "External Control" on the UR-Teachpad.
Now you should be able to control the robot arm via Rviz.

### Controller Laptop
Clone/ download the ros2-package `neobotix_mmo500_driver` and launch it and connect to the Neobotics pc via ssh with the following commands:
```
git clone https://github.com/mathias31415/neobotix_beerpong.git
```
build and run the docker images (NOTE: at this point its recommendet to deactivate your WIFI, because if not there can be interfernces with the container running on the neobotix):
```
cd neobotix_beerpong/ros2-packages/neobotix_mmo500_driver
./build_docker.sh
./start_docker.sh
````
now switch back to the shared WIFI and connect one terminal to the running container:
```
docker exec -it neobotix_mmo500_bringup bash
```
start Rviz visualization for navigation or mapping:
```
ros2 launch neo_nav2_bringup rviz_launch.py
```
at this point there should be not that much visible in Rviz, you can check if you can see the neobotix-hosted nodes in the terminal to verify the connection between the containers.

open another terminal and connect via ssh to the neobotix-pc:
```
ssh neobotix@172.22.32.11 (password: neobotix)
```
[**]
connect to the driver container and start the navigation:
```
docker exec -it neobotix_mmo500_bringup bash
ros2 launch neo_mpo_500-2 navigation.launch.py
```
Wait until all lifecycle nodes are up, now you should see the robot model, map and costmaps in Rviz.
After setting the initial pose of the robot in the map, you use the navigation by publishing Nav2Goals.

If you dont want to use the navigation mode, but want to record a naw map, switch back to [**], pass the navigation instructions and follow the mapping instructions: 
connect to the driver container and start the mapping:
```
docker exec -it neobotix_mmo500_bringup bash
ros2 launch neo_mpo_500-2 mapping.launch.py
```
Now drive the neobotix with the controller round the area you want to map (Note: only the front scanner is used for mapping):
The map will be created step by step. The process can be viewed in the poened Rviz window.

If you are finished, save the map:
open another terminal, connect it vis ssh to the neobotix and attach it to the running container:
```
ssh neobotix@172.22.32.11 (password: neobotix)
docker exec -it neobotix_mmo500_bringup bash
```
save the map with your name:
```
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/neo_mpo_500-2-humble/configs/navigation/maps/<your_map_name_without_file_extention>
```
if finished you can close the 2 terminals and run the navigation with your new map (launch argument map:=<your_map_name.yaml>

Moreover you have to start the realsense camera nodes on the neobotix.
Therefore, plug in the USB-Cable from the camera into an open neobotix port (Node: cable has to be outside of the scanner safety-field)
open another Terminal, connect via ssh to the neobotix and start the realsense container:
```
ssh neobotix@172.22.32.11 (password: neobotix)
cd neobotix_beerpong/ros2-packages/realsense_driver
./start_docker.sh
```
all nodes should be launched in autostart. If you recieve the feedback "Realsense Node is up!" in the terminal, everything is fine.


### User Laptop
Clone/ download the ros2-packages `neobotix_coordinator` and `docker_website_beerpong` and launch them with the following commands:
```
TODO
```
You can then access the website at http://127.0.0.1:8080/ and choose the table.
