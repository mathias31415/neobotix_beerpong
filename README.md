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
Clone/ download the ros2-packages `neobotix_mmo500_driver` and `realsense_driver` and launch them with the following commands:
```
TODO
```

### Laptop on the Neobotix
This laptop is placed on the Neobotix platform and connected to it via an Ethernet cable. Additionally, the laptop is on the same Wi-Fi network as the other laptops. Clone/ download the ros2-package `ur_ros2_driver_humble` and launch it with the following commands:
```
TODO
```

### Controller Laptop
Clone/ download the ros2-package `neobotix_mmo500_driver` and launch it and connect to the Neobotics pc via ssh with the following commands:
```
TODO
```

### User Laptop
Clone/ download the ros2-packages `neobotix_coordinator` and `docker_website_beerpong` and launch them with the following commands:
```
TODO
```
You can then access the website at http://127.0.0.1:8080/ and choose the table.