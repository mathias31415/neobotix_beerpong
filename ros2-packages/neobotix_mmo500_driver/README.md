# neobotix_mmo500_ros2

This is a ROS2 humble based package to use the Neobotix MMO500 platform.

## How to use?

Build and run the container with
```bash
cd neobotix_mmo500_ros2
source start_docker.sh
```

### Control the robot with the logitech controller

In the container run
```bash
ros2 launch neo_mpo_500-2 bringup.launch.py
```

### Autonomous Navigation

You need to have a client-pc with this docker container in the same network.

On the **robot-pc** open another shell in the container

```bash
docker exec -it neobotix_mmo500_bringup bash
```
and start the navigation in that shell
```bash
ros2 launch neo_mpo_500-2 navigation.launch.py
```

On the **client-pc** run in the container
```bash
ros2 launch neo_mpo_500-2 rviz.launch.py
```
You should see the robot and the map loaded in. Now give a pose estimation on where the robot is currently on the real map. If the robot localized itself you can give goal poses.

#### Move to a goal pose via topic
```bash
ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: -6.7, y: 6.8, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: 0, w: 1.0}}}"
```

### Mapping
Switch to noetic branch for better mapping.
