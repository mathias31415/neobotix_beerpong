# Neobotix Coordinator

front test pose:
ros2 service call /move_to_pose_lin  moveit_wrapper/srv/MoveToPose "{pose: {position:{x: 0.11, y: 0.31, z: 0.422}, orientation: {x: 0.0 ,y: 1.0, z: 0.0, w: 0.0}}}"

cup pose:
ros2 service call /move_to_pose_lin  moveit_wrapper/srv/MoveToPose "{pose: {position:{x: -0.26, y: -0.32, z: 0.56}, orientation: {x: 0.005 ,y: -0.001, z: -0.015, w: 0.999}}}"

ros2 service call /move_to_pose_lin  moveit_wrapper/srv/MoveToPose "{pose: {position:{x: -0.26, y: -0.32, z: 0.58}, orientation: {x: 0.0 ,y: 0.0, z: 0.0, w: 1.0}}}"


