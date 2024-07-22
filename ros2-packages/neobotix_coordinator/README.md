# Neobotix Coordinator

This repository contains the high level task control for the Neobotix application implemented with the Behavior Tree package in C++.

It uses the [BehaviorTee.IRAS](https://github.com/AndreasZachariae/BehaviorTree.IRAS) framework which is a wrapper around the [BehaviorTree.cpp v3.8](https://www.behaviortree.dev/) library and extends it for the combined use of behavior trees with ROS 2 Humble.

The library of actions can be arranged freely with the graphical user interface [Groot](https://github.com/BehaviorTree/Groot).

This ReadMe covers the following topics: 
- How to start a Behavior Tree? 
- How to start the Groot editor?
- How to design a new Behavior Tree?
- How to create a custom node?
- Further information (including the usage of custom interfaces)


## How to start a Behavior Tree?

Build and start the docker container
```bash
source build_docker.sh
source start_docker.sh
```

Inside the container launch the Coordinator node with parameters:
```
# in neobotix_coordinator docker

ros2 launch neobotix_coordinator neobotix.launch.py
```

The to be executed Behavior Tree can be adjusted in the corresponding launch file (e.g. neobotix.launch.py) located in 'src/neobotix_coordinator/launch'. In order to change it to an other behavior XML File, please adapt the parameters=[{'main_tree_path: ...<file path to the behavior file>......}]. Alternative you can us the params.yaml file. 


## How to start the Groot editor?

You can start Groot as the graphical user interface for the Behavior Tree.

Attach to a new shell and start groot:

```bash
docker exec -it coordinator bash
ros2 run groot Groot
```

## How to design a new Behavior Trees?

1. Start the docker container as normal
    ```bash
    source start_docker.sh
    ```
2. If new BT nodes were added or ports changed regenerate the GrootPalette by starting a test BT.
   ```bash
   # in ~/ros2_ws/
   colcon build
   ros2 launch neobotix_coordinator test.launch.py
   ```
3. Start Groot in Editor mode
    ```bash
    ros2 run groot Groot
    # Click "Editor" and "START"
    ```
4. Load GrootPalette with all custom nodes.  
   Click on ->  <img src="https://raw.githubusercontent.com/BehaviorTree/Groot/master/bt_editor/resources/svg/download.svg" alt="load" width="18"/> to load palette from file.  
   Choose the file from: `/home/docker/ros2_ws/src/neobotix_coordinator/behaviors/GrootPalette_nbtx.xml`
5. Build BT via drag and drop
6. Save tree to file.  
   Click on ->  <img src="https://raw.githubusercontent.com/BehaviorTree/Groot/master/bt_editor/resources/svg/save_dark.svg" alt="load" width="18"/> to save.  
   Choose location as: `/home/docker/ros2_ws/src/neobotix_coordinator/behaviors/<your_folder_name>/`
7. Create a new launch file with this parameter or adjust a existing launch file with this parameter:
    ```python
    # in /home/docker/ros2_ws/src/neobotix_coordinator/launch/<your_launch_file>.launch.py>
    parameters=[{'main_tree_path': "/home/docker/ros2_ws/src/neobotix_coordinator/behaviors/<your_folder_name>/<your_tree_name>.xml",
                 'groot_palette_path': "/home/docker/ros2_ws/src/neobotix_coordinator/behaviors/GrootPalette_nbtx.xml"}],
    ```
8. Launch your node 

Alternatively, you can adjust the .xml files diretly. 


## How to create a new custom node

There are currently 4 different types of nodes supported:
- **actions** Use for ROS2 action client
- **services** Use for ROS2 service client
- **conditions** Use for classic BT condition with ROS2 node handle access
- **nodes** Use for classic BT action with ROS2 node handle access

This instructions gives an example for a ROS2 action client

1. Add a new header file in the corresponding folder at `neobotix_coordinator/include/neobotix_coordinator/actions`. For this, copy an existing file from that folder and rename. Use this structure as template. Copy `MoveBase.h` and rename to `MyCustomActionNode.h`.
2. Add a new source file in `neobotix_coordinator/src/actions`. Copy `MoveBase.cpp` and rename to `MyCustomActionNode.cpp`.
3. In this source file change the first line to include your newly added header.  
   Replace: ~~`#include <neobotix_coordinator/actions/MoveBase.h>`~~
    ```cpp
    // in MyCustomActionNode.cpp
    #include <neobotix_coordinator/actions/MyCustomActionNode.h>
    ```
4. In your header file `MyCustomActionNode.h` include the header files of your ROS2 interface you want to use. In this example it is located in the `_interfaces` package.  
Replace: ~~`#include <nav2_msgs/action/navigate_to_pose.hpp>`~~  
**Important**: Interface header files are generated automatically. If your Interface file is called `MyCustomAction.action` (PascalCase) the generated header will be `my_custom_action.hpp` (snake_case).
    ```cpp
    // in MyCustomActionNode.h
    #include <iras_interfaces/action/my_custom_action.hpp>
    ```
5. Give an alias as shorter name.  
Replace: ~~`using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;`~~
    ```cpp
    // in MyCustomActionNode.h
    using MyCustomAction = neobotix_interfaces::action::MyCustomAction;
    ```
6. Replace all occurences where old alias is used with new one in `.ccp` and `.h` file. Use VSCode find and replace (strg + f) or rename symbol (F2) shortcut.
7. Change the class name to the same name as the file.  
   Replace: ~~`class MoveArm : public RosAction<MoveArmMoveIt>`~~  
   **Important**: The class name must be different from the given alias.
    ```cpp
    // in MyCustomActionNode.h
    class MyCustomActionNode : public RosAction<MyCustomAction>
    ```
8. Replace all occurences of the old class name in the source file `.ccp` with new one. Use VSCode find and replace (strg + f) or rename symbol (F2) shortcut.  
   Replace: For every function: ~~`std::string MoveBase::ros2_action_name()`~~
    ```cpp
    // in MyCustomActionNode.cpp
    // for every function
    std::string MyCustomActionNode::ros2_action_name()
    /* ... */
    ```
9. Set the topic name of the ROS2 action server to connect with as string.
    ```cpp
    // in MyCustomActionNode.cpp
    std::string MyCustomActionNode::ros2_action_name()
    {
        return "my_custom_action_topic";
    }
    ```
10. Set the list of ports provided by the BT node.
    ```cpp
    // in MyCustomActionNode.cpp

    /* New port:
    *      direction = [BT::InputPort, BT::OutputPort, BT::BidirectionalPort]
    *      data_type = <[float, int, std::string]>
    *      name = ("name") */
    BT::PortsList MyCustomActionNode::providedPorts()
    {
        return {BT::InputPort<std::string>("string_input"),
                BT::OutputPort<float>("float_output"),
                BT::BidirectionalPort<int>("int_bidirectional")
                /* ... */};
    }
    ```
11. Set the content of the goal message which is sent to the ROS2 action server.
    ```cpp
    // in MyCustomActionNode.cpp
    void MyCustomActionNode::on_send(MyCustomAction::Goal &goal)
    {
        goal.header.frame_id = "custom_frame";
        goal.header.stamp = get_node_handle()->now();
        /* ... */
        log("Custom goal sent");
    }
    ```
12. Define what happens when recieving feedback from the ROS2 action server.
    ```cpp
    // in MyCustomActionNode.cpp
    void MyCustomActionNode::on_feedback(const std::shared_ptr<const NavigateToPoseAction::Feedback> feedback)
    {
        /* ... */
        log("Feedback no. " + std::to_string(feedback.counter) + " recieved");
    }
    ```
13. Define what happens when recieving the result from the ROS2 action server.
    ```cpp
    // in MyCustomActionNode.cpp
    void MyCustomActionNode::on_result(const rclcpp_action::ClientGoalHandle<MyCustomAction>::WrappedResult &result, const MyCustomAction::Goal &goal)
    {
        /* ... */
        log("Action finished");
    }
    ```
14. Include your header file in the Coordinator node at `neobotix_coordinator/src/node.cpp`
    ```cpp
    // in node.cpp
    #include <neobotix_coordinator/actions/MyCustomActionNode.h>
    ```
15. Register your node in the BehaviorTreeFactory.  
    **Important**: The string given here defines the name of the node in BT XML representation and Groot.
    ```cpp
    // in node.cpp
    factory.registerNodeType<MyCustomActionNode>("MyCustomActionNode");
    ```
16. Rebuild and start the container as described above. This will generate an updated GrootPalette to use in the graphical editor Groot as described in "How to design a new Behavior Tree"


## Further information 

Most parts of the Behavior Trees are implemented with services/ actions based on the server/ client principle.

To learn more about ROS2 service clients in general, please investigate the following [link](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html).

For further examples on the creation of custom nodes and the usage of a custom interface message, please review the  [aip_coordinator](https://github.com/IRAS-HKA/aip_coordinator) repository and especially the "How_To_Use_AIP_Coordinator.md" file.


## License
<a rel="license" href="http://creativecommons.org/licenses/by-nc/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc/4.0/">Creative Commons Attribution-NonCommercial 4.0 International License</a>.
