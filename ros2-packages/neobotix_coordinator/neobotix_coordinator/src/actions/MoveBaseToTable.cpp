#include <neobotix_coordinator/actions/MoveBaseToTable.h>

/**
 * @brief Set the name of the ROS2 action server to connect with.
 * @return Topic name as a string.
 */
std::string MoveBaseToTable::ros2_action_name()
{
    return "navigate_to_pose";
}

/**
 * @brief Set the list of ports provided by the BT node.
 *
 * New port:
 *      direction = [BT::InputPort, BT::OutputPort, BT::BidirectionalPort]
 *      data_type = <[float, int, std::string]>
 *      name = ("name")
 *
 * @return List of provided ports.
 */
BT::PortsList MoveBaseToTable::providedPorts()
{
    return {BT::InputPort<int>("table_id"),
            BT::InputPort<int>("table_side"),
            BT::InputPort<int>("type") 
            };
            
}

/**
 * @brief Set the content of the goal message which is sent to the ROS2 action server.
 */
void MoveBaseToTable::on_send(MoveBaseToTableAction::Goal &goal)
{
    int table_id = ports.get_value<int>("table_id");
    int table_side = ports.get_value<int>("table_side");
    int type = ports.get_value<int>("type");

    // Switch to Case Handling after concept test
    switch (table_id) {
        case 1:
            switch (table_side){
                case 1:
                    switch (type){
                        case 1:
                            goal.pose.header.frame_id = "map";
                            goal.pose.pose.position.x = 5.10;
                            goal.pose.pose.position.y = 0.22;
                            goal.pose.pose.position.z = 0.0;                    
                            goal.pose.pose.orientation.x = 0.0;
                            goal.pose.pose.orientation.y = 0.0;
                            goal.pose.pose.orientation.z = -0.1462;
                            goal.pose.pose.orientation.w = 0.98926;
                            goal.pose.header.stamp = get_node_handle()->now();
                            log("Goal: (x=" + Converter::ftos(goal.pose.pose.position.x) + ", y=" + Converter::ftos(goal.pose.pose.position.y) + ")");
                            break;
                        case 2:
                            goal.pose.header.frame_id = "map";
                            goal.pose.pose.position.x = 5.63;
                            goal.pose.pose.position.y = 0.05;
                            goal.pose.pose.position.z = 0.0;                    
                            goal.pose.pose.orientation.x = 0.0;
                            goal.pose.pose.orientation.y = 0.0;
                            goal.pose.pose.orientation.z = -0.13109;
                            goal.pose.pose.orientation.w = 0.99; 
                            goal.pose.header.stamp = get_node_handle()->now();
                            log("Goal: (x=" + Converter::ftos(goal.pose.pose.position.x) + ", y=" + Converter::ftos(goal.pose.pose.position.y) + ")");
                            break;

                    }
                break;

                case 2:
                    switch (type){
                        case 1:
                            goal.pose.header.frame_id = "map";
                            goal.pose.pose.position.x = 9.26;
                            goal.pose.pose.position.y = -0.95;
                            goal.pose.pose.position.z = 0.0;                     
                            goal.pose.pose.orientation.x = 0.0;
                            goal.pose.pose.orientation.y = 0.0;
                            goal.pose.pose.orientation.z = 0.99255;
                            goal.pose.pose.orientation.w = 0.121; 
                            goal.pose.header.stamp = get_node_handle()->now();
                            log("Goal: (x=" + Converter::ftos(goal.pose.pose.position.x) + ", y=" + Converter::ftos(goal.pose.pose.position.y) + ")");
                            break;
                        
                        case 2:
                            goal.pose.header.frame_id = "map";
                            goal.pose.pose.position.x = 8.60;
                            goal.pose.pose.position.y = -0.75;
                            goal.pose.pose.position.z = 0.0;                     
                            goal.pose.pose.orientation.x = 0.0;
                            goal.pose.pose.orientation.y = 0.0;
                            goal.pose.pose.orientation.z = 0.99;
                            goal.pose.pose.orientation.w = 0.137; 
                            goal.pose.header.stamp = get_node_handle()->now();
                            log("Goal: (x=" + Converter::ftos(goal.pose.pose.position.x) + ", y=" + Converter::ftos(goal.pose.pose.position.y) + ")");
                            break;

                    }
                break;
            }
        break;
        case 2:
            switch (table_side){
                case 1:
                    switch (type){
                        case 1:
                            goal.pose.header.frame_id = "map";
                            goal.pose.pose.position.x = 7.48;
                            goal.pose.pose.position.y = -2.99;
                            goal.pose.pose.position.z = 0.0;                     
                            goal.pose.pose.orientation.x = 0.0;
                            goal.pose.pose.orientation.y = 0.0;
                            goal.pose.pose.orientation.z = -0.6199;
                            goal.pose.pose.orientation.w = 0.78468; 
                            goal.pose.header.stamp = get_node_handle()->now();
                            log("Goal: (x=" + Converter::ftos(goal.pose.pose.position.x) + ", y=" + Converter::ftos(goal.pose.pose.position.y) + ")");
                            break;

                        case 2:
                            goal.pose.header.frame_id = "map";
                            goal.pose.pose.position.x = 7.60;
                            goal.pose.pose.position.y = -3.46;
                            goal.pose.pose.position.z = 0.0;                     
                            goal.pose.pose.orientation.x = 0.0;
                            goal.pose.pose.orientation.y = 0.0;
                            goal.pose.pose.orientation.z = -0.6337;
                            goal.pose.pose.orientation.w = 0.77; 
                            goal.pose.header.stamp = get_node_handle()->now();
                            log("Goal: (x=" + Converter::ftos(goal.pose.pose.position.x) + ", y=" + Converter::ftos(goal.pose.pose.position.y) + ")");
                            break;
                    }
                break;
                
                case 2:
                    switch (type){
                        case 1:
                            goal.pose.header.frame_id = "map";
                            goal.pose.pose.position.x = 8.36;
                            goal.pose.pose.position.y = -7.21;
                            goal.pose.pose.position.z = 0.0;                     
                            goal.pose.pose.orientation.x = 0.0;
                            goal.pose.pose.orientation.y = 0.0;
                            goal.pose.pose.orientation.z = 0.7758;
                            goal.pose.pose.orientation.w = 0.630; 
                            goal.pose.header.stamp = get_node_handle()->now();
                            log("Goal: (x=" + Converter::ftos(goal.pose.pose.position.x) + ", y=" + Converter::ftos(goal.pose.pose.position.y) + ")");
                            break;
                        case 2:
                            goal.pose.header.frame_id = "map";
                            goal.pose.pose.position.x = 8.17;
                            goal.pose.pose.position.y = -6.63;
                            goal.pose.pose.position.z = 0.0;                     
                            goal.pose.pose.orientation.x = 0.0;
                            goal.pose.pose.orientation.y = 0.0;
                            goal.pose.pose.orientation.z = 0.7763;
                            goal.pose.pose.orientation.w = 0.630304; 
                            goal.pose.header.stamp = get_node_handle()->now();
                            log("Goal: (x=" + Converter::ftos(goal.pose.pose.position.x) + ", y=" + Converter::ftos(goal.pose.pose.position.y) + ")");
                            break;
                    }
                break;            
            }
        break;
        
        case 3:
            switch (table_side){
                case 1:
                    switch (type){
                        case 1:
                            goal.pose.header.frame_id = "map";
                            goal.pose.pose.position.x = 9.24;
                            goal.pose.pose.position.y = -12.58;
                            goal.pose.pose.position.z = 0.0;                     
                            goal.pose.pose.orientation.x = 0.0;
                            goal.pose.pose.orientation.y = 0.0;
                            goal.pose.pose.orientation.z = 0.9953;
                            goal.pose.pose.orientation.w = -0.09; 
                            goal.pose.header.stamp = get_node_handle()->now();
                            log("Goal: (x=" + Converter::ftos(goal.pose.pose.position.x) + ", y=" + Converter::ftos(goal.pose.pose.position.y) + ")");
                            break;

                        case 2:
                            goal.pose.header.frame_id = "map";
                            goal.pose.pose.position.x = 8.62;
                            goal.pose.pose.position.y = -12.7;
                            goal.pose.pose.position.z = 0.0; 
                            goal.pose.pose.orientation.x = 0.0;
                            goal.pose.pose.orientation.y = 0.0;
                            goal.pose.pose.orientation.z = 0.9954;
                            goal.pose.pose.orientation.w = -0.09; 
                            goal.pose.header.stamp = get_node_handle()->now();
                            log("Goal: (x=" + Converter::ftos(goal.pose.pose.position.x) + ", y=" + Converter::ftos(goal.pose.pose.position.y) + ")");
                            break;
                    }
                break;
                
                case 2:
                    switch (type){
                        case 1:
                            goal.pose.header.frame_id = "map";
                            goal.pose.pose.position.x = 5.07;
                            goal.pose.pose.position.y = -13.44;
                            goal.pose.pose.position.z = 0.0;                    
                            goal.pose.pose.orientation.x = 0.0;
                            goal.pose.pose.orientation.y = 0.0;
                            goal.pose.pose.orientation.z = 0.10827;
                            goal.pose.pose.orientation.w = 0.994; 
                            goal.pose.header.stamp = get_node_handle()->now();
                            log("Goal: (x=" + Converter::ftos(goal.pose.pose.position.x) + ", y=" + Converter::ftos(goal.pose.pose.position.y) + ")");
                            break;

                        case 2:
                            goal.pose.header.frame_id = "map";
                            goal.pose.pose.position.x = 5.65;
                            goal.pose.pose.position.y = -13.32;
                            goal.pose.pose.position.z = 0.0;                     
                            goal.pose.pose.orientation.x = 0.0;
                            goal.pose.pose.orientation.y = 0.0;
                            goal.pose.pose.orientation.z = 0.10664;
                            goal.pose.pose.orientation.w = 0.9943; 
                            goal.pose.header.stamp = get_node_handle()->now();                            
                            log("Goal: (x=" + Converter::ftos(goal.pose.pose.position.x) + ", y=" + Converter::ftos(goal.pose.pose.position.y) + ")");
                            break;
                    }
                break;
            }
        break;
        
        default:
            log("Note: No correct table id provided. Please request again.");
            break;
    }
}

/**
 * @brief Define what happens when recieving feedback from the ROS2 action server.
 */
void MoveBaseToTable::on_feedback(const std::shared_ptr<const MoveBaseToTableAction::Feedback>)
{
    // log("Current position: (x=" + Converter::ftos((float)feedback->current_pose.pose.position.x
    //     ", y=" + Converter::ftos((float)feedback->current_pose.pose.position.y) +
    //      "), Time elapsed: " + std::to_string(feedback->navigation_time.sec) + "s");
}

/**
 * @brief Define what happens when recieving the result from the ROS2 action server.
 */
void MoveBaseToTable::on_result(const rclcpp_action::ClientGoalHandle<MoveBaseToTableAction>::WrappedResult &, const MoveBaseToTableAction::Goal &goal)
{
    log("Goal reached! (x=" + Converter::ftos(goal.pose.pose.position.x) +
        ", y=" + Converter::ftos(goal.pose.pose.position.y) +
        "), Total time: " + std::to_string((int)get_node_handle()->now().seconds() - goal.pose.header.stamp.sec) + "s");
}