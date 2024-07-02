#include <neobotix_coordinator/services/GetArucoPosition.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
*/
std::string GetArucoPosition::ros2_service_name()
{
    return "/get_aruco_pose";
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
BT::PortsList GetArucoPosition::providedPorts()
{
    return {BT::InputPort<int>("aruco_id"),
            BT::OutputPort<float>("x"),
            BT::OutputPort<float>("y"),
            BT::OutputPort<float>("z"),
            BT::OutputPort<float>("q_x"),
            BT::OutputPort<float>("q_y"),
            BT::OutputPort<float>("q_z"),
            BT::OutputPort<float>("q_w"),
            };
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void GetArucoPosition::on_send(std::shared_ptr<GetArucoPositionSrv::Request> request)
{
    request->aruco_id = ports.get_value<int>("aruco_id");

    log("GetArucoPosition for Aruco ID " + Converter::ftos(request->aruco_id) + ".");
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool GetArucoPosition::on_result(std::shared_ptr<GetArucoPositionSrv::Response> response, std::shared_ptr<GetArucoPositionSrv::Request>)
{  
    geometry_msgs::msg::TransformStamped aruco_pose = response->transform_stamped;
    bool aruco_found = response->found;
    float x=0, y=0, z=0, q_x=0, q_y=0, q_z=0, q_w=0;

    x = aruco_pose.transform.translation.x;
    y = aruco_pose.transform.translation.y;
    z = aruco_pose.transform.translation.z;
    q_x = aruco_pose.transform.rotation.x;
    q_y = aruco_pose.transform.rotation.y;
    q_z = aruco_pose.transform.rotation.z;
    q_w = aruco_pose.transform.rotation.w;


    ports.set_value<float>("x", x);
    ports.set_value<float>("y", y);
    ports.set_value<float>("z", z);
    ports.set_value<float>("q_x", q_x);
    ports.set_value<float>("q_y", q_y);
    ports.set_value<float>("q_z", q_z);
    ports.set_value<float>("q_w", q_w);


    log("GetArucoPosition completed");
    log("Response Aruco_Found = " + std::to_string(aruco_found));
    log("Marker Position: x=" + std::to_string(x) + ", y=" + std::to_string(y) + ", z=" + std::to_string(z) + ", q_x=" + std::to_string(q_x) +", q_y=" + std::to_string(q_y) + ", q_z="+std::to_string(q_z) + ", q_w=" + std::to_string(q_w));
    log("On /tf_static Topic tf2 TransformStamped published with Name aruco_" + std::to_string(ports.get_value<int>("aruco_id")));
    return true;
}


