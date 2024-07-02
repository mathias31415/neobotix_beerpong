#include <neobotix_coordinator/services/MoveArmToPosePtp.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string MoveArmToPosePtp::ros2_service_name()
{
    return "/move_to_pose_ptp";
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
BT::PortsList MoveArmToPosePtp::providedPorts()
{
    return {BT::InputPort<float>("x"),
            BT::InputPort<float>("y"),
            BT::InputPort<float>("z"),
            BT::InputPort<float>("q_x"),
            BT::InputPort<float>("q_y"),
            BT::InputPort<float>("q_z"),
            BT::InputPort<float>("q_w")};
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void MoveArmToPosePtp::on_send(std::shared_ptr<MoveToPosePtpSrv::Request> request)
{
    request->pose.position.x = ports.get_value<float>("x");
    request->pose.position.y = ports.get_value<float>("y");
    request->pose.position.z = ports.get_value<float>("z");
    request->pose.orientation.x = ports.get_value<float>("q_x");
    request->pose.orientation.y = ports.get_value<float>("q_y");
    request->pose.orientation.z = ports.get_value<float>("q_z");
    request->pose.orientation.w = ports.get_value<float>("q_w");

    log("Move arm ptp to pose (" + Converter::ftos(request->pose.position.x) + ", " + Converter::ftos(request->pose.position.y) + ", " + Converter::ftos(request->pose.position.z) + ")");
    log("Orientation (" + Converter::ftos(request->pose.orientation.x) + ", " + Converter::ftos(request->pose.orientation.y) + ", " + Converter::ftos(request->pose.orientation.z) + ", " + Converter::ftos(request->pose.orientation.w) + ")");
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool MoveArmToPosePtp::on_result(std::shared_ptr<MoveToPosePtpSrv::Response>, std::shared_ptr<MoveToPosePtpSrv::Request>)
{
    log("MoveArmToPosePtp completed");
    return true;
}