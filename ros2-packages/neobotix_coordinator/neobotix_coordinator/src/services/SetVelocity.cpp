#include <neobotix_coordinator/services/SetVelocity.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string SetVelocity::ros2_service_name()
{
    return "/setVelocityScaling";
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
BT::PortsList SetVelocity::providedPorts()
{
    return {BT::InputPort<float>("velocity_scaling")
            };
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void SetVelocity::on_send(std::shared_ptr<SetVelocitySrv::Request> request)
{

    request->velocity_scaling = ports.get_value<float>("velocity_scaling");

    log("Scale velocity to (" + Converter::ftos(request->velocity_scaling) + ".");
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool SetVelocity::on_result(std::shared_ptr<SetVelocitySrv::Response>, std::shared_ptr<SetVelocitySrv::Request>)
{
    log("SetVelocity completed");
    return true;
}