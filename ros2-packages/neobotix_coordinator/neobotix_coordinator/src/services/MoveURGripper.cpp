#include <neobotix_coordinator/services/MoveURGripper.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string MoveURGripper::ros2_service_name()
{
    return "/io_and_status_controller/set_io";
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
BT::PortsList MoveURGripper::providedPorts()
{
    return {BT::InputPort<int>("fun"),
            BT::InputPort<int>("pin"),
            BT::InputPort<float>("state"),
            };
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void MoveURGripper::on_send(std::shared_ptr<MoveURGripperSrv::Request> request)
{

    request->fun = ports.get_value<int>("fun");
    request->pin = ports.get_value<int>("pin");
    request->state = ports.get_value<float>("state");

    log("MoveURGripper to state " + Converter::ftos(request->state) + ".");
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool MoveURGripper::on_result(std::shared_ptr<MoveURGripperSrv::Response>, std::shared_ptr<MoveURGripperSrv::Request>)
{
    log("MoveURGripper completed");
    return true;
}