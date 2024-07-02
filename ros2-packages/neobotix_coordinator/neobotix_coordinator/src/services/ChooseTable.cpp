#include <neobotix_coordinator/services/ChooseTable.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string ChooseTable::ros2_service_name()
{
    return "/table_server";
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
BT::PortsList ChooseTable::providedPorts()
{
    return {BT::InputPort<bool>("Send_Request"),
            BT::OutputPort<int>("Requested_Table")
            };
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void ChooseTable::on_send(std::shared_ptr<ChooseTableSrv::Request> request)
{

    request->request = ports.get_value<bool>("Send_Request");

    log("Requested Table server to get user table selection: (" + Converter::ftos(request->request));
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool ChooseTable::on_result(std::shared_ptr<ChooseTableSrv::Response> response, std::shared_ptr<ChooseTableSrv::Request>)
{

    ports.set_value<int>("Requested_Table", response->table_id);


    log("User has chosen table:  " + Converter::ftos(response->table_id));

    log("ChooseTable completed");
    return true;
}