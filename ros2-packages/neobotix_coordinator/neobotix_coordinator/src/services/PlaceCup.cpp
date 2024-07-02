#include <neobotix_coordinator/services/PlaceCup.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string PlaceCup::ros2_service_name()
{
    return "/move_to_pose_lin";
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
BT::PortsList PlaceCup::providedPorts()
{
    return {BT::InputPort<int>("cup_id"),
            BT::InputPort<float>("x"),
            BT::InputPort<float>("y"),
            BT::InputPort<float>("z"),
            BT::InputPort<float>("q_x"),
            BT::InputPort<float>("q_y"),
            BT::InputPort<float>("q_z"),
            BT::InputPort<float>("q_w")};

    // return {BT::InputPort<int>("cup_id")};
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void PlaceCup::on_send(std::shared_ptr<PlaceCupSrv::Request> request)
{
    float x, y, z, q_x, q_y, q_z, q_w;
    int cup_id;

    x = ports.get_value<float>("x");
    y = ports.get_value<float>("y");
    z = ports.get_value<float>("z");
    q_x = ports.get_value<float>("q_x");
    q_y = ports.get_value<float>("q_y");
    q_z = ports.get_value<float>("q_z");
    q_w = ports.get_value<float>("q_w");
    
    cup_id = ports.get_value<int>("cup_id");

    float x_offset, y_offset, z_offset, q_x_offset, q_y_offset, q_z_offset, q_w_offset;

    switch (cup_id)
      {
         case 1:
            x_offset = -0.15;
            y_offset = 0.1;
            z_offset = 0.11;
            q_x_offset = 0.0;
            q_y_offset = 0.0;
            q_z_offset = 0.0;
            q_w_offset = 0.0;
            log("Case 1");
            break;
         case 2:
            x_offset = 0.0;
            y_offset = 0.1;
            z_offset = 0.11;
            q_x_offset = 0.0;
            q_y_offset = 0.0;
            q_z_offset = 0.0;
            q_w_offset = 0.0;
            log("Case 2");
            break;
        case 3:
            x_offset = 0.15;
            y_offset = 0.1;
            z_offset = 0.11;          
            q_x_offset = 0.0;
            q_y_offset = 0.0;
            q_z_offset = 0.0;
            q_w_offset = 0.0;
            break;
        case 4:
            x_offset = -0.065;
            y_offset = 0.1;
            z_offset = 0.12;
            q_x_offset = 0.0;
            q_y_offset = 0.0;
            q_z_offset = 0.0;
            q_w_offset = 0.0;
            break;
        case 5:  
            x_offset = 0.065;
            y_offset = 0.1;
            z_offset = 0.12;
            q_x_offset = 0.0;
            q_y_offset = 0.0;
            q_z_offset = 0.0;
            q_w_offset = 0.0;
            break;
        case 6:  
            x_offset = 0.0 ;
            y_offset = 0.18;
            z_offset = 0.12;
            q_x_offset = 0.0;
            q_y_offset = 0.0;
            q_z_offset = 0.0;
            q_w_offset = 0.0;
            break;
      }
    
    float x_new, y_new, z_new, q_x_new, q_y_new, q_z_new, q_w_new;

    x_new = x + x_offset;
    y_new = y + y_offset;
    z_new = z + z_offset;
    q_x_new = q_x + q_x_offset;
    q_y_new = q_y + q_y_offset;
    q_z_new = q_z + q_z_offset;
    q_w_new = q_w + q_w_offset;

    request->pose.position.x = x_new;
    request->pose.position.y = y_new;
    request->pose.position.z = z_new;
    request->pose.orientation.x = q_x_new;
    request->pose.orientation.y = q_y_new;
    request->pose.orientation.z = q_z_new;
    request->pose.orientation.w = q_w_new;

    log("Move arm lin to pose (" + Converter::ftos(request->pose.position.x) + ", " + Converter::ftos(request->pose.position.y) + ", " + Converter::ftos(request->pose.position.z) + ")");
    log("Orientation ("     + Converter::ftos(request->pose.orientation.x) + ", " + Converter::ftos(request->pose.orientation.y) + ", " + Converter::ftos(request->pose.orientation.z) + ", " + Converter::ftos(request->pose.orientation.w) + ")");
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool PlaceCup::on_result(std::shared_ptr<PlaceCupSrv::Response> response, std::shared_ptr<PlaceCupSrv::Request>)
{

    

    bool result = response.get()->success;


    log("Resultat von dem PlaceCup" + Converter::ftos(response.get()->success));

    //result = ports.get_value<bool>("success");

    if (result = true)
    {
        log("PlaceCupSrv completed");
        return true;
    } 
    else
    {
        log("PlaceCupSrv failed");
        return false;
    } 
}