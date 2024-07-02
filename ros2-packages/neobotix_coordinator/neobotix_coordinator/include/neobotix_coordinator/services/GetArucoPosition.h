/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Service client "GetArucoPosition"
 *
 * @author 
 * @since 1.0.0 (2023.05.19)
 *********************************************************/


#pragma once

#include <neobotix_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosService.h>
#include <camera_interfaces/srv/get_aruco_pose.hpp>



using GetArucoPositionSrv = camera_interfaces::srv::GetArucoPose;

class GetArucoPosition : public RosService<GetArucoPositionSrv>
{
public:
    static BT::PortsList providedPorts();

    GetArucoPosition(const std::string &name, const BT::NodeConfiguration &config) : RosService(name, config) {}

    std::string ros2_service_name() override;

    void on_send(std::shared_ptr<GetArucoPositionSrv::Request> request) override;
    bool on_result(std::shared_ptr<GetArucoPositionSrv::Response> response, std::shared_ptr<GetArucoPositionSrv::Request> request) override;
};

