/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Service client "MoveToPosePtpSrv"
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2023.05.19)
 *********************************************************/
#pragma once

#include <neobotix_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosService.h>
#include <moveit_wrapper/srv/move_to_pose.hpp>

using MoveToPosePtpSrv = moveit_wrapper::srv::MoveToPose;

class MoveArmToPosePtp : public RosService<MoveToPosePtpSrv>
{
public:
    static BT::PortsList providedPorts();

    MoveArmToPosePtp(const std::string &name, const BT::NodeConfiguration &config) : RosService(name, config) {}

    std::string ros2_service_name() override;

    void on_send(std::shared_ptr<MoveToPosePtpSrv::Request> request) override;
    bool on_result(std::shared_ptr<MoveToPosePtpSrv::Response> response, std::shared_ptr<MoveToPosePtpSrv::Request> request) override;
};