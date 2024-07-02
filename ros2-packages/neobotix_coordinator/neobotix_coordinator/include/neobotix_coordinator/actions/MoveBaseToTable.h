/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Action client for "navigate_to_pose"(NavigateToPose)
 *
 * @author 
 * @since 1.0.0 (2022.05.12)
 *********************************************************/
#pragma once

#include <neobotix_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosAction.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>

using MoveBaseToTableAction = nav2_msgs::action::NavigateToPose;

class MoveBaseToTable : public RosAction<MoveBaseToTableAction>
{
public:
    static BT::PortsList providedPorts();

    MoveBaseToTable(const std::string &name, const BT::NodeConfiguration &config) : RosAction(name, config) {}

    std::string ros2_action_name() override;

    void on_send(MoveBaseToTableAction::Goal &goal) override;
    void on_feedback(const std::shared_ptr<const MoveBaseToTableAction::Feedback> feedback) override;
    void on_result(const rclcpp_action::ClientGoalHandle<MoveBaseToTableAction>::WrappedResult &result, const MoveBaseToTableAction::Goal &goal) override;
};