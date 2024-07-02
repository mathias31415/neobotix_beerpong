/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Service client "ChooseTableSrv"
 *
 * @author 
 * @since 1.0.0 (2023.05.19)
 *********************************************************/
#pragma once

#include <neobotix_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosService.h>
#include <select_table_interfaces/srv/table_select.hpp>


using ChooseTableSrv = select_table_interfaces::srv::TableSelect;

class ChooseTable : public RosService<ChooseTableSrv>
{
public:
    static BT::PortsList providedPorts();

    ChooseTable(const std::string &name, const BT::NodeConfiguration &config) : RosService(name, config) {}

    std::string ros2_service_name() override;

    void on_send(std::shared_ptr<ChooseTableSrv::Request> request) override;
    bool on_result(std::shared_ptr<ChooseTableSrv::Response> response, std::shared_ptr<ChooseTableSrv::Request> request) override;
};