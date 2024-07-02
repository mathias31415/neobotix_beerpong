#include <neobotix_coordinator/nodes/GetTfMarkerToBase.h>

#define RAD2DEG(x) ((x) * 180.0f / 3.1415)
#define DEG2RAD(x) ((x) * 3.1415 / 180.0f)
#define OFFSET_Y -0.035 // Offset from marker corner to center of the marker (For a arcuo marker with 0.07m side length)

using namespace std::chrono_literals;

/**
 * @brief Constructor of the node, initialize e.g. ROS2 subscriber.
 */
GetTfMarkerToBase::GetTfMarkerToBase(const std::string &name, const BT::NodeConfiguration &config) : RosNode(name, config)
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node_handle()->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
BT::PortsList GetTfMarkerToBase::providedPorts()
{
    return {BT::InputPort<int>("aruco_id"), // default: 1
            BT::InputPort<int>("max_seconds"),  // default: 5
            BT::InputPort<std::string>("base_frame"), // default: "base_link"
            BT::OutputPort<float>("x"),
            BT::OutputPort<float>("y"),
            BT::OutputPort<float>("z"),
            BT::OutputPort<float>("q_x"),
            BT::OutputPort<float>("q_y"),
            BT::OutputPort<float>("q_z"),
            BT::OutputPort<float>("q_w")};
}

/**
 * @brief Define what happens when this node is ticked for the first time.
 * @return BT::NodeStatus RUNNING (Has to return RUNNING to allow on_running to be called)
 */
BT::NodeStatus GetTfMarkerToBase::on_start()
{
    start_time_ = time(NULL);
    last_checked_ = time(NULL);

    return BT::NodeStatus::RUNNING;
}

/**
 * @brief Define what happens when this node is ticked in RUNNING mode.
 * @return BT::NodeStatus SUCCESS or FAILURE or RUNNING
 */
BT::NodeStatus GetTfMarkerToBase::on_running()
{
    if ((time(NULL) - start_time_) < ports.get_value<int>("max_seconds"))
    {
        if ((time(NULL) - last_checked_) > 1)
        {
            geometry_msgs::msg::TransformStamped transformStamped;
            last_checked_ = time(NULL);

            try
            {
                std::string marker_frame = "aruco_" + std::to_string(ports.get_value<int>("aruco_id"));
                RCLCPP_INFO(
                    get_node_handle()->get_logger(), "Calculate transform from base_link to %s", marker_frame.c_str());

                // tf_buffer_->canTransform("base_link", aruco_{aruco_id});
                transformStamped = tf_buffer_->lookupTransform(
                    ports.get_value<std::string>("base_frame"), marker_frame, // get_node_handle()->get_clock()->now(), 500ms);
                    tf2::TimePointZero);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_INFO(
                    get_node_handle()->get_logger(), "Could not transform: %s", ex.what());

                return BT::NodeStatus::RUNNING;
            }

            double x = 0, y = 0, z = 0, q_x = 0, q_y = 0, q_z = 0, q_w = 0;

            x = transformStamped.transform.translation.x;
            y = transformStamped.transform.translation.y;
            z = transformStamped.transform.translation.z; 
            
            q_x = transformStamped.transform.rotation.x;
            q_y = transformStamped.transform.rotation.y;
            q_z = transformStamped.transform.rotation.z;
            q_w = transformStamped.transform.rotation.w;

            ports.set_value<float>("x", x);
            ports.set_value<float>("y", y);
            ports.set_value<float>("z", z);
            ports.set_value<float>("q_x", q_x);
            ports.set_value<float>("q_y", q_y);
            ports.set_value<float>("q_z", q_z);
            ports.set_value<float>("q_w", q_w);

            log("Marker Pose relative to base_link/ world Frame: ");
            log("MarkerPosition: (x=" + Converter::ftos(x) + ", y=" + Converter::ftos(y) + ", z=" + Converter::ftos(z) + ")");
            log("MarkerPosition: (q_x=" + Converter::ftos(q_x) + ", q_y=" + Converter::ftos(q_y) + ", q_z=" + Converter::ftos(q_z) + ", q_w=" + Converter::ftos(q_w) + ")");

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::RUNNING;
        }
    }

    return BT::NodeStatus::FAILURE;
}