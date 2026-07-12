#ifndef ARM_TELEOP__ARM_SAFETY_NODE_HPP_
#define ARM_TELEOP__ARM_SAFETY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <algorithm>
#include <string>
#include <vector>

const std::string ARM_HARDWARE_COMPONENT = "ArmSystem"
const std::vector<std::string> ODRIVE_NODE_NS = {
  "odrive_node_0", "odrive_node_1", "odrive_node_2",
  "odrive_node_3", "odrive_node_4", "odrive_node_5",
};

class ArmSafetyNode : public rclcpp::Node{
  public:
    ArmSafetyNode();

  private:
    bool sim_mode_ = true;
    bool estopped_ = false;
    bool rearm_pending_ = false;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estopped_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_estop_srv_;
    rclcpp::Client<controller_manager_srvs::srv::SetHardwareComponentState>::SharedPtr hw_state_client_;

    void publish_estopped_status();
    void begin_rearm_sequence();
    void set_hardware_state(const std::string & label);
    void estop_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void reset_estop_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

#endif  // ARM_TELEOP__ARM_SAFETY_NODE_HPP_