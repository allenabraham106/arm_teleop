#ifndef ARM_TELEOP__ARM_SAFETY_NODE_HPP_
#define ARM_TELEOP__ARM_SAFETY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <arm_teleop_messages/msg/estop_status.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <algorithm>
#include <string>
#include <vector>

const std::string ARM_HARDWARE_COMPONENT = "ArmSystem";

class ArmSafetyNode : public rclcpp::Node{
  public:
    ArmSafetyNode();

  private:
    bool estopped_ = false;
    bool rearm_pending_ = false;

    rclcpp::Publisher<arm_teleop_messages::msg::EstopStatus>::SharedPtr estopped_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_estop_srv_;
    rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedPtr hw_state_client_;
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