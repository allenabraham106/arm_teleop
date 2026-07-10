#ifndef ARM_TELEOP__ARM_SAFETY_NODE_HPP_
#define ARM_TELEOP__ARM_SAFETY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <odrive_can/srv/axis_state.hpp>
#include <algorithm>
#include <string>
#include <vector>

constexpr uint32_t AXIS_STATE_IDLE = 1;
constexpr uint32_t AXIS_STATE_CLOSED_LOOP_CONTROL = 8;

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
    std::vector<bool> joint_ready_;

    std::vector<rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr> axis_state_clients_;
    std::vector<rclcpp::Client<std_srvs::srv::Empty>::SharedPtr>       clear_errors_clients_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estopped_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_estop_srv_;

    void publish_estopped_status();
    void request_all_axis_states(uint32_t state);
    void begin_rearm_sequence();
    void request_joint_rearm(size_t i);
    void check_rearm_complete();
    void estop_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void reset_estop_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

#endif  // ARM_TELEOP__ARM_SAFETY_NODE_HPP_