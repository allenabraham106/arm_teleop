#ifndef ARM_TELEOP__ARM_TELEOP_NODE_HPP_
#define ARM_TELEOP__ARM_TELEOP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <odrive_can/msg/control_message.hpp>
#include <odrive_can/srv/axis_state.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <algorithm>
#include <string>
#include <vector>
#include <utility>

constexpr uint32_t CONTROL_MODE_VELOCITY  = 2;
constexpr uint32_t INPUT_MODE_PASSTHROUGH = 1;
constexpr uint32_t AXIS_STATE_IDLE = 1;
constexpr uint32_t AXIS_STATE_CLOSED_LOOP_CONTROL = 8;

const std::vector<std::string> JOINT_TOPICS = {
  "odrive_node_0/control_message",  // J1 - base
  "odrive_node_1/control_message",  // J2 - shoulder
  "odrive_node_2/control_message",  // J3 - elbow
  "odrive_node_3/control_message",  // J4 - forearm
  "odrive_node_4/control_message",  // J5 - wrist pitch
  "odrive_node_5/control_message",  // J6 - wrist roll
};

const std::vector<std::string> ODRIVE_NODE_NS = {
  "odrive_node_0", "odrive_node_1", "odrive_node_2",
  "odrive_node_3", "odrive_node_4", "odrive_node_5",
};

class ArmTeleopNode : public rclcpp::Node{
  public:
    ArmTeleopNode();
    ~ArmTeleopNode();

  private:
    bool enabled_= false;
    bool estopped_ = false;
    bool rearm_pending_ = false;
    bool sim_mode_ = true;
    bool prev_btn_enable_= false;
    bool prev_btn_estop_ = false;
    bool prev_btn_speed_ = false;

    // Axis indices
    int axis_left_x_, axis_left_y_, axis_right_x_, axis_right_y_;
    int axis_l2_, axis_r2_, axis_dpad_x_, axis_dpad_y_;

    // Button indices
    int btn_enable_, btn_estop_, btn_speed_;

    // Velocity params
    double  max_vel_, deadband_, dpad_vel_, gripper_vel_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Sim mode publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sim_pub_;

    // Real hardware publishers
    std::vector<rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr> joint_pubs_;

    // Real hardware state controllers (estop and rearm)
    std::vector<rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr> axis_state_clients_;
    std::vector<rclcpp::Client<std_srvs::srv::Empty>::SharedPtr>       clear_errors_clients_;
    std::vector<bool> joint_ready_;

    // Ground-station-only recovery. Nothing on the pad can call this.
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_estop_srv_;

    double deadband(double val) const;
    double normalize_trigger(double raw) const;
    std::pair<double, double> dominant_axis_lock(double x, double y) const;
    void publish_velocity(size_t joint_idx, double velocity);
    void publish_zeros();
    void request_all_axis_states(uint32_t state);
    void begin_rearm_sequence();
    void request_joint_rearm(size_t i);
    void check_rearm_complete();
    void reset_estop_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
};

#endif  // ARM_TELEOP__ARM_TELEOP_NODE_HPP_