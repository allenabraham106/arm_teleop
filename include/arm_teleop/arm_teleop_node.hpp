#ifndef ARM_TELEOP__ARM_TELEOP_NODE_HPP_
#define ARM_TELEOP__ARM_TELEOP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <utility>
#include <string>
#include <vector>

constexpr uint32_t CONTROL_MODE_VELOCITY  = 2;
constexpr uint32_t INPUT_MODE_PASSTHROUGH = 1;

class ArmTeleopNode : public rclcpp::Node{
  public:
    ArmTeleopNode();
    ~ArmTeleopNode();

  private:
    bool enabled_= false;
    bool estopped_ = false;
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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estopped_sub_;

    // Sim mode publisher (also used for real hardware, once arm_bringup's
    // real-hardware launch exists — same forward_velocity_controller topic
    // either way. See ros2_control follow-up.)
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr estop_client_;

    double deadband(double val) const;
    double normalize_trigger(double raw) const;
    std::pair<double, double> dominant_axis_lock(double x, double y) const;
    void publish_zeros();
    void estopped_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
};

#endif