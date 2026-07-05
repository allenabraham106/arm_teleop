#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <odrive_can/msg/control_message.hpp>

constexpr int CONTROL_MODE_VELOCITY  = 2;
constexpr int INPUT_MODE_PASSTHROUGH = 1;

const std::vector<std::string> JOINT_TOPICS = {
  "/odrive_node_0/control_message",  // J1 - base
  "/odrive_node_1/control_message",  // J2 - shoulder
  "/odrive_node_2/control_message",  // J3 - elbow
  "/odrive_node_3/control_message",  // J4 - forearm
  "/odrive_node_4/control_message",  // J5 - wrist pitch
  "/odrive_node_5/control_message",  // J6 - wrist roll
};

class ArmTeleopNode : public rclcpp::Node{
  public:
    ArmTeleopNode() : Node("arm_teleop_node"){
      this->declare_parameter("sim_mode", true);
      sim_mode_ = this->get_parameter("sim_mode").as_bool();
      // Axis params
      this->declare_parameter("axis_left_x",  0);
      this->declare_parameter("axis_left_y",  1);
      this->declare_parameter("axis_right_x", 3);
      this->declare_parameter("axis_right_y", 4);
      this->declare_parameter("axis_l2",      2);
      this->declare_parameter("axis_r2",      5);
      this->declare_parameter("axis_dpad_x",  6);
      this->declare_parameter("axis_dpad_y",  7);

      // Button params
      this->declare_parameter("btn_enable", 0);
      this->declare_parameter("btn_estop",  1);
      this->declare_parameter("btn_rearm",  9);
      this->declare_parameter("btn_speed",  4);

      // Velocity params
      this->declare_parameter("max_velocity_slow", 0.5);
      this->declare_parameter("max_velocity_fast", 1.5);
      this->declare_parameter("deadband",          0.08);
      this->declare_parameter("dpad_velocity",     0.3);
      this->declare_parameter("gripper_velocity",  0.5);

      // Load axis indices
      axis_left_x_  = this->get_parameter("axis_left_x").as_int();
      axis_left_y_  = this->get_parameter("axis_left_y").as_int();
      axis_right_x_ = this->get_parameter("axis_right_x").as_int();
      axis_right_y_ = this->get_parameter("axis_right_y").as_int();
      axis_l2_      = this->get_parameter("axis_l2").as_int();
      axis_r2_      = this->get_parameter("axis_r2").as_int();
      axis_dpad_x_  = this->get_parameter("axis_dpad_x").as_int();
      axis_dpad_y_  = this->get_parameter("axis_dpad_y").as_int();

      // Load button indices
      btn_enable_ = this->get_parameter("btn_enable").as_int();
      btn_estop_  = this->get_parameter("btn_estop").as_int();
      btn_rearm_  = this->get_parameter("btn_rearm").as_int();
      btn_speed_  = this->get_parameter("btn_speed").as_int();

      // Load velocity params
      max_vel_slow_   = this->get_parameter("max_velocity_slow").as_double();
      max_vel_fast_   = this->get_parameter("max_velocity_fast").as_double();
      deadband_       = this->get_parameter("deadband").as_double();
      dpad_vel_       = this->get_parameter("dpad_velocity").as_double();
      gripper_vel_    = this->get_parameter("gripper_velocity").as_double();

      // Publishers
      if (sim_mode_) {
        sim_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "/forward_velocity_controller/commands", 10);
        RCLCPP_INFO(this->get_logger(), "Running in SIM mode.");
      } else {
        for (const auto & topic : JOINT_TOPICS) {
          joint_pubs_.push_back(
            this->create_publisher<odrive_can::msg::ControlMessage>(topic, 10));
        }
        RCLCPP_INFO(this->get_logger(), "Running in REAL hardware mode.");
      }

      // Subscriber
      joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&ArmTeleopNode::joy_callback, this, std::placeholders::_1));

      RCLCPP_INFO(this->get_logger(),
        "Arm teleop ready. Cross to enable. Circle for e-stop. Options to re-arm.");
    }

    ~ArmTeleopNode() { publish_zeros(); }

  private:
    bool enabled_= false;
    bool estopped_ = false;
    bool fast_mode_= false;
    bool sim_mode_ = true;
    bool prev_btn_enable_= false;
    bool prev_btn_estop_ = false;
    bool prev_btn_rearm_ = false;
    bool prev_btn_speed_ = false;

    // Axis indices
    int axis_left_x_, axis_left_y_, axis_right_x_, axis_right_y_;
    int axis_l2_, axis_r2_, axis_dpad_x_, axis_dpad_y_;

    // Button indices
    int btn_enable_, btn_estop_, btn_rearm_, btn_speed_;

    // Velocity params
    double max_vel_slow_, max_vel_fast_, deadband_, dpad_vel_, gripper_vel_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Sim mode publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr sim_pub_;

    // Real hardware publishers
    std::vector<rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr> joint_pubs_;

    double deadband(double val) const{
      if (std::abs(val) > deadband_){
        return val;
      }else{
        return 0.0;
      }
    }

    double normalize_trigger(double raw) const{
      return (raw + 1.0) / 2.0; 
    }

    void publish_velocity(size_t joint_idx, double velocity){
      if (joint_idx >= joint_pubs_.size()) return;
      odrive_can::msg::ControlMessage msg;
      msg.control_mode = CONTROL_MODE_VELOCITY;
      msg.input_mode   = INPUT_MODE_PASSTHROUGH;
      msg.input_vel    = static_cast<float>(velocity);
      msg.input_pos    = 0.0f;
      msg.input_torque = 0.0f;
      joint_pubs_[joint_idx]->publish(msg);
    }

    void publish_zeros(){
      if (sim_mode_){
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        sim_pub_->publish(msg);
      }else{
        for (size_t i = 0; i < joint_pubs_.size(); ++i) {
          publish_velocity(i, 0.0);
        }
      }
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
      bool btn_enable = msg->buttons[btn_enable_];
      bool btn_estop  = msg->buttons[btn_estop_];
      bool btn_rearm  = msg->buttons[btn_rearm_];
      bool btn_speed  = msg->buttons[btn_speed_];

      // Cross: toggle enable
      if (btn_enable && !prev_btn_enable_){
        if (!estopped_) {
          enabled_ = !enabled_;
          RCLCPP_INFO(this->get_logger(), "Teleop %s", enabled_ ? "ENABLED" : "DISABLED");
        } else {
          RCLCPP_WARN(this->get_logger(), "E-stop active — press Options to re-arm first.");
        }
      }

      // Circle: e-stop
      if (btn_estop && !prev_btn_estop_){
        estopped_ = true;
        enabled_  = false;
        publish_zeros();
        RCLCPP_ERROR(this->get_logger(), "E-STOP! Press Options to re-arm.");
      }

      // Options: re-arm
      if (btn_rearm && !prev_btn_rearm_ && estopped_){
        estopped_ = false;
        RCLCPP_INFO(this->get_logger(), "Re-armed. Press Cross to enable.");
      }

      // L1: speed toggle
      if (btn_speed && !prev_btn_speed_) {
        fast_mode_ = !fast_mode_;
        RCLCPP_INFO(this->get_logger(), "Speed: %s", fast_mode_ ? "FAST" : "SLOW");
      }

      prev_btn_enable_ = btn_enable;
      prev_btn_estop_  = btn_estop;
      prev_btn_rearm_  = btn_rearm;
      prev_btn_speed_  = btn_speed;

      if (!enabled_ || estopped_){
        publish_zeros();
        return;
      }

      double scale;
      if (fast_mode_){
        scale = max_vel_fast_;
      }else {
        scale = max_vel_slow_;
      }

      double j1 =  deadband(msg->axes[axis_left_x_])  * scale;
      double j2 = -deadband(msg->axes[axis_left_y_])  * scale;
      double j3 =  deadband(msg->axes[axis_right_x_]) * scale;
      double j4 = -deadband(msg->axes[axis_right_y_]) * scale;
      double j5 =  msg->axes[axis_dpad_y_] * dpad_vel_;
      double j6 =  msg->axes[axis_dpad_x_] * dpad_vel_;

      double lt = normalize_trigger(msg->axes[axis_l2_]);
      double rt = normalize_trigger(msg->axes[axis_r2_]);
      double gripper = (lt - rt) * gripper_vel_;  // + = close, - = open

      if (sim_mode_) {
        std_msgs::msg::Float64MultiArray vel_msg;
        vel_msg.data = {j1, j2, j3, j4, j5, j6};
        sim_pub_->publish(vel_msg);
      } else {
        publish_velocity(0, j1);
        publish_velocity(1, j2);
        publish_velocity(2, j3);
        publish_velocity(3, j4);
        publish_velocity(4, j5);
        publish_velocity(5, j6);
      }

      // TODO: wire gripper to its actual topic
      (void)gripper;
    }
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmTeleopNode>());
  rclcpp::shutdown();
  return 0;
}