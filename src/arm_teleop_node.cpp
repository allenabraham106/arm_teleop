#include "arm_teleop/arm_teleop_node.hpp"

ArmTeleopNode::ArmTeleopNode() : Node("arm_teleop_node"){
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
  this->declare_parameter("btn_speed",  4);

  // Velocity params
  this->declare_parameter("deadband",          0.08);
  this->declare_parameter("dpad_velocity",     0.3);
  this->declare_parameter("gripper_velocity",  0.5);
  this->declare_parameter("max_velocity", 1.5);

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
  btn_speed_  = this->get_parameter("btn_speed").as_int();

  // Load velocity params
  deadband_ = this->get_parameter("deadband").as_double();
  dpad_vel_ = this->get_parameter("dpad_velocity").as_double();
  gripper_vel_ = this->get_parameter("gripper_velocity").as_double();
  max_vel_ = this->get_parameter("max_velocity").as_double();

  // Publishers
  vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "forward_velocity_controller/commands",
    10
  );

  // Subscriber
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&ArmTeleopNode::joy_callback, this, std::placeholders::_1)
  );

  rclcpp::QoS latched_qos(1);
  latched_qos.transient_local();
  estopped_sub_ = this->create_subscription<arm_teleop_messages::msg::EstopStatus>(
    "estopped",
    latched_qos,
    std::bind(&ArmTeleopNode::estopped_callback, this, std::placeholders::_1)
  );

  estop_client_ = this->create_client<std_srvs::srv::Trigger>(
    "estop"
  );

  RCLCPP_INFO(this->get_logger(),
    "Arm teleop ready. Cross to enable. Circle for e-stop. "
    "E-stop recovery is via arm_safety_node's reset_estop service only."
  );
}

ArmTeleopNode::~ArmTeleopNode() { publish_zeros(); }

double ArmTeleopNode::deadband(double val) const{
  if (std::abs(val) > deadband_){
    return val;
  }else{
    return 0.0;
  }
}

double ArmTeleopNode::normalize_trigger(double raw) const{
  return (raw + 1.0) / 2.0;
}

// joystick dominant possession
std::pair<double, double> ArmTeleopNode::dominant_axis_lock(double x, double y) const{
  if(std::abs(x) >= std::abs(y)){
    return {x, 0};
  }else{
   return {0, y};
 }
}

void ArmTeleopNode::publish_zeros(){
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0
  };
  vel_pub_->publish(msg);
}

void ArmTeleopNode::estopped_callback(const arm_teleop_messages::msg::EstopStatus::SharedPtr msg){
  estopped_ = msg->estopped;
  rearm_pending_ = msg->rearm_pending;
}

void ArmTeleopNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
  bool btn_enable = msg->buttons[btn_enable_];
  bool btn_estop  = msg->buttons[btn_estop_];
  bool btn_speed  = msg->buttons[btn_speed_];

  // Cross: toggle enable
  if (btn_enable && !prev_btn_enable_){
    if (!estopped_) {
      enabled_ = !enabled_;
      RCLCPP_INFO(this->get_logger(), "Teleop %s", enabled_ ? "ENABLED" : "DISABLED");
    } else {
      RCLCPP_WARN(this->get_logger(), rearm_pending_
        ? "E-stop latched, re-arm already in progress."
        : "E-stop latched. Recovery is via arm_safety_node's reset_estop service, not a controller button."
      );
    }
  }

  // Circle: e-stop
  if (btn_estop && !prev_btn_estop_){
    estopped_ = true;
    enabled_  = false;
    publish_zeros();

    if (!estop_client_->service_is_ready()){
      RCLCPP_ERROR(this->get_logger(), "arm_safety_node's estop service not available!");
    } else {
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      estop_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
          auto result = future.get();
          if (!result->success){
            RCLCPP_ERROR(this->get_logger(), "estop service call failed: %s", result->message.c_str());
          }
        });
    }
    RCLCPP_ERROR(this->get_logger(), "E-STOP triggered.");
  }

  prev_btn_enable_ = btn_enable;
  prev_btn_estop_  = btn_estop;
  prev_btn_speed_  = btn_speed;

  if (!enabled_ || estopped_){
    publish_zeros();
    return;
  }

  double scale = max_vel_;

  double left_x = deadband(msg->axes[axis_left_x_]) * scale;
  double left_y = -deadband(msg->axes[axis_left_y_]) * scale;
  auto [j1, j2] = dominant_axis_lock(left_x, left_y);
  
  double right_x = -deadband(msg->axes[axis_right_x_]) * scale;
  double right_y = -deadband(msg->axes[axis_right_y_]) * scale;
  auto [j3, j4] = dominant_axis_lock(right_x, right_y);

  double pitch = msg->axes[axis_dpad_y_] * dpad_vel_;
  double roll = msg->axes[axis_dpad_x_] * dpad_vel_;
  double j5 = pitch + roll;
  double j6 = pitch - roll;

  double largest = std::max(std::abs(j5), std::abs(j6));
  if (largest > dpad_vel_){
    double norm = dpad_vel_ / largest;
    j5 *= norm;
    j6 *= norm;
  }
  double lt = normalize_trigger(msg->axes[axis_l2_]);
  double rt = normalize_trigger(msg->axes[axis_r2_]);
  double gripper = (lt - rt) * gripper_vel_;  // + = close, - = open

  std_msgs::msg::Float64MultiArray vel_msg;
  vel_msg.data = {j1, j2, j3, j4, j5, j6};
  vel_pub_->publish(vel_msg);

  // TODO: wire gripper to its actual topic
  (void)gripper;
}

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmTeleopNode>());
  rclcpp::shutdown();
  return 0;
}