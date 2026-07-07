#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <odrive_can/msg/control_message.hpp>
#include <odrive_can/srv/axis_state.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <algorithm>

constexpr int CONTROL_MODE_VELOCITY  = 2;
constexpr int INPUT_MODE_PASSTHROUGH = 1;
constexpr uint32_t AXIS_STATE_IDLE = 1;
constexpr uint32_t AXIS_STATE_CLOSED_LOOP_CONTROL = 8;

const std::vector<std::string> JOINT_TOPICS = {
  "/odrive_node_0/control_message",  // J1 - base
  "/odrive_node_1/control_message",  // J2 - shoulder
  "/odrive_node_2/control_message",  // J3 - elbow
  "/odrive_node_3/control_message",  // J4 - forearm
  "/odrive_node_4/control_message",  // J5 - wrist pitch
  "/odrive_node_5/control_message",  // J6 - wrist roll
};

const std::vector<std::string> ODRIVE_NODE_NS ={
  "odrive_node_0", "odrive_node_1", "odrive_node_2",
  "odrive_node_3", "odrive_node_4", "odrive_node_5",
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

      joint_ready_.resize(ODRIVE_NODE_NS.size(), false);

      // Publishers
      if (sim_mode_) {
        sim_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "/forward_velocity_controller/commands", 10);
        RCLCPP_INFO(this->get_logger(), "Running in SIM mode.");
      } else {
        for (const auto & topic : JOINT_TOPICS){
          joint_pubs_.push_back(
            this->create_publisher<odrive_can::msg::ControlMessage>(topic, 10));
        }
        for (const auto & ns: ODRIVE_NODE_NS){
          axis_state_clients_.push_back(
          this->create_client<odrive_can::srv::AxisState>(ns + "/request_axis_state")
        );
        clear_errors_clients_.push_back(
          this->create_client<std_srvs::srv::Empty>(ns + "/clear_errors")
        );
        }
        RCLCPP_INFO(this->get_logger(), "Running in REAL hardware mode.");
      }

      // Subscriber
      joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&ArmTeleopNode::joy_callback, this, std::placeholders::_1)
      );
      
      // recovery from estop is only on groundstation not controller
      reset_estop_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/reset_estop",
        std::bind(&ArmTeleopNode::reset_estop_callback, this, std::placeholders::_1, std::placeholders::_2)
      );

      RCLCPP_INFO(this->get_logger(),
        "Arm teleop ready. Cross to enable. Circle for e-stop. Options to re-arm.");
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_estop_srv_;


    ~ArmTeleopNode() { publish_zeros(); }

  private:
    bool enabled_= false;
    bool estopped_ = false;
    bool sim_mode_ = true;
    bool rearm_pending_ = false;
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
        for (size_t i = 0; i < joint_pubs_.size(); ++i){
          publish_velocity(i, 0.0);
        }
      }
    }

    void request_all_axis_states(uint32_t state){
      if(sim_mode_){
        return; 
      }
      auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
      request->axis_requested_state = state;
      for(size_t i = 0; i < axis_state_clients_.size(); ++i){
        if(!axis_state_clients_[i]->service_is_ready()){
          RCLCPP_WARN(this->get_logger(), "Joint %zu axis_state service not available", i);
          continue;
        }
        axis_state_clients_[i]->async_send_request(
          request,
          [this, i](rclcpp::Client<odrive_can::srv::AxisState>::SharedFuture future){
            auto result = future.get();
            if(result->procedure_result != 0){
              RCLCPP_ERROR(this->get_logger(),
                "Joint %zu failed to reach requested state (procedure_result=%u, active_errors=0x%x)",
                i, result->procedure_result, result->active_errors);
            }
          }
        );
      }
    }

    void begin_rearm_sequence(){
      if(sim_mode_){
        estopped_ = false;
        RCLCPP_INFO(this->get_logger(), "Re-armed in sim");
        return;
      }
      rearm_pending_ = true;
      std::fill(joint_ready_.begin(), joint_ready_.end(), false);
      RCLCPP_INFO(this->get_logger(), "Re-arm requested, waiting for joint confirmation");
      for(size_t i = 0; i < clear_errors_clients_.size(); ++i){
        request_joint_rearm(i);
      }
    }

    void request_joint_rearm(size_t i){
      if (!clear_errors_clients_[i]->service_is_ready()){
        RCLCPP_WARN(this->get_logger(), "Joint %zu clear_errors not available yet", i);
        return;
      }
      auto empty_req = std::make_shared<std_srvs::srv::Empty::Request>();
      clear_errors_clients_[i]->async_send_request(
        empty_req,
        [this, i](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture){
          if (!axis_state_clients_[i]->service_is_ready()) return;
          auto state_req = std::make_shared<odrive_can::srv::AxisState::Request>();
          state_req->axis_requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
          axis_state_clients_[i]->async_send_request(state_req,
            [this, i](rclcpp::Client<odrive_can::srv::AxisState>::SharedFuture future){
              auto result = future.get();
              if (result->procedure_result == 0 && result->axis_state == AXIS_STATE_CLOSED_LOOP_CONTROL){
                joint_ready_[i] = true;
                RCLCPP_INFO(this->get_logger(), "Joint %zu confirmed closed-loop.", i);
                check_rearm_complete();
              } else {
                RCLCPP_ERROR(this->get_logger(),
                  "Joint %zu did NOT reach closed-loop (procedure_result=%u, axis_state=%u, active_errors=0x%x)",
                  i, result->procedure_result, result->axis_state, result->active_errors);
              }
            });
        });
    }

    void check_rearm_complete(){
      if (std::all_of(joint_ready_.begin(), joint_ready_.end(), [](bool b){ return b; })){
        estopped_ = false;
        rearm_pending_ = false;
        RCLCPP_INFO(this->get_logger(), "All joints confirmed. Re-armed. Press Cross to enable.");
      }
    }

    // only from the groundstation only not available fro the controller
    void reset_estop_callback(
       const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response){
      if (!estopped_){
        response->success = false;
        response->message = "Not e-stopped.";
        return;
      }
      if (rearm_pending_){
        response->success = false;
        response->message = "Re-arm already in progress.";
        return;
      }
      begin_rearm_sequence();
      response->success = true;
      response->message = "Re-arm sequence started — check node logs for per-joint confirmation.";
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
      bool btn_enable = msg->buttons[btn_enable_];
      bool btn_estop  = msg->buttons[btn_estop_];
      bool btn_speed  = msg->buttons[btn_speed_];

      // Cross: toggle enable
      if (btn_enable && !prev_btn_enable_){
        if (!estopped_) {
          enabled_ = !enabled_;
          RCLCPP_INFO(this->get_logger(), "Teleop %s", enabled_ ? "ENABLED" : "DISABLED");
        } else {
          RCLCPP_WARN(this->get_logger(), "E-stop active.");
        }
      }

      // Circle: e-stop
      if (btn_estop && !prev_btn_estop_){
        estopped_      = true;
        enabled_       = false;
        rearm_pending_ = false;
        std::fill(joint_ready_.begin(), joint_ready_.end(), false);
        publish_zeros();
        request_all_axis_states(AXIS_STATE_IDLE);
        RCLCPP_ERROR(this->get_logger(), "E-STOP! All joints commanded IDLE. Recovery via reset_estop service only.");
      }

      prev_btn_enable_ = btn_enable;
      prev_btn_estop_  = btn_estop;
      prev_btn_speed_  = btn_speed;

      if (!enabled_ || estopped_){
        publish_zeros();
        return;
      }

      double scale = max_vel_; 

      // left joystick domaninant axis lock
      double left_x = deadband(msg->axes[axis_left_x_]) * scale;
      double left_y = -deadband(msg->axes[axis_left_y_]) * scale;
      double j1, j2;
      if(std::abs(left_x) >= std::abs(left_y)){
        j1 = left_x;
        j2 = 0.0;
      }else{
        j1 = 0.0; 
        j2 = left_y;
      }

      // right joystick axis lock
      double right_x = -deadband(msg->axes[axis_right_x_]) * scale;
      double right_y = deadband(msg->axes[axis_right_y_]) * scale;
      double j3, j4; 
      if(std::abs(right_y) >= std::abs(right_x)){
        j3 = right_y;
        j4 = 0.0;
      }else{
        j3 = 0.0; 
        j4 = right_x;
      }

      double pitch = msg->axes[axis_dpad_y_] * dpad_vel_;
      double roll = msg->axes[axis_dpad_x_] * dpad_vel_;
      double j5 = pitch + roll; 
      double j6 = pitch - roll;
 
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