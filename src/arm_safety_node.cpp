#include "arm_teleop/arm_safety_node.hpp"

ArmSafetyNode::ArmSafetyNode() : Node("arm_safety_node"){
  this->declare_parameter("sim_mode", true);
  sim_mode_ = this->get_parameter("sim_mode").as_bool();

  joint_ready_.resize(ODRIVE_NODE_NS.size(), false);

  if (!sim_mode_){
    for (const auto & ns : ODRIVE_NODE_NS){
      axis_state_clients_.push_back(
        this->create_client<odrive_can::srv::AxisState>(ns + "/request_axis_state"));
      clear_errors_clients_.push_back(
        this->create_client<std_srvs::srv::Empty>(ns + "/clear_errors"));
    }
  }

  rclcpp::QoS latched_qos(1);
  latched_qos.transient_local();
  estopped_pub_ = this->create_publisher<std_msgs::msg::Bool>("estopped", latched_qos);

  estop_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "estop",
    std::bind(&ArmSafetyNode::estop_callback, this, std::placeholders::_1, std::placeholders::_2));

  reset_estop_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "reset_estop",
    std::bind(&ArmSafetyNode::reset_estop_callback, this, std::placeholders::_1, std::placeholders::_2));

  publish_estopped_status();

  RCLCPP_INFO(this->get_logger(),
    "arm_safety_node ready (%s). estop/reset_estop services active.",
    sim_mode_ ? "sim" : "real hardware");
}

void ArmSafetyNode::publish_estopped_status(){
  std_msgs::msg::Bool msg;
  msg.data = estopped_;
  estopped_pub_->publish(msg);
}

void ArmSafetyNode::request_all_axis_states(uint32_t state){
  if (sim_mode_) return;
  auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
  request->axis_requested_state = state;
  for (size_t i = 0; i < axis_state_clients_.size(); ++i){
    if (!axis_state_clients_[i]->service_is_ready()){
      RCLCPP_WARN(this->get_logger(), "Joint %zu axis_state service not available", i);
      continue;
    }
    axis_state_clients_[i]->async_send_request(
      request,
      [this, i](rclcpp::Client<odrive_can::srv::AxisState>::SharedFuture future){
        auto result = future.get();
        if (result->procedure_result != 0){
          RCLCPP_ERROR(this->get_logger(),
            "Joint %zu failed to reach requested state (procedure_result=%u, active_errors=0x%x)",
            i, result->procedure_result, result->active_errors);
        }
      });
  }
}

void ArmSafetyNode::begin_rearm_sequence(){
  if (sim_mode_){
    estopped_ = false;
    publish_estopped_status();
    RCLCPP_INFO(this->get_logger(), "Re-armed (sim).");
    return;
  }
  rearm_pending_ = true;
  std::fill(joint_ready_.begin(), joint_ready_.end(), false);
  RCLCPP_WARN(this->get_logger(), "Re-arm requested — waiting on all joints to confirm closed-loop.");
  for (size_t i = 0; i < clear_errors_clients_.size(); ++i){
    request_joint_rearm(i);
  }
}

void ArmSafetyNode::request_joint_rearm(size_t i){
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

void ArmSafetyNode::check_rearm_complete(){
  if (std::all_of(joint_ready_.begin(), joint_ready_.end(), [](bool b){ return b; })){
    estopped_ = false;
    rearm_pending_ = false;
    publish_estopped_status();
    RCLCPP_INFO(this->get_logger(), "All joints confirmed. Re-armed.");
  }
}

void ArmSafetyNode::estop_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response){
  estopped_ = true;
  rearm_pending_ = false;
  std::fill(joint_ready_.begin(), joint_ready_.end(), false);
  publish_estopped_status();
  request_all_axis_states(AXIS_STATE_IDLE);
  RCLCPP_ERROR(this->get_logger(), "E-STOP! All joints commanded IDLE. Recovery via reset_estop service only.");
  response->success = true;
  response->message = "E-stop engaged. All joints commanded IDLE.";
}

// only from the groundstation, not available from the controller
void ArmSafetyNode::reset_estop_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
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

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmSafetyNode>());
  rclcpp::shutdown();
  return 0;
}