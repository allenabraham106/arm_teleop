#include "arm_teleop/arm_safety_node.hpp"

ArmSafetyNode::ArmSafetyNode() : Node("arm_safety_node"){
  hw_state_client_ = this->create_client<controller_manager_msgs::srv::SetHardwareComponentState>(
  "/controller_manager/set_hardware_component_state");

  rclcpp::QoS latched_qos(1);
  latched_qos.transient_local();
  estopped_pub_ = this->create_publisher<arm_teleop::msg::EstopStatus>("estopped", latched_qos);

  estop_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "estop",
    std::bind(&ArmSafetyNode::estop_callback, this, std::placeholders::_1, std::placeholders::_2));

  reset_estop_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "reset_estop",
    std::bind(&ArmSafetyNode::reset_estop_callback, this, std::placeholders::_1, std::placeholders::_2));

  publish_estopped_status();

  RCLCPP_INFO(this->get_logger(), "arm_safety_node ready. estop/reset_estop services active.");
}

void ArmSafetyNode::publish_estopped_status(){
  arm_teleop::msg::EstopStatus msg;
  msg.estopped = estopped_;
  msg.rearm_pending = rearm_pending_;
  estopped_pub_->publish(msg);
}

void ArmSafetyNode::begin_rearm_sequence(){
  rearm_pending_ = true;
  set_hardware_state("active");
}

void ArmSafetyNode::set_hardware_state(const std::string & label){
  if (!hw_state_client_->service_is_ready()){
    RCLCPP_ERROR(this->get_logger(), "controller_manager's set_hardware_component_state not available!");
    return;
  }
  auto request = std::make_shared<controller_manager_msgs::srv::SetHardwareComponentState::Request>();
  request->name = ARM_HARDWARE_COMPONENT;
  request->target_state.label = label;
  hw_state_client_->async_send_request(request,
    [this, label](rclcpp::Client<controller_manager_msgs::srv::SetHardwareComponentState>::SharedFuture future){
      auto result = future.get();
      if (!result->ok){
        RCLCPP_ERROR(this->get_logger(), "Failed to set ArmSystem to '%s'", label.c_str());
        return;
      }
      if (label == "active"){
        estopped_ = false;
        rearm_pending_ = false;
        publish_estopped_status();
        RCLCPP_INFO(this->get_logger(), "ArmSystem confirmed active. Re-armed.");
      }
    });
}

void ArmSafetyNode::estop_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response){
  estopped_ = true;
  rearm_pending_ = false;
  publish_estopped_status();
  set_hardware_state("inactive");
  RCLCPP_ERROR(this->get_logger(), "E-STOP! Setting ArmSystem to inactive.");
  response->success = true;
  response->message = "E-stop engaged. ArmSystem set to inactive.";

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
  response->message = "Re-arm requested. Check node logs for ArmSystem confirmation.";
}

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmSafetyNode>());
  rclcpp::shutdown();
  return 0;
}