#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <odrive_can/msg/control_message.hpp>
#include <chrono>

const std::vector<std::string> JOINT_NAMES = {
  "Joint_1", "Joint_2", "Joint_3",
  "Joint_4", "Joint_5", "Joint_6"
};

const std::vector<std::string> JOINT_TOPICS = {
  "/odrive_node_0/control_message",
  "/odrive_node_1/control_message",
  "/odrive_node_2/control_message",
  "/odrive_node_3/control_message",
  "/odrive_node_4/control_message",
  "/odrive_node_5/control_message",
};

class ArmJointSimNode : public rclcpp::Node
{
public:
  ArmJointSimNode() : Node("arm_joint_sim_node")
  {
    positions_.resize(6, 0.0);
    velocities_.resize(6, 0.0);
    last_time_ = this->now();

    // Subscribe to each ODrive control message topic
    for (size_t i = 0; i < JOINT_TOPICS.size(); ++i) {
      subs_.push_back(
        this->create_subscription<odrive_can::msg::ControlMessage>(
          JOINT_TOPICS[i], 10,
          [this, i](const odrive_can::msg::ControlMessage::SharedPtr msg) {
            velocities_[i] = msg->input_vel;
          }
        )
      );
    }

    // Publisher
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", 10);

    // Timer at 50Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ArmJointSimNode::update, this));

    RCLCPP_INFO(this->get_logger(), "Arm joint sim node started.");
  }

private:
  std::vector<double> positions_;
  std::vector<double> velocities_;
  rclcpp::Time last_time_;

  std::vector<rclcpp::Subscription<odrive_can::msg::ControlMessage>::SharedPtr> subs_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void update()
  {
    auto now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    // Integrate velocity -> position
    for (size_t i = 0; i < 6; ++i) {
      positions_[i] += velocities_[i] * dt;
    }

    // Publish joint states
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now;
    msg.name     = JOINT_NAMES;
    msg.position = positions_;
    msg.velocity = velocities_;
    joint_state_pub_->publish(msg);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmJointSimNode>());
  rclcpp::shutdown();
  return 0;
}