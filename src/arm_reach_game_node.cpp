#include "arm_teleop/arm_reach_game_node.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <chrono>

ArmReachGameNode::ArmReachGameNode() : Node("arm_reach_game_node"){
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    leg_start_time_ = this->now();

    // TODO: Confirm that the options button is the correct mapping
    this->declare_parameter("btn_stop", 7);
    this->declare_parameter("base_frame", "Link_1");
    this->declare_parameter("ee_frame", "Gripper");
    this->declare_parameter("box_a_x", 0.3);
    this->declare_parameter("box_a_y", 0.2);
    this->declare_parameter("box_a_z", 0.3);
    this->declare_parameter("box_b_x", 0.2);
    this->declare_parameter("box_b_y", -0.22);
    this->declare_parameter("box_b_z", 0.15);
    this->declare_parameter("hit_radius", 0.05);

    btn_stop_ = this->get_parameter("btn_stop").as_int();
    base_frame_ = this->get_parameter("base_frame").as_string();
    ee_frame_ = this->get_parameter("ee_frame").as_string();
    box_a_pos_ = {
        this->get_parameter("box_a_x").as_double(),
        this->get_parameter("box_a_y").as_double(),
        this->get_parameter("box_a_z").as_double()
    };
    box_b_pos_ = {
        this->get_parameter("box_b_x").as_double(),
        this->get_parameter("box_b_y").as_double(),
        this->get_parameter("box_b_z").as_double()
    };
    hit_radius_ = this->get_parameter("hit_radius").as_double();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy_arm", 
        10, 
        std::bind(&ArmReachGameNode::joy_callback, this, std::placeholders::_1)
    );

    leg_time_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "reach_game_leg_time", 
        10
    );

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "reach_game_markers",
        10
    );

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&ArmReachGameNode::timer_callback, this)
    );
}

void ArmReachGameNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
    bool btn_stop = msg->buttons[btn_stop_];
    if(btn_stop && !prev_btn_stop_ && !game_stopped_){
        game_stopped_ = true;
        RCLCPP_INFO(this->get_logger(), "Game Completed.\n Legs completed %d\n Time taken: %.2fs", run_count_, best_time_sec_);
    }
    prev_btn_stop_ = btn_stop;
}

void ArmReachGameNode::timer_callback(){
    if(game_stopped_){
        return;
    }
    publish_markers();

    geometry_msgs::msg::TransformStamped tf;
    try{
        tf = tf_buffer_->lookupTransform(base_frame_, ee_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Can't get %s -> %s transform yet: %s", base_frame_.c_str(), ee_frame_.c_str(), ex.what());
        return;
    }

    const auto & target_box = (current_target_ == Target::BOX_A) ? box_a_pos_ : box_b_pos_;

    double x = tf.transform.translation.x; 
    double y = tf.transform.translation.y;
    double z = tf.transform.translation.z;
    double dx = target_box[0] - x; 
    double dy = target_box[1] - y;
    double dz = target_box[2] - z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    double elapsed = (this->now() - leg_start_time_).seconds();
    std_msgs::msg::Float64 msg;
    msg.data = elapsed;

    if (dist > hit_radius_){
        return;
    }

    if (!game_started_){
        game_started_ = true;
        leg_start_time_ = this->now();
        current_target_ = Target::BOX_B;
        RCLCPP_INFO(this->get_logger(), "Heading to Box B");
        return;
    }

    run_count_++;

    if(best_time_sec_ < 0.0 || elapsed < best_time_sec_){
        best_time_sec_ = elapsed;
    }

    leg_time_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Leg %d: %.2fs (best: %.2fs)", run_count_, elapsed, best_time_sec_);
    current_target_ = (current_target_ == Target::BOX_A) ? Target::BOX_B : Target::BOX_A;
    leg_start_time_ = this->now();
}

void ArmReachGameNode::publish_markers(){
    visualization_msgs::msg::MarkerArray markers;
    auto make_box = [&](int id, const std::array<double, 3> & pos, bool is_target){
        visualization_msgs::msg::Marker m;
        m.header.frame_id = base_frame_;
        m.header.stamp = this->now();
        m.ns = "reach_game";
        m.id = id;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = pos[0];
        m.pose.position.y = pos[1];
        m.pose.position.z = pos[2];
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = hit_radius_ * 2.0;
        m.color.a = 0.8;
        m.color.r = is_target ? 0.0 : 0.5;
        m.color.g = is_target ? 1.0 : 0.5;
        m.color.b = is_target ? 0.0 : 0.5;
        return m;
    };

    markers.markers.push_back(make_box(0, box_a_pos_, current_target_ == Target::BOX_A));
    markers.markers.push_back(make_box(1, box_b_pos_, current_target_ == Target::BOX_B));
    marker_pub_->publish(markers);
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmReachGameNode>());
    rclcpp::shutdown();
    return 0;
}