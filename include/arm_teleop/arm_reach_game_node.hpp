#ifndef ARM_TELEOP__ARM_REACH_GAME_NODE_HPP_
#define ARM_TELEOP__ARM_REACH_GAME_NODE_HPP_

#include <sensor_msgs/msg/joy.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <string>
#include <array>


class ArmReachGameNode : public rclcpp::Node{
    public:
        ArmReachGameNode();
    private: 
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        rclcpp::Time leg_start_time_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr leg_time_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string base_frame_;
        std::string ee_frame_;
        std::array<double, 3> box_a_pos_;
        std::array<double, 3> box_b_pos_;
        
        int btn_stop_;
        bool prev_btn_stop_ = false;
        bool game_stopped_ = false;
        bool game_started_ = false;
        double best_time_sec_ = -1.0;
        int run_count_ = 0;
        enum class Target {BOX_A, BOX_B};
        Target current_target_ = Target::BOX_A;
        double hit_radius_;


        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_sub_);
        void timer_callback();
        void publish_markers();
};

#endif