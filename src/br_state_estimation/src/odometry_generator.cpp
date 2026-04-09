#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class OdometryConverter : public rclcpp::Node
{
    public:
    OdometryConverter() : Node("odometry_generator"){

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 5, std::bind(&OdometryConverter::cmdVelCallback, this, _1));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("arti_odom", 100);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        odom_pub_timer_ = this->create_wall_timer(10ms, std::bind(&OdometryConverter::odomPubTimerCallback, this));

    }
    private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::ConstPtr msg){
        this->last_twist_ = *msg;
    }
    void odomPubTimerCallback(){
        // geometry_msgs::msg::TransformStamped tf_result;
        // try{
        //     tf_result = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePoint(0ms));
        // }
        // catch (tf2::TransformException & ex) {
        //     RCLCPP_WARN_SKIPFIRST_THROTTLE(
        //         get_logger(), *get_clock(), (5000ms).count(),
        //         "cannot get map to base_link transform. %s", ex.what());
        //     return;
        // }
        
        nav_msgs::msg::Odometry out_msg;
        out_msg.twist.twist = this->last_twist_;
        // out_msg.pose.pose.position.x = tf_result.transform.translation.x;
        // out_msg.pose.pose.position.y = tf_result.transform.translation.y;
        // out_msg.pose.pose.position.z = tf_result.transform.translation.z;
        // out_msg.pose.pose.orientation = tf_result.transform.rotation;
        out_msg.header.stamp = this->get_clock()->now();
        out_msg.header.frame_id = "odom";
        out_msg.child_frame_id = "base_link";
        for(size_t i = 0; i < 36; i += 7) out_msg.twist.covariance[i] = 0.05;
        for(size_t i = 0; i < 36; i += 7) out_msg.pose.covariance[i] = 0.1;

        this->odom_pub_->publish(out_msg);
        RCLCPP_DEBUG(this->get_logger(), "Sent out odometry msg");
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::Twist last_twist_;
    rclcpp::TimerBase::SharedPtr odom_pub_timer_;

    

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    OdometryConverter::SharedPtr oc(new OdometryConverter());
    rclcpp::spin(oc);
}