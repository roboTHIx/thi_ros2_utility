#ifndef THIODOMTOTF_H_
#define THIODOMTOTF_H_


#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "nav_msgs/msg/odometry.hpp"

class ThiOdomToTf : public rclcpp::Node{
public:
  ThiOdomToTf() : Node("thi_odom_to_tf_node")
  {
    //params
    // this->declare_parameter<std::string>("target_frame",              _params.target_frame);
    // _params.target_frame            = this->get_parameter("target_frame").as_string();

    // RCLCPP_INFO(_logger, "------------------------------------------------------");
    // RCLCPP_INFO(_logger, "-------    Parameter, THI_OdomToTf_Node      ----------");
    // RCLCPP_INFO(_logger, "------------------------------------------------------");
    // RCLCPP_INFO(_logger, "target_frame            : %s", _params.target_frame.c_str());
    // RCLCPP_INFO(_logger, "------------------------------------------------------");
    // RCLCPP_INFO(_logger, "------------------------------------------------------");

    _clock = this->get_clock();
    _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    _sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&ThiOdomToTf::sub_odom_callback, this, std::placeholders::_1));
    //  this->create_publisher<std_msgs::msg::String>("topic", 10);
    _pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom_base", 10);

  }
  virtual ~ThiOdomToTf() = default;

private: //fcn
  void sub_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // RCLCPP_INFO(_logger, "got odom");
    geometry_msgs::msg::TransformStamped t;
    auto new_msg = *msg;
    new_msg.header.frame_id = "odom_base";
    new_msg.header.stamp = _clock->now();
    _pub_odom->publish(new_msg);

    t.header = msg->header;
    t.header.frame_id = "odom_base";
    t.header.stamp = _clock->now();
    t.child_frame_id = msg->child_frame_id;

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;

    _tf_broadcaster->sendTransform(t);
  }

  
private:
  // struct ThiOdomToTf_params
  // {
  //   std::string target_frame = "base_link";
  // } _params;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  _sub_odom;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_odom;
  std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
  
  rclcpp::Logger _logger {rclcpp::get_logger("thi_odom_to_tf_node")};

  rclcpp::Clock::SharedPtr _clock;
};

#endif  //THIODOMTOTF_H_