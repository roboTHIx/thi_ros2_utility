#include "thi_odom_to_tf/ThiOdomToTf.hpp"

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThiOdomToTf>());
  rclcpp::shutdown();

  return 0;
}