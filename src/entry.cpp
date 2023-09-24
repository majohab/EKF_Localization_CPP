#include "rclcpp/rclcpp.hpp"
#include "ekf_localization/ekf_localization.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfROSWrapper>());
  rclcpp::shutdown();
  return 0;
}