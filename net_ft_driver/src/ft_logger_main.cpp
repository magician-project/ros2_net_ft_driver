#include <rclcpp/rclcpp.hpp>
#include <net_ft_driver/FT_logger.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<net_ft_driver::FTLogger>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}