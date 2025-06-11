#include <rclcpp/rclcpp.hpp>
#include <net_ft_driver/ATIFTSensor.hpp>

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<net_ft_driver::ATIFTSensor>());
    rclcpp::shutdown();
    return 0;
}