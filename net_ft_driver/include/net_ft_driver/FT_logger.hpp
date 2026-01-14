#pragma once

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

//Log
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
#include <ament_index_cpp/get_package_share_directory.hpp>


namespace net_ft_driver
{
class FTLogger : public rclcpp::Node
{
public:
	// Construct with optional NodeOptions so this can be composed or run standalone
	FTLogger (const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:

    rclcpp::TimerBase::SharedPtr timer_;
    double rate_;
    void timer_callback();


    bool initLogging();
    bool logData();
    Eigen::Matrix<double, 6, 1> wrench_eigen_;
    std::string logging_file_name_;
    XBot::MatLogger2::Ptr mat_logger_;
    XBot::MatAppender::Ptr mat_appender_;

	void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
	std::string topic_;
	rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_;
    std::mutex wrench_mutex_;

    rclcpp::Clock ros_clock_;
    rclcpp::Clock system_clock_;
    rclcpp::Time timestamp_msg_;
};
}  // namespace net_ft_driver