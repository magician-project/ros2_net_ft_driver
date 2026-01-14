#include <net_ft_driver/FT_logger.hpp>

using namespace net_ft_driver;

FTLogger::FTLogger(const rclcpp::NodeOptions & options)
    : Node("ft_logger", options),
    ros_clock_(RCL_ROS_TIME),
    system_clock_(RCL_SYSTEM_TIME)
{
    this->declare_parameter("topic_name", "/ati_ft_sensor/wrench_sensed");
    this->declare_parameter("logging_file_name", "ft_logger_data");
    this->declare_parameter("rate", 500.0);

    topic_ = this->get_parameter("topic_name").as_string(); 
    std::string log_name = this->get_parameter("logging_file_name").as_string();
    rate_ = this->get_parameter("rate").as_double();

    wrench_eigen_.setZero();

    sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&FTLogger::wrench_callback, this, std::placeholders::_1));

    if (!log_name.empty()) {
        logging_file_name_ =
          ament_index_cpp::get_package_share_directory("net_ft_driver") +
          "/logs/" + log_name;
        
        if (! initLogging()) {
            throw std::runtime_error("failed to init logging");
        }
    } 

    auto period = std::chrono::duration<double>(1.0 / rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&FTLogger::timer_callback, this));

    timestamp_msg_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    RCLCPP_INFO(this->get_logger(), "FTLogger started at %.1f Hz", rate_);
}

void FTLogger::wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(wrench_mutex_);

    timestamp_msg_ = msg->header.stamp;
    wrench_eigen_(0) = msg->wrench.force.x;
    wrench_eigen_(1) = msg->wrench.force.y;
    wrench_eigen_(2) = msg->wrench.force.z;
    wrench_eigen_(3) = msg->wrench.torque.x;
    wrench_eigen_(4) = msg->wrench.torque.y;
    wrench_eigen_(5) = msg->wrench.torque.z;
}

bool FTLogger::initLogging() {

    // date-time automatically appended
    mat_logger_ = XBot::MatLogger2::MakeLogger(logging_file_name_);
    mat_appender_ = XBot::MatAppender::MakeInstance();
    if (!mat_appender_->add_logger(mat_logger_)) {
        std::cout <<  "Matlogger is  null or it was already registered." << std::endl;
        return false;
    }
    mat_appender_->start_flush_thread(); //automatically flush, so DO NOT call flush_available_data()!
    
    RCLCPP_INFO(this->get_logger(),
                "Logging to %s",
                mat_logger_->get_filename().c_str());    

    return true;
}

bool FTLogger::logData() {

    //log wrench data
    std::lock_guard<std::mutex> lock(wrench_mutex_);

    mat_logger_->add("wrench", wrench_eigen_);
    mat_logger_->add("timestamp_msg_ns", timestamp_msg_.nanoseconds());
    mat_logger_->add("time_ros_ns", ros_clock_.now().nanoseconds());
    mat_logger_->add("time_system_ns", system_clock_.now().nanoseconds());

    return true;
}

void FTLogger::timer_callback()
{
    logData();
}
