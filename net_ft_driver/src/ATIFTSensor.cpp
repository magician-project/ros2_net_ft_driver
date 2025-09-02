#include <net_ft_driver/ATIFTSensor.hpp>

using namespace net_ft_driver;

ATIFTSensor::ATIFTSensor() : Node("ati_ft_sensor")
{

    //This class
    lost_packets_ = 0;
    packet_count_ = 0;
    out_of_order_count_ = 0;
    status_ = 0;

    this->declare_parameter("sensor_ip", "192.168.1.1");
    const std::string sensor_ip = this->get_parameter("sensor_ip").as_string();
    //ati_ft_interface_ = std::make_unique<AtiFTInterface>(sensor_ip);

    ati_ft_interface_ = static_unique_pointer_cast<AtiFTInterface>(
        std::move(NetFTInterface::create("ati", sensor_ip)));

    if (!ati_ft_interface_) {
        throw std::runtime_error("Failed to create ATI FT Interface");
    }

    this->declare_parameter("sensor_filter_value", 0);
    const int sensor_filter = this->get_parameter("sensor_filter_value").as_int();
    if (!ati_ft_interface_->set_internal_filter(sensor_filter)){
        throw std::runtime_error("Failed to set internal filter for ATI FT Sensor");
    }

    this->declare_parameter("sensor_sampling_rate", 500);
    const int sensor_sampling_rate = this->get_parameter("sensor_sampling_rate").as_int();
    if (!ati_ft_interface_->set_sampling_rate(sensor_sampling_rate)){
        throw std::runtime_error("Failed to set sampling rate for ATI FT Sensor");
    }

    if (!ati_ft_interface_->start_streaming()) {
        throw std::runtime_error("Failed to start streaming for ATI FT Sensor");
    }

    if (!ati_ft_interface_->set_bias()){
        throw std::runtime_error("Failed to set bias for ATI FT Sensor");
    }

    //just a check for data incoming
    std::unique_ptr<SensorData> data = ati_ft_interface_->receive_data();
    if (!data) {
        throw std::runtime_error("Failed to receive data from ATI FT Sensor");
    }
    ft_sensor_measurements_ = data->ft_values;

    //ROS

    this->declare_parameter("wrench_topic_name", this->get_name()+std::string("/wrench_sensed"));
    const std::string wrench_topic_name = this->get_parameter("wrench_topic_name").as_string();
    wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(wrench_topic_name, 1);

    wrench_msg_ = geometry_msgs::msg::WrenchStamped();
    this->declare_parameter("sensing_frame", "ft_sensing_frame");
    wrench_msg_.header.frame_id = this->get_parameter("sensing_frame").as_string();
    wrench_msg_.wrench.force.x = 0.0;
    wrench_msg_.wrench.force.y = 0.0;
    wrench_msg_.wrench.force.z = 0.0;
    wrench_msg_.wrench.torque.x = 0.0;
    wrench_msg_.wrench.torque.y = 0.0;
    wrench_msg_.wrench.torque.z = 0.0;

    this->declare_parameter("diagnostic_topic_name", this->get_name()+std::string("/diagnostic"));
    const std::string diagnostic_topic_name = this->get_parameter("diagnostic_topic_name").as_string();
    diagnostic_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostic_topic_name, 1);

    this->declare_parameter("reset_bias_service_name", this->get_name()+std::string("/reset_bias"));
    const std::string reset_bias_service_name = this->get_parameter("reset_bias_service_name").as_string();
    reset_bias_service_ = this->create_service<std_srvs::srv::Trigger>(
        reset_bias_service_name, 
        std::bind(&ATIFTSensor::reset_bias, this, std::placeholders::_1, std::placeholders::_2));

    this->declare_parameter("set_filter_service_name", this->get_name()+std::string("/set_filter"));
    const std::string set_filter_service_name = this->get_parameter("set_filter_service_name").as_string();
    set_filter_service_ = this->create_service<net_ft_driver::srv::SetInt>(
        set_filter_service_name, 
        std::bind(&ATIFTSensor::set_filter, this, std::placeholders::_1, std::placeholders::_2));

    this->declare_parameter("set_sampling_rate_service_name", this->get_name()+std::string("/set_sampling_rate"));
    const std::string set_sampling_rate_service_name = this->get_parameter("set_sampling_rate_service_name").as_string();
    set_sampling_rate_service_ = this->create_service<net_ft_driver::srv::SetInt>(
        set_sampling_rate_service_name, 
        std::bind(&ATIFTSensor::set_sampling_rate, this, std::placeholders::_1, std::placeholders::_2));


    this->declare_parameter("node_rate", 100.0);
    const double rate = this->get_parameter("node_rate").as_double();
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        std::bind(&ATIFTSensor::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Initialized F/T Sensor");
}

ATIFTSensor::~ATIFTSensor() {
    if (ati_ft_interface_) {
        if (!ati_ft_interface_->stop_streaming()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to stop streaming for ATI FT Sensor");
        }
        RCLCPP_INFO(this->get_logger(), "Successfully stopped data streaming!");
    } else {
        RCLCPP_INFO(this->get_logger(), "ATI FT Interface was not initialized");
    }
}

bool ATIFTSensor::read() {
    auto data = ati_ft_interface_->receive_data();
    if (!data) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read data from ATI FT Sensor");
        return false;
    }

    ft_sensor_measurements_ = data->ft_values;
    lost_packets_ = static_cast<uint32_t>(data->lost_packets);
    packet_count_ = static_cast<uint32_t>(data->packet_count);
    out_of_order_count_ = static_cast<uint32_t>(data->out_of_order_count);
    status_ = static_cast<uint32_t>(data->status);

    return true;
}

bool ATIFTSensor::publish() {
    wrench_msg_.header.stamp = this->now();
    wrench_msg_.wrench.force.x = -ft_sensor_measurements_[0];
    wrench_msg_.wrench.force.y = ft_sensor_measurements_[1];
    wrench_msg_.wrench.force.z = ft_sensor_measurements_[2];
    wrench_msg_.wrench.torque.x = -ft_sensor_measurements_[3];
    wrench_msg_.wrench.torque.y = ft_sensor_measurements_[4];
    wrench_msg_.wrench.torque.z = ft_sensor_measurements_[5];

    wrench_pub_->publish(wrench_msg_);
    return true;
}

void ATIFTSensor::publish_diagnostic()
{
  diag_array_.status.clear();
  diagnostic_updater::DiagnosticStatusWrapper diag_status;

  if (last_packet_count_ == packet_count_) {
    diag_status.mergeSummary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "No new data received!");
  }
  if (status_ != 0) {
    diag_status.mergeSummaryf(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Net F/T driver reports error 0x%08x",
                              status_);
    RCLCPP_ERROR(this->get_logger(), "SERIOUS ERROR: STATUS IS NOT HEALTHY!!!!!!!!!!!!!!!!!!");

  }
  diag_status.clear();
  diag_status.addf("System status", "0x%08x", status_);
  diag_status.addf("Good packets", "%u", packet_count_);
  diag_status.addf("Lost packets", "%u", lost_packets_);
  diag_status.addf("Out-of-order packets", "%u", out_of_order_count_);

  last_packet_count_ = packet_count_;
  diag_array_.status.push_back(diag_status);
  diag_array_.header.stamp = this->get_clock()->now();

  diagnostic_publisher_->publish(diag_array_);
}

void ATIFTSensor::reset_bias(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (ati_ft_interface_->set_bias()) {
        response->success = true;
        response->message = "Bias reset successfully";
        RCLCPP_INFO(this->get_logger(), "Bias reset successfully");
    } else {
        response->success = false;
        response->message = "Failed to reset bias";
        RCLCPP_ERROR(this->get_logger(), "Failed to reset bias");
    }
}

void ATIFTSensor::set_filter(
    const std::shared_ptr<net_ft_driver::srv::SetInt::Request> request,
    std::shared_ptr<net_ft_driver::srv::SetInt::Response> response)
{
    if (ati_ft_interface_->set_internal_filter(request->value)) {
        response->success = true;
        response->message = "Filter set successfully";
        RCLCPP_INFO(this->get_logger(), "Filter set successfully");
    } else {
        response->success = false;
        response->message = "Failed to set filter";
        RCLCPP_ERROR(this->get_logger(), "Failed to set filter");
    }
}

void ATIFTSensor::set_sampling_rate(
    const std::shared_ptr<net_ft_driver::srv::SetInt::Request> request,
    std::shared_ptr<net_ft_driver::srv::SetInt::Response> response)
{
    if (ati_ft_interface_->set_sampling_rate(request->value)) {
        response->success = true;
        response->message = "Sampling rate set successfully";
        RCLCPP_INFO(this->get_logger(), "Sampling rate set successfully");
    } else {
        response->success = false;
        response->message = "Failed to set Sampling Rate";
        RCLCPP_ERROR(this->get_logger(), "Failed to set Sampling Rate");
    }
}

void ATIFTSensor::timer_callback()
{
    read();
    publish();
    publish_diagnostic();
}