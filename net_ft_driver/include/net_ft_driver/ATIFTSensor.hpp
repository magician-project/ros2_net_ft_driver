#ifndef ATI_FT_SENSOR_HPP
#define ATI_FT_SENSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <net_ft_driver/srv/set_int.hpp>

#include <memory>

#include <net_ft_driver/interfaces/ati_ft_interface.hpp>

namespace net_ft_driver
{

class ATIFTSensor : public rclcpp::Node
{
public:
    ATIFTSensor();
    ~ATIFTSensor();

private:
    // ATI FT Sensor
    std::unique_ptr<AtiFTInterface> ati_ft_interface_;

    //ROS
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
    geometry_msgs::msg::WrenchStamped wrench_msg_;

    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
    diagnostic_msgs::msg::DiagnosticArray diag_array_;
    uint32_t last_packet_count_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_bias_service_;
    void reset_bias(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response>      response);

    rclcpp::Service<net_ft_driver::srv::SetInt>::SharedPtr set_filter_service_;
    void set_filter(
        const std::shared_ptr<net_ft_driver::srv::SetInt::Request> request,
        std::shared_ptr<net_ft_driver::srv::SetInt::Response>      response);

    rclcpp::Service<net_ft_driver::srv::SetInt>::SharedPtr set_sampling_rate_service_;
    void set_sampling_rate(
        const std::shared_ptr<net_ft_driver::srv::SetInt::Request> request,
        std::shared_ptr<net_ft_driver::srv::SetInt::Response>      response);

    // Internal
    std::array<double, 6> ft_sensor_measurements_;
    uint32_t packet_count_;
    uint32_t lost_packets_;
    uint32_t out_of_order_count_;
    uint32_t status_;

    bool read();
    bool publish();
    void publish_diagnostic();


    //https://stackoverflow.com/questions/36120424/alternatives-of-static-pointer-cast-for-unique-ptr
    template<typename TO, typename FROM>
    std::unique_ptr<TO> static_unique_pointer_cast (std::unique_ptr<FROM>&& old){
        return std::unique_ptr<TO>{static_cast<TO*>(old.release())};
        // conversion: unique_ptr<FROM>->FROM*->TO*->unique_ptr<TO>
    }
};

}  // namespace net_ft_driver

#endif  // ATI_FT_SENSOR_HPP