#ifndef FCI_LIBRARY_HPP
#define FCI_LIBRARY_HPP

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <stdint.h>


using namespace px4_msgs::msg;
using namespace std::chrono_literals;

class FlightControllerInterface : public rclcpp::Node
{
public:
    FlightControllerInterface();
    void arm();
    void disarm();

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr bodyrate_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    std::atomic<uint64_t> timestamp_;
    uint64_t offboard_setpoint_counter_;

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_bodyrate_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

#endif // FCI_LIBRARY_HPP
