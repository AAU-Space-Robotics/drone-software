#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>  // include thrust setpoint message type
#include <chrono>

using namespace std::chrono_literals;

class FlightControlInterface : public rclcpp::Node
{
public:
    explicit FlightControlInterface() 
        : Node("flight_control_interface"), thrust_command_{0.0, 0.0, 1.0} // updated thrust setpoint for hovering
    {
        // QoS for subscriptions and publications
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Subscriber for sensor_combined data
        subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
            "/fmu/out/sensor_combined", qos,
            [this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received Sensor Data:");
                RCLCPP_INFO(this->get_logger(), "Timestamp: %lu", msg->timestamp);
                RCLCPP_INFO(this->get_logger(), "Gyro X: %f", msg->gyro_rad[0]);
                RCLCPP_INFO(this->get_logger(), "Gyro Y: %f", msg->gyro_rad[1]);
                RCLCPP_INFO(this->get_logger(), "Gyro Z: %f", msg->gyro_rad[2]);
                RCLCPP_INFO(this->get_logger(), "Accel X: %f", msg->accelerometer_m_s2[0]);
                RCLCPP_INFO(this->get_logger(), "Accel Y: %f", msg->accelerometer_m_s2[1]);
                RCLCPP_INFO(this->get_logger(), "Accel Z: %f", msg->accelerometer_m_s2[2]);

                // Additional processing can be added here to adjust thrust_command_ dynamically
            });

        // Publisher for thrust setpoint
        thrust_publisher_ = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
            "/fmu/in/vehicle_thrust_setpoint", qos);

        // Timer to periodically publish thrust setpoint
        timer_ = this->create_wall_timer(
            100ms, [this]() { this->publish_thrust_command(); });
    }

private:
    // Method to publish thrust command
    void publish_thrust_command()
    {
        auto message = px4_msgs::msg::VehicleThrustSetpoint();
        message.timestamp = this->now().nanoseconds() / 1000;  // microseconds
        message.timestamp_sample = message.timestamp;
        
        // Setting thrust values along XYZ axes
        message.xyz[0] = thrust_command_[0];  // X-axis thrust (typically zero for hovering)
        message.xyz[1] = thrust_command_[1];  // Y-axis thrust (typically zero for hovering)
        message.xyz[2] = thrust_command_[2];  // Z-axis thrust (set for hovering)

        RCLCPP_INFO(this->get_logger(), "Publishing thrust command: [%f, %f, %f]",
                    message.xyz[0], message.xyz[1], message.xyz[2]);
        thrust_publisher_->publish(message);
    }

    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscription_;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Thrust command for hovering (approximately -0.7 for Z-axis in a typical drone)
    std::array<float, 3> thrust_command_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlightControlInterface>());
    rclcpp::shutdown();
    return 0;
}
