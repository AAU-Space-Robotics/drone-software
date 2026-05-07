#pragma once

#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/battery_status.hpp>

#include "common/mavlink.h"
#include "transport.h"

// Runs on the GCS machine. Transport is either UDP or a serial SiK radio.
// Sends to drone:       heartbeat, commands, RTK corrections.
// Receives from drone:  heartbeat, position, attitude, battery.
class CommsGcs : public rclcpp::Node {
public:
    CommsGcs();
    ~CommsGcs();

private:
    // Receive path
    void recv_loop();
    void handle_message(const mavlink_message_t& msg);

    // Send path
    void send_mavlink(mavlink_message_t& msg);
    void send_heartbeat();
    void send_rtcm(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

    std::unique_ptr<ITransport> transport_;

    uint8_t system_id_   {255};  // GCS conventionally uses system ID 255
    uint8_t component_id_{  0};

    // Receive side
    std::thread       recv_thread_;
    std::atomic<bool> running_{true};

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                  uav_heartbeat_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr position_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitude>::SharedPtr       attitude_pub_;
    rclcpp::Publisher<px4_msgs::msg::BatteryStatus>::SharedPtr         battery_pub_;

    // Send side
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr rtcm_sub_;
    uint8_t rtcm_seq_{0};  // rolling sequence number (0-31) per RTCM message
};
