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
#include "udp_socket.h"

// Runs on the drone. One bidirectional UDP socket (later: serial/SiK).
// Receives from GCS: heartbeat, commands, RTK corrections.
// Sends to GCS:      heartbeat, position, attitude, battery.
class CommsUav : public rclcpp::Node {
public:
    CommsUav();
    ~CommsUav();

private:
    // Receive path
    void recv_loop();
    void handle_message(const mavlink_message_t& msg);

    // Send path
    void send_mavlink(mavlink_message_t& msg);
    void send_heartbeat();
    void on_global_position(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void on_attitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void on_battery(const px4_msgs::msg::BatteryStatus::SharedPtr msg);

    std::unique_ptr<UdpSocket> socket_;

    uint8_t system_id_   {1};
    uint8_t component_id_{1};

    // Receive side — background thread + publishers
    std::thread          recv_thread_;
    std::atomic<bool>    running_{true};

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr            heartbeat_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr rtcm_pub_;

    // Send side — timer + subscribers
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr       att_sub_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr         bat_sub_;
};
