#pragma once

#include <array>
#include <atomic>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/gps_inject_data.hpp>
#include <interfaces/msg/drone_state.hpp>

#include "common/mavlink.h"
#include "transport.h"

// Runs on the drone. Transport is either UDP or a serial SiK radio.
// Receives from GCS: heartbeat, RTK corrections.
// Sends to GCS:      heartbeat, DroneState (position, attitude, battery).
class CommsUav : public rclcpp::Node {
public:
    CommsUav();
    ~CommsUav();

private:
    // Receive path
    void recv_loop();
    void handle_message(const mavlink_message_t& msg);
    void handle_rtcm(const mavlink_gps_rtcm_data_t& rtcm);
    void publish_gps_inject(const uint8_t* data, size_t len);

    // Send path
    void send_mavlink(mavlink_message_t& msg);
    void send_heartbeat();
    void send_drone_state();
    void on_drone_state(const interfaces::msg::DroneState::SharedPtr msg);

    std::unique_ptr<ITransport> transport_;

    uint8_t system_id_   {1};
    uint8_t component_id_{1};

    // Receive side — background thread + publishers
    std::thread       recv_thread_;
    std::atomic<bool> running_{true};

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr          heartbeat_pub_;
    rclcpp::Publisher<px4_msgs::msg::GpsInjectData>::SharedPtr gps_inject_pub_;

    // Fragment reassembly buffer — 32 sequence slots, 4 fragments each.
    // MAVLink GPS_RTCM_DATA flags: bit0=fragmented, bits1-2=frag_idx, bits3-7=seq.
    struct FragBuf {
        std::array<std::vector<uint8_t>, 4> frags;
        uint8_t received_mask{0};
        uint8_t last_frag_idx{0};
        bool    last_known{false};

        void reset() {
            for (auto& f : frags) f.clear();
            received_mask = 0;
            last_frag_idx = 0;
            last_known    = false;
        }
    };
    std::array<FragBuf, 32> rtcm_frags_{};

    // Send side
    interfaces::msg::DroneState latest_state_{};
    std::mutex                  state_mutex_;

    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_;

    rclcpp::Subscription<interfaces::msg::DroneState>::SharedPtr drone_state_sub_;
};
