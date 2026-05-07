#pragma once

#include <array>
#include <atomic>
#include <cstring>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/gps_inject_data.hpp>
#include <asr_comms/msg/telemetry_position.hpp>
#include <asr_comms/msg/telemetry_attitude.hpp>
#include <asr_comms/msg/telemetry_battery.hpp>
#include <asr_comms/msg/telemetry_gps.hpp>
#include <asr_comms/msg/telemetry_status.hpp>

#include "common/mavlink.h"
#include "transport.h"

// Runs on the drone. Transport is either UDP or a serial SiK radio.
// Receives from GCS: heartbeat, RTK corrections.
// Sends to GCS:      heartbeat + telemetry (position, attitude, battery, GPS, status) at ≤10 Hz.
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
    void on_position(const asr_comms::msg::TelemetryPosition::SharedPtr msg);
    void on_attitude(const asr_comms::msg::TelemetryAttitude::SharedPtr msg);
    void on_battery(const asr_comms::msg::TelemetryBattery::SharedPtr msg);
    void on_gps(const asr_comms::msg::TelemetryGPS::SharedPtr msg);
    void on_status(const asr_comms::msg::TelemetryStatus::SharedPtr msg);

    std::unique_ptr<ITransport> transport_;

    uint8_t system_id_   {1};
    uint8_t component_id_{1};

    // Receive side
    std::thread       recv_thread_;
    std::atomic<bool> running_{true};

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr          heartbeat_pub_;
    rclcpp::Publisher<px4_msgs::msg::GpsInjectData>::SharedPtr gps_inject_pub_;

    // Fragment reassembly buffer for incoming RTCM.
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
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    rclcpp::Subscription<asr_comms::msg::TelemetryPosition>::SharedPtr position_sub_;
    rclcpp::Subscription<asr_comms::msg::TelemetryAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<asr_comms::msg::TelemetryBattery>::SharedPtr  battery_sub_;
    rclcpp::Subscription<asr_comms::msg::TelemetryGPS>::SharedPtr      gps_sub_;
    rclcpp::Subscription<asr_comms::msg::TelemetryStatus>::SharedPtr   status_sub_;

    static constexpr uint16_t ASR_MSG_TELEMETRY_STATUS = 0x9001u;
};
