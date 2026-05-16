#pragma once

#include <array>
#include <atomic>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/gps_inject_data.hpp>
#include <asr_comms/msg/telemetry_position.hpp>
#include <asr_comms/msg/telemetry_attitude.hpp>
#include <asr_comms/msg/telemetry_battery.hpp>
#include <asr_comms/msg/telemetry_gps.hpp>
#include <asr_comms/msg/telemetry_status.hpp>
#include <asr_comms/msg/gcs_heartbeat.hpp>
#include <asr_comms/msg/manual_control_input.hpp>
#include <asr_comms/msg/servo_command.hpp>
#include <asr_comms/action/drone_command.hpp>

#include "common/mavlink.h"
#include "dedup.h"
#include "transport.h"
#include "udp_socket.h"

// Runs on the drone. Transport is either UDP or a serial SiK radio.
// Receives from GCS: heartbeat, RTK corrections, UAVCommand.
// Sends to GCS:      heartbeat + telemetry + CommandAck.
class CommsUav : public rclcpp::Node {
public:
    CommsUav();
    ~CommsUav();

private:
    using DroneCommand        = asr_comms::action::DroneCommand;
    using GoalHandleDroneCmd  = rclcpp_action::ClientGoalHandle<DroneCommand>;

    // Receive path
    void recv_loop();
    void wifi_recv_loop();
    void handle_message(const mavlink_message_t& msg);
    void handle_rtcm(const mavlink_gps_rtcm_data_t& rtcm);
    void handle_peer_beacon(const mavlink_v2_extension_t& ext);
    void publish_gps_inject(const uint8_t* data, size_t len);
    void forward_command(const mavlink_command_long_t& cmd);

    // Send path
    void send_mavlink(mavlink_message_t& msg);
    void send_heartbeat();
    void send_rx_kbps();
    void send_radio_stats();
    void on_position(const asr_comms::msg::TelemetryPosition::SharedPtr msg);
    void on_attitude(const asr_comms::msg::TelemetryAttitude::SharedPtr msg);
    void on_battery(const asr_comms::msg::TelemetryBattery::SharedPtr msg);
    void on_gps(const asr_comms::msg::TelemetryGPS::SharedPtr msg);
    void on_status(const asr_comms::msg::TelemetryStatus::SharedPtr msg);

    std::unique_ptr<ITransport> transport_;
    std::unique_ptr<UdpSocket>  wifi_transport_;  // client socket, nullptr until beacon received

    uint8_t system_id_   {1};
    uint8_t component_id_{1};
    uint16_t wifi_port_{14553};

    // Receive side
    std::thread       recv_thread_;
    std::thread       wifi_recv_thread_;
    std::atomic<bool> running_{true};
    std::mutex        recv_mutex_;   // serialises handle_message across both recv threads
    std::mutex        send_mutex_;
    DedupFilter       dedup_;
    std::atomic<size_t> rx_bytes_{0};
    float               uav_rx_kbps_{0.0f};
    std::chrono::steady_clock::time_point rx_rate_ts_{std::chrono::steady_clock::now()};

    rclcpp::Publisher<asr_comms::msg::GcsHeartbeat>::SharedPtr        heartbeat_pub_;
    rclcpp::Publisher<px4_msgs::msg::GpsInjectData>::SharedPtr        gps_inject_pub_;
    rclcpp::Publisher<asr_comms::msg::ManualControlInput>::SharedPtr  manual_input_pub_;
    rclcpp::Publisher<asr_comms::msg::ServoCommand>::SharedPtr        servo_command_pub_;

    // UAV-side radio stats (populated from local RADIO_STATUS, forwarded to GCS)
    std::atomic<uint8_t>  uav_radio_txbuf_{255};
    std::atomic<uint8_t>  uav_radio_rssi_{255};
    std::atomic<uint8_t>  uav_radio_noise_{255};
    std::atomic<uint16_t> uav_radio_rxerrors_{0};

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

    // Command forwarding: COMMAND_LONG → autopilot action → COMMAND_ACK
    rclcpp_action::Client<DroneCommand>::SharedPtr action_client_;

    static constexpr uint16_t ASR_MSG_TELEMETRY_STATUS = 0x9001u;
    static constexpr uint16_t ASR_MSG_SERVO_COMMAND    = 0x9002u;
    static constexpr uint16_t ASR_MSG_PEER_BEACON      = 0x9003u;

    // ASR custom MAVLink command IDs (local experiment range ≥ 32768)
    static constexpr uint16_t ASR_CMD_GOTO              = 32768u;
    static constexpr uint16_t ASR_CMD_MANUAL            = 32769u;
    static constexpr uint16_t ASR_CMD_MANUAL_AIDED      = 32770u;
    static constexpr uint16_t ASR_CMD_SET_ORIGIN        = 32771u;
    static constexpr uint16_t ASR_CMD_SET_LINEAR_SPEED  = 32772u;
    static constexpr uint16_t ASR_CMD_SET_ANGULAR_SPEED = 32773u;
    static constexpr uint16_t ASR_CMD_SPIN              = 32774u;
    static constexpr uint16_t ASR_CMD_ELAND             = 32775u;
    static constexpr uint16_t ASR_CMD_VELOCITY          = 32776u;
    static constexpr uint16_t ASR_CMD_MULTI_WAYPOINT    = 32777u;
};
