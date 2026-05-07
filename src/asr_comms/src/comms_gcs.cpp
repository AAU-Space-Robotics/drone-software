#include "comms_gcs.h"
#include "serial_port.h"
#include "udp_socket.h"

#include <chrono>
#include <cstring>
#include <glob.h>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

static constexpr uint16_t DEFAULT_BIND_PORT   = 14550;
static constexpr char     DEFAULT_TARGET_IP[] = "127.0.0.1";
static constexpr uint16_t DEFAULT_TARGET_PORT = 14551;
static constexpr size_t   UDP_BUF_SIZE        = 2048;

static std::string auto_detect_serial()
{
    for (const char* pattern : {"/dev/ttyUSB*", "/dev/ttyACM*"}) {
        glob_t g{};
        if (glob(pattern, 0, nullptr, &g) == 0 && g.gl_pathc > 0) {
            std::string path = g.gl_pathv[0];
            globfree(&g);
            return path;
        }
        globfree(&g);
    }
    throw std::runtime_error("No serial radio found — is the USB radio module plugged in?");
}

CommsGcs::CommsGcs() : Node("comms_gcs")
{
    declare_parameter("bind_port",    static_cast<int>(DEFAULT_BIND_PORT));
    declare_parameter("target_ip",    DEFAULT_TARGET_IP);
    declare_parameter("target_port",  static_cast<int>(DEFAULT_TARGET_PORT));
    declare_parameter("serial_port",  std::string{});
    declare_parameter("baud_rate",    57600);
    declare_parameter("system_id",    static_cast<int>(system_id_));
    declare_parameter("component_id", static_cast<int>(component_id_));

    system_id_    = static_cast<uint8_t>(get_parameter("system_id").as_int());
    component_id_ = static_cast<uint8_t>(get_parameter("component_id").as_int());

    const auto serial_port = get_parameter("serial_port").as_string();
    if (!serial_port.empty()) {
        const auto dev  = (serial_port == "auto") ? auto_detect_serial() : serial_port;
        const int  baud = static_cast<int>(get_parameter("baud_rate").as_int());
        transport_ = std::make_unique<SerialPort>(dev, baud);
        RCLCPP_INFO(get_logger(), "GCS comms — serial %s @ %d baud", dev.c_str(), baud);
    } else {
        const auto bind_port   = static_cast<uint16_t>(get_parameter("bind_port").as_int());
        const auto target_ip   = get_parameter("target_ip").as_string();
        const auto target_port = static_cast<uint16_t>(get_parameter("target_port").as_int());
        transport_ = std::make_unique<UdpSocket>(bind_port, target_ip, target_port);
        RCLCPP_INFO(get_logger(), "GCS comms — UDP listening on :%u, sending to %s:%u",
                    bind_port, target_ip.c_str(), target_port);
    }

    // Publishers — incoming telemetry from UAV
    uav_heartbeat_pub_ = create_publisher<std_msgs::msg::Bool>("/comms/uav_heartbeat", 10);
    position_pub_ = create_publisher<asr_comms::msg::TelemetryPosition>("/comms/telemetry/position", 10);
    attitude_pub_ = create_publisher<asr_comms::msg::TelemetryAttitude>("/comms/telemetry/attitude", 10);
    battery_pub_  = create_publisher<asr_comms::msg::TelemetryBattery>( "/comms/telemetry/battery",  10);
    gps_pub_      = create_publisher<asr_comms::msg::TelemetryGPS>(     "/comms/telemetry/gps",      10);
    status_pub_   = create_publisher<asr_comms::msg::TelemetryStatus>(  "/comms/telemetry/status",   10);

    // Send side
    heartbeat_timer_ = create_wall_timer(1s, std::bind(&CommsGcs::send_heartbeat, this));

    rtcm_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/rtcm", 10,
        std::bind(&CommsGcs::send_rtcm, this, std::placeholders::_1));

    recv_thread_ = std::thread(&CommsGcs::recv_loop, this);
}

CommsGcs::~CommsGcs()
{
    running_ = false;
    if (recv_thread_.joinable()) recv_thread_.join();
}

// --- Receive path ---

void CommsGcs::recv_loop()
{
    uint8_t           buf[UDP_BUF_SIZE];
    mavlink_message_t msg{};
    mavlink_status_t  status{};

    while (running_) {
        const ssize_t n = transport_->recv(buf, sizeof(buf));
        if (n <= 0) continue;

        for (ssize_t i = 0; i < n; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
                handle_message(msg);
        }
    }
}

void CommsGcs::handle_message(const mavlink_message_t& msg)
{
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t hb{};
        mavlink_msg_heartbeat_decode(&msg, &hb);
        RCLCPP_DEBUG(get_logger(), "UAV heartbeat — type %u", hb.type);
        std_msgs::msg::Bool out{};
        out.data = true;
        uav_heartbeat_pub_->publish(out);
        break;
    }

    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
        mavlink_local_position_ned_t p{};
        mavlink_msg_local_position_ned_decode(&msg, &p);

        asr_comms::msg::TelemetryPosition out{};
        out.timestamp    = p.time_boot_ms / 1e3;
        out.position     = {p.x, p.y, p.z};
        out.velocity     = {p.vx, p.vy, p.vz};
        // target_position not carried in LOCAL_POSITION_NED — left at default zero
        position_pub_->publish(out);
        break;
    }

    case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t a{};
        mavlink_msg_attitude_decode(&msg, &a);

        asr_comms::msg::TelemetryAttitude out{};
        out.timestamp   = a.time_boot_ms / 1e3;
        out.orientation = {a.roll, a.pitch, a.yaw};
        attitude_pub_->publish(out);
        break;
    }

    case MAVLINK_MSG_ID_BATTERY_STATUS: {
        mavlink_battery_status_t b{};
        mavlink_msg_battery_status_decode(&msg, &b);

        asr_comms::msg::TelemetryBattery out{};
        out.timestamp      = get_clock()->now().seconds();
        out.voltage        = b.voltages[0] / 1000.0f;
        out.current        = b.current_battery / 100.0f;
        out.percentage     = static_cast<float>(b.battery_remaining);
        out.discharged_mah = static_cast<float>(b.current_consumed);
        battery_pub_->publish(out);
        break;
    }

    case MAVLINK_MSG_ID_GPS_RAW_INT: {
        mavlink_gps_raw_int_t g{};
        mavlink_msg_gps_raw_int_decode(&msg, &g);

        asr_comms::msg::TelemetryGPS out{};
        out.timestamp       = g.time_usec / 1e6;
        out.latitude        = g.lat / 1e7;
        out.longitude       = g.lon / 1e7;
        out.satellites_used = static_cast<int32_t>(g.satellites_visible);
        gps_pub_->publish(out);
        break;
    }

    case MAVLINK_MSG_ID_V2_EXTENSION: {
        mavlink_v2_extension_t ext{};
        mavlink_msg_v2_extension_decode(&msg, &ext);
        if (ext.message_type != ASR_MSG_TELEMETRY_STATUS) break;

#pragma pack(push, 1)
        struct StatusPod {
            double   timestamp;
            double   flight_time;
            float    actuator_speeds[4];
            int32_t  probes_found;
            int16_t  flight_mode;
            int16_t  led_mode;
            uint8_t  arming_state;
            uint8_t  trajectory_mode;
            uint8_t  estop;
        };
#pragma pack(pop)

        StatusPod pod{};
        std::memcpy(&pod, ext.payload, sizeof(pod));

        asr_comms::msg::TelemetryStatus out{};
        out.timestamp       = pod.timestamp;
        out.flight_time     = pod.flight_time;
        out.probes_found    = pod.probes_found;
        out.flight_mode     = pod.flight_mode;
        out.led_mode        = pod.led_mode;
        out.arming_state    = pod.arming_state;
        out.trajectory_mode = pod.trajectory_mode;
        out.estop           = pod.estop;
        out.actuator_speeds = {pod.actuator_speeds[0], pod.actuator_speeds[1],
                               pod.actuator_speeds[2], pod.actuator_speeds[3]};
        status_pub_->publish(out);
        break;
    }

    default:
        break;
    }
}

// --- Send path ---

void CommsGcs::send_mavlink(mavlink_message_t& msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    transport_->send(buf, len);
}

void CommsGcs::send_heartbeat()
{
    mavlink_message_t msg{};
    mavlink_msg_heartbeat_pack(system_id_, component_id_, &msg,
        MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
        0, 0, MAV_STATE_ACTIVE);
    send_mavlink(msg);
}

void CommsGcs::send_rtcm(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
    const auto& data   = msg->data;
    const size_t total = data.size();
    if (total == 0) return;

    if (total <= 180) {
        uint8_t payload[180]{};
        std::memcpy(payload, data.data(), total);
        mavlink_message_t mav{};
        mavlink_msg_gps_rtcm_data_pack(system_id_, component_id_, &mav,
            0, static_cast<uint8_t>(total), payload);
        send_mavlink(mav);
        return;
    }

    // Fragmented: up to 4 × 180-byte chunks.
    // flags byte: bit0=fragmented, bits1-2=frag_idx, bits3-7=seq.
    const uint8_t seq = rtcm_seq_++ & 0x1F;
    size_t  offset   = 0;
    uint8_t frag_idx = 0;

    while (offset < total) {
        if (frag_idx >= 4) {
            RCLCPP_WARN(get_logger(),
                "RTCM message too large (>720 B) — %zu bytes dropped", total - offset);
            break;
        }
        const size_t chunk = std::min<size_t>(total - offset, 180);
        uint8_t payload[180]{};
        std::memcpy(payload, data.data() + offset, chunk);

        const uint8_t flags = 0x01u
            | static_cast<uint8_t>(frag_idx << 1)
            | static_cast<uint8_t>(seq      << 3);

        mavlink_message_t mav{};
        mavlink_msg_gps_rtcm_data_pack(system_id_, component_id_, &mav,
            flags, static_cast<uint8_t>(chunk), payload);
        send_mavlink(mav);

        offset += chunk;
        ++frag_idx;
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommsGcs>());
    rclcpp::shutdown();
    return 0;
}
