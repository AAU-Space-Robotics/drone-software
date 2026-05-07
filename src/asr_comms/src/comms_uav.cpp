#include "comms_uav.h"
#include "serial_port.h"
#include "udp_socket.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <glob.h>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

static constexpr uint16_t DEFAULT_BIND_PORT   = 14551;
static constexpr char     DEFAULT_TARGET_IP[] = "127.0.0.1";
static constexpr uint16_t DEFAULT_TARGET_PORT = 14550;
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

CommsUav::CommsUav()
: Node("comms_uav")
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
        RCLCPP_INFO(get_logger(), "UAV comms — serial %s @ %d baud", dev.c_str(), baud);
    } else {
        const auto bind_port   = static_cast<uint16_t>(get_parameter("bind_port").as_int());
        const auto target_ip   = get_parameter("target_ip").as_string();
        const auto target_port = static_cast<uint16_t>(get_parameter("target_port").as_int());
        transport_ = std::make_unique<UdpSocket>(bind_port, target_ip, target_port);
        RCLCPP_INFO(get_logger(), "UAV comms — UDP listening on :%u, sending to %s:%u",
                    bind_port, target_ip.c_str(), target_port);
    }

    // Publishers (incoming from GCS)
    heartbeat_pub_   = create_publisher<std_msgs::msg::Bool>("/comms/gcs_heartbeat", 10);
    gps_inject_pub_  = create_publisher<px4_msgs::msg::GpsInjectData>("/fmu/in/gps_inject_data", 10);

    // Subscribers (outgoing to GCS)
    heartbeat_timer_ = create_wall_timer(1s, std::bind(&CommsUav::send_heartbeat, this));

    pos_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/fmu/out/vehicle_global_position", 10,
        std::bind(&CommsUav::on_global_position, this, std::placeholders::_1));

    att_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude", 10,
        std::bind(&CommsUav::on_attitude, this, std::placeholders::_1));

    bat_sub_ = create_subscription<px4_msgs::msg::BatteryStatus>(
        "/fmu/out/battery_status", 10,
        std::bind(&CommsUav::on_battery, this, std::placeholders::_1));

    recv_thread_ = std::thread(&CommsUav::recv_loop, this);
}

CommsUav::~CommsUav()
{
    running_ = false;
    if (recv_thread_.joinable()) recv_thread_.join();
}

// --- Receive path ---

void CommsUav::recv_loop()
{
    uint8_t          buf[UDP_BUF_SIZE];
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

void CommsUav::handle_message(const mavlink_message_t& msg)
{
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT: {
        mavlink_heartbeat_t hb{};
        mavlink_msg_heartbeat_decode(&msg, &hb);
        RCLCPP_DEBUG(get_logger(), "GCS heartbeat — type %u", hb.type);
        std_msgs::msg::Bool out{};
        out.data = true;
        heartbeat_pub_->publish(out);
        break;
    }

    case MAVLINK_MSG_ID_GPS_RTCM_DATA: {
        mavlink_gps_rtcm_data_t rtcm{};
        mavlink_msg_gps_rtcm_data_decode(&msg, &rtcm);
        handle_rtcm(rtcm);
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_LONG: {
        mavlink_command_long_t cmd{};
        mavlink_msg_command_long_decode(&msg, &cmd);
        RCLCPP_INFO(get_logger(), "COMMAND_LONG: cmd=%u param1=%.2f", cmd.command, cmd.param1);
        // TODO: publish to /comms/gcs_command via interfaces msg
        break;
    }

    default:
        break;
    }
}

void CommsUav::handle_rtcm(const mavlink_gps_rtcm_data_t& rtcm)
{
    if (!(rtcm.flags & 0x01u)) {
        // Not fragmented — publish directly
        publish_gps_inject(rtcm.data, rtcm.len);
        return;
    }

    // Fragmented: buffer by sequence number until all fragments arrive.
    // flags: bit0=1 (fragmented), bits1-2=fragment index, bits3-7=sequence (0-31)
    const uint8_t seq      = (rtcm.flags >> 3) & 0x1Fu;
    const uint8_t frag_idx = (rtcm.flags >> 1) & 0x03u;

    auto& buf = rtcm_frags_[seq];
    buf.frags[frag_idx].assign(rtcm.data, rtcm.data + rtcm.len);
    buf.received_mask |= static_cast<uint8_t>(1u << frag_idx);

    // The last fragment is identified by having len < 180 (not a full chunk)
    if (rtcm.len < 180) {
        buf.last_frag_idx = frag_idx;
        buf.last_known    = true;
    }

    if (!buf.last_known) return;

    // Check all fragments 0..last_frag_idx are present
    const uint8_t expected = static_cast<uint8_t>((1u << (buf.last_frag_idx + 1)) - 1u);
    if ((buf.received_mask & expected) != expected) return;

    // Reassemble into a contiguous buffer
    std::vector<uint8_t> assembled;
    assembled.reserve(static_cast<size_t>(buf.last_frag_idx + 1) * 180);
    for (uint8_t i = 0; i <= buf.last_frag_idx; ++i)
        assembled.insert(assembled.end(), buf.frags[i].begin(), buf.frags[i].end());

    buf.reset();

    publish_gps_inject(assembled.data(), assembled.size());
}

void CommsUav::publish_gps_inject(const uint8_t* data, size_t len)
{
    // GpsInjectData carries max 300 bytes; split into chunks if needed.
    // PX4 queues up to ORB_QUEUE_LENGTH=8 messages so back-to-back sends are safe.
    size_t offset = 0;
    while (offset < len) {
        const size_t chunk = std::min<size_t>(len - offset, 300u);

        px4_msgs::msg::GpsInjectData out{};
        out.timestamp = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
        out.len       = static_cast<uint16_t>(chunk);
        out.flags     = (len > 300u) ? 1u : 0u;  // flag as fragmented when split
        std::memcpy(out.data.data(), data + offset, chunk);

        gps_inject_pub_->publish(out);
        offset += chunk;
    }
}

// --- Send path ---

void CommsUav::send_mavlink(mavlink_message_t& msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    transport_->send(buf, len);
}

void CommsUav::send_heartbeat()
{
    mavlink_message_t msg{};
    mavlink_msg_heartbeat_pack(system_id_, component_id_, &msg,
        MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
        MAV_MODE_FLAG_AUTO_ENABLED, 0, MAV_STATE_ACTIVE);
    send_mavlink(msg);
}

void CommsUav::on_global_position(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr p)
{
    mavlink_message_t msg{};
    mavlink_msg_global_position_int_pack(system_id_, component_id_, &msg,
        0,
        static_cast<int32_t>(p->lat * 1e7),
        static_cast<int32_t>(p->lon * 1e7),
        static_cast<int32_t>(p->alt * 1e3),
        static_cast<int32_t>(p->alt * 1e3),  // TODO: use AGL altitude
        0, 0, 0,                              // TODO: velocity from VehicleLocalPosition
        UINT16_MAX);
    send_mavlink(msg);
}

void CommsUav::on_attitude(const px4_msgs::msg::VehicleAttitude::SharedPtr a)
{
    const float w = a->q[0], x = a->q[1], y = a->q[2], z = a->q[3];
    const float roll  = std::atan2(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y));
    const float pitch = std::asin( 2.0f*(w*y - z*x));
    const float yaw   = std::atan2(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z));

    mavlink_message_t msg{};
    mavlink_msg_attitude_pack(system_id_, component_id_, &msg,
        0, roll, pitch, yaw, 0.0f, 0.0f, 0.0f);
    send_mavlink(msg);
}

void CommsUav::on_battery(const px4_msgs::msg::BatteryStatus::SharedPtr b)
{
    uint16_t cell_voltages[10];
    std::fill(std::begin(cell_voltages), std::end(cell_voltages), UINT16_MAX);
    cell_voltages[0] = static_cast<uint16_t>(b->voltage_v * 1000.0f);

    mavlink_message_t msg{};
    mavlink_msg_battery_status_pack(system_id_, component_id_, &msg,
        0, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO,
        INT16_MAX,
        cell_voltages,
        static_cast<int16_t>(b->current_a * 100.0f),
        static_cast<int32_t>(b->discharged_mah),
        -1,
        static_cast<int8_t>(b->remaining * 100.0f),
        0, MAV_BATTERY_CHARGE_STATE_OK, nullptr, 0, 0);
    send_mavlink(msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommsUav>());
    rclcpp::shutdown();
    return 0;
}
