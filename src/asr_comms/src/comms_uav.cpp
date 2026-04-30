#include "comms_uav.h"

#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

static constexpr uint16_t DEFAULT_BIND_PORT   = 14551;
static constexpr char     DEFAULT_TARGET_IP[] = "127.0.0.1";
static constexpr uint16_t DEFAULT_TARGET_PORT = 14550;
static constexpr size_t   UDP_BUF_SIZE        = 2048;

CommsUav::CommsUav()
: Node("comms_uav")
{
    declare_parameter("bind_port",    static_cast<int>(DEFAULT_BIND_PORT));
    declare_parameter("target_ip",    DEFAULT_TARGET_IP);
    declare_parameter("target_port",  static_cast<int>(DEFAULT_TARGET_PORT));
    declare_parameter("system_id",    static_cast<int>(system_id_));
    declare_parameter("component_id", static_cast<int>(component_id_));

    const auto bind_port  = static_cast<uint16_t>(get_parameter("bind_port").as_int());
    const auto target_ip  = get_parameter("target_ip").as_string();
    const auto target_port = static_cast<uint16_t>(get_parameter("target_port").as_int());
    system_id_    = static_cast<uint8_t>(get_parameter("system_id").as_int());
    component_id_ = static_cast<uint8_t>(get_parameter("component_id").as_int());

    socket_ = std::make_unique<UdpSocket>(bind_port, target_ip, target_port);
    RCLCPP_INFO(get_logger(), "UAV comms — listening on :%u, sending to %s:%u",
                bind_port, target_ip.c_str(), target_port);

    // Publishers (incoming from GCS)
    heartbeat_pub_ = create_publisher<std_msgs::msg::Bool>("/comms/gcs_heartbeat", 10);
    rtcm_pub_      = create_publisher<std_msgs::msg::UInt8MultiArray>("/comms/rtcm_data", 10);

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
        const ssize_t n = socket_->recv(buf, sizeof(buf));
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
        std_msgs::msg::UInt8MultiArray out{};
        out.data.assign(rtcm.data, rtcm.data + rtcm.len);
        rtcm_pub_->publish(out);
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

// --- Send path ---

void CommsUav::send_mavlink(mavlink_message_t& msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    socket_->send(buf, len);
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
