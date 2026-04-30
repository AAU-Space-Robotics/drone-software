#include "comms_gcs.h"

#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

static constexpr uint16_t DEFAULT_BIND_PORT   = 14550;
static constexpr char     DEFAULT_TARGET_IP[] = "127.0.0.1";
static constexpr uint16_t DEFAULT_TARGET_PORT = 14551;
static constexpr size_t   UDP_BUF_SIZE        = 2048;

CommsGcs::CommsGcs()
: Node("comms_gcs")
{
    declare_parameter("bind_port",    static_cast<int>(DEFAULT_BIND_PORT));
    declare_parameter("target_ip",    DEFAULT_TARGET_IP);
    declare_parameter("target_port",  static_cast<int>(DEFAULT_TARGET_PORT));
    declare_parameter("system_id",    static_cast<int>(system_id_));
    declare_parameter("component_id", static_cast<int>(component_id_));

    const auto bind_port   = static_cast<uint16_t>(get_parameter("bind_port").as_int());
    const auto target_ip   = get_parameter("target_ip").as_string();
    const auto target_port = static_cast<uint16_t>(get_parameter("target_port").as_int());
    system_id_    = static_cast<uint8_t>(get_parameter("system_id").as_int());
    component_id_ = static_cast<uint8_t>(get_parameter("component_id").as_int());

    socket_ = std::make_unique<UdpSocket>(bind_port, target_ip, target_port);
    RCLCPP_INFO(get_logger(), "GCS comms — listening on :%u, sending to %s:%u",
                bind_port, target_ip.c_str(), target_port);

    // Publishers (incoming from UAV)
    uav_heartbeat_pub_ = create_publisher<std_msgs::msg::Bool>("/comms/uav_heartbeat", 10);
    position_pub_      = create_publisher<px4_msgs::msg::VehicleGlobalPosition>("/comms/uav_position", 10);
    attitude_pub_      = create_publisher<px4_msgs::msg::VehicleAttitude>("/comms/uav_attitude", 10);
    battery_pub_       = create_publisher<px4_msgs::msg::BatteryStatus>("/comms/uav_battery", 10);

    // Send side
    heartbeat_timer_ = create_wall_timer(1s, std::bind(&CommsGcs::send_heartbeat, this));

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
        const ssize_t n = socket_->recv(buf, sizeof(buf));
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

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t p{};
        mavlink_msg_global_position_int_decode(&msg, &p);

        px4_msgs::msg::VehicleGlobalPosition out{};
        out.lat = p.lat / 1e7;
        out.lon = p.lon / 1e7;
        out.alt = p.alt / 1e3f;
        position_pub_->publish(out);
        break;
    }

    case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t a{};
        mavlink_msg_attitude_decode(&msg, &a);

        // Convert Euler back to quaternion for px4_msgs
        const float cr = std::cos(a.roll  * 0.5f);
        const float sr = std::sin(a.roll  * 0.5f);
        const float cp = std::cos(a.pitch * 0.5f);
        const float sp = std::sin(a.pitch * 0.5f);
        const float cy = std::cos(a.yaw   * 0.5f);
        const float sy = std::sin(a.yaw   * 0.5f);

        px4_msgs::msg::VehicleAttitude out{};
        out.q[0] = cr*cp*cy + sr*sp*sy;
        out.q[1] = sr*cp*cy - cr*sp*sy;
        out.q[2] = cr*sp*cy + sr*cp*sy;
        out.q[3] = cr*cp*sy - sr*sp*cy;
        attitude_pub_->publish(out);
        break;
    }

    case MAVLINK_MSG_ID_BATTERY_STATUS: {
        mavlink_battery_status_t b{};
        mavlink_msg_battery_status_decode(&msg, &b);

        px4_msgs::msg::BatteryStatus out{};
        out.voltage_v     = b.voltages[0] / 1000.0f;
        out.current_a     = b.current_battery / 100.0f;
        out.discharged_mah = static_cast<float>(b.current_consumed);
        out.remaining     = b.battery_remaining / 100.0f;
        battery_pub_->publish(out);
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
    socket_->send(buf, len);
}

void CommsGcs::send_heartbeat()
{
    mavlink_message_t msg{};
    mavlink_msg_heartbeat_pack(system_id_, component_id_, &msg,
        MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
        0, 0, MAV_STATE_ACTIVE);
    send_mavlink(msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommsGcs>());
    rclcpp::shutdown();
    return 0;
}
