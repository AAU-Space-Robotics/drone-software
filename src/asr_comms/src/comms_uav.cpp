#include "comms_uav.h"
#include "serial_port.h"
#include "udp_socket.h"

#include <algorithm>
#include <chrono>
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
    declare_parameter("baud_rate",    115200);
    declare_parameter("system_id",    static_cast<int>(system_id_));
    declare_parameter("component_id", static_cast<int>(component_id_));

    std::cout << "\n"
              << "=============================\n"
              << "    * COMMS UAV STARTING... *\n"
              << "=============================\n"
              << std::endl;

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

    // Incoming from GCS — QoS must match the autopilot's subscriber profiles
    auto qos_be_tl = rclcpp::QoS(10).best_effort().transient_local();
    auto qos_be_vo = rclcpp::QoS(10).best_effort().durability_volatile();

    heartbeat_pub_      = create_publisher<asr_comms::msg::GcsHeartbeat>("in/gcs_heartbeat", qos_be_tl);
    gps_inject_pub_     = create_publisher<px4_msgs::msg::GpsInjectData>("/fmu/in/gps_inject_data", 10);
    manual_input_pub_   = create_publisher<asr_comms::msg::ManualControlInput>("in/manual_input", qos_be_tl);
    servo_command_pub_  = create_publisher<asr_comms::msg::ServoCommand>("in/servo_command", qos_be_vo);

    // Outgoing to GCS — one subscription per telemetry topic.
    // best_effort + depth 1: only the latest sample matters; never queue stale data
    // that would arrive at the GCS as delayed or "duplicate" bursts.
    heartbeat_timer_ = create_wall_timer(1s, [this]() {
        uav_rx_kbps_ = static_cast<float>(rx_bytes_.exchange(0)) / 1024.0f;
        send_heartbeat();
    });

    auto qos_rt = rclcpp::QoS(1).best_effort();

    position_sub_ = create_subscription<asr_comms::msg::TelemetryPosition>(
        "out/telemetry/position", qos_rt,
        std::bind(&CommsUav::on_position, this, std::placeholders::_1));

    attitude_sub_ = create_subscription<asr_comms::msg::TelemetryAttitude>(
        "out/telemetry/attitude", qos_rt,
        std::bind(&CommsUav::on_attitude, this, std::placeholders::_1));

    battery_sub_ = create_subscription<asr_comms::msg::TelemetryBattery>(
        "out/telemetry/battery", qos_rt,
        std::bind(&CommsUav::on_battery, this, std::placeholders::_1));

    gps_sub_ = create_subscription<asr_comms::msg::TelemetryGPS>(
        "out/telemetry/gps", qos_rt,
        std::bind(&CommsUav::on_gps, this, std::placeholders::_1));

    status_sub_ = create_subscription<asr_comms::msg::TelemetryStatus>(
        "out/telemetry/status", qos_rt,
        std::bind(&CommsUav::on_status, this, std::placeholders::_1));

    // Action client — forwards COMMAND_LONG from GCS to the autopilot action server
    action_client_ = rclcpp_action::create_client<DroneCommand>(this, "in/drone_command");

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
    uint8_t           buf[UDP_BUF_SIZE];
    mavlink_message_t msg{};
    mavlink_status_t  status{};

    while (running_) {
        const ssize_t n = transport_->recv(buf, sizeof(buf));
        if (n <= 0) continue;

        rx_bytes_ += static_cast<size_t>(n);
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
        asr_comms::msg::GcsHeartbeat out{};
        out.timestamp   = get_clock()->now().seconds();
        out.gcs_nominal = static_cast<int8_t>(hb.base_mode);
        heartbeat_pub_->publish(out);
        break;
    }

    case MAVLINK_MSG_ID_MANUAL_CONTROL: {
        mavlink_manual_control_t mc{};
        mavlink_msg_manual_control_decode(&msg, &mc);

        asr_comms::msg::ManualControlInput out{};
        out.pitch        =  mc.x / 1000.0f;
        out.roll         =  mc.y / 1000.0f;
        out.thrust       =  mc.z / 1000.0f;
        out.yaw_velocity =  mc.r / 1000.0f;
        out.arm          =  (mc.buttons >> 0) & 0x01u;
        out.estop        =  (mc.buttons >> 1) & 0x01u;
        out.selfdestruct =  (mc.buttons >> 2) & 0x01u;
        manual_input_pub_->publish(out);
        break;
    }

    case MAVLINK_MSG_ID_V2_EXTENSION: {
        mavlink_v2_extension_t ext{};
        mavlink_msg_v2_extension_decode(&msg, &ext);
        if (ext.message_type != ASR_MSG_SERVO_COMMAND) break;

#pragma pack(push, 1)
        struct ServoPod {
            uint64_t timestamp;
            int32_t  aux_index;
            int32_t  id;
            float    value;
        };
#pragma pack(pop)

        ServoPod pod{};
        std::memcpy(&pod, ext.payload, sizeof(pod));

        asr_comms::msg::ServoCommand out{};
        out.timestamp = pod.timestamp;
        out.aux_index = pod.aux_index;
        out.id        = pod.id;
        out.value     = pod.value;
        servo_command_pub_->publish(out);
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
        forward_command(cmd);
        break;
    }

    default:
        break;
    }
}

void CommsUav::forward_command(const mavlink_command_long_t& cmd)
{
    DroneCommand::Goal goal{};

    switch (cmd.command) {
    case MAV_CMD_COMPONENT_ARM_DISARM:
        if (cmd.param2 > 20000.0f)  goal.command_type = "estop";
        else if (cmd.param1 > 0.5f) goal.command_type = "arm";
        else                         goal.command_type = "disarm";
        break;
    case MAV_CMD_NAV_LAND:
        goal.command_type = "land";
        break;
    case MAV_CMD_NAV_TAKEOFF:
        goal.command_type = "takeoff";
        goal.target_pose  = {static_cast<double>(cmd.param7)};
        break;
    case ASR_CMD_GOTO:
        goal.command_type = "goto";
        goal.target_pose  = {static_cast<double>(cmd.param1),
                              static_cast<double>(cmd.param2),
                              static_cast<double>(cmd.param3)};
        goal.yaw = static_cast<double>(cmd.param4);
        break;
    case ASR_CMD_MANUAL:        goal.command_type = "manual";        break;
    case ASR_CMD_MANUAL_AIDED:  goal.command_type = "manual_aided";  break;
    case ASR_CMD_SET_ORIGIN:    goal.command_type = "set_origin";    break;
    case ASR_CMD_ELAND:         goal.command_type = "eland";         break;
    case ASR_CMD_SET_LINEAR_SPEED:
        goal.command_type = "set_linear_speed";
        goal.target_pose  = {static_cast<double>(cmd.param1)};
        break;
    case ASR_CMD_SET_ANGULAR_SPEED:
        goal.command_type = "set_angular_speed";
        goal.target_pose  = {static_cast<double>(cmd.param1)};
        break;
    case ASR_CMD_SPIN:
        goal.command_type = "spin";
        goal.target_pose  = {static_cast<double>(cmd.param1),
                              static_cast<double>(cmd.param2),
                              static_cast<double>(cmd.param3)};
        break;
    case ASR_CMD_VELOCITY:
        goal.command_type = "velocity";
        goal.target_pose  = {static_cast<double>(cmd.param1)};
        break;
    case ASR_CMD_MULTI_WAYPOINT:
        goal.command_type = "test_multi_waypoint";
        break;
    default:
        RCLCPP_WARN(get_logger(), "Ignoring unknown COMMAND_LONG id %u", cmd.command);
        return;
    }

    if (!action_client_->action_server_is_ready()) {
        RCLCPP_WARN(get_logger(), "Autopilot action server not ready — dropping command '%s'",
                    goal.command_type.c_str());
        mavlink_message_t mav{};
        mavlink_msg_command_ack_pack(system_id_, component_id_, &mav,
            cmd.command, MAV_RESULT_TEMPORARILY_REJECTED, 255, 0, 0, 0);
        send_mavlink(mav);
        return;
    }

    const uint16_t cmd_id = cmd.command;
    auto opts = rclcpp_action::Client<DroneCommand>::SendGoalOptions{};

    opts.goal_response_callback = [this, cmd_id](const GoalHandleDroneCmd::SharedPtr& handle) {
        if (!handle) {
            mavlink_message_t mav{};
            mavlink_msg_command_ack_pack(system_id_, component_id_, &mav,
                cmd_id, MAV_RESULT_DENIED, 255, 0, 0, 0);
            send_mavlink(mav);
        }
    };

    opts.result_callback = [this, cmd_id](const GoalHandleDroneCmd::WrappedResult& res) {
        uint8_t mav_result;
        if (res.code == rclcpp_action::ResultCode::SUCCEEDED && res.result->success)
            mav_result = MAV_RESULT_ACCEPTED;
        else if (res.code == rclcpp_action::ResultCode::CANCELED)
            mav_result = MAV_RESULT_CANCELLED;
        else
            mav_result = MAV_RESULT_FAILED;

        mavlink_message_t mav{};
        mavlink_msg_command_ack_pack(system_id_, component_id_, &mav,
            cmd_id, mav_result, 255, 0, 0, 0);
        send_mavlink(mav);
    };

    action_client_->async_send_goal(goal, opts);
}

void CommsUav::handle_rtcm(const mavlink_gps_rtcm_data_t& rtcm)
{
    if (!(rtcm.flags & 0x01u)) {
        publish_gps_inject(rtcm.data, rtcm.len);
        return;
    }

    const uint8_t seq      = (rtcm.flags >> 3) & 0x1Fu;
    const uint8_t frag_idx = (rtcm.flags >> 1) & 0x03u;

    auto& buf = rtcm_frags_[seq];
    buf.frags[frag_idx].assign(rtcm.data, rtcm.data + rtcm.len);
    buf.received_mask |= static_cast<uint8_t>(1u << frag_idx);

    if (rtcm.len < 180) {
        buf.last_frag_idx = frag_idx;
        buf.last_known    = true;
    }

    if (!buf.last_known) return;

    const uint8_t expected = static_cast<uint8_t>((1u << (buf.last_frag_idx + 1)) - 1u);
    if ((buf.received_mask & expected) != expected) return;

    std::vector<uint8_t> assembled;
    assembled.reserve(static_cast<size_t>(buf.last_frag_idx + 1) * 180);
    for (uint8_t i = 0; i <= buf.last_frag_idx; ++i)
        assembled.insert(assembled.end(), buf.frags[i].begin(), buf.frags[i].end());

    buf.reset();
    publish_gps_inject(assembled.data(), assembled.size());
}

void CommsUav::publish_gps_inject(const uint8_t* data, size_t len)
{
    size_t offset = 0;
    while (offset < len) {
        const size_t chunk = std::min<size_t>(len - offset, 300u);

        px4_msgs::msg::GpsInjectData out{};
        out.timestamp = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
        out.len       = static_cast<uint16_t>(chunk);
        out.flags     = (len > 300u) ? 1u : 0u;
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
    std::lock_guard<std::mutex> lock(send_mutex_);
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

void CommsUav::on_position(const asr_comms::msg::TelemetryPosition::SharedPtr msg)
{
    mavlink_message_t mav{};
    mavlink_msg_local_position_ned_pack(system_id_, component_id_, &mav,
        static_cast<uint32_t>(msg->timestamp * 1e3),
        msg->position[0], msg->position[1], msg->position[2],
        msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    send_mavlink(mav);
}

void CommsUav::on_attitude(const asr_comms::msg::TelemetryAttitude::SharedPtr msg)
{
    // Angular rates not available in TelemetryAttitude — send zero
    mavlink_message_t mav{};
    mavlink_msg_attitude_pack(system_id_, component_id_, &mav,
        static_cast<uint32_t>(msg->timestamp * 1e3),
        msg->orientation[0], msg->orientation[1], msg->orientation[2],
        0.0f, 0.0f, 0.0f);
    send_mavlink(mav);
}

void CommsUav::on_battery(const asr_comms::msg::TelemetryBattery::SharedPtr msg)
{
    uint16_t cell_voltages[10];
    std::fill(std::begin(cell_voltages), std::end(cell_voltages), UINT16_MAX);
    cell_voltages[0] = static_cast<uint16_t>(msg->voltage * 1000.0f);

    const int8_t pct = static_cast<int8_t>(std::clamp(msg->percentage * 100.0f, 0.0f, 100.0f));

    mavlink_message_t mav{};
    mavlink_msg_battery_status_pack(system_id_, component_id_, &mav,
        0, MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO,
        INT16_MAX,
        cell_voltages,
        static_cast<int16_t>(msg->current * 100.0f),
        static_cast<int32_t>(msg->discharged_mah),
        -1, pct,
        0, MAV_BATTERY_CHARGE_STATE_OK, nullptr, 0, 0);
    send_mavlink(mav);
}

void CommsUav::on_gps(const asr_comms::msg::TelemetryGPS::SharedPtr msg)
{
    mavlink_message_t mav{};
    mavlink_msg_gps_raw_int_pack(system_id_, component_id_, &mav,
        static_cast<uint64_t>(msg->timestamp * 1e6),
        GPS_FIX_TYPE_3D_FIX,
        static_cast<int32_t>(msg->latitude  * 1e7),
        static_cast<int32_t>(msg->longitude * 1e7),
        0,           // alt (mm)
        UINT16_MAX,  // eph (unknown)
        UINT16_MAX,  // epv (unknown)
        UINT16_MAX,  // vel (unknown)
        UINT16_MAX,  // cog (unknown)
        static_cast<uint8_t>(msg->satellites_used),
        0,           // alt_ellipsoid
        UINT32_MAX,  // h_acc (unknown)
        UINT32_MAX,  // v_acc (unknown)
        UINT32_MAX,  // vel_acc (unknown)
        UINT32_MAX,  // hdg_acc (unknown)
        0);          // yaw (unknown)
    send_mavlink(mav);
}

void CommsUav::on_status(const asr_comms::msg::TelemetryStatus::SharedPtr msg)
{
#pragma pack(push, 1)
    struct StatusPod {
        double   timestamp;
        double   flight_time;
        float    actuator_speeds[4];
        float    uav_rx_kbps;
        int32_t  probes_found;
        int16_t  flight_mode;
        int16_t  led_mode;
        uint8_t  arming_state;
        uint8_t  trajectory_mode;
        uint8_t  estop;
    };
#pragma pack(pop)
    static_assert(sizeof(StatusPod) <= 249);

    StatusPod pod{};
    pod.timestamp       = msg->timestamp;
    pod.flight_time     = msg->flight_time;
    pod.probes_found    = msg->probes_found;
    pod.flight_mode     = msg->flight_mode;
    pod.led_mode        = msg->led_mode;
    pod.arming_state    = msg->arming_state;
    pod.trajectory_mode = msg->trajectory_mode;
    pod.estop           = msg->estop;
    pod.uav_rx_kbps     = uav_rx_kbps_;
    for (int i = 0; i < 4; ++i)
        pod.actuator_speeds[i] = msg->actuator_speeds.size() > size_t(i)
                                 ? msg->actuator_speeds[i] : 0.f;

    uint8_t payload[249]{};
    std::memcpy(payload, &pod, sizeof(pod));

    mavlink_message_t mav{};
    mavlink_msg_v2_extension_pack(system_id_, component_id_, &mav,
        0, 0, 0, ASR_MSG_TELEMETRY_STATUS, payload);
    send_mavlink(mav);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommsUav>());
    rclcpp::shutdown();
    return 0;
}
