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
    declare_parameter("baud_rate",    115200);
    declare_parameter("system_id",    static_cast<int>(system_id_));
    declare_parameter("component_id", static_cast<int>(component_id_));
    declare_parameter("wifi_port",    14552);
    declare_parameter("wifi_enabled", true);
    declare_parameter("camera_port",  static_cast<int>(camera_port_));

    system_id_    = static_cast<uint8_t>(get_parameter("system_id").as_int());
    component_id_ = static_cast<uint8_t>(get_parameter("component_id").as_int());
    wifi_port_    = static_cast<uint16_t>(get_parameter("wifi_port").as_int());
    camera_port_  = static_cast<uint16_t>(get_parameter("camera_port").as_int());
    const bool wifi_enabled = get_parameter("wifi_enabled").as_bool();

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
    uav_heartbeat_pub_ = create_publisher<std_msgs::msg::Bool>("uav_heartbeat", 10);
    position_pub_ = create_publisher<asr_comms::msg::TelemetryPosition>("telemetry/position", 10);
    attitude_pub_ = create_publisher<asr_comms::msg::TelemetryAttitude>("telemetry/attitude", 10);
    battery_pub_  = create_publisher<asr_comms::msg::TelemetryBattery>( "telemetry/battery",  10);
    gps_pub_      = create_publisher<asr_comms::msg::TelemetryGPS>(     "telemetry/gps",      10);
    status_pub_   = create_publisher<asr_comms::msg::TelemetryStatus>(  "telemetry/status",   10);
    command_ack_pub_ = create_publisher<asr_comms::msg::CommandAck>("command_ack", 10);
    camera_pub_      = create_publisher<sensor_msgs::msg::CompressedImage>("camera/image/compressed", 10);

    camera_stream_sub_ = create_subscription<asr_comms::msg::CameraStreamRequest>(
        "in/camera_stream", 1,
        std::bind(&CommsGcs::on_camera_stream_request, this, std::placeholders::_1));

    // Send side
    heartbeat_timer_  = create_wall_timer(1s, std::bind(&CommsGcs::send_heartbeat, this));
    stats_timer_      = create_wall_timer(1s, std::bind(&CommsGcs::publish_link_stats, this));
    link_stats_pub_   = create_publisher<asr_comms::msg::LinkStats>("link_stats", 10);

    rtcm_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        "rtcm", 10,
        std::bind(&CommsGcs::send_rtcm, this, std::placeholders::_1));

    uav_command_sub_ = create_subscription<asr_comms::msg::UAVCommand>(
        "in/uav_command", 1,
        std::bind(&CommsGcs::on_uav_command, this, std::placeholders::_1));

    auto qos_rt = rclcpp::QoS(1).best_effort();

    gcs_heartbeat_sub_ = create_subscription<asr_comms::msg::GcsHeartbeat>(
        "in/gcs_heartbeat", rclcpp::QoS(10).best_effort(),
        std::bind(&CommsGcs::on_gcs_heartbeat, this, std::placeholders::_1));

    manual_input_sub_ = create_subscription<asr_comms::msg::ManualControlInput>(
        "in/manual_input", qos_rt,
        std::bind(&CommsGcs::on_manual_input, this, std::placeholders::_1));

    servo_command_sub_ = create_subscription<asr_comms::msg::ServoCommand>(
        "in/servo_command", qos_rt,
        std::bind(&CommsGcs::on_servo_command, this, std::placeholders::_1));

    if (!wifi_enabled) {
        RCLCPP_INFO(get_logger(), "WiFi dual-path disabled (wifi_enabled: false)");
    } else {
        // Auto-detect local WiFi IP via routing trick (no packets sent).
        int s = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (s >= 0) {
            sockaddr_in dummy{};
            dummy.sin_family = AF_INET;
            dummy.sin_port   = htons(80);
            inet_pton(AF_INET, "8.8.8.8", &dummy.sin_addr);
            if (connect(s, reinterpret_cast<sockaddr*>(&dummy), sizeof(dummy)) == 0) {
                sockaddr_in local{};
                socklen_t   len = sizeof(local);
                getsockname(s, reinterpret_cast<sockaddr*>(&local), &len);
                wifi_ip_ = local.sin_addr.s_addr;
                char ip_str[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &local.sin_addr, ip_str, sizeof(ip_str));
                RCLCPP_INFO(get_logger(), "WiFi peer beacon: advertising %s:%u", ip_str, wifi_port_);
            } else {
                RCLCPP_WARN(get_logger(), "No default route — WiFi dual-path beacon disabled");
            }
            close(s);
        }

        // Open the WiFi server socket (peer address learned from first incoming packet).
        try {
            auto sock = std::make_unique<UdpSocket>(wifi_port_);
            sock->set_recv_timeout_ms(200);
            wifi_transport_ = std::move(sock);
            wifi_recv_thread_ = std::thread(&CommsGcs::wifi_recv_loop, this);
            RCLCPP_INFO(get_logger(), "WiFi server socket bound on :%u", wifi_port_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to open WiFi socket: %s — dual-path disabled", e.what());
        }

        beacon_timer_ = create_wall_timer(1s, std::bind(&CommsGcs::send_peer_beacon, this));

        // Camera receive socket — UAV streams JPEG frames here over WiFi only.
        try {
            auto cam_sock = std::make_unique<UdpSocket>(camera_port_);
            cam_sock->set_recv_timeout_ms(200);
            camera_transport_ = std::move(cam_sock);
            camera_recv_thread_ = std::thread(&CommsGcs::camera_recv_loop, this);
            RCLCPP_INFO(get_logger(), "Camera RX socket bound on :%u", camera_port_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to open camera socket: %s", e.what());
        }
    }

    mavlink_set_proto_version(MAVLINK_COMM_0, 2);
    mavlink_set_proto_version(MAVLINK_COMM_1, 2);
    recv_thread_ = std::thread(&CommsGcs::recv_loop, this);
}

CommsGcs::~CommsGcs()
{
    running_ = false;
    if (recv_thread_.joinable())        recv_thread_.join();
    if (wifi_recv_thread_.joinable())   wifi_recv_thread_.join();
    if (camera_recv_thread_.joinable()) camera_recv_thread_.join();
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

        radio_rx_bytes_ += static_cast<size_t>(n);
        last_rx_ns_.store(static_cast<uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count()));
        for (ssize_t i = 0; i < n; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                if (dedup_.check(msg.sysid, msg.compid, msg.seq)) {
                    ++radio_rx_msgs_;
                    std::lock_guard<std::mutex> lock(recv_mutex_);
                    handle_message(msg);
                }
            }
        }
    }
}

void CommsGcs::wifi_recv_loop()
{
    uint8_t           buf[UDP_BUF_SIZE];
    mavlink_message_t msg{};
    mavlink_status_t  status{};

    while (running_) {
        const ssize_t n = wifi_transport_->recv(buf, sizeof(buf));
        if (n <= 0) continue;

        wifi_rx_bytes_ += static_cast<size_t>(n);
        last_rx_ns_.store(static_cast<uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count()));

        for (ssize_t i = 0; i < n; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_1, buf[i], &msg, &status)) {
                if (dedup_.check(msg.sysid, msg.compid, msg.seq)) {
                    ++wifi_rx_msgs_;
                    std::lock_guard<std::mutex> lock(recv_mutex_);
                    handle_message(msg);
                }
            }
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
        out.timestamp       = get_clock()->now().seconds();
        out.voltage         = b.voltages[0] / 1000.0f;
        out.current         = b.current_battery / 100.0f;
        out.percentage      = static_cast<float>(b.battery_remaining) / 100.0f;
        out.discharged_mah  = static_cast<float>(b.current_consumed);
        out.average_current = out.current;
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

    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t ack{};
        mavlink_msg_command_ack_decode(&msg, &ack);

        static const char* result_strings[] = {
            "accepted", "temporarily rejected", "denied",
            "unsupported", "failed", "in progress", "cancelled"
        };
        const char* result_str = (ack.result < 7) ? result_strings[ack.result] : "unknown";

        asr_comms::msg::CommandAck out{};
        out.command_type = pending_command_type_;
        out.result       = ack.result;
        out.message      = std::string(pending_command_type_) + ": " + result_str;
        command_ack_pub_->publish(out);
        RCLCPP_INFO(get_logger(), "Command ack — '%s' %s",
                    pending_command_type_.c_str(), result_str);
        break;
    }

    case MAVLINK_MSG_ID_RADIO_STATUS: {
        mavlink_radio_status_t radio{};
        mavlink_msg_radio_status_decode(&msg, &radio);
        radio_rssi_.store(radio.rssi);
        radio_remrssi_.store(radio.remrssi);
        radio_noise_.store(radio.noise);
        radio_remnoise_.store(radio.remnoise);
        radio_txbuf_.store(radio.txbuf);
        radio_rxerrors_.store(radio.rxerrors);
        radio_fixed_.store(radio.fixed);
        last_radio_ns_.store(static_cast<uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count()));
        break;
    }

    case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT: {
        mavlink_named_value_float_t nv{};
        mavlink_msg_named_value_float_decode(&msg, &nv);
        if      (std::strncmp(nv.name, "rx_kbps",  sizeof(nv.name)) == 0)
            uav_rx_kbps_.store(nv.value);
        else if (std::strncmp(nv.name, "uav_txbuf", sizeof(nv.name)) == 0) {
            uav_radio_txbuf_.store(static_cast<uint8_t>(nv.value));
            last_uav_radio_ns_.store(static_cast<uint64_t>(
                std::chrono::steady_clock::now().time_since_epoch().count()));
        }
        else if (std::strncmp(nv.name, "uav_rssi",  sizeof(nv.name)) == 0)
            uav_radio_rssi_.store(static_cast<uint8_t>(nv.value));
        else if (std::strncmp(nv.name, "uav_noise", sizeof(nv.name)) == 0)
            uav_radio_noise_.store(static_cast<uint8_t>(nv.value));
        else if (std::strncmp(nv.name, "uav_rxerr",   sizeof(nv.name)) == 0)
            uav_radio_rxerrors_.store(static_cast<uint16_t>(nv.value));
        else if (std::strncmp(nv.name, "cam_stream",  sizeof(nv.name)) == 0)
            camera_streaming_.store(nv.value >= 0.5f);
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
    tx_bytes_ += len;
    if (wifi_transport_)
        wifi_transport_->send(buf, len);
}

void CommsGcs::send_peer_beacon()
{
    if (wifi_ip_ == 0) return;

    // Discovery: 1 Hz until the WiFi peer is known. Keepalive: every 10 s.
    const bool connected = wifi_transport_ && wifi_transport_->peer_known();
    if (connected && (++beacon_tick_ % 10 != 0)) return;
    if (!connected) beacon_tick_ = 0;  // reset so reconnection re-enters discovery

#pragma pack(push, 1)
    struct PeerBeacon { uint32_t ip; uint16_t port; };
#pragma pack(pop)
    PeerBeacon beacon{ wifi_ip_, htons(wifi_port_) };
    uint8_t payload[249]{};
    std::memcpy(payload, &beacon, sizeof(beacon));
    mavlink_message_t msg{};
    mavlink_msg_v2_extension_pack(system_id_, component_id_, &msg,
        0, 0, 0, ASR_MSG_PEER_BEACON, payload);
    send_mavlink(msg);
    RCLCPP_DEBUG(get_logger(), "Peer beacon sent (connected=%s)", connected ? "yes" : "no");
}

void CommsGcs::publish_link_stats()
{
    const uint64_t now_ns = static_cast<uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    constexpr uint64_t LINK_TIMEOUT_NS  = 2'000'000'000ULL; // 2 s
    constexpr uint64_t RADIO_TIMEOUT_NS = 2'000'000'000ULL; // 2 s

    const uint64_t last_rx    = last_rx_ns_.load();
    const uint64_t last_radio = last_radio_ns_.load();

    const size_t radio_bytes = radio_rx_bytes_.exchange(0);
    const size_t wifi_bytes  = wifi_rx_bytes_.exchange(0);

    asr_comms::msg::LinkStats out{};

    out.mavlink_connected = (last_rx != 0) && ((now_ns - last_rx) < LINK_TIMEOUT_NS);
    out.wifi_connected    = wifi_transport_ && wifi_transport_->peer_known();
    out.camera_streaming  = camera_streaming_.load();

    out.radio_tx_kbps   = static_cast<float>(tx_bytes_.exchange(0)) * 8.0f / 1000.0f;
    out.mavlink_rx_kbps = static_cast<float>(radio_bytes + wifi_bytes) * 8.0f / 1000.0f;
    out.radio_rx_kbps   = static_cast<float>(radio_bytes) * 8.0f / 1000.0f;
    out.wifi_rx_kbps    = static_cast<float>(wifi_bytes)  * 8.0f / 1000.0f;
    out.radio_rx_msgs   = radio_rx_msgs_.exchange(0);
    out.wifi_rx_msgs    = wifi_rx_msgs_.exchange(0);
    out.uav_rx_kbps     = uav_rx_kbps_.load();

    out.camera_rx_kbps = static_cast<float>(camera_rx_bytes_.exchange(0)) * 8.0f / 1000.0f;

    out.radio_ok      = (last_radio != 0) && ((now_ns - last_radio) < RADIO_TIMEOUT_NS);
    out.radio_rssi    = radio_rssi_.load();
    out.radio_remrssi = radio_remrssi_.load();
    out.radio_noise   = radio_noise_.load();
    out.radio_remnoise = radio_remnoise_.load();
    out.radio_txbuf   = radio_txbuf_.load();
    out.radio_rxerrors = radio_rxerrors_.load();
    out.radio_fixed   = radio_fixed_.load();

    using LS = asr_comms::msg::LinkStats;
    if (!out.radio_ok) {
        out.signal_quality = LS::QUALITY_NO_RADIO;
    } else {
        const int snr = static_cast<int>(out.radio_rssi) - static_cast<int>(out.radio_noise);
        if      (snr <  20) out.signal_quality = LS::QUALITY_CRITICAL;
        else if (snr <  40) out.signal_quality = LS::QUALITY_POOR;
        else if (snr <  70) out.signal_quality = LS::QUALITY_FAIR;
        else if (snr < 100) out.signal_quality = LS::QUALITY_GOOD;
        else                out.signal_quality = LS::QUALITY_EXCELLENT;
    }

    const uint64_t last_uav_radio = last_uav_radio_ns_.load();
    out.uav_radio_ok  = (last_uav_radio != 0) && ((now_ns - last_uav_radio) < RADIO_TIMEOUT_NS);
    out.uav_rssi      = uav_radio_rssi_.load();
    out.uav_noise     = uav_radio_noise_.load();
    out.uav_txbuf     = uav_radio_txbuf_.load();
    out.uav_rxerrors  = uav_radio_rxerrors_.load();

    link_stats_pub_->publish(out);
}

void CommsGcs::send_heartbeat()
{
    mavlink_message_t msg{};
    mavlink_msg_heartbeat_pack(system_id_, component_id_, &msg,
        MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
        static_cast<uint8_t>(gcs_nominal_), 0, MAV_STATE_ACTIVE);
    send_mavlink(msg);
}

void CommsGcs::on_gcs_heartbeat(const asr_comms::msg::GcsHeartbeat::SharedPtr msg)
{
    gcs_nominal_ = msg->gcs_nominal;
}

void CommsGcs::on_manual_input(const asr_comms::msg::ManualControlInput::SharedPtr msg)
{
    const int16_t x = static_cast<int16_t>(msg->pitch        * 1000.0f);
    const int16_t y = static_cast<int16_t>(msg->roll         * 1000.0f);
    const int16_t z = static_cast<int16_t>(msg->thrust       * 1000.0f);
    const int16_t r = static_cast<int16_t>(msg->yaw_velocity * 1000.0f);
    const uint16_t buttons =
          static_cast<uint16_t>(msg->arm          & 0x01u)
        | static_cast<uint16_t>((msg->estop        & 0x01u) << 1)
        | static_cast<uint16_t>((msg->selfdestruct & 0x01u) << 2);

    mavlink_message_t mav{};
    mavlink_msg_manual_control_pack(system_id_, component_id_, &mav,
        1 /* target_system */, x, y, z, r, buttons,
        0 /* buttons2 */, 0 /* enabled_extensions */,
        0, 0, 0, 0, 0, 0, 0, 0 /* aux1-6, s, t */);
    send_mavlink(mav);
}

void CommsGcs::on_servo_command(const asr_comms::msg::ServoCommand::SharedPtr msg)
{
#pragma pack(push, 1)
    struct ServoPod {
        uint64_t timestamp;
        int32_t  aux_index;
        int32_t  id;
        float    value;
    };
#pragma pack(pop)
    static_assert(sizeof(ServoPod) <= 249);

    ServoPod pod{};
    pod.timestamp = msg->timestamp;
    pod.aux_index = msg->aux_index;
    pod.id        = msg->id;
    pod.value     = msg->value;

    uint8_t payload[249]{};
    std::memcpy(payload, &pod, sizeof(pod));

    mavlink_message_t mav{};
    mavlink_msg_v2_extension_pack(system_id_, component_id_, &mav,
        0, 0, 0, ASR_MSG_SERVO_COMMAND, payload);
    send_mavlink(mav);
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

void CommsGcs::on_uav_command(const asr_comms::msg::UAVCommand::SharedPtr msg)
{
    const auto now = std::chrono::steady_clock::now();
    const auto ms_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_command_sent_).count();
    if (msg->command_type == pending_command_type_ && ms_since_last < 300) {
        RCLCPP_WARN(get_logger(), "Ignoring duplicate '%s' command (%ld ms since last)",
                    msg->command_type.c_str(), ms_since_last);
        return;
    }
    last_command_sent_ = now;
    pending_command_type_ = msg->command_type;
    const auto& cmd = msg->command_type;
    mavlink_message_t mav{};

    auto pack_long = [&](uint16_t command,
                         float p1=0, float p2=0, float p3=0, float p4=0,
                         float p5=0, float p6=0, float p7=0) {
        mavlink_msg_command_long_pack(system_id_, component_id_, &mav,
            1, 1, command, 0, p1, p2, p3, p4, p5, p6, p7);
    };

    const auto& tp = msg->target_pose;
    auto p = [&](size_t i) -> float {
        return (i < tp.size()) ? static_cast<float>(tp[i]) : 0.0f;
    };

    if      (cmd == "arm")    pack_long(MAV_CMD_COMPONENT_ARM_DISARM, 1.0f);
    else if (cmd == "disarm") pack_long(MAV_CMD_COMPONENT_ARM_DISARM, 0.0f);
    else if (cmd == "estop")  pack_long(MAV_CMD_COMPONENT_ARM_DISARM, 0.0f, 21196.0f);
    else if (cmd == "land")   pack_long(MAV_CMD_NAV_LAND);
    else if (cmd == "takeoff")       pack_long(MAV_CMD_NAV_TAKEOFF, 0,0,0,0,0,0, p(0));
    else if (cmd == "goto")          pack_long(ASR_CMD_GOTO,        p(0), p(1), p(2),
                                               static_cast<float>(msg->yaw));
    else if (cmd == "manual")        pack_long(ASR_CMD_MANUAL);
    else if (cmd == "manual_aided")  pack_long(ASR_CMD_MANUAL_AIDED);
    else if (cmd == "set_origin")    pack_long(ASR_CMD_SET_ORIGIN);
    else if (cmd == "eland")         pack_long(ASR_CMD_ELAND);
    else if (cmd == "set_linear_speed")  pack_long(ASR_CMD_SET_LINEAR_SPEED,  p(0));
    else if (cmd == "set_angular_speed") pack_long(ASR_CMD_SET_ANGULAR_SPEED, p(0));
    else if (cmd == "spin")      pack_long(ASR_CMD_SPIN,         p(0), p(1), p(2));
    else if (cmd == "velocity")  pack_long(ASR_CMD_VELOCITY,     p(0));
    else if (cmd == "test_multi_waypoint") pack_long(ASR_CMD_MULTI_WAYPOINT);
    else {
        RCLCPP_WARN(get_logger(), "Unknown command_type: '%s'", cmd.c_str());
        return;
    }

    send_mavlink(mav);
    RCLCPP_INFO(get_logger(), "Sent command '%s' over MAVLink", cmd.c_str());
}

// --- Camera streaming ---

void CommsGcs::camera_recv_loop()
{
    static constexpr size_t BUF_SIZE = sizeof(VideoFrameHeader) + CAMERA_MAX_FRAG_PAYLOAD;
    std::vector<uint8_t> buf(BUF_SIZE);

    while (running_) {
        const ssize_t n = camera_transport_->recv(buf.data(), buf.size());
        if (n < static_cast<ssize_t>(sizeof(VideoFrameHeader))) continue;
        camera_rx_bytes_ += static_cast<size_t>(n);

        VideoFrameHeader hdr{};
        std::memcpy(&hdr, buf.data(), sizeof(hdr));

        const size_t payload_len = static_cast<size_t>(n) - sizeof(VideoFrameHeader);
        if (hdr.payload_size > payload_len || hdr.total_frags == 0) continue;

        const uint8_t* payload = buf.data() + sizeof(VideoFrameHeader);

        std::lock_guard<std::mutex> lock(camera_assembler_mutex_);

        if (hdr.frame_id != camera_assembler_.frame_id) {
            camera_assembler_.frame_id       = hdr.frame_id;
            camera_assembler_.frame_size     = hdr.frame_size;
            camera_assembler_.total_frags    = hdr.total_frags;
            camera_assembler_.received_count = 0;
            camera_assembler_.frags.assign(hdr.total_frags, {});
        }

        if (hdr.frag_idx >= camera_assembler_.frags.size()) continue;
        auto& frag = camera_assembler_.frags[hdr.frag_idx];
        if (frag.received) continue;

        frag.data.assign(payload, payload + hdr.payload_size);
        frag.received = true;
        ++camera_assembler_.received_count;

        if (camera_assembler_.received_count == camera_assembler_.total_frags) {
            std::vector<uint8_t> frame;
            frame.reserve(camera_assembler_.frame_size);
            for (auto& f : camera_assembler_.frags)
                frame.insert(frame.end(), f.data.begin(), f.data.end());

            sensor_msgs::msg::CompressedImage img{};
            img.header.stamp = get_clock()->now();
            img.format       = "jpeg";
            img.data         = std::move(frame);
            camera_pub_->publish(img);
        }
    }
}

void CommsGcs::on_camera_stream_request(
    const asr_comms::msg::CameraStreamRequest::SharedPtr msg)
{
    if (!wifi_transport_ || !wifi_transport_->peer_known()) {
        RCLCPP_WARN(get_logger(), "Camera stream request ignored — WiFi peer not connected");
        return;
    }
    mavlink_message_t mav{};
    if (msg->enabled) {
        mavlink_msg_command_long_pack(system_id_, component_id_, &mav,
            1, 1, MAV_CMD_VIDEO_START_STREAMING,
            0, 0, 0, 0, 0, 0, 0, 0);
        camera_streaming_ = true;
        RCLCPP_INFO(get_logger(), "Requesting camera stream start");
    } else {
        mavlink_msg_command_long_pack(system_id_, component_id_, &mav,
            1, 1, MAV_CMD_VIDEO_STOP_STREAMING,
            0, 0, 0, 0, 0, 0, 0, 0);
        camera_streaming_ = false;
        RCLCPP_INFO(get_logger(), "Stopping camera stream");
    }
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &mav);
    wifi_transport_->send(buf, len);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommsGcs>());
    rclcpp::shutdown();
    return 0;
}
