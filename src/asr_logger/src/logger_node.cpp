#include "asr_logger/logger_node.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

using std::placeholders::_1;

namespace asr_logger {

// ---------------------------------------------------------------------------

static std::string makeLogFilename(const std::string& dir)
{
    std::filesystem::create_directories(dir);
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream ss;
    ss << dir << "/flight_"
       << std::put_time(&tm, "%Y%m%d_%H%M%S")
       << ".ulg";
    return ss.str();
}

static uint64_t nowUs()
{
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
        .count());
}

// ---------------------------------------------------------------------------

LoggerNode::LoggerNode()
: Node("asr_logger")
{
    declare_parameter("log_dir",  std::string{"~/flight_logs"});
    declare_parameter("log_mode", std::string{"general"});

    log_dir_  = get_parameter("log_dir").as_string();
    log_mode_ = get_parameter("log_mode").as_string();

    if (!log_dir_.empty() && log_dir_[0] == '~') {
        const char* home = std::getenv("HOME");
        if (home) log_dir_ = std::string(home) + log_dir_.substr(1);
    }

    initWriter();

    auto qos = rclcpp::SensorDataQoS();

    position_sub_ = create_subscription<asr_comms::msg::TelemetryPosition>(
        "out/telemetry/position", qos,
        std::bind(&LoggerNode::onPosition, this, _1));

    attitude_sub_ = create_subscription<asr_comms::msg::TelemetryAttitude>(
        "out/telemetry/attitude", qos,
        std::bind(&LoggerNode::onAttitude, this, _1));

    battery_sub_ = create_subscription<asr_comms::msg::TelemetryBattery>(
        "out/telemetry/battery", qos,
        std::bind(&LoggerNode::onBattery, this, _1));

    status_sub_ = create_subscription<asr_comms::msg::TelemetryStatus>(
        "out/telemetry/status", qos,
        std::bind(&LoggerNode::onStatus, this, _1));

    // DroneScope subscription always active; only written in control_inspection mode
    drone_scope_sub_ = create_subscription<asr_comms::msg::DroneScope>(
        "out/drone_scope", qos,
        std::bind(&LoggerNode::onScope, this, _1));

    trajectory_setpoint_sub_ = create_subscription<asr_comms::msg::TrajectorySetpoint>(
        "out/trajectory_setpoint", qos,
        std::bind(&LoggerNode::onTrajectorySetpoint, this, _1));

    control_detail_sub_ = create_subscription<asr_comms::msg::ControlDetail>(
        "out/control_detail", qos,
        std::bind(&LoggerNode::onControlDetail, this, _1));

    comms_health_sub_ = create_subscription<asr_comms::msg::CommsHealth>(
        "out/comms_health", qos,
        std::bind(&LoggerNode::onCommsHealth, this, _1));

    flush_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&LoggerNode::onFlushTimer, this));

    RCLCPP_INFO(get_logger(), "Logger started — mode: %s", log_mode_.c_str());
}

LoggerNode::~LoggerNode()
{
    if (writer_) {
        writer_->fsync();
    }
}

void LoggerNode::initWriter()
{
    // Field definitions live here to avoid the static-initialization-order fiasco:
    // Field() looks up Field::kBasicTypes (defined in messages.cpp) and those two
    // translation units have no guaranteed init order at file scope.
    using F = ulog_cpp::Field;
    const std::vector<F> position_fields = {
        {"uint64_t", "timestamp"},
        {"float", "pos_x"},    {"float", "pos_y"},    {"float", "pos_z"},
        {"float", "vel_x"},    {"float", "vel_y"},    {"float", "vel_z"},
        {"float", "target_x"}, {"float", "target_y"}, {"float", "target_z"},
    };
    const std::vector<F> attitude_fields = {
        {"uint64_t", "timestamp"},
        {"float", "roll"}, {"float", "pitch"}, {"float", "yaw"},
        {"float", "accel_x"}, {"float", "accel_y"}, {"float", "accel_z"},
    };
    const std::vector<F> battery_fields = {
        {"uint64_t", "timestamp"},
        {"float", "voltage"}, {"float", "current"}, {"float", "percentage"},
        {"float", "discharged_mah"}, {"float", "average_current"},
    };
    const std::vector<F> status_fields = {
        {"uint64_t", "timestamp"},
        {"float",   "flight_time"},
        {"float",   "actuator_speed_0"}, {"float", "actuator_speed_1"},
        {"float",   "actuator_speed_2"}, {"float", "actuator_speed_3"},
        {"int32_t", "probes_found"},
        {"int16_t", "flight_mode"}, {"int16_t", "led_mode"},
        {"uint8_t", "arming_state"}, {"uint8_t", "trajectory_mode"}, {"uint8_t", "estop"},
    };
    const std::vector<F> control_scope_fields = {
        {"uint64_t", "timestamp"},
        {"float", "control_output_z"}, {"float", "target_z"},
    };
    const std::vector<F> trajectory_setpoint_fields = {
        {"uint64_t", "timestamp"},
        {"float", "ref_pos_x"}, {"float", "ref_pos_y"}, {"float", "ref_pos_z"},
        {"float", "ref_vel_x"}, {"float", "ref_vel_y"}, {"float", "ref_vel_z"},
        {"float", "ref_acc_x"}, {"float", "ref_acc_y"}, {"float", "ref_acc_z"},
        {"float", "ref_yaw"},
    };
    const std::vector<F> control_detail_fields = {
        {"uint64_t", "timestamp"},
        {"float", "pos_ctrl_x"}, {"float", "pos_ctrl_y"}, {"float", "pos_ctrl_z"},
        {"float", "pos_err_x"},  {"float", "pos_err_y"},  {"float", "pos_err_z"},
        {"float", "pos_int_x"},  {"float", "pos_int_y"},  {"float", "pos_int_z"},
        {"float", "vel_cmd_x"},  {"float", "vel_cmd_y"},  {"float", "vel_cmd_z"},
        {"float", "hover_thrust_estimate"},
    };
    const std::vector<F> comms_health_fields = {
        {"uint64_t", "timestamp"},
        {"float",   "rx_kbps"}, {"float", "gcs_msg_age_ms"},
        {"uint8_t", "gcs_connected"},
    };

    const std::string path = makeLogFilename(log_dir_);

    writer_ = std::make_unique<ulog_cpp::SimpleWriter>(path, nowUs());
    writer_->writeInfo("sys_name", std::string{"asr_thyra"});
    writer_->writeInfo("log_mode", log_mode_);

    writer_->writeMessageFormat("position",             position_fields);
    writer_->writeMessageFormat("attitude",             attitude_fields);
    writer_->writeMessageFormat("battery",              battery_fields);
    writer_->writeMessageFormat("status",               status_fields);
    writer_->writeMessageFormat("control_scope",        control_scope_fields);
    writer_->writeMessageFormat("trajectory_setpoint",  trajectory_setpoint_fields);
    writer_->writeMessageFormat("control_detail",       control_detail_fields);
    writer_->writeMessageFormat("comms_health",         comms_health_fields);

    writer_->headerComplete();

    id_position_       = writer_->writeAddLoggedMessage("position");
    id_attitude_       = writer_->writeAddLoggedMessage("attitude");
    id_battery_        = writer_->writeAddLoggedMessage("battery");
    id_status_         = writer_->writeAddLoggedMessage("status");
    id_control_scope_        = writer_->writeAddLoggedMessage("control_scope");
    id_trajectory_setpoint_  = writer_->writeAddLoggedMessage("trajectory_setpoint");
    id_control_detail_ = writer_->writeAddLoggedMessage("control_detail");
    id_comms_health_   = writer_->writeAddLoggedMessage("comms_health");

    RCLCPP_INFO(get_logger(), "Writing to: %s", path.c_str());
}

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

void LoggerNode::onPosition(const asr_comms::msg::TelemetryPosition& msg)
{
    UlogPosition frame{
        .timestamp = toUs(msg.timestamp),
        .pos_x = msg.position[0], .pos_y = msg.position[1], .pos_z = msg.position[2],
        .vel_x = msg.velocity[0], .vel_y = msg.velocity[1], .vel_z = msg.velocity[2],
        .target_x = msg.target_position[0],
        .target_y = msg.target_position[1],
        .target_z = msg.target_position[2],
    };
    writer_->writeData(id_position_, frame);
}

void LoggerNode::onAttitude(const asr_comms::msg::TelemetryAttitude& msg)
{
    UlogAttitude frame{
        .timestamp = toUs(msg.timestamp),
        .roll  = msg.orientation[0],
        .pitch = msg.orientation[1],
        .yaw   = msg.orientation[2],
        .accel_x = msg.acceleration[0],
        .accel_y = msg.acceleration[1],
        .accel_z = msg.acceleration[2],
    };
    writer_->writeData(id_attitude_, frame);
}

void LoggerNode::onBattery(const asr_comms::msg::TelemetryBattery& msg)
{
    UlogBattery frame{
        .timestamp       = toUs(msg.timestamp),
        .voltage         = msg.voltage,
        .current         = msg.current,
        .percentage      = msg.percentage,
        .discharged_mah  = msg.discharged_mah,
        .average_current = msg.average_current,
    };
    writer_->writeData(id_battery_, frame);
}

void LoggerNode::onStatus(const asr_comms::msg::TelemetryStatus& msg)
{
    UlogStatus frame{
        .timestamp       = toUs(msg.timestamp),
        .flight_time     = static_cast<float>(msg.flight_time),
        .actuator_speed_0 = msg.actuator_speeds[0],
        .actuator_speed_1 = msg.actuator_speeds[1],
        .actuator_speed_2 = msg.actuator_speeds[2],
        .actuator_speed_3 = msg.actuator_speeds[3],
        .probes_found    = msg.probes_found,
        .flight_mode     = msg.flight_mode,
        .led_mode        = msg.led_mode,
        .arming_state    = msg.arming_state,
        .trajectory_mode = msg.trajectory_mode,
        .estop           = msg.estop,
    };
    writer_->writeData(id_status_, frame);
}

void LoggerNode::onScope(const asr_comms::msg::DroneScope& msg)
{
    if (log_mode_ != "control_inspection") {
        return;
    }
    UlogControlScope frame{
        .timestamp        = toUs(msg.timestamp),
        .control_output_z = static_cast<float>(msg.control_output_z),
        .target_z         = static_cast<float>(msg.target_z),
    };
    writer_->writeData(id_control_scope_, frame);
}

void LoggerNode::onTrajectorySetpoint(const asr_comms::msg::TrajectorySetpoint& msg)
{
    UlogTrajectorySetpoint frame{
        .timestamp = toUs(msg.timestamp),
        .ref_pos_x = msg.ref_pos_x, .ref_pos_y = msg.ref_pos_y, .ref_pos_z = msg.ref_pos_z,
        .ref_vel_x = msg.ref_vel_x, .ref_vel_y = msg.ref_vel_y, .ref_vel_z = msg.ref_vel_z,
        .ref_acc_x = msg.ref_acc_x, .ref_acc_y = msg.ref_acc_y, .ref_acc_z = msg.ref_acc_z,
        .ref_yaw   = msg.ref_yaw,
    };
    writer_->writeData(id_trajectory_setpoint_, frame);
}

void LoggerNode::onControlDetail(const asr_comms::msg::ControlDetail& msg)
{
    if (log_mode_ != "control_inspection") {
        return;
    }
    UlogControlDetail frame{
        .timestamp             = toUs(msg.timestamp),
        .pos_ctrl_x            = msg.pos_ctrl_x,
        .pos_ctrl_y            = msg.pos_ctrl_y,
        .pos_ctrl_z            = msg.pos_ctrl_z,
        .pos_err_x             = msg.pos_err_x,
        .pos_err_y             = msg.pos_err_y,
        .pos_err_z             = msg.pos_err_z,
        .pos_int_x             = msg.pos_int_x,
        .pos_int_y             = msg.pos_int_y,
        .pos_int_z             = msg.pos_int_z,
        .vel_cmd_x             = msg.vel_cmd_x,
        .vel_cmd_y             = msg.vel_cmd_y,
        .vel_cmd_z             = msg.vel_cmd_z,
        .hover_thrust_estimate = msg.hover_thrust_estimate,
    };
    writer_->writeData(id_control_detail_, frame);
}

void LoggerNode::onFlushTimer()
{
    if (writer_) {
        writer_->fsync();
    }
}

void LoggerNode::onCommsHealth(const asr_comms::msg::CommsHealth& msg)
{
    UlogCommsHealth frame{
        .timestamp      = toUs(msg.timestamp),
        .rx_kbps        = msg.rx_kbps,
        .gcs_msg_age_ms = msg.gcs_msg_age_ms,
        .gcs_connected  = msg.gcs_connected ? uint8_t{1} : uint8_t{0},
    };
    writer_->writeData(id_comms_health_, frame);
}

}  // namespace asr_logger

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<asr_logger::LoggerNode>());
    rclcpp::shutdown();
    return 0;
}
