#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "asr_comms/msg/telemetry_position.hpp"
#include "asr_comms/msg/telemetry_attitude.hpp"
#include "asr_comms/msg/telemetry_battery.hpp"
#include "asr_comms/msg/telemetry_status.hpp"
#include "asr_comms/msg/drone_scope.hpp"
#include "asr_comms/msg/trajectory_setpoint.hpp"
#include "asr_comms/msg/control_detail.hpp"
#include "asr_comms/msg/comms_health.hpp"

#include "simple_writer.hpp"

namespace asr_logger {

// ---------------------------------------------------------------------------
// ULOG data structs
// Fields must be ordered by decreasing type size — no internal padding allowed.
// Tail padding is fine (SimpleWriter copies only the declared field bytes).
// ---------------------------------------------------------------------------

struct UlogPosition {
    uint64_t timestamp;                          // us
    float    pos_x,  pos_y,  pos_z;             // m
    float    vel_x,  vel_y,  vel_z;             // m/s
    float    target_x, target_y, target_z;      // m
};

struct UlogAttitude {
    uint64_t timestamp;                          // us
    float    roll, pitch, yaw;                   // rad
    float    accel_x, accel_y, accel_z;          // m/s^2
};

struct UlogBattery {
    uint64_t timestamp;                          // us
    float    voltage;                            // V
    float    current;                            // A
    float    percentage;                         // 0-100
    float    discharged_mah;
    float    average_current;                    // A
};

struct UlogStatus {
    uint64_t timestamp;                          // us
    float    flight_time;                        // s
    float    actuator_speed_0, actuator_speed_1, actuator_speed_2, actuator_speed_3;  // 0.0-1.0
    int32_t  probes_found;
    int16_t  flight_mode;
    int16_t  led_mode;
    uint8_t  arming_state;
    uint8_t  trajectory_mode;
    uint8_t  estop;
    // 1 byte tail padding possible — SimpleWriter handles sizeof >= expected
};

// DroneScope — only written in control_inspection mode
struct UlogControlScope {
    uint64_t timestamp;                          // us
    float    control_output_z;
    float    target_z;
};

// Trajectory reference point from the path planner (always logged when active)
struct UlogTrajectorySetpoint {
    uint64_t timestamp;                          // us
    float    ref_pos_x, ref_pos_y, ref_pos_z;   // m  — where we should be on the path
    float    ref_vel_x, ref_vel_y, ref_vel_z;   // m/s — feedforward velocity
    float    ref_acc_x, ref_acc_y, ref_acc_z;   // m/s^2 — feedforward acceleration
    float    ref_yaw;                            // rad
};

// Position controller internals (control_inspection only)
struct UlogControlDetail {
    uint64_t timestamp;                          // us
    float    pos_ctrl_x, pos_ctrl_y, pos_ctrl_z; // position PID output (FRD)
    float    pos_err_x,  pos_err_y,  pos_err_z;  // position error per axis
    float    pos_int_x,  pos_int_y,  pos_int_z;  // integrator state
    float    vel_cmd_x,  vel_cmd_y,  vel_cmd_z;  // combined velocity command (NED)
    float    hover_thrust_estimate;
};

// Comms health from comms_uav (always logged, 1 Hz)
struct UlogCommsHealth {
    uint64_t timestamp;                          // us
    float    rx_kbps;                            // kbit/s from GCS
    float    gcs_msg_age_ms;                     // ms since last GCS message
    uint8_t  gcs_connected;                      // 0/1
};

// ---------------------------------------------------------------------------

class LoggerNode : public rclcpp::Node {
public:
    explicit LoggerNode();
    ~LoggerNode();

private:
    void initWriter();

    static uint64_t toUs(double ros_stamp_s) {
        return static_cast<uint64_t>(ros_stamp_s * 1e6);
    }

    void onPosition(const asr_comms::msg::TelemetryPosition& msg);
    void onAttitude(const asr_comms::msg::TelemetryAttitude& msg);
    void onBattery(const asr_comms::msg::TelemetryBattery& msg);
    void onStatus(const asr_comms::msg::TelemetryStatus& msg);
    void onScope(const asr_comms::msg::DroneScope& msg);
    void onTrajectorySetpoint(const asr_comms::msg::TrajectorySetpoint& msg);
    void onControlDetail(const asr_comms::msg::ControlDetail& msg);
    void onCommsHealth(const asr_comms::msg::CommsHealth& msg);
    void onFlushTimer();

    std::unique_ptr<ulog_cpp::SimpleWriter> writer_;
    std::string log_dir_;
    std::string log_mode_;   // "general" | "control_inspection"

    uint16_t id_position_{};
    uint16_t id_attitude_{};
    uint16_t id_battery_{};
    uint16_t id_status_{};
    uint16_t id_control_scope_{};
    uint16_t id_trajectory_setpoint_{};
    uint16_t id_control_detail_{};
    uint16_t id_comms_health_{};

    rclcpp::Subscription<asr_comms::msg::TelemetryPosition>::SharedPtr position_sub_;
    rclcpp::Subscription<asr_comms::msg::TelemetryAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<asr_comms::msg::TelemetryBattery>::SharedPtr battery_sub_;
    rclcpp::Subscription<asr_comms::msg::TelemetryStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<asr_comms::msg::DroneScope>::SharedPtr drone_scope_sub_;
    rclcpp::Subscription<asr_comms::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_sub_;
    rclcpp::Subscription<asr_comms::msg::ControlDetail>::SharedPtr control_detail_sub_;
    rclcpp::Subscription<asr_comms::msg::CommsHealth>::SharedPtr comms_health_sub_;
    rclcpp::TimerBase::SharedPtr flush_timer_;
};

}  // namespace asr_logger
