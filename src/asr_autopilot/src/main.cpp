#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcutils/logging.h>
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/actuator_outputs.hpp>
#include "px4_msgs/msg/sensor_gps.hpp"
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/actuator_test.hpp>

#include <asr_comms/action/drone_command.hpp>
#include <asr_comms/msg/manual_control_input.hpp>
#include <asr_comms/msg/motion_capture_pose.hpp>
#include <asr_comms/msg/telemetry_position.hpp>
#include <asr_comms/msg/telemetry_attitude.hpp>
#include <asr_comms/msg/telemetry_battery.hpp>
#include <asr_comms/msg/telemetry_gps.hpp>
#include <asr_comms/msg/telemetry_status.hpp>
#include <asr_comms/msg/drone_scope.hpp>
#include <asr_comms/msg/gcs_heartbeat.hpp>
#include <asr_comms/msg/probe_global_locations.hpp>
#include <asr_comms/msg/attitude_setpoint_rpy.hpp>
#include <asr_comms/msg/servo_command.hpp>

#include "controller.h"
#include "state_manager.h"
#include "transformations.h"
#include "path_planner.h"



using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class AAUAutopilot : public rclcpp::Node
{
public:
    using DroneCommand = asr_comms::action::DroneCommand;
    using GoalHandleDroneCommand = rclcpp_action::ServerGoalHandle<DroneCommand>;

    ~AAUAutopilot()
    {
        if (execute_thread_.joinable()) {
            execute_thread_.join();
        }
    }

    AAUAutopilot()
    : Node("aau_autopilot_node"),
      controller_(transformations_),
      offboard_setpoint_counter_(0),
      offboard_mode_set_(false),
      timeout_threshold_(0.2)
    {
        // ROS 2 QoS settings
        // PX4 telemetry/out topics
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

        // Servo command path (in/servo_command -> /fmu/in/actuator_*)
        rclcpp::QoS servo_qos(10);
        servo_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        servo_qos.durability(rclcpp::DurabilityPolicy::Volatile);
        
         std::cout << "\n"
          << "=============================\n"
          << "    * THYRA STARTING... *\n"
          << "=============================\n"
          << std::endl;

        // Load and apply logging configuration from YAML
        this->declare_parameter("logging.enabled", true);
        this->declare_parameter("logging.console_level", "INFO");

        bool logging_enabled = this->get_parameter("logging.enabled").as_bool();
        std::string console_level = this->get_parameter("logging.console_level").as_string();

        if (logging_enabled) {
            // Map string to logger severity level
            int severity = RCUTILS_LOG_SEVERITY_INFO;
            
            if (console_level == "DEBUG") {
                severity = RCUTILS_LOG_SEVERITY_DEBUG;
            } else if (console_level == "INFO") {
                severity = RCUTILS_LOG_SEVERITY_INFO;
            } else if (console_level == "WARN") {
                severity = RCUTILS_LOG_SEVERITY_WARN;
            } else if (console_level == "ERROR") {
                severity = RCUTILS_LOG_SEVERITY_ERROR;
            } else {
                RCLCPP_WARN(this->get_logger(), 
                            "Unknown log level '%s', defaulting to INFO", 
                            console_level.c_str());
                severity = RCUTILS_LOG_SEVERITY_INFO;
            }
            
            auto ret = rcutils_logging_set_logger_level(this->get_logger().get_name(), severity);
            if (ret != RCUTILS_RET_OK) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set logger level");
            }
        } else {
            // Disable logging by setting to FATAL level (only fatal errors will show)
            auto ret = rcutils_logging_set_logger_level(this->get_logger().get_name(), 
                                                        RCUTILS_LOG_SEVERITY_FATAL);
            if (ret != RCUTILS_RET_OK) {
                RCLCPP_ERROR(this->get_logger(), "Failed to disable logging");
            }
        }
        // Determine time source at initialization - simulated or system time
        bool use_sim_time = false;
        if (!this->has_parameter("use_sim_time")) {
            this->declare_parameter("use_sim_time", false);
        }
        this->get_parameter("use_sim_time", use_sim_time);
        clock_ = std::make_shared<rclcpp::Clock>(use_sim_time ? RCL_ROS_TIME : RCL_SYSTEM_TIME);
        RCLCPP_INFO(get_logger(), "Using %s time source", use_sim_time ? "simulation" : "system");
        
        // Declare position source parameter
        std::string position_source = "px4";
        if (!this->has_parameter("position_source")) {
            this->declare_parameter("position_source", "px4");
        }
        this->get_parameter("position_source", position_source);
        RCLCPP_INFO(get_logger(), "Using %s position source", position_source.c_str());

        // Load PID gains
        PIDControllerGains pid_gains;
        load_pid_gains("pitch", pid_gains.pitch, 0.0, 0.0, 0.0);
        load_pid_gains("roll", pid_gains.roll, 0.0, 0.0, 0.0);
        load_pid_gains("yaw", pid_gains.yaw, 0.0, 0.0, 0.0);
        load_pid_gains("thrust", pid_gains.thrust, 0.0, 0.0, 0.0);

        // Load position gains
        // Position controller gains - X, Y, Z
        PIDPosControllerGains pos_pid_gains;
        this->declare_parameter("controller.position_controller.x.Kp", 1.0);
        this->declare_parameter("controller.position_controller.x.Ki", 0.0);
        this->declare_parameter("controller.position_controller.x.Kd", 0.1);
        this->get_parameter("controller.position_controller.x.Kp", pos_pid_gains.x.Kp);
        this->get_parameter("controller.position_controller.x.Ki", pos_pid_gains.x.Ki);
        this->get_parameter("controller.position_controller.x.Kd", pos_pid_gains.x.Kd);
        RCLCPP_INFO(get_logger(), "Position X PID gains: Kp=%.2f, Ki=%.2f, Kd=%.2f",
                pos_pid_gains.x.Kp, pos_pid_gains.x.Ki, pos_pid_gains.x.Kd);

        this->declare_parameter("controller.position_controller.y.Kp", 1.0);
        this->declare_parameter("controller.position_controller.y.Ki", 0.0);
        this->declare_parameter("controller.position_controller.y.Kd", 0.1);
        this->get_parameter("controller.position_controller.y.Kp", pos_pid_gains.y.Kp);
        this->get_parameter("controller.position_controller.y.Ki", pos_pid_gains.y.Ki);
        this->get_parameter("controller.position_controller.y.Kd", pos_pid_gains.y.Kd);
        RCLCPP_INFO(get_logger(), "Position Y PID gains: Kp=%.2f, Ki=%.2f, Kd=%.2f",
                pos_pid_gains.y.Kp, pos_pid_gains.y.Ki, pos_pid_gains.y.Kd);

        this->declare_parameter("controller.position_controller.z.Kp", 1.0);
        this->declare_parameter("controller.position_controller.z.Ki", 0.0);
        this->declare_parameter("controller.position_controller.z.Kd", 0.1);
        this->get_parameter("controller.position_controller.z.Kp", pos_pid_gains.z.Kp);
        this->get_parameter("controller.position_controller.z.Ki", pos_pid_gains.z.Ki);
        this->get_parameter("controller.position_controller.z.Kd", pos_pid_gains.z.Kd);
        RCLCPP_INFO(get_logger(), "Position Z PID gains: Kp=%.2f, Ki=%.2f, Kd=%.2f",
                pos_pid_gains.z.Kp, pos_pid_gains.z.Ki, pos_pid_gains.z.Kd);

        // Apply position PID gains to controller
        controller_.setPositionPIDGains(pos_pid_gains);


        this->declare_parameter("controller.ema.alpha",0.0);
        this->get_parameter("controller.ema.alpha", controller_.ema_filter_alpha_);

        RCLCPP_INFO(get_logger(), "Controller EMA filter alpha: %.2f", controller_.ema_filter_alpha_);

        // Motion constraints (used by both path planner and position controller)
        this->declare_parameter("motion_constraints.limits.min_linear_velocity",0.0);
        this->declare_parameter("motion_constraints.limits.min_angular_velocity",0.0);
        this->declare_parameter("motion_constraints.limits.max_linear_velocity",0.1);
        this->declare_parameter("motion_constraints.limits.max_angular_velocity",0.1);
        this->declare_parameter("motion_constraints.limits.max_tilt_angle", 20.0);  // degrees
        this->declare_parameter("motion_constraints.default.linear_velocity",0.05);
        this->declare_parameter("motion_constraints.default.angular_velocity",0.05);


        this->get_parameter("motion_constraints.limits.min_linear_velocity", path_planner_.min_linear_velocity_);
        this->get_parameter("motion_constraints.limits.min_angular_velocity", path_planner_.min_angular_velocity_);
        this->get_parameter("motion_constraints.limits.max_linear_velocity", path_planner_.max_linear_velocity_);
        this->get_parameter("motion_constraints.limits.max_angular_velocity", path_planner_.max_angular_velocity_);
        this->get_parameter("motion_constraints.default.linear_velocity", path_planner_.current_linear_velocity_);
        this->get_parameter("motion_constraints.default.angular_velocity", path_planner_.current_angular_velocity_);

        // Get max tilt angle in degrees and convert to radians
        double max_tilt_angle_deg = 20.0;
        this->get_parameter("motion_constraints.limits.max_tilt_angle", max_tilt_angle_deg);
        controller_.max_tilt_angle_ = max_tilt_angle_deg * M_PI / 180.0;

        // Use motion constraints for controller velocity saturation
        controller_.max_horizontal_velocity_ = path_planner_.max_linear_velocity_;
        controller_.max_vertical_velocity_ = path_planner_.max_linear_velocity_;

        RCLCPP_INFO(get_logger(),
            "Motion constraints (path planner & controller):\n"
            "  min_linear_velocity     = %.2f m/s\n"
            "  min_angular_velocity    = %.2f rad/s\n"
            "  max_linear_velocity     = %.2f m/s\n"
            "  max_angular_velocity    = %.2f rad/s\n"
            "  max_tilt_angle          = %.1f degrees (%.3f rad)\n"
            "  default_linear_velocity = %.2f m/s\n"
            "  default_angular_velocity= %.2f rad/s",
            path_planner_.min_linear_velocity_,
            path_planner_.min_angular_velocity_,
            path_planner_.max_linear_velocity_,
            path_planner_.max_angular_velocity_,
            max_tilt_angle_deg,
            controller_.max_tilt_angle_,
            path_planner_.current_linear_velocity_,
            path_planner_.current_angular_velocity_);
        
        RCLCPP_INFO(get_logger(),
            "Controller velocity saturation:\n"
            "  max_horizontal_velocity = %.2f m/s\n"
            "  max_vertical_velocity   = %.2f m/s",
            controller_.max_horizontal_velocity_,
            controller_.max_vertical_velocity_);


        // Set PID gains in controller
        controller_.setPIDGains(pid_gains);

        // Load safety parameters
        this->declare_parameter("safety.check_gcs_timeout", true);
        this->declare_parameter("safety.check_position_timeout", true);
        this->declare_parameter("safety.gcs_timeout_threshold", 0.2);
        this->declare_parameter("safety.position_timeout_threshold", 0.2);
        this->declare_parameter("safety.safety_thrust_initial", -0.8);
        this->declare_parameter("safety.safety_thrust_final", -0.45);
        this->declare_parameter("safety.safety_thrustdown_rate", 0.0005);
        this->declare_parameter("safety.check_battery", true);
        this->declare_parameter("safety.check_geofence", true);
        this->declare_parameter("safety.geofence_radius", 2.0);
        this->declare_parameter("safety.battery_threshold", 0.10); // 5% battery threshold
        this->declare_parameter("safety.geofence_height", 2.0); // Geofence height (meters)
        this->declare_parameter("safety.do_mode_change_delay", false);
        this->declare_parameter("safety.mode_change_delay", 5.0);

        this->get_parameter("safety.check_gcs_timeout", check_gcs_timeout_);
        this->get_parameter("safety.check_position_timeout", check_position_timeout_);
        this->get_parameter("safety.gcs_timeout_threshold", gcs_timeout_threshold_);
        this->get_parameter("safety.position_timeout_threshold", position_timeout_threshold_);
        this->get_parameter("safety.safety_thrust_initial", safety_thrust_initial_);
        this->get_parameter("safety.safety_thrust_final", safety_thrust_final_);
        this->get_parameter("safety.safety_thrustdown_rate", safety_thrustdown_rate_);
        this->get_parameter("safety.check_battery", safety_check_battery_);
        this->get_parameter("safety.battery_threshold", safety_battery_threshold_);
        this->get_parameter("safety.check_geofence", safety_check_geofence_);
        this->get_parameter("safety.geofence_radius", safety_geofence_radius_);
        this->get_parameter("safety.geofence_height", safety_geofence_height_);
        this->get_parameter("safety.do_mode_change_delay", do_mode_change_delay_);
        this->get_parameter("safety.mode_change_delay", mode_change_delay_);

        RCLCPP_INFO(get_logger(),
            "Safety parameters:\n"
            "  gcs_timeout            = %.2f s\n"
            "  position_timeout       = %.2f s\n"
            "  safety_thrust_initial  = %.2f\n"
            "  safety_thrust_final    = %.2f\n"
            "  safety_thrustdown_rate = %.4f\n"
            "  check_battery          = %s\n"
            "  battery_threshold      = %.2f\n"
            "  check_geofence         = %s\n"
            "  geofence_radius        = %.2f m\n"
            "  geofence_height        = %.2f m\n"
            "  do_mode_change_delay   = %s\n"
            "  mode_change_delay      = %.2f s",
            gcs_timeout_threshold_,
            position_timeout_threshold_,
            safety_thrust_initial_,
            safety_thrust_final_,
            safety_thrustdown_rate_,
            safety_check_battery_ ? "true" : "false",
            safety_battery_threshold_,
            safety_check_geofence_ ? "true" : "false",
            safety_geofence_radius_,
            safety_geofence_height_,
            do_mode_change_delay_ ? "true" : "false",
            mode_change_delay_);

        // Load setup parameters
        this->declare_parameter("setup.lidar_offset", 0.0);
        this->get_parameter("setup.lidar_offset", lidar_offset_);
        this->declare_parameter("setup.takeoff_height_", -0.5);
        this->get_parameter("setup.takeoff_height_", takeoff_height_);
        this->declare_parameter("setup.motor_speed_min", 1000.0);
        this->get_parameter("setup.motor_speed_min", motor_speed_min_);
        this->declare_parameter("setup.motor_speed_max", 1900.0);
        this->get_parameter("setup.motor_speed_max", motor_speed_max_);

        RCLCPP_INFO(get_logger(), "Lidar offset: %.2f", lidar_offset_);
        RCLCPP_INFO(get_logger(), "Takeoff height: %.2f", takeoff_height_);
        RCLCPP_INFO(get_logger(), "Motor speed min: %.2f", motor_speed_min_);
        RCLCPP_INFO(get_logger(), "Motor speed max: %.2f", motor_speed_max_);

        // hover_thrust_estimate_
        this->declare_parameter("controller.pid_gains.hover_thrust_setpoint.base_value", -0.5);
        this->declare_parameter("controller.pid_gains.hover_thrust_setpoint.learning_rate", 0.0001);
        this->get_parameter("controller.pid_gains.hover_thrust_setpoint.base_value", controller_.hover_thrust_estimate_);
        this->get_parameter("controller.pid_gains.hover_thrust_setpoint.learning_rate", controller_.hover_learning_rate_);

        RCLCPP_INFO(get_logger(), "Hover thrust estimate: %.3f", controller_.hover_thrust_estimate_);
        RCLCPP_INFO(get_logger(), "Hover thrust learning rate: %.6f", controller_.hover_learning_rate_);

        // Set initial state
        state_manager_.setHeartbeat(GCSHeartbeat(get_time(),0));
        state_manager_.setGlobalPosition(Stamped3DVector(get_time(), 0.0, 0.0, 0.0));
        state_manager_.setOrigin(Stamped3DVector(get_time(), 0.0, 0.0, 0.0));
        state_manager_.setGlobalVelocity(Stamped3DVector(get_time(), 0.0, 0.0, 0.0));
        state_manager_.setTargetPositionProfile(Stamped4DVector(get_time(), 0.0, 0.0, 0.0, 0.0));
        state_manager_.setAttitude(StampedQuaternion(get_time(), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)));
        state_manager_.setManualControlInput(Stamped4DVector(get_time(), 0.0, 0.0, 0.0, 0.0));
        state_manager_.setGroundDistanceState(Stamped3DVector(get_time(), 0.0, 0.0, 0.0));
        timestamp_last_position_control_ = get_time();


        
        // Publishers
        offboard_control_mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        attitude_setpoint_pub_ = create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
        attitude_setpoint_rpy_thrust_pub_ = create_publisher<asr_comms::msg::AttitudeSetpointRPY>("/fmu/in/attitude_setpoint_rpy_thrust", 10);
        vehicle_command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        telemetry_position_pub_ = create_publisher<asr_comms::msg::TelemetryPosition>("out/telemetry/position", 10);
        telemetry_attitude_pub_ = create_publisher<asr_comms::msg::TelemetryAttitude>("out/telemetry/attitude", 10);
        telemetry_battery_pub_  = create_publisher<asr_comms::msg::TelemetryBattery>( "out/telemetry/battery",  10);
        telemetry_gps_pub_      = create_publisher<asr_comms::msg::TelemetryGPS>(     "out/telemetry/gps",      10);
        telemetry_status_pub_   = create_publisher<asr_comms::msg::TelemetryStatus>(  "out/telemetry/status",   10);
        origin_offset_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("out/origin_offset", 10);
        actuator_servos_pub_ = create_publisher<px4_msgs::msg::ActuatorServos>("/fmu/in/actuator_servos", servo_qos);
        actuator_test_pub_ = create_publisher<px4_msgs::msg::ActuatorTest>("/fmu/in/actuator_test", servo_qos);
        
        

        // Subscribers
        if (position_source == "px4"){
            local_position_sub_ = create_subscription<VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position", qos,
                [this](const VehicleLocalPosition::SharedPtr msg) { localPositionCallback(msg); });
        }
        else if (position_source == "mocap"){
            motion_capture_local_position_sub_ = create_subscription<asr_comms::msg::MotionCapturePose>(
                "in/motion_capture_pose", qos,
                [this](const asr_comms::msg::MotionCapturePose::SharedPtr msg)
                { motionCaptureLocalPositionCallback(msg); });
        }

        gcs_heartbeat_sub_ = create_subscription<asr_comms::msg::GcsHeartbeat>(
            "in/gcs_heartbeat", qos,
            [this](const asr_comms::msg::GcsHeartbeat::SharedPtr msg)
            { gcsHeartbeatCallback(msg); });

        attitude_sub_ = create_subscription<VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            [this](const VehicleAttitude::SharedPtr msg)
            { attitudeCallback(msg); });
        status_sub_ = create_subscription<VehicleStatus>(
            "/fmu/out/vehicle_status", qos,
            [this](const VehicleStatus::SharedPtr msg)
            { vehicleStatusCallback(msg); });
        manual_input_sub_ = create_subscription<asr_comms::msg::ManualControlInput>(
            "in/manual_input", qos,
            [this](const asr_comms::msg::ManualControlInput::SharedPtr msg)
            { manualControlInputCallback(msg); });
        battery_status_sub_ = create_subscription<BatteryStatus>(
            "/fmu/out/battery_status", qos,
            [this](const BatteryStatus::SharedPtr msg)
            { batteryStatusCallback(msg); });
        ground_distance_sub_ = create_subscription<DistanceSensor>(
            "out/distance_sensor", qos,
            [this](const DistanceSensor::SharedPtr msg)
            { GroundDistanceCallback(msg);});
        actuator_output_sub_ = create_subscription<ActuatorOutputs>(
            "/fmu/out/actuator_outputs", qos,
            [this](const ActuatorOutputs::SharedPtr msg)
            { ActuatorOutputCallback(msg);});
        probe_global_locations_sub_ = create_subscription<asr_comms::msg::ProbeGlobalLocations>(
            "/probe_detector/global_probe_locations", qos,
            [this](const asr_comms::msg::ProbeGlobalLocations::SharedPtr msg)
            { GlobalProbeLocationsCallback(msg); });
        gps_sub_ = create_subscription<SensorGps>(
            "/fmu/out/vehicle_gps_position", qos,
            [this](const SensorGps::SharedPtr msg)
            { gpsCallback(msg); });
        servo_command_sub_ = create_subscription<asr_comms::msg::ServoCommand>("in/servo_command", servo_qos,
            [this](const asr_comms::msg::ServoCommand::SharedPtr msg) 
            {setServo(msg->aux_index, msg->id, msg->value);});

        //Action server
        drone_command_server_ = rclcpp_action::create_server<DroneCommand>(
            this, "in/drone_command",
            [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const DroneCommand::Goal> goal)
            {
                return handleDroneCommand(uuid, goal);
            },
            [this](const std::shared_ptr<GoalHandleDroneCommand> goal_handle)
            {
                return handleCancel(goal_handle);
            },
            [this](const std::shared_ptr<GoalHandleDroneCommand> goal_handle)
            {
                handleAccepted(goal_handle);
            });

        // Timers
        offboard_timer_ = create_wall_timer(200ms, [this](){ setOffboardMode(); });
        drone_state_timer = create_wall_timer(100ms, [this](){ publish_drone_state(); });
        safety_timer_ = create_wall_timer(200ms, [this]() { safetyCheckCallback(); });
        offset_timer = create_wall_timer(1000ms, [this]() { publish_origin_offset(); });
        servo_hold_timer_ = create_wall_timer(100ms, [this]() { publishHeldDisarmedServoCommand(); });
        gimbal_timeout_timer_ = this->create_wall_timer(2000ms, [this]() {RCLCPP_WARN(get_logger(),
             "Gimbal manual control timeout - switching to auto");
                gimbal_is_manual_ = false;
                gimbal_timeout_timer_->cancel(); });
        gimbal_timeout_timer_->cancel(); 

        std::cout << "\n"
          << "=============================\n"
          << "    * THYRA OPERATIONAL *\n"
          << "=============================\n"
          << std::endl;
        
        //Set origin
        Stamped3DVector current_position = state_manager_.getGlobalPosition();
        state_manager_.setOrigin(current_position);
    }

    rclcpp::Time get_time() const {
        return clock_->now();
    }

private:

    void load_pid_gains(const std::string& controller, PIDGains& gains, 
        double kp_default, double ki_default, double kd_default)
    {
    // Declare parameters
    this->declare_parameter("controller.pid_gains." + controller + ".Kp", kp_default);
    this->declare_parameter("controller.pid_gains." + controller + ".Ki", ki_default);
    this->declare_parameter("controller.pid_gains." + controller + ".Kd", kd_default);

    // Get parameters
    this->get_parameter("controller.pid_gains." + controller + ".Kp", gains.Kp);
    this->get_parameter("controller.pid_gains." + controller + ".Ki", gains.Ki);
    this->get_parameter("controller.pid_gains." + controller + ".Kd", gains.Kd);

    // Log gains
    RCLCPP_INFO(get_logger(), "%s PID gains: Kp=%.2f, Ki=%.2f, Kd=%.2f",
    controller.c_str(), gains.Kp, gains.Ki, gains.Kd);
    }

    void GroundDistanceCallback(const DistanceSensor::SharedPtr msg)
    {
        // Retrieve measurement
        double ground_distance = msg->current_distance - lidar_offset_;

        // Shift old values to make space for the newest one
        for (int i = 0; i < max_ground_distance_readings_ - 1; ++i) {
            ground_distance_readings_[i] = ground_distance_readings_[i + 1];
            ground_distance_timestamps_[i] = ground_distance_timestamps_[i + 1];
        }

        // Add newest reading and timestamp at the end
        ground_distance_readings_[max_ground_distance_readings_ - 1] = static_cast<float>(ground_distance);
        ground_distance_timestamps_[max_ground_distance_readings_ - 1] = get_time();

        // Track number of valid readings
        static int valid_readings_ = 0;
        if (valid_readings_ < max_ground_distance_readings_) {
            valid_readings_++;
        }

        // --- Calculate average ---
        float sum = 0.0f;
        int valid_count = std::min(valid_readings_, max_ground_distance_readings_);
        for (int i = max_ground_distance_readings_ - valid_count; i < max_ground_distance_readings_; ++i) {
            sum += ground_distance_readings_[i];
        }
        float avg_ground_distance = valid_count > 0 ? sum / static_cast<float>(valid_count) : 0.0f;

        // --- Calculate velocity ---
        float ground_distance_velocity = 0.0f;
        double dt_sec_vel = 0.0;
        bool valid_velocity = false;
        if (valid_readings_ >= 2) {
            rclcpp::Duration dt_vel = ground_distance_timestamps_[max_ground_distance_readings_ - 1] -
                                    ground_distance_timestamps_[max_ground_distance_readings_ - 2];
            dt_sec_vel = dt_vel.seconds();
            if (dt_sec_vel > 0.0 &&
                ground_distance_timestamps_[max_ground_distance_readings_ - 1].get_clock_type() ==
                ground_distance_timestamps_[max_ground_distance_readings_ - 2].get_clock_type()) {
                ground_distance_velocity = (ground_distance_readings_[max_ground_distance_readings_ - 1] -
                                        ground_distance_readings_[max_ground_distance_readings_ - 2]) / dt_sec_vel;
                valid_velocity = true;
            } else {
                RCLCPP_WARN(get_logger(), "Invalid time difference for velocity calculation: dt=%.6f, same_source=%d",
                            dt_sec_vel, ground_distance_timestamps_[max_ground_distance_readings_ - 1].get_clock_type() ==
                            ground_distance_timestamps_[max_ground_distance_readings_ - 2].get_clock_type());
            }
        }

        // --- Calculate acceleration ---
        float ground_distance_acceleration = 0.0f;
        if (valid_readings_ >= 3 && valid_velocity) {
            rclcpp::Duration dt_acc = ground_distance_timestamps_[max_ground_distance_readings_ - 2] -
                                    ground_distance_timestamps_[max_ground_distance_readings_ - 3];
            double dt_sec_acc = dt_acc.seconds();
            if (dt_sec_acc > 0.0 &&
                ground_distance_timestamps_[max_ground_distance_readings_ - 2].get_clock_type() ==
                ground_distance_timestamps_[max_ground_distance_readings_ - 3].get_clock_type()) {
                float prev_velocity = (ground_distance_readings_[max_ground_distance_readings_ - 2] -
                                    ground_distance_readings_[max_ground_distance_readings_ - 3]) / dt_sec_acc;
                ground_distance_acceleration = (ground_distance_velocity - prev_velocity) / dt_sec_vel;
            } else {
                RCLCPP_WARN(get_logger(), "Invalid time difference for acceleration calculation: dt_acc=%.6f, same_source=%d",
                            dt_sec_acc, ground_distance_timestamps_[max_ground_distance_readings_ - 2].get_clock_type() ==
                            ground_distance_timestamps_[max_ground_distance_readings_ - 3].get_clock_type());
            }
        }

        // Set the ground distance state in the state manager
        Stamped3DVector ground_distance_state(get_time(), avg_ground_distance, ground_distance_velocity, ground_distance_acceleration);
        state_manager_.setGroundDistanceState(ground_distance_state);
    }

    void gcsHeartbeatCallback(const asr_comms::msg::GcsHeartbeat::SharedPtr msg)
    {
        // Use the arrival time of the GCS heartbeat message as the heartbeat time
        state_manager_.setHeartbeat(GCSHeartbeat(get_time(), msg->gcs_nominal));
    }

    bool geofence_violated(Eigen::Vector3d position)
    {
        // Return false if geofence check is disabled - no violation
        if (!safety_check_geofence_){
            return false;
        }
        Eigen::Vector2d xy_position(position.x(),
        position.y());
        float z_position = position.z();
        float xy_distance = xy_position.norm();
        float z_distance = std::abs(z_position);

        // Check if outside geofence
        return (xy_distance >= safety_geofence_radius_ || z_distance >= safety_geofence_height_);
    }
    
    void safetyCheckCallback()
    {
        DroneState drone_state = state_manager_.getDroneState();

        // Skip checks if drone is disarmed or already in a fail-safe mode
        if (drone_state.arming_state != ArmingState::ARMED ||
            drone_state.flight_mode == FlightMode::SAFETYLAND_BLIND ||
            drone_state.flight_mode == FlightMode::LANDED)
        {
            return;
        }

        bool trigger_emergency_land = false;
        bool trigger_position_land = false;
        bool trigger_blind_land = false;
        bool landing_position_mode = drone_state.flight_mode == FlightMode::LAND_POSITION;

        // Check battery status if enabled
        if (safety_check_battery_ && !landing_position_mode)
        {
            // Get the latest battery state
            BatteryState battery_state = state_manager_.getBatteryState();
            if (battery_state.charge_remaining < safety_battery_threshold_)
            {
                RCLCPP_WARN(get_logger(), "Battery charge low (%.2f < %.2f), triggering emergency land",
                            battery_state.charge_remaining, safety_battery_threshold_);
                trigger_emergency_land = true;
            }
        }

        // Check GCS input freshness
        if (check_gcs_timeout_ && !landing_position_mode)
        {
            gcs_stale_ = (get_time() - state_manager_.getHeartbeat().timestamp).seconds() > gcs_timeout_threshold_;
            if (gcs_stale_)
            {
                RCLCPP_WARN(get_logger(), "No GCS input received in the last %.2f seconds!", gcs_timeout_threshold_);
            }
        }

        // Check position data freshness
        if (check_position_timeout_)
        {
            position_stale_ = (get_time() - state_manager_.getGlobalPosition().getTime()).seconds() > position_timeout_threshold_;
            if (position_stale_)
            {
                RCLCPP_WARN(get_logger(), "No position data received in the last %.2f seconds!", position_timeout_threshold_);
            }
        }

        // Check Geofence violations
        if (safety_check_geofence_)
        {
            if (geofence_violated(state_manager_.getGlobalPosition().vector()))
            {
                RCLCPP_WARN(get_logger(), "Geofence violation detected!");
                trigger_emergency_land = true;
            }
        }

        // Determine fail-safe action
        if (position_stale_) {
            RCLCPP_WARN(get_logger(), "Entering safety land mode due to stale data (GCS: %s, Position: %s)",
                gcs_stale_ ? "stale" : "fresh", position_stale_ ? "stale" : "fresh");
            trigger_blind_land = true;
        } else if ((gcs_stale_ || trigger_emergency_land) && !landing_position_mode) {
            RCLCPP_WARN(get_logger(), "Entering safety land mode due to stale GCS input or low battery (charge_remaining=%.2f < threshold=%.2f)",
                        state_manager_.getBatteryState().charge_remaining, safety_battery_threshold_);
            trigger_position_land = true;
        }  

        // Apply fail-safe
        if (trigger_position_land && !landing_position_mode)
            {
                setDroneMode(FlightMode::BEGIN_LAND_POSITION);
                ensureControlLoopRunning(2);
                landPositionMode();
            }
        else if (trigger_blind_land)
        {
            setDroneMode(FlightMode::SAFETYLAND_BLIND);
            ensureControlLoopRunning(3); // Switch to safetyLandBlindMode
        }
    }

    void localPositionCallback(const VehicleLocalPosition::SharedPtr msg)
    {
        // Note that local position refers to coordinates being expressed in cartesian coordinates from some origin point.
        Stamped3DVector origin = state_manager_.getOrigin();
        Stamped3DVector local_position(get_time(), msg->x - origin.x(), msg->y - origin.y(), msg->z - origin.z());
        state_manager_.setGlobalPosition(local_position);

        // Set the velocity in the state manager
        Stamped3DVector local_velocity(get_time(), msg->vx, msg->vy, msg->vz);
        state_manager_.setGlobalVelocity(local_velocity);

        // set the acceleration in the state manager
        Stamped3DVector local_acceleration(get_time(), msg->ax, msg->ay, msg->az);
        state_manager_.setGlobalAcceleration(local_acceleration);
    }

    void motionCaptureLocalPositionCallback(const asr_comms::msg::MotionCapturePose::SharedPtr msg)
    {
        Stamped3DVector origin = state_manager_.getOrigin();
        Stamped3DVector local_position(get_time(), msg->x - origin.x(), msg->y - origin.y(), msg->z- origin.z());

        state_manager_.setGlobalPosition(local_position);
    }

    void batteryStatusCallback(const BatteryStatus::SharedPtr msg)
    {
        BatteryState battery_state;
        battery_state.timestamp = get_time();
        battery_state.cell_count = msg->cell_count;
        battery_state.voltage = msg->voltage_v;
        battery_state.charge_remaining = msg->remaining;
        battery_state.discharged_mah = msg->discharged_mah;
        battery_state.average_current = msg->current_a;
        
        state_manager_.setBatteryState(battery_state);
    }

    void attitudeCallback(const VehicleAttitude::SharedPtr msg)
    {
        StampedQuaternion attitude(get_time(), Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]));
        state_manager_.setAttitude(attitude);
    }

    void vehicleStatusCallback(const VehicleStatus::SharedPtr msg)
    {
        DroneState drone_state = state_manager_.getDroneState();
        
        bool arming = (drone_state.arming_state == ArmingState::DISARMED && msg->arming_state == 2);
        bool disarming = (drone_state.arming_state == ArmingState::ARMED && msg->arming_state != 2);
        // Reset the control loop if the arming state changes. Only if the drone is asked to be armed, while armed, do nothing.
        if ((arming || disarming))
        {
            if (disarming && !(drone_state.flight_mode == FlightMode::EMERGENCY_STOP))
            {
                drone_state.flight_mode = FlightMode::STANDBY;
                drone_state.flight_mode_trait = getFlightModeTraits(drone_state.flight_mode)[0];
            }
            else if (arming && (drone_state.flight_mode == FlightMode::EMERGENCY_STOP))
            {
                drone_state.flight_mode = FlightMode::STANDBY;
                drone_state.flight_mode_trait = getFlightModeTraits(drone_state.flight_mode)[0];
            }
            cleanupControlLoop();
        }

        // Set the new drone state
        drone_state.timestamp = get_time();
        drone_state.arming_state = (msg->arming_state == 2) ? ArmingState::ARMED : ArmingState::DISARMED;
        state_manager_.setDroneState(drone_state);
    }

    void manualControlInputCallback(const asr_comms::msg::ManualControlInput::SharedPtr msg)
    {
        Stamped4DVector manual_input_sub_ = state_manager_.getManualControlInput();
        manual_input_sub_.setTime(get_time());
        manual_input_sub_.setX(controller_.mapNormToAngle(msg->roll));
        manual_input_sub_.setY(controller_.mapNormToAngle(msg->pitch));
        manual_input_sub_.setZ(manual_input_sub_.z() + controller_.mapNormToAngle(msg->yaw_velocity * yaw_sensitivity_));
        manual_input_sub_.setW(msg->thrust);
        state_manager_.setManualControlInput(manual_input_sub_);
    }

    void ActuatorOutputCallback(const ActuatorOutputs::SharedPtr msg)
    {
        // Set the actuator speeds in the state manager
        float motor_speed_range = motor_speed_max_ - motor_speed_min_;
        float output[4];
        for (int i = 0; i < 4; ++i)
        {
            output[i] = (msg->output[i] - motor_speed_min_) / motor_speed_range;
            output[i] = std::clamp(output[i], 0.0f, 1.0f);
        }

        Stamped4DVector actuator_speeds(get_time(), output[0], output[1],output[2], output[3]);
        state_manager_.setActuatorSpeeds(actuator_speeds);
    }

    void GlobalProbeLocationsCallback(const asr_comms::msg::ProbeGlobalLocations::SharedPtr msg)
    {
        // Set the probe locations in the state manager
        GlobalProbeLocations probe_locations;
        probe_locations.stamp = get_time();
        probe_locations.probe_count = msg->probe_count;
        probe_locations.x = msg->x;
        probe_locations.y = msg->y;
        probe_locations.z = msg->z;
        probe_locations.confidence = msg->confidence;
        probe_locations.contribution = msg->contribution;
        state_manager_.setGlobalProbeLocations(probe_locations);
    }

    double unwrapYaw(double yaw) {
        while (yaw > M_PI) yaw -= 2.0 * M_PI;
        while (yaw < -M_PI) yaw += 2.0 * M_PI;
        return yaw;
    }
    void gpsCallback(const SensorGps::SharedPtr msg)
    {
        GPSState gps_state;
        gps_state.latitude = msg->latitude_deg;
        gps_state.longitude = msg->longitude_deg;
        gps_state.satellites_used = msg->satellites_used;
        state_manager_.setGPSState(gps_state);

    }

    // Publish the origin offset
    void publish_origin_offset()
    {
        geometry_msgs::msg::PoseStamped msg{};
        msg.header.stamp = get_time();
        msg.header.frame_id = "px4_offset";
        Stamped3DVector Current_origin = state_manager_.getOrigin();
        msg.pose.position.x = Current_origin.x();
        msg.pose.position.y = Current_origin.y();
        msg.pose.position.z = Current_origin.z();
        origin_offset_pub_->publish(msg);
    }
    
    inline double wrapToPi(double angle) {
        angle = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0)
            angle += 2.0 * M_PI;
        angle -= M_PI;
        return angle * 180.0 / M_PI; // Convert to degrees
    }


    // Drone state publisher
    void publish_drone_state()
    {
        const DroneState&        drone_state    = state_manager_.getDroneState();
        const Stamped3DVector&   position       = state_manager_.getGlobalPosition();
        const Stamped3DVector&   velocity       = state_manager_.getGlobalVelocity();
        const StampedQuaternion& attitude       = state_manager_.getAttitude();
        const Stamped4DVector&   target_profile = state_manager_.getTargetPositionProfile();
        const auto&              battery        = state_manager_.getBatteryState();
        const Stamped4DVector&   actuators      = state_manager_.getActuatorSpeeds();
        const auto&              gps            = state_manager_.getGPSState();
        const Eigen::Vector3d    euler          = transformations_.quaternionToEuler(attitude.quaternion());

        {
            asr_comms::msg::TelemetryPosition msg{};
            msg.timestamp        = position.timestamp.seconds();
            msg.position         = {static_cast<float>(position.x()),
                                    static_cast<float>(position.y()),
                                    static_cast<float>(position.z())};
            msg.velocity         = {static_cast<float>(velocity.x()),
                                    static_cast<float>(velocity.y()),
                                    static_cast<float>(velocity.z())};
            msg.target_position  = {static_cast<float>(target_profile.x()),
                                    static_cast<float>(target_profile.y()),
                                    static_cast<float>(target_profile.z())};
            telemetry_position_pub_->publish(msg);
        }

        {
            asr_comms::msg::TelemetryAttitude msg{};
            msg.timestamp   = attitude.timestamp.seconds();
            msg.orientation = {static_cast<float>(euler.x()),   // roll
                               static_cast<float>(euler.y()),   // pitch
                               static_cast<float>(euler.z())};  // yaw
            telemetry_attitude_pub_->publish(msg);
        }

        {
            asr_comms::msg::TelemetryBattery msg{};
            msg.timestamp       = battery.timestamp.seconds();
            msg.voltage         = battery.voltage;
            msg.current         = battery.average_current;
            msg.percentage      = battery.charge_remaining;
            msg.discharged_mah  = battery.discharged_mah;
            msg.average_current = battery.average_current;
            telemetry_battery_pub_->publish(msg);
        }

        {
            asr_comms::msg::TelemetryGPS msg{};
            msg.timestamp       = get_clock()->now().seconds();
            msg.latitude        = gps.latitude;
            msg.longitude       = gps.longitude;
            msg.satellites_used = gps.satellites_used;
            telemetry_gps_pub_->publish(msg);
        }

        {
            asr_comms::msg::TelemetryStatus msg{};
            msg.timestamp       = get_clock()->now().seconds();
            msg.arming_state    = static_cast<uint8_t>(drone_state.arming_state);
            msg.trajectory_mode = static_cast<uint8_t>(drone_state.trajectory_mode);
            msg.flight_mode     = static_cast<int16_t>(drone_state.flight_mode);
            msg.led_mode        = static_cast<int16_t>(drone_state.flight_mode_trait);
            msg.flight_time     = drone_state.flight_time.seconds();
            msg.probes_found    = state_manager_.getGlobalProbeLocations().getProbeCount();
            msg.actuator_speeds = {static_cast<float>(actuators.x()),
                                   static_cast<float>(actuators.y()),
                                   static_cast<float>(actuators.z()),
                                   static_cast<float>(actuators.w())};
            telemetry_status_pub_->publish(msg);
        }
    }
    // Offboard mode handling
    void setOffboardMode()
    {
        publishOffboardControlMode();
        if (!offboard_mode_set_ && offboard_setpoint_counter_ < 11)
        {
            if (offboard_setpoint_counter_ == 10)
            {
                publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                offboard_mode_set_ = true;
            }
            offboard_setpoint_counter_++;
        }
    }

    void publishOffboardControlMode()
    {
        OffboardControlMode msg{};
        msg.position = false;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = true;
        msg.body_rate = false;
        msg.timestamp = get_time().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(msg);
    }

    // Control modes
    Eigen::Vector4d manualMode()
    {
        Stamped4DVector manual_input_sub_ = state_manager_.getManualControlInput();
        
        return {manual_input_sub_.x(), manual_input_sub_.y(), manual_input_sub_.z(), manual_input_sub_.w()};
    }

    Eigen::Vector4d positionAndVelocityControl()
    {
        DroneState drone_state = state_manager_.getDroneState();

        // Sample trajectory if active, else if landed, disarm
        if (drone_state.trajectory_mode == TrajectoryMode::ACTIVE) {
            double dt = (get_time() - drone_state.trajectory_start_time_).seconds();
            if (dt > path_planner_.getTotalTime()) {
                drone_state.trajectory_mode = TrajectoryMode::COMPLETED;
                state_manager_.setDroneState(drone_state);
                FullTrajectoryPoint final_point = path_planner_.getTrajectoryPoint(path_planner_.getTotalTime(), trajectoryMethod::MIN_SNAP);
                Stamped4DVector target_profile(get_time(), final_point.position.x(), final_point.position.y(), final_point.position.z(), 0.0);
                Stamped3DVector target_velocity_profile(get_time(), final_point.velocity.x(), final_point.velocity.y(), final_point.velocity.z());
                state_manager_.setTargetPositionProfile(target_profile);
              
                state_manager_.setTargetAttitude(StampedQuaternion(get_time(), final_point.orientation));
            }
            else { // If trajectory is still active
                FullTrajectoryPoint target_point = path_planner_.getTrajectoryPoint(dt, trajectoryMethod::MIN_SNAP);
                Stamped4DVector target_profile(get_time(), target_point.position.x(), target_point.position.y(), target_point.position.z(), 0.0);
                Stamped3DVector target_velocity_profile(get_time(), target_point.velocity.x(), target_point.velocity.y(), target_point.velocity.z());
                state_manager_.setTargetPositionProfile(target_profile);
                
                state_manager_.setTargetAttitude(StampedQuaternion(get_time(), target_point.orientation));
            }
        }
        else if (drone_state.flight_mode == FlightMode::LAND_POSITION && drone_state.trajectory_mode == TrajectoryMode::COMPLETED) {
            // Drone is now landed, update state and disarm
            RCLCPP_INFO(get_logger(), "Drone has landed, disarming...");
            drone_state.flight_mode = FlightMode::LANDED;
            drone_state.flight_mode_trait = getFlightModeTraits(drone_state.flight_mode)[0];
            drone_state.arming_state = ArmingState::DISARMED;
            drone_state.command = Command::DISARM;
            drone_state.trajectory_mode = TrajectoryMode::COMPLETED;
            state_manager_.setDroneState(drone_state);
            disarm(true);
            cleanupControlLoop();
        }
        // Get target profiles
        Stamped4DVector target_profile = state_manager_.getTargetPositionProfile();

        Stamped3DVector target_velocity_profile = state_manager_.getTargetVelocityProfile();
        Stamped3DVector target_position_3d(target_profile.timestamp, target_profile.vector().x(), target_profile.vector().y(), target_profile.vector().z());
        
        // Get current states
        Stamped3DVector position = state_manager_.getGlobalPosition();
        Stamped3DVector velocity = state_manager_.getGlobalVelocity();
        StampedQuaternion attitude = state_manager_.getAttitude();
        
        double dt = (get_time() - position.getTime()).seconds(); //

        if (dt > timeout_threshold_) {
            RCLCPP_WARN(get_logger(), "No position or velocity data received!");
            return Eigen::Vector4d::Zero();
        }

        // Keep trajectory velocity in NED frame
        Eigen::Vector3d vel_cmd_ned = target_velocity_profile.vector();  // NED frame

        // Position control loop
        if (position_loop_counter_ >= position_loop_trigger_) {
            // Position controller outputs velocity in FRD frame
            Eigen::Vector3d pos_control_output_frd = controller_.positionControl(
                dt, 
                prev_position_error_, 
                position, 
                attitude, 
                target_position_3d, 
                state_manager_.getLatestControlSignalPosition()
            );
            
            // Transform position control output from FRD to NED
            Eigen::Vector3d pos_control_output_ned = attitude.quaternion() * pos_control_output_frd;
            
            // Add in NED frame
            vel_cmd_ned += pos_control_output_ned;
            
            position_loop_counter_ = 0;
            state_manager_.setLatestControlSignalPosition(pos_control_output_frd);
        }
        else {
            // Transform previous output from FRD to NED
            Eigen::Vector3d prev_output_frd = state_manager_.getLatestControlSignalPosition();
            Eigen::Vector3d prev_output_ned = attitude.quaternion() * prev_output_frd;
            vel_cmd_ned += prev_output_ned;
        }
        position_loop_counter_++;

        // Pass NED velocity to velocity controller
        Stamped3DVector target_velocity_3d(get_time(), vel_cmd_ned.x(), vel_cmd_ned.y(), vel_cmd_ned.z());
        
        // Velocity controller handles NED input correctly
        Eigen::Vector4d output = controller_.velocityControl(
            dt, 
            prev_velocity_error_, 
            velocity, 
            attitude, 
            target_velocity_3d, 
            state_manager_.getLatestControlSignalVelocity()
        );

        state_manager_.setLatestControlSignalVelocity(output);

        // Set yaw from trajectory
        Eigen::Vector3d target_euler = transformations_.quaternionToEuler(
            state_manager_.getTargetAttitude().quaternion()
        );
        output.z() = target_euler.x();

        return output;
    }


    Eigen::Vector4d manualAidedMode()
    {
        Stamped4DVector manual_input = state_manager_.getManualControlInput();
        Stamped3DVector velocity = state_manager_.getGlobalVelocity();
        StampedQuaternion attitude = state_manager_.getAttitude();
        double dt = (get_time() - velocity.getTime()).seconds();

        // Map manual controls to velocity commands using the controller's map_controls function
        // This converts normalized stick inputs to velocity commands in body frame (FRD)
        Eigen::Vector4d vel_cmd = controller_.map_controls(manual_input);
        
        // Velocity commands from map_controls are in body frame (FRD)
        // Transform to NED frame for velocity controller
        Eigen::Vector3d vel_cmd_body(vel_cmd.x(), vel_cmd.y(), vel_cmd.w());
        Eigen::Vector3d vel_cmd_ned = attitude.quaternion() * vel_cmd_body;
        
        Stamped3DVector target_velocity_3d(get_time(), vel_cmd_ned.x(), vel_cmd_ned.y(), vel_cmd_ned.z());
        
        Eigen::Vector4d output = controller_.velocityControl(
           dt, 
           prev_velocity_error_, 
           velocity, 
           attitude, 
           target_velocity_3d, 
           state_manager_.getLatestControlSignalVelocity()
        );
        
        // Set yaw rate from manual input (vel_cmd.z contains yaw_velocity)
        output.z() = vel_cmd.z();
        
        return output;
    }

    void controlLoop()
    {
        // Update flight time
        DroneState drone_state = state_manager_.getDroneState();
        rclcpp::Time current_time = get_time();
        drone_state.flight_time = drone_state.flight_time + (current_time - drone_state.timestamp);
        drone_state.timestamp = current_time;
        state_manager_.setDroneState(drone_state);

        //Lock the current control mode
        std::lock_guard<std::mutex> lock(current_control_mode_mutex_);


        Eigen::Vector4d control_input;
        switch (current_control_mode_)
        {
        case 0:
            control_input = manualMode();
            break;
        case 1:
            
            control_input = manualAidedMode();
            break;
        case 2:
            control_input = positionAndVelocityControl();
            break;
        case 3:
            control_input = safetyLandBlindMode();
            break;
        default:
            RCLCPP_WARN(get_logger(), "Unknown control mode: %d", current_control_mode_);
            control_input = Eigen::Vector4d::Zero();
            break;
        }

        publishAttitudeSetpoint(control_input);
        publishAttitudeSetpointRollPitchYawThrust(control_input);
    }

    void cleanupControlLoop()
    {
        if (control_timer_)
        {
            // Stop the control loop and reset target profile and attitude setpoint
            try
            {
                control_timer_->cancel();
                RCLCPP_INFO(get_logger(), "Control loop stopped.");
                Stamped4DVector target_profile(get_time(), 0.0, 0.0, 0.0, 0.0);
                state_manager_.setTargetPositionProfile(target_profile);
                publishAttitudeSetpoint(Eigen::Vector4d(0.0, 0.0, 0.0, 0.0));

                DroneState drone_state = state_manager_.getDroneState();
                drone_state.timestamp = get_time();
                drone_state.flight_time = rclcpp::Duration(0, 0);
                state_manager_.setDroneState(drone_state);
            }
            catch (const rclcpp::exceptions::RCLError &e)
            {
                RCLCPP_ERROR(get_logger(), "Error stopping control loop: %s", e.what());
            }
            control_timer_.reset();
        }
    }

    void ensureControlLoopRunning(int mode) {
        std::lock_guard<std::mutex> lock(current_control_mode_mutex_);
        if (current_control_mode_ != mode) {
            current_control_mode_ = mode;
            RCLCPP_INFO(get_logger(), "Control mode updated to: %d", mode);
        }
        DroneState drone_state = state_manager_.getDroneState();
        if (!control_timer_ && drone_state.arming_state == ArmingState::ARMED) {
            drone_state.timestamp = get_time();
            drone_state.flight_time = rclcpp::Duration(0, 0);
            state_manager_.setDroneState(drone_state);
            control_timer_ = create_wall_timer(10ms, [this]() { controlLoop(); });
            RCLCPP_INFO(get_logger(), "Control loop started with mode: %d", mode);
        }
    }

    Eigen::Vector4d safetyLandBlindMode(){
        DroneState drone_state = state_manager_.getDroneState();
        // Get current yaw of the drone
        StampedQuaternion attitude = state_manager_.getAttitude(); 
        Eigen::Vector3d euler = transformations_.quaternionToEuler(attitude.quaternion());
        
        if (drone_state.flight_mode != FlightMode::SAFETYLAND_BLIND)
        {
            // Make the drone go into safety land mode
            RCLCPP_INFO(get_logger(), "Safety land mode activated.");
            drone_state.timestamp = get_time();
            drone_state.flight_mode = FlightMode::SAFETYLAND_BLIND;
            drone_state.flight_mode_trait = getFlightModeTraits(drone_state.flight_mode)[0];
            state_manager_.setDroneState(drone_state);
            safety_land_start_time_ = get_time();
        }
        else if (drone_state.flight_mode == FlightMode::SAFETYLAND_BLIND)
        {
            double time_elapsed = (get_time() - safety_land_start_time_).seconds();

            // Exponential decay: thrust = final + (initial - final) * e^(-k * t)
            safety_thrust_ = safety_thrust_final_ + (safety_thrust_initial_ - safety_thrust_final_) * std::exp(-safety_thrustdown_rate_ * time_elapsed);

            RCLCPP_INFO(get_logger(), "Safety thrust: %.2f", safety_thrust_);
    
            if (safety_thrust_ >= safety_thrust_final_ - 0.02)
            {
                drone_state.timestamp = get_time();
                drone_state.flight_mode = FlightMode::LANDED;
                drone_state.flight_mode_trait = getFlightModeTraits(drone_state.flight_mode)[0];

                // Clean up control loop
                disarm(true);
                cleanupControlLoop();
                RCLCPP_INFO(get_logger(), "Drone landed safely, maybe?.");
            }
        }

        return Eigen::Vector4d(euler.x(), euler.y(), euler.z(), safety_thrust_);
    }

    void landPositionMode()
    {
        // Get current state of the drone
        DroneState drone_state = state_manager_.getDroneState();

        if (drone_state.flight_mode == FlightMode::BEGIN_LAND_POSITION)
        {
            // Make the drone go into safety land mode
            RCLCPP_INFO(get_logger(), "Position based land mode activated.");

            // Get trajectory initialization state with current position, velocity, and acceleration
            TrajectoryInitState init_state = state_manager_.getTrajectoryInitState();
            Stamped3DVector GroundDistance = state_manager_.getGroundDistanceState();

            // Calculate the landing position
            double z_landing = 0.0; // Default landing height
            if ((get_time() - GroundDistance.getTime()).seconds() < 0.3)
            {
                z_landing = init_state.position.z() + GroundDistance.vector().x();
                RCLCPP_INFO(get_logger(), "Using ground distance sensor for landing: z_landing=%.2f", z_landing);
            }
            
            // Set the target landing position and orientation (maintain current orientation)
            Eigen::Vector3d target_position = {init_state.position.x(), init_state.position.y(), z_landing};
            Eigen::Quaterniond target_quat = init_state.orientation; // Preserve current orientation

            // Generate landing trajectory
            path_planner_.GenerateTrajectory(init_state.position, target_position, init_state.orientation, 
                                            target_quat, init_state.velocity, init_state.acceleration, 
                                            trajectoryMethod::MIN_SNAP);
            float trajectory_duration = path_planner_.getTotalTime();

            // Set trajectory start time and update flight mode
            drone_state.flight_mode = FlightMode::LAND_POSITION;
            drone_state.flight_mode_trait = getFlightModeTraits(drone_state.flight_mode)[0];
            drone_state.trajectory_start_time_ = get_time();
            drone_state.trajectory_mode = TrajectoryMode::ACTIVE;
            drone_state.trajectory_duration = rclcpp::Duration::from_seconds(trajectory_duration);
            state_manager_.setDroneState(drone_state);
        }
    }

    bool isModeChangePending()
    {
        return get_clock()->now() < mode_change_deadline_;
    }

    void setDroneMode(FlightMode mode)
    {
        DroneState drone_state = state_manager_.getDroneState();
        FlightMode old_flight_mode = drone_state.flight_mode;

        if (old_flight_mode != mode)
        {
            drone_state.timestamp = get_time();
            drone_state.flight_mode = mode;
            drone_state.flight_mode_trait = getFlightModeTraits(mode)[0];
            state_manager_.setDroneState(drone_state);

            // Only delay if switching between autonomous/manual, but NOT if switching to standby or between estop and standby
            if (do_mode_change_delay_ &&
                (getFlightModeTraits(mode)[0] != getFlightModeTraits(old_flight_mode)[0]) &&
                mode != FlightMode::STANDBY &&
                mode != FlightMode::EMERGENCY_STOP &&
                !( (mode == FlightMode::EMERGENCY_STOP && old_flight_mode == FlightMode::STANDBY) ||
                   (mode == FlightMode::STANDBY && old_flight_mode == FlightMode::EMERGENCY_STOP) )
            )
            {
                // Record deadline instead of blocking — callers check isModeChangePending()
                mode_change_deadline_ = get_clock()->now() + rclcpp::Duration::from_seconds(mode_change_delay_);
                RCLCPP_INFO(get_logger(), "Mode change delay of %.2f s started (non-blocking)", mode_change_delay_);
            }
        }
    }

    // Command functions
    void arm()
    {
        publishAttitudeSetpoint(Eigen::Vector4d(0.0, 0.0, 0.0, 0.0));
        RCLCPP_INFO(get_logger(), "Sending arm command...");
        publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
    }

    void disarm(bool kill = false)
    {
        publishAttitudeSetpoint(Eigen::Vector4d(0.0, 0.0, 0.0, 0.0));

        float param2 = kill ? 21196.0 : 0.0;
        RCLCPP_INFO(get_logger(), "Sending disarm command (kill=%d)...", kill);
        publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, param2);
    }

    void publishAttitudeSetpoint(const Eigen::Vector4d &input)
    {
        Eigen::Quaterniond q = transformations_.eulerToQuaternion(input.x(), input.y(), input.z());
        VehicleAttitudeSetpoint msg{};
        msg.timestamp = get_time().nanoseconds() / 1000;
        msg.q_d = {static_cast<float>(q.w()), static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z())};
        msg.thrust_body = {0.0f, 0.0f, static_cast<float>(input.w())};
        attitude_setpoint_pub_->publish(msg);
    }

    void setServo(int aux_index, int id, float value)
    {
       // aux_index: 0 = AUX1, 1 = AUX2, etc.
       // value: -1.0 to 1.0

      
        GimbalState gimbal_id = static_cast<GimbalState>(id);
       if (aux_index < 0 || aux_index >= 8) {
           RCLCPP_WARN(this->get_logger(), "Ignoring servo command with invalid aux_index=%d", aux_index);
           return;
       }
       if (gimbal_id != GimbalState::Manual && gimbal_id != GimbalState::Auto) {
           RCLCPP_WARN(this->get_logger(), "Ignoring servo command with invalid gimbal_id=%d", id);
           return;
       }

       if (gimbal_id == GimbalState::Manual) {
            gimbal_is_manual_ = true;
            gimbal_timeout_timer_->reset();
            value = std::clamp(value, -1.0f, 1.0f);
        }
        else if (gimbal_id == GimbalState::Auto) {
            if (gimbal_is_manual_) {
                return;  
            }
            value = std::clamp(value, -1.0f, 1.0f);
        }
         
       bool armed = (state_manager_.getDroneState().arming_state == ArmingState::ARMED);
       const int64_t now_ns = get_time().nanoseconds();

       latest_servo_aux_index_ = aux_index;
       latest_servo_value_ = value;
       latest_servo_command_ns_ = now_ns;
       has_latest_servo_command_ = true;

       if (armed) {
           // Armed: use actuator_servos — goes through proper mixer pipeline
           px4_msgs::msg::ActuatorServos msg{};
           msg.timestamp        = get_time().nanoseconds() / 1000;
           msg.timestamp_sample = get_time().nanoseconds() / 1000;
           msg.control.fill(0.0f);  // zero-initialize
           msg.control[aux_index] = value;
           actuator_servos_pub_->publish(msg);
       } else {
           // Disarmed: use actuator_test — bypasses arming gate
           publishActuatorTestServo(aux_index, value);
       }
    }

    void publishActuatorTestServo(int aux_index, float value)
    {
        px4_msgs::msg::ActuatorTest msg{};
        msg.timestamp  = get_time().nanoseconds() / 1000;
        msg.function   = 201 + aux_index;  // 201=Servo1(AUX1), 202=Servo2(AUX2), etc.
        msg.value      = value;
        msg.timeout_ms = servo_test_timeout_ms_;
        msg.action     = 1;    // 1 = set value
        actuator_test_pub_->publish(msg);
    }

    void publishHeldDisarmedServoCommand()
    {
        if (!has_latest_servo_command_) {
            return;
        }

        const int64_t now_ns = get_time().nanoseconds();
        if ((now_ns - latest_servo_command_ns_) > servo_hold_timeout_ns_) {
            has_latest_servo_command_ = false;
            return;
        }

        bool armed = (state_manager_.getDroneState().arming_state == ArmingState::ARMED);
        if (armed) {
            return;
        }

        // Refresh ActuatorTest before timeout so disarmed servo command remains stable.
        publishActuatorTestServo(latest_servo_aux_index_, latest_servo_value_);
    }

    void publishAttitudeSetpointRollPitchYawThrust(const Eigen::Vector4d &input)
    {
        asr_comms::msg::AttitudeSetpointRPY msg{};
        msg.timestamp = get_time().nanoseconds() / 1000;
        // Some message definitions do not include a 'mode' field, so we skip assigning it
        msg.orientation = {
            static_cast<float>(wrapToPi(input.x())), // roll
            static_cast<float>(wrapToPi(input.y())), // pitch
            static_cast<float>(wrapToPi(input.z()))  // yaw
        };
        msg.thrust = static_cast<double>(input.w());
        attitude_setpoint_rpy_thrust_pub_->publish(msg);
    }

    void publishVehicleCommand(uint16_t command, float param1, float param2)
    {
        VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = get_time().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);
    }

    // Action server handlers
    rclcpp_action::GoalResponse handleDroneCommand(const rclcpp_action::GoalUUID & /*uuid*/,
                                                std::shared_ptr<const DroneCommand::Goal> goal)
    {
        static const std::vector<std::string> allowed_commands = {"arm", "disarm", "takeoff", "goto", "land", "estop", "eland", "manual", "manual_aided", "set_origin", "set_linear_speed", "set_angular_speed", "spin", "test_multi_waypoint"};
        RCLCPP_INFO(get_logger(), "Received goal request with command_type: %s", goal->command_type.c_str());

        DroneState drone_state = state_manager_.getDroneState();
        if (std::find(allowed_commands.begin(), allowed_commands.end(), goal->command_type) == allowed_commands.end())
        {
            RCLCPP_WARN(get_logger(), "Rejected invalid command_type: %s", goal->command_type.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->command_type == "arm") {
            if (drone_state.arming_state == ArmingState::ARMED) {
                RCLCPP_WARN(get_logger(), "Rejected: drone already armed.");
                return rclcpp_action::GoalResponse::REJECT;
            }
        } else if (goal->command_type == "disarm") {
            if (drone_state.arming_state == ArmingState::DISARMED) {
                RCLCPP_WARN(get_logger(), "Rejected: drone already disarmed.");
                return rclcpp_action::GoalResponse::REJECT;
            }
        } else if (goal->command_type == "land" && false) {
            RCLCPP_WARN(get_logger(), "Rejected: invalid land parameters or drone not armed.");
            return rclcpp_action::GoalResponse::REJECT;
        } else if (goal->command_type == "manual") {
            if (drone_state.flight_mode == FlightMode::MANUAL || drone_state.arming_state != ArmingState::ARMED) {
                RCLCPP_WARN(get_logger(), "Rejected: already in manual mode or drone not armed.");
                return rclcpp_action::GoalResponse::REJECT;
            }
        } else if (goal->command_type == "set_origin") {
            if (drone_state.arming_state != ArmingState::DISARMED) {
                RCLCPP_WARN(get_logger(), "Rejected: Drone was not disarmed");
                return rclcpp_action::GoalResponse::REJECT;
            }
        } else if (goal->command_type == "set_linear_speed") {
            if (goal->target_pose.size() != 1 || drone_state.arming_state != ArmingState::DISARMED) {
                RCLCPP_WARN(get_logger(), "Rejected: invalid set_linear_speed parameters or drone not armed.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            float linear_speed = goal->target_pose[0];
            if (!std::isfinite(linear_speed)) {
                RCLCPP_WARN(get_logger(), "Rejected: linear speed is not a finite real number (value: %f).", linear_speed);
                return rclcpp_action::GoalResponse::REJECT;
            }
        } else if (goal->command_type == "set_angular_speed") {
            if (goal->target_pose.size() != 1 || drone_state.arming_state != ArmingState::DISARMED) {
                RCLCPP_WARN(get_logger(), "Rejected: invalid set_angular_speed parameters or drone not armed.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            float angular_speed = goal->target_pose[0];
            if (!std::isfinite(angular_speed)) {
                RCLCPP_WARN(get_logger(), "Rejected: angular speed is not a finite real number (value: %f).", angular_speed);
                return rclcpp_action::GoalResponse::REJECT;
            }
        } else if (goal->command_type == "spin") {
            if (goal->target_pose.size() != 3 || drone_state.arming_state != ArmingState::ARMED || 
                goal->target_pose[1] < 0.0 || (goal->target_pose[2] != 0.0 && goal->target_pose[2] != 1.0) ||
                !std::isfinite(goal->target_pose[0]) || !std::isfinite(goal->target_pose[1]) || !std::isfinite(goal->target_pose[2])) {
                RCLCPP_WARN(get_logger(), "Rejected: invalid spin parameters, drone not armed, negative rotations, invalid path selection, or non-finite values.");
                return rclcpp_action::GoalResponse::REJECT;
            }
        } else if (goal->command_type == "takeoff") {
            if (goal->target_pose.size() != 1) {
                RCLCPP_WARN(get_logger(), "Rejected: invalid takeoff parameters, expected single altitude value.");
                return rclcpp_action::GoalResponse::REJECT;
            } else if (drone_state.arming_state != ArmingState::ARMED) {
                RCLCPP_WARN(get_logger(), "Rejected: drone not armed for takeoff.");
                return rclcpp_action::GoalResponse::REJECT;
            } else if (!std::isfinite(goal->target_pose[0])){
                RCLCPP_WARN(get_logger(), "Rejected: takeoff altitude is not a finite real number (value: %f).", goal->target_pose[0]);
                return rclcpp_action::GoalResponse::REJECT;
            } else if (geofence_violated(Eigen::Vector3d(0, 0, goal->target_pose[0]))) {
                RCLCPP_WARN(get_logger(), "Rejected: takeoff altitude %f would violate geofence.", goal->target_pose[0]);
                return rclcpp_action::GoalResponse::REJECT;
            }
        } else if (goal->command_type == "goto") {
            if (goal->target_pose.size() != 3) {
                RCLCPP_WARN(get_logger(), "Rejected: invalid goto parameters, expected x, y, z coordinates.");
                return rclcpp_action::GoalResponse::REJECT;
            } else if (drone_state.arming_state != ArmingState::ARMED) {
                RCLCPP_WARN(get_logger(), "Rejected: drone not armed for goto.");
                return rclcpp_action::GoalResponse::REJECT;
            } else if (!std::isfinite(goal->target_pose[0]) || !std::isfinite(goal->target_pose[1]) || !std::isfinite(goal->target_pose[2])) {
                RCLCPP_WARN(get_logger(), "Rejected: goto coordinates are not finite real numbers (x: %f, y: %f, z: %f).", 
                            goal->target_pose[0], goal->target_pose[1], goal->target_pose[2]);
                return rclcpp_action::GoalResponse::REJECT;
            } else if (!std::isfinite(goal->yaw)) {
                RCLCPP_WARN(get_logger(), "Rejected: goto yaw is not a finite real number (value: %f).", goal->yaw);
                return rclcpp_action::GoalResponse::REJECT;
            } else if (geofence_violated(Eigen::Vector3d(goal->target_pose[0], goal->target_pose[1], goal->target_pose[2]))) {
                RCLCPP_WARN(get_logger(), "Rejected: target position (x: %f, y: %f, z: %f) would violate geofence.", 
                            goal->target_pose[0], goal->target_pose[1], goal->target_pose[2]);
                return rclcpp_action::GoalResponse::REJECT;
            }
            else if (goal->command_type == "manual_aided") {
            if (drone_state.flight_mode == FlightMode::MANUAL_AIDED || drone_state.arming_state != ArmingState::ARMED) {
                RCLCPP_WARN(get_logger(), "Rejected: already in manual aided mode or drone not armed.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            }
        } else if (goal->command_type == "test_multi_waypoint") {
            if (drone_state.arming_state != ArmingState::ARMED) {
                RCLCPP_WARN(get_logger(), "Rejected: drone not armed for test_multi_waypoint.");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }


    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleDroneCommand> /*goal_handle*/)
    {
        RCLCPP_INFO(get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleDroneCommand> goal_handle)
    {
        if (execute_thread_.joinable()) {
            execute_thread_.join();
        }
        execute_thread_ = std::thread([this, goal_handle]() { execute(goal_handle); });
    }

    // Command execution handlers
    void activateTrajectory(float trajectory_duration) {
        DroneState drone_state = state_manager_.getDroneState();
        drone_state.trajectory_start_time_ = get_time();
        drone_state.trajectory_mode = TrajectoryMode::ACTIVE;
        drone_state.trajectory_duration = rclcpp::Duration::from_seconds(trajectory_duration);
        state_manager_.setDroneState(drone_state);
    }

    void executeEstop(std::shared_ptr<DroneCommand::Result> result) {
        setDroneMode(FlightMode::EMERGENCY_STOP);
        disarm(true);
        cleanupControlLoop();
        result->success = true;
        result->message = "Emergency stop executed.";
    }

    void executeEland(std::shared_ptr<DroneCommand::Result> result) {
        ensureControlLoopRunning(3);
        result->success = true;
        result->message = "Emergency landing executed.";
    }

    void executeArm(std::shared_ptr<DroneCommand::Result> result) {
        Stamped4DVector target_profile = state_manager_.getTargetPositionProfile();
        Stamped3DVector global_pos = state_manager_.getGlobalPosition();
        target_profile.setTime(get_time());
        target_profile.setX(global_pos.x());
        target_profile.setY(global_pos.y());
        target_profile.setZ(global_pos.z());
        target_profile.setW(0.0);
        state_manager_.setTargetPositionProfile(target_profile);
        arm();
        result->success = true;
        result->message = "Drone armed.";
    }

    void executeDisarm(std::shared_ptr<DroneCommand::Result> result) {
        disarm();
        cleanupControlLoop();
        result->success = true;
        result->message = "Drone disarmed.";
    }

    void executeTakeoff(const std::shared_ptr<const DroneCommand::Goal> goal, 
                        std::shared_ptr<DroneCommand::Result> result) {
        setDroneMode(FlightMode::POSITION);
        ensureControlLoopRunning(2);

        TrajectoryInitState init_state = state_manager_.getTrajectoryInitState();
        Eigen::Quaterniond target_quat = transformations_.eulerToQuaternion(0.0, 0.0, init_state.yaw).normalized();
        Eigen::Vector3d target_position = {init_state.position.x(), init_state.position.y(), 
                                          std::min(goal->target_pose[0], takeoff_height_ + init_state.position.z())};

        path_planner_.GenerateTrajectory(init_state.position, target_position, init_state.orientation, 
                                        target_quat, init_state.velocity, init_state.acceleration, 
                                        trajectoryMethod::MIN_SNAP);
        activateTrajectory(path_planner_.getTotalTime());

        result->success = true;
        result->message = "Drone taking off.";
        std::cout << "Takeoff target: " << target_position.transpose() << std::endl;
    }

    void executeGoto(const std::shared_ptr<const DroneCommand::Goal> goal, 
                     std::shared_ptr<DroneCommand::Result> result) {
        setDroneMode(FlightMode::POSITION);
        ensureControlLoopRunning(2);

        TrajectoryInitState init_state = state_manager_.getTrajectoryInitState();
        float target_yaw = transformations_.unwrapAngle(goal->yaw, 2*M_PI, 0);
        Eigen::Vector3d target_position = {goal->target_pose[0], goal->target_pose[1], goal->target_pose[2]};
        Eigen::Quaterniond target_quat = transformations_.eulerToQuaternion(0.0, 0.0, target_yaw).normalized();

        path_planner_.GenerateTrajectory(init_state.position, target_position, init_state.orientation, 
                                        target_quat, init_state.velocity, init_state.acceleration, 
                                        trajectoryMethod::MIN_SNAP);
        activateTrajectory(path_planner_.getTotalTime());

        result->success = true;
        result->message = "Drone moving to target position.";
    }

    void executeSpin(const std::shared_ptr<const DroneCommand::Goal> goal, 
                     std::shared_ptr<DroneCommand::Result> result) {
        setDroneMode(FlightMode::POSITION);
        ensureControlLoopRunning(2);

        TrajectoryInitState init_state = state_manager_.getTrajectoryInitState();
        double target_yaw = goal->target_pose[0];
        double num_rotations = goal->target_pose[1];
        bool use_longest_path = (goal->target_pose[2] == 1.0);

        path_planner_.GenerateSpinTrajectory(init_state.position, init_state.orientation, 
                                            target_yaw, num_rotations, use_longest_path, 
                                            trajectoryMethod::MIN_SNAP);
        activateTrajectory(path_planner_.getTotalTime());

        result->success = true;
        result->message = "Drone spinning to yaw " + std::to_string(target_yaw) + " with " + 
                         std::to_string(num_rotations) + " rotations.";
        RCLCPP_INFO(get_logger(), "Spin target: yaw=%.2f, rotations=%.1f, path=%s", 
                   target_yaw, num_rotations, use_longest_path ? "longest" : "shortest");
    }

    void executeTestMultiWaypoint(std::shared_ptr<DroneCommand::Result> result) {
        setDroneMode(FlightMode::POSITION);
        ensureControlLoopRunning(2);

        TrajectoryInitState init_state = state_manager_.getTrajectoryInitState();
        std::vector<Waypoint> waypoints;
        
        waypoints.push_back({init_state.position, init_state.yaw, 0.0, 0.0});
        waypoints.push_back({init_state.position + Eigen::Vector3d(1.0, 0.0, 0.0), init_state.yaw, 0.0, 0.0});
        waypoints.push_back({init_state.position + Eigen::Vector3d(1.0, 1.0, 0.0), init_state.yaw + 1.57, 0.0, 0.0});
        waypoints.push_back({init_state.position + Eigen::Vector3d(0.0, 1.0, 0.0), init_state.yaw + 3.14, 0.0, 0.0});
        waypoints.push_back({init_state.position, init_state.yaw, 0.0, 0.0});

        bool success = path_planner_.GenerateMultiWaypointTrajectory(
            waypoints, init_state.velocity, init_state.acceleration, init_state.yaw, trajectoryMethod::MIN_SNAP);

        if (!success) {
            result->success = false;
            result->message = "Failed to generate multi-waypoint trajectory.";
            RCLCPP_ERROR(get_logger(), "Multi-waypoint trajectory generation failed!");
            return;
        }

        float trajectory_duration = path_planner_.getTotalTime();
        auto constraint_result = path_planner_.checkConstraints(200);
        if (!constraint_result.satisfied) {
            RCLCPP_WARN(get_logger(), "Trajectory violates constraints! Max velocity: %.3f m/s (limit: %.3f m/s)", 
                       constraint_result.max_velocity, path_planner_.max_linear_velocity_);
        }
        
        auto segments = path_planner_.getSegmentInfo();
        RCLCPP_INFO(get_logger(), "Multi-waypoint trajectory: %zu segments, total time: %.2f s", 
                   segments.size(), trajectory_duration);
        for (size_t i = 0; i < segments.size(); ++i) {
            RCLCPP_INFO(get_logger(), "  Segment %zu: duration=%.2f s", i, segments[i].duration);
        }

        activateTrajectory(trajectory_duration);

        result->success = true;
        result->message = "Testing multi-waypoint trajectory: " + std::to_string(waypoints.size()) + 
                         " waypoints, " + std::to_string(segments.size()) + " segments, " + 
                         std::to_string(trajectory_duration) + " seconds.";
    }

    void executeManual(std::shared_ptr<DroneCommand::Result> result) {
        cleanupControlLoop();
        setDroneMode(FlightMode::MANUAL);
        ensureControlLoopRunning(0);
        result->success = true;
        result->message = "Drone in manual mode.";
    }

    void executeManualAided(std::shared_ptr<DroneCommand::Result> result) {
        setDroneMode(FlightMode::MANUAL_AIDED);
        ensureControlLoopRunning(1);
        result->success = true;
        result->message = "Drone in manual aided mode.";
    }

    void executeLand(std::shared_ptr<DroneCommand::Result> result) {
        setDroneMode(FlightMode::BEGIN_LAND_POSITION);
        ensureControlLoopRunning(2);
        landPositionMode();
        result->success = true;
        result->message = "Drone landing.";
    }

    void executeSetOrigin(std::shared_ptr<DroneCommand::Result> result) {
        Stamped3DVector global_position = state_manager_.getGlobalPosition();
        Stamped3DVector Current_origin = state_manager_.getOrigin();
        global_position.vector().x() = global_position.vector().x() + Current_origin.vector().x();
        global_position.vector().y() = global_position.vector().y() + Current_origin.vector().y();
        global_position.vector().z() = global_position.vector().z() + Current_origin.vector().z();

        state_manager_.setOrigin(global_position);
        RCLCPP_INFO(get_logger(), "Origin set to current position: (%.2f, %.2f, %.2f)", 
                   global_position.x(), global_position.y(), global_position.z());
        result->success = true;
        result->message = "Origin set to current position.";
    }

    void executeSetLinearSpeed(const std::shared_ptr<const DroneCommand::Goal> goal, 
                               std::shared_ptr<DroneCommand::Result> result) {
        float linear_speed = goal->target_pose[0];
        bool velocity_set = path_planner_.setLinearVelocity(linear_speed);
        result->success = velocity_set;
        if (velocity_set) {
            result->message = "Linear speed set to " + std::to_string(linear_speed) + " m/s.";
        } else {
            result->message = "Failed to set linear speed.";
        }
    }

    void executeSetAngularSpeed(const std::shared_ptr<const DroneCommand::Goal> goal, 
                                std::shared_ptr<DroneCommand::Result> result) {
        float angular_speed = goal->target_pose[0];
        bool velocity_set = path_planner_.setAngularVelocity(angular_speed);
        result->success = velocity_set;
        if (velocity_set) {
            result->message = "Angular speed set to " + std::to_string(angular_speed) + " rad/s.";
        } else {
            result->message = "Failed to set angular speed.";
        }
    }

    void execute(const std::shared_ptr<GoalHandleDroneCommand> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<DroneCommand::Result>();

        try
        {
            if (goal->command_type == "estop") {
                executeEstop(result);
            }
            else if (goal->command_type == "eland") {
                executeEland(result);
            }
            else if (goal->command_type == "arm") {
                executeArm(result);
            }
            else if (goal->command_type == "disarm") {
                executeDisarm(result);
            }
            else if (goal->command_type == "takeoff") {
                executeTakeoff(goal, result);
            }
            else if (goal->command_type == "goto") {
                executeGoto(goal, result);
            } else if (goal->command_type == "spin") {
                executeSpin(goal, result);
            }
            else if (goal->command_type == "test_multi_waypoint") {
                executeTestMultiWaypoint(result);
            }
            else if (goal->command_type == "manual") {
                executeManual(result);
            }
            else if (goal->command_type == "manual_aided") {
                executeManualAided(result);
            }
            else if (goal->command_type == "land") {
                executeLand(result);
            }
            else if (goal->command_type == "set_origin") {
                executeSetOrigin(result);
            }
            else if (goal->command_type == "set_linear_speed") {
                executeSetLinearSpeed(goal, result);
            }
            else if (goal->command_type == "set_angular_speed") {
                executeSetAngularSpeed(goal, result);
            }
            else
            {
                result->success = false;
                result->message = "Invalid command.";
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Execution error: %s", e.what());
            result->success = false;
            result->message = "Execution failed.";
        }

        if (rclcpp::ok())
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal execution completed.");
        }
    }

    // Member variables
    std::shared_ptr<rclcpp::Clock> clock_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_;
    rclcpp::Publisher<asr_comms::msg::AttitudeSetpointRPY>::SharedPtr attitude_setpoint_rpy_thrust_pub_;

    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<asr_comms::msg::TelemetryPosition>::SharedPtr telemetry_position_pub_;
    rclcpp::Publisher<asr_comms::msg::TelemetryAttitude>::SharedPtr telemetry_attitude_pub_;
    rclcpp::Publisher<asr_comms::msg::TelemetryBattery>::SharedPtr  telemetry_battery_pub_;
    rclcpp::Publisher<asr_comms::msg::TelemetryGPS>::SharedPtr      telemetry_gps_pub_;
    rclcpp::Publisher<asr_comms::msg::TelemetryStatus>::SharedPtr   telemetry_status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr origin_offset_pub_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr actuator_servos_pub_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorTest>::SharedPtr actuator_test_pub_;

    rclcpp::Subscription<asr_comms::msg::GcsHeartbeat>::SharedPtr gcs_heartbeat_sub_;
    rclcpp::Subscription<asr_comms::msg::MotionCapturePose>::SharedPtr motion_capture_local_position_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<asr_comms::msg::ManualControlInput>::SharedPtr manual_input_sub_;
    rclcpp::Subscription<BatteryStatus>::SharedPtr battery_status_sub_;
    rclcpp::Subscription<DistanceSensor>::SharedPtr ground_distance_sub_;
    rclcpp::Subscription<ActuatorOutputs>::SharedPtr actuator_output_sub_;
    rclcpp::Subscription<asr_comms::msg::ProbeGlobalLocations>::SharedPtr probe_global_locations_sub_;
    rclcpp::Subscription<SensorGps>::SharedPtr gps_sub_;
    rclcpp::Subscription<asr_comms::msg::ServoCommand>::SharedPtr servo_command_sub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr offboard_timer_;
    rclcpp::TimerBase::SharedPtr drone_state_timer;
    rclcpp::TimerBase::SharedPtr safety_timer_;
    rclcpp::TimerBase::SharedPtr offset_timer;
    rclcpp::TimerBase::SharedPtr servo_hold_timer_;
    rclcpp_action::Server<DroneCommand>::SharedPtr drone_command_server_;
    rclcpp::TimerBase::SharedPtr gimbal_timeout_timer_;


    Transformations transformations_;
    StateManager state_manager_;
    Controller controller_;
    PathPlanner path_planner_;

    PositionError prev_position_error_;
    
    VelocityError prev_velocity_error_;
    AccelerationError prev_acceleration_error_;
    static constexpr float yaw_sensitivity_ = 1.0f / 20.0f; // !! Why define this here?....

    // Setup variables
    float lidar_offset_; // Describes what the Lidar measures, in meteres when standing on the ground

    //Safety variables
    bool gcs_stale_ = false;
    bool position_stale_ = false;
    bool check_gcs_timeout_;
    bool check_position_timeout_;
    float mode_change_delay_;
    bool do_mode_change_delay_;
    rclcpp::Time mode_change_deadline_{0, 0, RCL_ROS_TIME};

    std::thread execute_thread_;
    float safety_thrust_;
    float safety_thrust_initial_;
    float safety_thrust_final_;
    float safety_thrustdown_rate_;
    float gcs_timeout_threshold_;
    float position_timeout_threshold_;
    rclcpp::Time safety_land_start_time_;
    bool safety_check_battery_;
    float safety_battery_threshold_;
    bool safety_check_geofence_;
    float safety_geofence_radius_;
    float safety_geofence_height_;

    //controller specific variables
    rclcpp::Time timestamp_last_position_control_;
    double period_position_control_ = 1/25.0; // ms
    double period_velocity_control_ = 1/50.0; // ms

    int position_loop_counter_ = 2;
    const int position_loop_trigger_ = 2;

    // Logging variables
    bool logging_enabled_ = false;
    std::string console_level_ = "INFO";

    // Last X Ground Distance Sensor readings
    static constexpr int max_ground_distance_readings_ = 10;
    std::vector<float> ground_distance_readings_ = std::vector<float>(max_ground_distance_readings_, 0.0f);
    std::vector<rclcpp::Time> ground_distance_timestamps_ = std::vector<rclcpp::Time>(max_ground_distance_readings_, rclcpp::Time(0, 0));

    int offboard_setpoint_counter_;
    bool offboard_mode_set_;
    double timeout_threshold_;
    int current_control_mode_;
    std::mutex current_control_mode_mutex_;

    int latest_servo_aux_index_ = 0;
    float latest_servo_value_ = 0.0f;
    bool has_latest_servo_command_ = false;
    int64_t latest_servo_command_ns_ = 0;
    static constexpr uint32_t servo_test_timeout_ms_ = 500;
    static constexpr int64_t servo_hold_timeout_ns_ = 5000000000LL;
    bool gimbal_is_manual_ = false;

    // Setup variables
    float takeoff_height_;
    float motor_speed_min_;
    float motor_speed_max_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AAUAutopilot>());
    rclcpp::shutdown();
    return 0;
}   