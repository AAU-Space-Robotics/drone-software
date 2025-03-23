#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>

#include <interfaces/action/drone_command.hpp>
#include <interfaces/msg/manual_control_input.hpp>

#include "fci_controller.h"
#include "fci_state_manager.h"
#include "fci_transformations.h"

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class FlightControllerInterface : public rclcpp::Node {
public:
    using DroneCommand = interfaces::action::DroneCommand;
    using GoalHandleDroneCommand = rclcpp_action::ServerGoalHandle<DroneCommand>;

    FlightControllerInterface()
        : Node("flight_controller_interface"),
          controller_(transformations_),
          offboard_setpoint_counter_(0),
          offboard_mode_set_(false),
          timeout_threshold_(0.2) {
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

        offboard_control_mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        bodyrate_setpoint_pub_ = create_publisher<VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint", 10);
        attitude_setpoint_pub_ = create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
        vehicle_command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        gps_sub_ = create_subscription<VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position", qos,
            [this](const VehicleGlobalPosition::SharedPtr msg) { gpsCallback(msg); });
        attitude_sub_ = create_subscription<VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            [this](const VehicleAttitude::SharedPtr msg) { attitudeCallback(msg); });
        status_sub_ = create_subscription<VehicleStatus>(
            "/fmu/out/vehicle_status", qos,
            [this](const VehicleStatus::SharedPtr msg) { vehicleStatusCallback(msg); });
        manual_input_sub_ = create_subscription<interfaces::msg::ManualControlInput>(
            "drone/in/manual_input", qos,
            [this](const interfaces::msg::ManualControlInput::SharedPtr msg) { manualControlInputCallback(msg); });
        sensor_combined_sub_ = create_subscription<SensorCombined>(
            "/fmu/out/sensor_combined", qos,
            [this](const SensorCombined::SharedPtr msg) { sensorCombinedCallback(msg); });

        drone_command_server_ = rclcpp_action::create_server<DroneCommand>(
            this, "/fmu/in/drone_command",
            [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const DroneCommand::Goal> goal) {
                return handleDroneCommand(uuid, goal);
            },
            [this](const std::shared_ptr<GoalHandleDroneCommand> goal_handle) {
                return handleCancel(goal_handle);
            },
            [this](const std::shared_ptr<GoalHandleDroneCommand> goal_handle) {
                handleAccepted(goal_handle);
            });

        offboard_timer_ = create_wall_timer(200ms, [this]() { setOffboardMode(); });

        RCLCPP_INFO(get_logger(), "FlightControllerInterface initialized.");
    }

private:
    void gpsCallback(const VehicleGlobalPosition::SharedPtr msg) {
        if (!transformations_.isGPSOriginSet()) {
            transformations_.setGPSOrigin(now(), msg->lat, msg->lon, msg->alt);
        }
        state_manager_.setGlobalPosition(transformations_.convertGPSToGlobalPosition(now(), msg->lat, msg->lon, msg->alt));
    }

    void sensorCombinedCallback(const SensorCombined::SharedPtr msg) {
        Eigen::Vector3d acceleration_local{msg->accelerometer_m_s2[0], msg->accelerometer_m_s2[1], msg->accelerometer_m_s2[2]};
        Stamped3DVector acceleration_global = transformations_.accelerationLocalToGlobal(now(), state_manager_.getAttitude().quaternion(), acceleration_local);
        acceleration_global.vector().z() += 9.81; // Remove gravity
        state_manager_.setGlobalAcceleration(acceleration_global);
    }

    void attitudeCallback(const VehicleAttitude::SharedPtr msg) {
        StampedQuaternion attitude;
        attitude.setTime(now());
        attitude.quaternion() = Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
        state_manager_.setAttitude(attitude);
    }

    void vehicleStatusCallback(const VehicleStatus::SharedPtr msg) {
        DroneState drone_state = state_manager_.getDroneState();
        drone_state.timestamp = now();
        drone_state.arming_state = (msg->arming_state == 2) ? ArmingState::ARMED : ArmingState::DISARMED;
        state_manager_.setDroneState(drone_state);
    }

    void manualControlInputCallback(const interfaces::msg::ManualControlInput::SharedPtr msg) {
        Stamped4DVector manual_input = state_manager_.getManualControlInput();
        manual_input.setTime(now());
        manual_input.setX(controller_.mapNormToAngle(msg->roll));
        manual_input.setY(controller_.mapNormToAngle(msg->pitch));
        manual_input.setZ(manual_input.z() + controller_.mapNormToAngle(msg->yaw_velocity * yaw_sensitivity_));
        manual_input.setW(msg->thrust);
        state_manager_.setManualControlInput(manual_input);
    }

    void setOffboardMode() {
        publishOffboardControlMode();
        if (!offboard_mode_set_ && offboard_setpoint_counter_ < 11) {
            if (offboard_setpoint_counter_ == 10) {
                publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                offboard_mode_set_ = true;
            }
            offboard_setpoint_counter_++;
        }
    }

    void publishOffboardControlMode() {
        OffboardControlMode msg{};
        msg.position = false;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = true;
        msg.body_rate = false;
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(msg);
    }

    Eigen::Vector4d manualMode() {
        Stamped4DVector manual_input = state_manager_.getManualControlInput();
        return {manual_input.x(), manual_input.y(), manual_input.z(), manual_input.w()};
    }

    Eigen::Vector4d controlMode() {
        Stamped4DVector target_profile = state_manager_.getTargetPositionProfile();
        Stamped3DVector position = state_manager_.getGlobalPosition();
        StampedQuaternion attitude = state_manager_.getAttitude();
        Stamped3DVector target_position_3d;
        target_position_3d.setTime(target_profile.getTime());
        target_position_3d.vector() = target_profile.vector().head<3>(); // Extract x, y, z

        double dt = (now() - position.getTime()).seconds();
        if (dt > timeout_threshold_) {
            RCLCPP_WARN(get_logger(), "No position data received in the last %.2f seconds!", timeout_threshold_);
            return Eigen::Vector4d::Zero();
        }

        Eigen::Vector4d output = controller_.pidControl(dt, prev_position_error_, position, attitude, target_position_3d);
        return output;
    }

    Eigen::Vector4d manualAidedMode() {
        Stamped4DVector manual_input = state_manager_.getManualControlInput();
        Stamped4DVector target_profile = state_manager_.getTargetPositionProfile();
        target_profile.setZ(target_profile.z() + manual_input.w() / 10.0);
        state_manager_.setTargetPositionProfile(target_profile);

        Stamped3DVector position = state_manager_.getGlobalPosition();
        StampedQuaternion attitude = state_manager_.getAttitude();
        Stamped3DVector target_position_3d;
        target_position_3d.setTime(target_profile.getTime());
        target_position_3d.vector() = target_profile.vector().head<3>(); // Extract x, y, z

        double dt = (now() - position.getTime()).seconds();
        if (dt > timeout_threshold_) {
            RCLCPP_WARN(get_logger(), "No position data received in the last %.2f seconds!", timeout_threshold_);
            return Eigen::Vector4d::Zero();
        }

        Eigen::Vector4d output = controller_.pidControl(dt, prev_position_error_, position, attitude, target_position_3d);
        return {manual_input.x(), manual_input.y(), manual_input.z(), output.w()};
    }

    void controlLoop(int mode) {
        Eigen::Vector4d control_input;
        switch (mode) {
            case 0: control_input = manualMode(); break;
            case 1: control_input = manualAidedMode(); break;
            case 2: control_input = controlMode(); break;
            default:
                RCLCPP_WARN(get_logger(), "Unknown control mode: %d", mode);
                control_input = Eigen::Vector4d::Zero();
                break;
        }
        RCLCPP_INFO(get_logger(), "Control input: roll=%.2f, pitch=%.2f, yaw=%.2f, thrust=%.2f",
                    control_input.x(), control_input.y(), control_input.z(), control_input.w());
        publishAttitudeSetpoint(control_input);
    }

    void cleanupControlLoop() {
        if (control_timer_) {
            try {
                control_timer_->cancel();
                RCLCPP_INFO(get_logger(), "Control loop stopped.");
            } catch (const rclcpp::exceptions::RCLError& e) {
                RCLCPP_ERROR(get_logger(), "Error stopping control loop: %s", e.what());
            }
            control_timer_.reset();
        }
    }

    void ensureControlLoopRunning(int mode) {
        DroneState drone_state = state_manager_.getDroneState();
        if (!control_timer_ && drone_state.arming_state == ArmingState::ARMED) {
            control_timer_ = create_wall_timer(10ms, [this, mode]() { controlLoop(mode); });
            RCLCPP_INFO(get_logger(), "Control loop started with mode: %d", mode);
        } else if (!control_timer_) {
            RCLCPP_WARN(get_logger(), "Cannot start control loop: drone not armed.");
        }
    }

    void arm() {
        RCLCPP_INFO(get_logger(), "Sending arm command...");
        publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
    }

    void disarm(bool kill = false) {
        float param2 = kill ? 21196.0 : 0.0;
        RCLCPP_INFO(get_logger(), "Sending disarm command (kill=%d)...", kill);
        publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, param2);
    }

    void publishAttitudeSetpoint(const Eigen::Vector4d& input) {
        Eigen::Quaterniond q = transformations_.eulerToQuaternion(input.x(), input.y(), input.z());
        VehicleAttitudeSetpoint msg{};
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        msg.q_d = {static_cast<float>(q.w()), static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z())};
        msg.thrust_body = {0.0f, 0.0f, static_cast<float>(input.w())};
        attitude_setpoint_pub_->publish(msg);
    }

    void publishVehicleCommand(uint16_t command, float param1, float param2) {
        VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = get_clock()->now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);
    }

    rclcpp_action::GoalResponse handleDroneCommand(const rclcpp_action::GoalUUID& /*uuid*/,
                                                   std::shared_ptr<const DroneCommand::Goal> goal) {
        static const std::vector<std::string> allowed_commands = {"arm", "disarm", "takeoff", "goto", "land", "estop", "manual", "manual_aided"};
        RCLCPP_INFO(get_logger(), "Received goal request with command_type: %s", goal->command_type.c_str());

        DroneState drone_state = state_manager_.getDroneState();
        if (std::find(allowed_commands.begin(), allowed_commands.end(), goal->command_type) == allowed_commands.end()) {
            RCLCPP_WARN(get_logger(), "Rejected invalid command_type: %s", goal->command_type.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->command_type == "arm" && drone_state.arming_state == ArmingState::ARMED) {
            RCLCPP_WARN(get_logger(), "Rejected: drone already armed.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->command_type == "disarm" && drone_state.arming_state == ArmingState::DISARMED) {
            RCLCPP_WARN(get_logger(), "Rejected: drone already disarmed.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->command_type == "takeoff" && (goal->target_pose.size() != 1 || drone_state.arming_state != ArmingState::ARMED)) {
            RCLCPP_WARN(get_logger(), "Rejected: invalid takeoff parameters or drone not armed.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->command_type == "goto" && (goal->target_pose.size() != 3 || drone_state.arming_state != ArmingState::ARMED)) {
            RCLCPP_WARN(get_logger(), "Rejected: invalid goto parameters or drone not armed.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->command_type == "manual" && (drone_state.flight_mode == FlightMode::MANUAL || drone_state.arming_state != ArmingState::ARMED)) {
            RCLCPP_WARN(get_logger(), "Rejected: already in manual mode or drone not armed.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleDroneCommand> /*goal_handle*/) {
        RCLCPP_INFO(get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleDroneCommand> goal_handle) {
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
    }

    void execute(const std::shared_ptr<GoalHandleDroneCommand> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<DroneCommand::Result>();

        try {
            if (goal->command_type == "estop") {
                disarm(true);
                cleanupControlLoop();
                result->success = true;
                result->message = "Emergency stop executed.";
            } else if (goal->command_type == "arm") {
                arm();
                result->success = true;
                result->message = "Drone armed.";
            } else if (goal->command_type == "disarm") {
                disarm();
                cleanupControlLoop();
                result->success = true;
                result->message = "Drone disarmed.";
            } else if (goal->command_type == "takeoff") {
                ensureControlLoopRunning(2);
                Stamped4DVector target;
                target.setTime(now());
                target.setX(0.0);
                target.setY(0.0);
                target.setZ(std::max(goal->target_pose[0], -1.5));
                target.setW(goal->yaw);
                state_manager_.setTargetPositionProfile(target);
                result->success = true;
                result->message = "Drone taking off.";
            } else if (goal->command_type == "goto") {
                ensureControlLoopRunning(2);
                Stamped4DVector target;
                target.setTime(now());
                target.setX(goal->target_pose[0]);
                target.setY(goal->target_pose[1]);
                target.setZ(goal->target_pose[2]);
                target.setW(goal->yaw);
                state_manager_.setTargetPositionProfile(target);
                result->success = true;
                result->message = "Drone moving to target position.";
            } else if (goal->command_type == "manual") {
                cleanupControlLoop();
                ensureControlLoopRunning(0);
                result->success = true;
                result->message = "Drone in manual mode.";
            } else if (goal->command_type == "manual_aided") {
                ensureControlLoopRunning(1);
                result->success = true;
                result->message = "Drone in manual aided mode.";
            } else {
                result->success = false;
                result->message = "Invalid command.";
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Execution error: %s", e.what());
            result->success = false;
            result->message = "Execution failed.";
        }

        if (rclcpp::ok()) {
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal execution completed.");
        }
    }

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr bodyrate_setpoint_pub_;
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;

    rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr gps_sub_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<interfaces::msg::ManualControlInput>::SharedPtr manual_input_sub_;
    rclcpp::Subscription<SensorCombined>::SharedPtr sensor_combined_sub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr offboard_timer_;
    rclcpp_action::Server<DroneCommand>::SharedPtr drone_command_server_;

    FCI_Transformations transformations_;
    FCI_StateManager state_manager_;
    FCI_Controller controller_;

    PositionError prev_position_error_;
    AccelerationError prev_acceleration_error_;
    static constexpr float yaw_sensitivity_ = 1.0f / 20.0f;

    int offboard_setpoint_counter_;
    bool offboard_mode_set_;
    double timeout_threshold_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlightControllerInterface>());
    rclcpp::shutdown();
    return 0;
}