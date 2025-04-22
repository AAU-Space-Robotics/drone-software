#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

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


#include <interfaces/action/drone_command.hpp>
#include <interfaces/msg/manual_control_input.hpp>
#include <interfaces/msg/motion_capture_pose.hpp>
#include <interfaces/msg/drone_state.hpp>
#include <interfaces/msg/drone_scope.hpp>

#include "fci_controller.h"
#include "fci_state_manager.h"
#include "fci_transformations.h"
#include "fci_path_planner.h"

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class FlightControllerInterface : public rclcpp::Node
{
public:
    using DroneCommand = interfaces::action::DroneCommand;
    using GoalHandleDroneCommand = rclcpp_action::ServerGoalHandle<DroneCommand>;

    FlightControllerInterface()
        : Node("flight_controller_interface"),
          controller_(transformations_),
          offboard_setpoint_counter_(0),
          offboard_mode_set_(false),
          timeout_threshold_(0.2)
    {
        // ROS 2 QoS settings
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::TransientLocal);

        // Determine time source at initialization
        bool use_sim_time = false;
        this->get_parameter_or("use_sim_time", use_sim_time, false);
        clock_ = std::make_shared<rclcpp::Clock>(use_sim_time ? RCL_ROS_TIME : RCL_SYSTEM_TIME);
        RCLCPP_INFO(get_logger(), "Using %s time source", use_sim_time ? "simulation" : "system");

        // Declare position source parameter
        std::string position_source = "px4";
        this->declare_parameter("position_source", "px4");
        this->get_parameter("position_source", position_source);
        RCLCPP_INFO(get_logger(), "Using %s position source", position_source.c_str());

        // Set initial state
        state_manager_.setGlobalPosition(Stamped3DVector(get_time(), 0.0, 0.0, 0.0));
        state_manager_.setGlobalVelocity(Stamped3DVector(get_time(), 0.0, 0.0, 0.0));
        state_manager_.setTargetPositionProfile(Stamped4DVector(get_time(), 0.0, 0.0, 0.0, 0.0));

        // Publishers
        offboard_control_mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        attitude_setpoint_pub_ = create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
        vehicle_command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
        drone_state_pub_ = create_publisher<interfaces::msg::DroneState>("drone/out/drone_state", 10);

        // Subscribers
        if (position_source == "px4"){
            local_position_sub_ = create_subscription<VehicleLocalPosition>(
                "/fmu/out/vehicle_local_position", qos,
                [this](const VehicleLocalPosition::SharedPtr msg) { localPositionCallback(msg); });
        }
        else if (position_source == "mocap"){
            motion_capture_local_position_sub_ = create_subscription<interfaces::msg::MotionCapturePose>(
                "drone/in/motion_capture_pose", qos,
                [this](const interfaces::msg::MotionCapturePose::SharedPtr msg)
                { motionCaptureLocalPositionCallback(msg); });
        }

        attitude_sub_ = create_subscription<VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            [this](const VehicleAttitude::SharedPtr msg)
            { attitudeCallback(msg); });
        status_sub_ = create_subscription<VehicleStatus>(
            "/fmu/out/vehicle_status", qos,
            [this](const VehicleStatus::SharedPtr msg)
            { vehicleStatusCallback(msg); });
        manual_input_sub_ = create_subscription<interfaces::msg::ManualControlInput>(
            "drone/in/manual_input", qos,
            [this](const interfaces::msg::ManualControlInput::SharedPtr msg)
            { manualControlInputCallback(msg); });
        battery_status_sub_ = create_subscription<BatteryStatus>(
            "/fmu/out/battery_status", qos,
            [this](const BatteryStatus::SharedPtr msg)
            { batteryStatusCallback(msg); });


        // Action server
        drone_command_server_ = rclcpp_action::create_server<DroneCommand>(
            this, "/fmu/in/drone_command",
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
        offboard_timer_ = create_wall_timer(200ms, [this]()
                                            { setOffboardMode(); });

        RCLCPP_INFO(get_logger(), "FlightControllerInterface initialized.");

        drone_state_timer = create_wall_timer(100ms, [this]()
                                            { publish_drone_state(); });
    }

    rclcpp::Time get_time() const {
        return clock_->now();
    }

private:
    
    void localPositionCallback(const VehicleLocalPosition::SharedPtr msg)
    {
        // Note that local position refers to coordinates being expressed in cartesian coordinates
        Stamped3DVector local_position(get_time(), msg->x, msg->y, msg->z);
        state_manager_.setGlobalPosition(local_position);

        // Set the velocity in the state manager
        Stamped3DVector local_velocity(get_time(), msg->vx, msg->vy, msg->vz);
        state_manager_.setGlobalVelocity(local_velocity);

        // set the acceleration in the state manager
        Stamped3DVector local_acceleration(get_time(), msg->ax, msg->ay, msg->az);
        //Stamped3DVector local_acceleration_global = transformations_.accelerationLocalToGlobal(get_time(), state_manager_.getAttitude().quaternion(), local_acceleration);
        //local_acceleration_global.vector().z() += 9.81; // Remove gravity
        state_manager_.setGlobalAcceleration(local_acceleration);
        
    }

    void motionCaptureLocalPositionCallback(const interfaces::msg::MotionCapturePose::SharedPtr msg)
    {
        Stamped3DVector local_position(get_time(), msg->x, msg->y, msg->z);

        // RCLCPP_INFO(get_logger(), "Motion capture position: x=%.2f, y=%.2f, z=%.2f", local_position.x(), local_position.y(), local_position.z());

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
        drone_state.timestamp = get_time();
        drone_state.arming_state = (msg->arming_state == 2) ? ArmingState::ARMED : ArmingState::DISARMED;
        state_manager_.setDroneState(drone_state);
    }

    void manualControlInputCallback(const interfaces::msg::ManualControlInput::SharedPtr msg)
    {
        Stamped4DVector manual_input = state_manager_.getManualControlInput();
        manual_input.setTime(get_time());
        manual_input.setX(controller_.mapNormToAngle(msg->roll));
        manual_input.setY(controller_.mapNormToAngle(msg->pitch));
        manual_input.setZ(manual_input.z() + controller_.mapNormToAngle(msg->yaw_velocity * yaw_sensitivity_));
        manual_input.setW(msg->thrust);
        state_manager_.setManualControlInput(manual_input);
    }

    // Drone state publisher
    void publish_drone_state()
    {
        interfaces::msg::DroneState msg{};

        msg.timestamp = get_time().seconds();
        //int8 id
        //int8 mode

        //Get drone state
        Stamped3DVector position = state_manager_.getGlobalPosition();
        msg.position.resize(3);
        msg.position[0] = position.x(); 
        msg.position[1] = position.y(); 
        msg.position[2] = position.z();
        //float32[] velocity #x, y, z
        //float32[] orientation  #roll, pitch, yaw
        //float32[] target_position #x, y, z
        //float32[] acceleration
        msg.battery_voltage = state_manager_.getBatteryState().voltage;
        
        //float32 battery_percentage  # 0.0 to 100.0
        //uint8 arming_state           
        //uint8 estop     

        drone_state_pub_->publish(msg);
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
        Stamped4DVector manual_input = state_manager_.getManualControlInput();
        return {manual_input.x(), manual_input.y(), manual_input.z(), manual_input.w()};
    }

    Eigen::Vector4d positionMode()
    {

        if (is_trajectory_active_) {
            double dt = (get_time() - trajectory_start_time_).seconds();
            if (dt > path_planner_.getTotalTime()) {
                is_trajectory_active_ = false;
                Vector3d final_position = path_planner_.getTrajectoryPoint(path_planner_.getTotalTime(), trajectoryMethod::MIN_SNAP);
                Stamped4DVector target_profile(get_time(), final_position.x(), final_position.y(), final_position.z(), 0.0);
                state_manager_.setTargetPositionProfile(target_profile);
            }
            Vector3d target_position = path_planner_.getTrajectoryPoint(dt, trajectoryMethod::MIN_SNAP);
            Stamped4DVector target_profile(get_time(), target_position.x(), target_position.y(), target_position.z(), 0.0);
            state_manager_.setTargetPositionProfile(target_profile);
        }

        Stamped4DVector target_profile = state_manager_.getTargetPositionProfile();
        Stamped3DVector position = state_manager_.getGlobalPosition();
        StampedQuaternion attitude = state_manager_.getAttitude();
        Stamped3DVector target_position_3d(target_profile.timestamp, target_profile.vector().x(), target_profile.vector().y(), target_profile.vector().z());

        // Calculate global position error in NED frame
        Eigen::Vector3d global_error_ned = target_position_3d.vector() - position.vector();
    
        double dt = (get_time() - position.getTime()).seconds();
        if (dt > timeout_threshold_)
        {
            RCLCPP_WARN(get_logger(), "No position data received in the last %.2f seconds!", timeout_threshold_);
            return Eigen::Vector4d::Zero();
        }

        Eigen::Vector4d output = controller_.pidControl(dt, prev_position_error_, position, attitude, target_position_3d);

        // Get velocity from state manager and print it 
        Stamped3DVector velocity = state_manager_.getGlobalVelocity();
        RCLCPP_INFO(get_logger(), "Velocity: x=%.2f, y=%.2f, z=%.2f", velocity.x(), velocity.y(), velocity.z());
        // Get acceleration from state manager and print it
        Stamped3DVector acceleration = state_manager_.getGlobalAcceleration();
        RCLCPP_INFO(get_logger(), "Acceleration: x=%.2f, y=%.2f, z=%.2f", acceleration.x(), acceleration.y(), acceleration.z());

        return output;
    }

    Eigen::Vector4d controlModeACCELTEST()
    {
        Stamped4DVector target_profile = state_manager_.getTargetPositionProfile();
        Stamped3DVector acceleration = state_manager_.getGlobalAcceleration();
        StampedQuaternion attitude = state_manager_.getAttitude();
        Stamped3DVector target_position_3d(target_profile.timestamp, target_profile.vector().x(), target_profile.vector().y(), target_profile.vector().z());

        // Calculate global position error in NED frame
        Eigen::Vector3d global_error_ned = target_position_3d.vector() - acceleration.vector();

        double dt = (get_time() - acceleration.getTime()).seconds();
        if (dt > timeout_threshold_)
        {
            RCLCPP_WARN(get_logger(), "No position data received in the last %.2f seconds!", timeout_threshold_);
            return Eigen::Vector4d::Zero();
        }

        Eigen::Vector4d output = controller_.accelerationControl(dt, prev_acceleration_error_, acceleration, target_position_3d);
        return output;
    }

    Eigen::Vector4d manualAidedMode()
    {
        Stamped4DVector manual_input = state_manager_.getManualControlInput();
        Stamped4DVector target_profile = state_manager_.getTargetPositionProfile();
        target_profile.setZ(target_profile.z() + manual_input.w() / 10.0);
        state_manager_.setTargetPositionProfile(target_profile);
        Stamped3DVector position = state_manager_.getGlobalPosition();
        StampedQuaternion attitude = state_manager_.getAttitude();
        Stamped3DVector target_position_3d(target_profile.timestamp, target_profile.vector().x(), target_profile.vector().y(), target_profile.vector().z());

        double dt = (get_time() - position.getTime()).seconds();
        if (dt > timeout_threshold_)
        {
            RCLCPP_WARN(get_logger(), "No position data received in the last %.2f seconds!", timeout_threshold_);
            return Eigen::Vector4d::Zero();
        }

        Eigen::Vector4d output = controller_.pidControl(dt, prev_position_error_, position, attitude, target_position_3d);
        return {manual_input.x(), manual_input.y(), manual_input.z(), output.w()};
    }

    void controlLoop(int mode)
    {
        Eigen::Vector4d control_input;
        switch (mode)
        {
        case 0:
            control_input = manualMode();
            break;
        case 1:
            control_input = manualAidedMode();
            break;
        case 2:
            control_input = positionMode();
            break;
        default:
            RCLCPP_WARN(get_logger(), "Unknown control mode: %d", mode);
            control_input = Eigen::Vector4d::Zero();
            break;
        }
        // RCLCPP_INFO(get_logger(), "Control input: roll=%.2f, pitch=%.2f, yaw=%.2f, thrust=%.2f",
        //              control_input.x(), control_input.y(), control_input.z(), control_input.w());

        publishAttitudeSetpoint(control_input);
    }

    void cleanupControlLoop()
    {
        if (control_timer_)
        {
            try
            {
                control_timer_->cancel();
                RCLCPP_INFO(get_logger(), "Control loop stopped.");
            }
            catch (const rclcpp::exceptions::RCLError &e)
            {
                RCLCPP_ERROR(get_logger(), "Error stopping control loop: %s", e.what());
            }
            control_timer_.reset();
        }
    }

    void ensureControlLoopRunning(int mode)
    {
        DroneState drone_state = state_manager_.getDroneState();
        if (!control_timer_ && drone_state.arming_state == ArmingState::ARMED)
        {
            control_timer_ = create_wall_timer(10ms, [this, mode]()
                                               { controlLoop(mode); });
            RCLCPP_INFO(get_logger(), "Control loop started with mode: %d", mode);
        }
        else if (!control_timer_)
        {
            RCLCPP_WARN(get_logger(), "Cannot start control loop: drone not armed.");
        }
    }

    // Command functions
    void arm()
    {
        RCLCPP_INFO(get_logger(), "Sending arm command...");
        publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
    }

    void disarm(bool kill = false)
    {
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
        static const std::vector<std::string> allowed_commands = {"arm", "disarm", "takeoff", "goto", "land", "estop", "manual", "manual_aided"};
        RCLCPP_INFO(get_logger(), "Received goal request with command_type: %s", goal->command_type.c_str());

        DroneState drone_state = state_manager_.getDroneState();
        if (std::find(allowed_commands.begin(), allowed_commands.end(), goal->command_type) == allowed_commands.end())
        {
            RCLCPP_WARN(get_logger(), "Rejected invalid command_type: %s", goal->command_type.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (goal->command_type == "arm" && drone_state.arming_state == ArmingState::ARMED)
        {
            RCLCPP_WARN(get_logger(), "Rejected: drone already armed.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->command_type == "disarm" && drone_state.arming_state == ArmingState::DISARMED)
        {
            RCLCPP_WARN(get_logger(), "Rejected: drone already disarmed.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->command_type == "takeoff" && (goal->target_pose.size() != 1 || drone_state.arming_state != ArmingState::ARMED))
        {
            RCLCPP_WARN(get_logger(), "Rejected: invalid takeoff parameters or drone not armed.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->command_type == "goto" && (goal->target_pose.size() != 3 || drone_state.arming_state != ArmingState::ARMED))
        {
            RCLCPP_WARN(get_logger(), "Rejected: invalid goto parameters or drone not armed.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->command_type == "manual" && (drone_state.flight_mode == FlightMode::MANUAL || drone_state.arming_state != ArmingState::ARMED))
        {
            RCLCPP_WARN(get_logger(), "Rejected: already in manual mode or drone not armed.");
            return rclcpp_action::GoalResponse::REJECT;
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
        std::thread([this, goal_handle]()
                    { execute(goal_handle); })
            .detach();
    }

    void execute(const std::shared_ptr<GoalHandleDroneCommand> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<DroneCommand::Result>();

        try
        {
            if (goal->command_type == "estop")
            {
                disarm(true);
                cleanupControlLoop();
                result->success = true;
                result->message = "Emergency stop executed.";
            }
            else if (goal->command_type == "arm")
            {

                // Set the current target_position, to the current position
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
            else if (goal->command_type == "disarm")
            {
                disarm();
                cleanupControlLoop();
                result->success = true;
                result->message = "Drone disarmed.";
            }
            else if (goal->command_type == "takeoff")
            {
                ensureControlLoopRunning(2);
                
                // Set the takeoff position, to the current target to handle ssteady state errors by mitigating, free fall
                Stamped4DVector target_profile = state_manager_.getTargetPositionProfile();
                Vector3d takeoff_position = {target_profile.x(), target_profile.y(), target_profile.z()};
                // Set the target takeoff goal, based on the current position. Should at least be 1.5m above the current position
                Vector3d target_position = {takeoff_position.x(), takeoff_position.y(), std::min(goal->target_pose[0], -1.5)};
                Vector3d current_velocity = {0.0, 0.0, 0.0};
                Vector3d current_acceleration = {0.0, 0.0, 0.0};
                float takeoff_time = 5.0;

                // Generate takeoff trajectory
                path_planner_.GenerateTrajectory(takeoff_position, target_position, current_velocity, current_acceleration, takeoff_time, trajectoryMethod::MIN_SNAP);

                // set trajectory start time
                trajectory_start_time_ = get_time();
                is_trajectory_active_ = true;

                result->success = true;
                result->message = "Drone taking off.";
                std::cout << "Takeoff target: " << state_manager_.getTargetPositionProfile().vector().transpose() << std::endl;
            }
            else if (goal->command_type == "goto")
            {
                ensureControlLoopRunning(2);
                // Set the takeoff position, to the current target to handle ssteady state errors by mitigating, free fall

                Stamped4DVector target_profile = state_manager_.getTargetPositionProfile();
                Vector3d takeoff_position = {target_profile.x(), target_profile.y(), target_profile.z()};
                
                Vector3d target_position = {goal->target_pose[0], goal->target_pose[1], goal->target_pose[2]};
                Vector3d current_velocity = {0.0, 0.0, 0.0};
                Vector3d current_acceleration = {0.0, 0.0, 0.0};
                float takeoff_time = 5.0;

                // Generate trajectory
                path_planner_.GenerateTrajectory(takeoff_position, target_position, current_velocity, current_acceleration, takeoff_time, trajectoryMethod::MIN_SNAP);

                // set trajectory start time
                trajectory_start_time_ = get_time();
                is_trajectory_active_ = true;

                result->success = true;
                result->message = "Drone moving to target position.";
                std::cout << "Goto target: " << state_manager_.getTargetPositionProfile().vector().transpose() << std::endl;
            }
            else if (goal->command_type == "manual")
            {
                cleanupControlLoop();
                ensureControlLoopRunning(0);
                result->success = true;
                result->message = "Drone in manual mode.";
            }
            else if (goal->command_type == "manual_aided")
            {
                ensureControlLoopRunning(1);
                result->success = true;
                result->message = "Drone in manual aided mode.";
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
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<interfaces::msg::DroneState>::SharedPtr drone_state_pub_;

    // rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr gps_sub_;
    rclcpp::Subscription<interfaces::msg::MotionCapturePose>::SharedPtr motion_capture_local_position_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<interfaces::msg::ManualControlInput>::SharedPtr manual_input_sub_;
    rclcpp::Subscription<BatteryStatus>::SharedPtr battery_status_sub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr offboard_timer_;
    rclcpp::TimerBase::SharedPtr drone_state_timer;
    rclcpp_action::Server<DroneCommand>::SharedPtr drone_command_server_;

    FCI_Transformations transformations_;
    FCI_StateManager state_manager_;
    FCI_Controller controller_;
    FCI_PathPlanner path_planner_;

    PositionError prev_position_error_;
    AccelerationError prev_acceleration_error_;
    static constexpr float yaw_sensitivity_ = 1.0f / 20.0f;

    int offboard_setpoint_counter_;
    bool offboard_mode_set_;
    double timeout_threshold_;

    bool is_trajectory_active_ = false;
    rclcpp::Time trajectory_start_time_;
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlightControllerInterface>());
    rclcpp::shutdown();
    return 0;
}