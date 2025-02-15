#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <interfaces/action/drone_command.hpp>
#include <interfaces/msg/manual_control_input.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include "fci_controller.h"
#include "fci_utilities.h"

// cd PX4-Autopilot/ && make px4_sitl gz_x500
// MicroXRCEAgent udp4 -p 8888
// cd drone-software && source install/setup.bash && ros2 run fc_interface fci
// Open qGround

// cd drone-software && source install/setup.bash
// 

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

std::vector<std::string> allowed_commands = {"arm", "disarm","takeoff", "goto", "land", "estop", "manual", "manual_aided"};


class FlightControllerInterface : public rclcpp::Node
{
public:
    using DroneCommand = interfaces::action::DroneCommand;
    using GoalHandleDroneCommand = rclcpp_action::ServerGoalHandle<DroneCommand>;


    FlightControllerInterface() : Node("flight_controller_interface")
    {
        
        // ROS2 QoS settings
        rclcpp::QoS qos_settings(10);
        qos_settings.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_settings.durability(rclcpp::DurabilityPolicy::TransientLocal);

        // Create Publisher
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        bodyrate_setpoint_publisher_ = this->create_publisher<VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint", 10);
        attitude_setpoint_publisher_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

        // Create Subscribers
        drone_state_subscriber_ = this->create_subscription<VehicleGlobalPosition>("/fmu/out/vehicle_global_position", qos_settings, std::bind(&FlightControllerInterface::GPSCallback, this, std::placeholders::_1));
        drone_attitude_subscriber_ = this->create_subscription<VehicleAttitude>("/fmu/out/vehicle_attitude", qos_settings, std::bind(&FlightControllerInterface::AttitudeCallback, this, std::placeholders::_1));
        drone_status_subscriber_ = this->create_subscription<VehicleStatus>("/fmu/out/vehicle_status", qos_settings, std::bind(&FlightControllerInterface::VehicleStatusCallback, this, std::placeholders::_1));
        drone_manual_input_subscriber_ = this->create_subscription<interfaces::msg::ManualControlInput>("drone/in/manual_input", qos_settings, std::bind(&FlightControllerInterface::ManualControlInputCallback, this, std::placeholders::_1));
    
        //drone_local_position_subscriber_ = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos_settings, std::bind(&FlightControllerInterface::PositionNEDCallback, this, std::placeholders::_1)); Removed, as the cube orange does not provide local position data. Only the simulation
    
        // Action server
        drone_command_action_server_ = rclcpp_action::create_server<interfaces::action::DroneCommand>(this,"/fmu/in/drone_command",
        std::bind(&FlightControllerInterface::handle_drone_cmd, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&FlightControllerInterface::handle_cancel, this, std::placeholders::_1),
        std::bind(&FlightControllerInterface::handle_accepted, this, std::placeholders::_1));

        // Timers
        offboard_timer_ = this->create_wall_timer(200ms, std::bind(&FlightControllerInterface::set_Offboardmode, this));

        // Parameters
        timeout_threshold_ = 0.2; // seconds
        offboard_mode_set_ = false;
        offboard_setpoint_counter_ = 0;

        // Node is now initialized
        RCLCPP_INFO(this->get_logger(), "ControlNode initialized.");
    }

private:
    // Callback functions
    void GPSCallback(const VehicleGlobalPosition::SharedPtr msg)
    {
        // Check if origin is set
        if (!Utils.isGPSOriginSet())
        {
            // Set the GPS origin
            Utils.setGPSOrigin(this->now(),msg->lat, msg->lon, msg->alt);
        }

        // Get NED position
        PositionNED ned_data = Utils.convertGPSToNED(this->now(),msg->lat, msg->lon, msg->alt);

        // Update global data with the latest NED position
        Utils.setPositionNED(ned_data);

    }

    void PositionNEDCallback(const VehicleLocalPosition::SharedPtr msg)
    {
        // Make a thread-safe copy of the data
        PositionNED position_data = {this->now(), msg->x, msg->y, msg->z};

        // Update global data with the latest local position
        //Utils.setPositionNED(position_data); It should not be used
        //RCLCPP_INFO(this->get_logger(), "Received local position data: x=%.2f, y=%.2f, z=%.2f", msg->x, msg->y, msg->z);
    }

    void AttitudeCallback(const VehicleAttitude::SharedPtr msg)
    {
        // Make a thread-safe copy of the data
        Attitude attitude_data = {this->now(), msg->q[0], msg->q[1], msg->q[2], msg->q[3]};

        // Update global data with the latest attitude
        Utils.setAttitude(attitude_data);
    }

    void VehicleStatusCallback(const VehicleStatus::SharedPtr msg)
    {
        // Get a copy of the current drone state
        DroneState drone_state = Utils.getDroneState();

        // Rewrite the received data
        drone_state.timestamp = this->now();
        drone_state.arming_state = (msg->arming_state == 2) ? ArmingState::ARMED : ArmingState::DISARMED;

        // Update global data with the latest drone state
        Utils.setDroneState(drone_state);
    }

    void ManualControlInputCallback(const interfaces::msg::ManualControlInput::SharedPtr msg)
    {
        //Read the manual control input
        ManualControlInput manual_control_input = Utils.getManualControlInput();

        manual_control_input = {this->now(), msg->roll, msg->pitch, manual_control_input.yaw + msg->yaw_velocity, msg->thrust};

        // Update global data with the latest manual control input
        Utils.setManualControlInput(manual_control_input);

    }

    // Control loop and other control logic
    void set_Offboardmode(){

        publish_offboard_control_mode();

        if(!offboard_mode_set_){

            if (offboard_setpoint_counter_ == 10) {
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                offboard_mode_set_ = true;
            }
            
            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        }
    }

    void publish_offboard_control_mode()
    {
        OffboardControlMode msg{};
        msg.position = false;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = true;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void controlLoop(){   
        // Implement some functions to determine the target position

        TargetPositionProfile target_position_profile = Utils.getTargetPositionProfile();
        std::vector<double> target_position = {target_position_profile.x, target_position_profile.y, target_position_profile.z};

        // Get the current position and attitude data
        PositionNED position_data = Utils.getPositionNED();
        std::vector<double> NED_position = {position_data.x, position_data.y, position_data.z};

        Attitude attitude_data = Utils.getAttitude();
        std::vector<double> attitude = {attitude_data.qw, attitude_data.qx, attitude_data.qy, attitude_data.qz};

        // Callculate the time since the last GPS message
        auto now = this->now();
        double dt = (now -  position_data.timestamp).seconds();

        // Check for timeout --Remove position_data.received_local_position_data_
        if ( dt > timeout_threshold_)
        {
            //Implement logic to handle missing data

            RCLCPP_WARN(this->get_logger(), "No data received in the last %.2f seconds!", timeout_threshold_);
            //received_local_position_data_ = false; // Reset flag to avoid repeated warnings

            return;
        }
       
        // Control loop
        std::vector<double> controller_values = Controller.PID_control(dt, previous_pose_error_, integral_pose_error_, NED_position, attitude, target_position);

        // set attiude setpoint
        publish_attitude_setpoint(controller_values[0], controller_values[1], 0.0, controller_values[2]);
        
    }

    void manualControlLoop()
    {
        // Check for connection to the manual control input

        // read the manual control input
        ManualControlInput manual_control_input = Utils.getManualControlInput();

        //print the manual control input
        RCLCPP_INFO(this->get_logger(), "Manual control input: roll: %.2f, pitch: %.2f, yaw: %.2f, thrust: %.2f", manual_control_input.roll, manual_control_input.pitch, manual_control_input.yaw, manual_control_input.thrust);

        // Publish the manual control input
        publish_attitude_setpoint(manual_control_input.roll, manual_control_input.pitch, manual_control_input.yaw, manual_control_input.thrust);
        
    }

    void manualAidedControlLoop()
    {
        // Implement manual aided control logic
    }

    void cleanUpControlLoops()
    {
       // Try to stop all control loops
        if (control_timer_) {
            try {
                control_timer_->cancel();
                RCLCPP_INFO(this->get_logger(), "Control loop successfully stopped.");
            } catch (const rclcpp::exceptions::RCLError &e) {
                RCLCPP_ERROR(this->get_logger(), "Error while stopping control loop: %s", e.what());
            }
        }

        if (manual_control_timer_) {
            try {
                manual_control_timer_->cancel();
                RCLCPP_INFO(this->get_logger(), "Manual control loop successfully stopped.");
            } catch (const rclcpp::exceptions::RCLError &e) {
                RCLCPP_ERROR(this->get_logger(), "Error while stopping manual control loop: %s", e.what());
            }
        }
    }

    // Helper functions 
    void ensureControlLoopRunning()
    {
       //Check if the control loop is running
        DroneState drone_state = Utils.getDroneState();
        if (drone_state.flight_mode != FlightMode::POSITION)
        {
            // Start the control loop
            control_timer_ = this->create_wall_timer(10ms, std::bind(&FlightControllerInterface::controlLoop, this));
            drone_state.flight_mode = FlightMode::POSITION;
            Utils.setDroneState(drone_state);
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Control loop is already running or the drone is not armed.");
        }
    }

    void ensureManualControlLoopRunning()
    {
        //Check if the manual control loop is running
        DroneState drone_state = Utils.getDroneState();
        if (drone_state.flight_mode != FlightMode::MANUAL)
        {
            // Start the manual control loop
            manual_control_timer_ = this->create_wall_timer(20ms, std::bind(&FlightControllerInterface::manualControlLoop, this));
            drone_state.flight_mode = FlightMode::MANUAL;
            Utils.setDroneState(drone_state);
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Manual control loop is already running or the drone is not armed.");
        }
    }
    


    // Commander functions
    void arm()
    {
        // Publish the arm command
        RCLCPP_INFO(this->get_logger(), "Sending arm command...");
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
    }

    void disarm(int command = 0)
    {
        // Publish the disarm command, command = 0 for disarm and 1 for kill
        double param2 = command == 1 ? 21196.0 : 0.0; 

        RCLCPP_INFO(this->get_logger(), "Sending disarm command...");
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, param2);

    }

    void publish_attitude_setpoint(double roll, double pitch, double yaw, double thrust)
    {
        //Source: https://docs.px4.io/main/en/msg_docs/VehicleAttitudeSetpoint.html

        // Convert the Euler angles to quaternions
        std::vector<double> q = Utils.euler_to_quaternion(roll, pitch, yaw);

        VehicleAttitudeSetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.q_d = {static_cast<float>(q[0]), static_cast<float>(q[1]), static_cast<float>(q[2]), static_cast<float>(q[3])};
        msg.thrust_body = {0.0, 0.0, static_cast<float>(thrust)};

        //msg.yaw_sp_move_rate = 0.0;
        //msg.reset_integral = false;
        //msg.fw_control_yaw_wheel = false;
        
        attitude_setpoint_publisher_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1, float param2)
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
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    // Action server functions - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
    rclcpp_action::GoalResponse handle_drone_cmd(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const DroneCommand::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with command_type: %s", goal->command_type.c_str());

        // Read state of the drone
        DroneState drone_state = Utils.getDroneState();

        // Validate command_type by checking if it is in the allowed_commands list
        if (std::find(allowed_commands.begin(), allowed_commands.end(), goal->command_type) == allowed_commands.end())
        {
            RCLCPP_WARN(this->get_logger(), "Rejected invalid command_type: %s", goal->command_type.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Validate if the drone is already armed
        if (goal->command_type == "arm")
        {
            if (drone_state.arming_state == ArmingState::ARMED)
            {
                RCLCPP_WARN(this->get_logger(), "Rejected goal: drone is already armed.");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        else if (goal->command_type == "disarm")
        {
            if (drone_state.arming_state == ArmingState::DISARMED)
            {
                RCLCPP_WARN(this->get_logger(), "Rejected goal: drone is already disarmed.");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        // If "takeoff", validate the pose and yaw
        if (goal->command_type == "takeoff")
        {
            if (goal->target_pose.size() != 1)
            {
                RCLCPP_WARN(this->get_logger(), "Rejected goal: target_pose must have exactly 1 element.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            else if (drone_state.arming_state != ArmingState::ARMED) // Drone is only armed when armed state = 2
            {
                RCLCPP_WARN(this->get_logger(), "Rejected goal: drone must be armed.");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        // If "goto", validate the pose and yaw
        if (goal->command_type == "goto")
        {
            if (goal->target_pose.size() != 3)
            {
                RCLCPP_WARN(this->get_logger(), "Rejected goal: target_pose must have exactly 3 elements.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            else if (drone_state.arming_state != ArmingState::ARMED) // Drone is only armed when armed state = 2
            {
                RCLCPP_WARN(this->get_logger(), "Rejected goal: drone must be armed.");
                return rclcpp_action::GoalResponse::REJECT;
            }
        }

        // If "manual"
        if (goal->command_type == "manual")
        {
            // print arming state
            RCLCPP_INFO(this->get_logger(), "Arming state: %d", drone_state.arming_state);

            if (drone_state.flight_mode == FlightMode::MANUAL)
            {
                RCLCPP_WARN(this->get_logger(), "Rejected goal: drone is already in manual mode.");
                return rclcpp_action::GoalResponse::REJECT;
            }
            else if (drone_state.arming_state != ArmingState::ARMED) // Drone is only armed when armed state = 2
            {   

                RCLCPP_WARN(this->get_logger(), "Rejected goal: drone must be armed. DO you get that.");
                return rclcpp_action::GoalResponse::REJECT;
            }

        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleDroneCommand> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleDroneCommand> goal_handle)
    {
        // Use a separate thread to execute the goal
        std::thread{std::bind(&FlightControllerInterface::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleDroneCommand> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing command");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<DroneCommand::Feedback>();
        auto result = std::make_shared<DroneCommand::Result>();

        // Get state of the drone
        DroneState drone_state = Utils.getDroneState();

        try
        {
            if (goal->command_type == "estop")
            {
                // disarm the drone
                disarm(1);

                // Ensure the control loop is stopped
                cleanUpControlLoops();

                result->success = true;
                result->message = "Drone disarmed successfully.";
            }

            else if (goal->command_type == "arm")
            {
                // arm the drone
                arm();

                result->success = true;
                result->message = "Drone armed successfully.";
            }
            else if (goal->command_type == "disarm")
            {
                // disarm the drone
                disarm();

                // Ensure the control loop is stopped
                cleanUpControlLoops();

                result->success = true;
                result->message = "Drone disarmed successfully.";
            }
            else if (goal->command_type == "takeoff")
            {
                // Ensure the control loop is running
                ensureControlLoopRunning();

                // Read the target pose and yaw
                float z = goal->target_pose[0];
                if (z > -1.5) z = -1.5;

                std::vector<double> target_pose = {0.0, 0.0, goal->target_pose[0]};
                double target_yaw = goal->yaw;

                // Set the target position profile in the global data space
                TargetPositionProfile target_position_profile = {this->now(), target_pose[0], target_pose[1], target_pose[2], target_yaw};
                Utils.setTargetPositionProfile(target_position_profile);

                result->success = true;
                result->message = "Drone is taking off.";
            }
            else if (goal->command_type == "goto")
            {
                // Ensure the control loop is running
                ensureControlLoopRunning();

                // Read the target pose and yaw
                std::vector<double> target_pose = {goal->target_pose[0], goal->target_pose[1], goal->target_pose[2]};
                double target_yaw = goal->yaw;

                // Set the target position profile in the global data space
                TargetPositionProfile target_position_profile = {this->now(), target_pose[0], target_pose[1], target_pose[2], target_yaw};
                Utils.setTargetPositionProfile(target_position_profile);
               
                result->success = true;
                result->message = "Drone is moving to the target position.";
            }
            else if (goal->command_type == "manual")
            {
                // Ensure the control loop is stopped
                cleanUpControlLoops();

                // Listen to topic for manual control
                ensureManualControlLoopRunning();

                result->success = true;
                result->message = "Drone is in manual mode.";
            }
            else if (goal->command_type == "manual_aided")
            {
                // Ensure the control loop is running
                ensureControlLoopRunning();

                result->success = true;
                result->message = "Drone is in manual mode with assistance.";
            }
            else
            {
                result->success = false;
                result->message = "Invalid command type.";
            }



        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception during goal execution: %s", e.what());
            result->success = false;
            result->message = "An error occurred during execution.";
        }

        // Mark goal as succeeded if ROS is still running
        if (rclcpp::ok())
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal execution completed.");
        }

       
    }

    // Member variables
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr bodyrate_setpoint_publisher_;
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr drone_state_subscriber_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr drone_attitude_subscriber_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr drone_local_position_subscriber_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr drone_status_subscriber_;
    rclcpp::Subscription<interfaces::msg::ManualControlInput>::SharedPtr drone_manual_input_subscriber_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr offboard_timer_;
    rclcpp::TimerBase::SharedPtr manual_control_timer_;
    rclcpp_action::Server<interfaces::action::DroneCommand>::SharedPtr drone_command_action_server_;


    //Class Init
    FCI_Controller Controller;
    FCI_Utilities Utils;

    // Mutex and variables for attitude data
    std::mutex attitude_data_mutex_;
    VehicleAttitude::SharedPtr last_attitude_msg_;
    rclcpp::Time last_attitude_msg_time_;
    bool received_attitude_data_;

    // Controller variables
    std::vector<double> previous_pose_error_ = {0.0, 0.0, 0.0};
    std::vector<double> integral_pose_error_ = {0.0, 0.0, 0.0};

    // Mode variables
    int offboard_setpoint_counter_;
    bool offboard_mode_set_;

    // Parameters
    double timeout_threshold_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FlightControllerInterface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
