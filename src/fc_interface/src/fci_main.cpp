#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <stdint.h>
#include <cmath>
#include <vector>
#include <mutex>

#include "fci_controller.h"
#include "fci_utilities.h"

// cd PX4-Autopilot/ && make px4_sitl gz_x500
// MicroXRCEAgent udp4 -p 8888

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class FlightControllerInterface : public rclcpp::Node
{
public:
    FlightControllerInterface() : Node("flight_controller_interface"), last_gps_msg_time_(this->now()), received_gps_data_(false)
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

        drone_state_subscriber_ = this->create_subscription<VehicleGlobalPosition>("/fmu/out/vehicle_global_position", qos_settings, std::bind(&FlightControllerInterface::GPSCallback, this, std::placeholders::_1));
        drone_attitude_subscriber_ = this->create_subscription<VehicleAttitude>("/fmu/out/vehicle_attitude", qos_settings, std::bind(&FlightControllerInterface::AttitudeCallback, this, std::placeholders::_1));
        drone_local_position_subscriber_ = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos_settings, std::bind(&FlightControllerInterface::LocalPositionCallback, this, std::placeholders::_1));

        // Timers
        control_timer_ = this->create_wall_timer(10ms, std::bind(&FlightControllerInterface::controlLoop, this));
        offboard_timer_ = this->create_wall_timer(200ms, std::bind(&FlightControllerInterface::set_Offboardmode, this));

        // Parameters
        timeout_threshold_ = 0.2; // seconds
        origin_set_ = false;
        offboard_mode_set_ = false;
        offboard_setpoint_counter_ = 0;

        // Node is now initialized
        RCLCPP_INFO(this->get_logger(), "ControlNode initialized.");
    }

private:
    // Callback functions
    void GPSCallback(const VehicleGlobalPosition::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(gps_data_mutex_);
        last_gps_msg_ = msg;
        last_gps_msg_time_ = this->now();
        received_gps_data_ = true;
    }

    void LocalPositionCallback(const VehicleLocalPosition::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(local_position_data_mutex_);
        last_local_position_msg_ = msg;
        last_local_position_msg_time_ = this->now();
        received_local_position_data_ = true;
    }

    void AttitudeCallback(const VehicleAttitude::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(attitude_data_mutex_);
        last_attitude_msg_ = msg;
        last_attitude_msg_time_ = this->now();
        received_attitude_data_ = true;
    }

    // Control loop and other control logic
    void set_Offboardmode(){

        publish_offboard_control_mode();

        if(!offboard_mode_set_){

            if (offboard_setpoint_counter_ == 10) {
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                offboard_mode_set_ = true;
                // Arm
                arm();
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

    void controlLoop()
    {
        // Lock the mutex to read the GPS data - - - - - - - - - - - - - - - - - - - - - - - - - - -
        std::unique_lock<std::mutex> lock_position(local_position_data_mutex_);

        // Callculate the time since the last GPS message
        auto now = this->now();
        double dt = (now -  last_local_position_msg_time_).seconds();

        // Check for timeout
        if (!received_local_position_data_ && dt > timeout_threshold_)
        {
            //Implement logic to handle missing data

            RCLCPP_WARN(this->get_logger(), "No data received in the last %.2f seconds!", timeout_threshold_);
            received_local_position_data_ = false; // Reset flag to avoid repeated warnings

            return;
        }
        else if (!received_local_position_data_)
        {
            // No data received, but not yet timed out
            RCLCPP_WARN(this->get_logger(), "Skipping control computation due to missing data.");
            return;

        }

        // Read GPS data
        std::vector<double> NED_position = {last_local_position_msg_->x, last_local_position_msg_->y, last_local_position_msg_->z};

        // Unlock the mutex - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        lock_position.unlock();

        // Set the target pose
        std::vector<double> target_pose = {0.0, 0.0, -10.0};

        // Read attitude data - lock the mutex - - - - - - - - -
        std::unique_lock<std::mutex> lock_attitude(attitude_data_mutex_);
        std::vector<double> attitude = {last_attitude_msg_->q[0], last_attitude_msg_->q[1], last_attitude_msg_->q[2], last_attitude_msg_->q[3]};
        lock_attitude.unlock();
        // Unlock the mutex - - - - - - - - - - - - - - - - - - -


        // Control loop
        std::vector<double> controller_values = Controller.PID_control(dt, previous_pose_error_, NED_position, attitude, target_pose);

        // set attiude setpoint
        publish_attitude_setpoint(controller_values[0], controller_values[1], 0.0, controller_values[2]);


        // Clean up - - - Reset the flag for the next iteration - - - - - - - - - - - - - 

        received_gps_data_ = false; // Reset flag for next iteration
    }

    
    // Commander functions
    void arm(){
        // Publish the command to arm the vehicle, by setting the param1 to 1.0
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
        RCLCPP_INFO(this->get_logger(), "Arm command sent");
    }

    void disarm()
    {
        // Publish the command to disarm the vehicle, by setting the param1 to 0.0
        publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
        RCLCPP_INFO(this->get_logger(), "Disarm command sent");
    }

    void publish_attitude_setpoint(double roll, double pitch, double yaw, double thrust)
    {
        //Source: https://docs.px4.io/main/en/msg_docs/VehicleAttitudeSetpoint.html

        // Convert the Euler angles to quaternions
        std::vector<double> q = Utils.euler_to_quaternion(roll, pitch, yaw);

        VehicleAttitudeSetpoint msg{};
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        msg.q_d = {q[0], q[1], q[2], q[3]};
        msg.thrust_body = {0.0, 0.0, thrust};

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

    // Member variables
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr bodyrate_setpoint_publisher_;
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr drone_state_subscriber_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr drone_attitude_subscriber_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr drone_local_position_subscriber_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr offboard_timer_;

    //Class Init
    FCI_Controller Controller;
    FCI_Utilities Utils;

    // Mutex and variables for GPS data
    std::mutex gps_data_mutex_;
    VehicleGlobalPosition::SharedPtr last_gps_msg_;
    rclcpp::Time last_gps_msg_time_;
    bool received_gps_data_;
    bool origin_set_;
    std::vector<double> origin_position_;

    // Mutex and variables for attitude data
    std::mutex attitude_data_mutex_;
    VehicleAttitude::SharedPtr last_attitude_msg_;
    rclcpp::Time last_attitude_msg_time_;
    bool received_attitude_data_;

    // Mutex and variables for local position data
    std::mutex local_position_data_mutex_;
    VehicleLocalPosition::SharedPtr last_local_position_msg_;
    rclcpp::Time last_local_position_msg_time_;
    bool received_local_position_data_;

    // Controller variables
    std::vector<double> previous_pose_error_ = {0.0, 0.0, 0.0};

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
