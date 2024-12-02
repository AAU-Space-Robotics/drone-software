#include "fci_library.hpp"

FlightControllerInterface::FlightControllerInterface() : Node("flight_controller_interface")
{
    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    bodyrate_setpoint_publisher_ = this->create_publisher<VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    offboard_setpoint_counter_ = 0;

    auto timer_callback = [this]() -> void {
        if (offboard_setpoint_counter_ == 10) {
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            this->arm();
        }

        publish_offboard_control_mode();
        publish_bodyrate_setpoint();

        if (offboard_setpoint_counter_ < 11) {
            offboard_setpoint_counter_++;
        }
    };
    timer_ = this->create_wall_timer(100ms, timer_callback);
}


// ------ Commander

void FlightControllerInterface::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void FlightControllerInterface::disarm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void FlightControllerInterface::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void FlightControllerInterface::publish_trajectory_setpoint()
{
    TrajectorySetpoint msg{};
    msg.position = {0.0, 0.0, -5.0};
    msg.yaw = -3.14; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void FlightControllerInterface::publish_bodyrate_setpoint()
{
    VehicleRatesSetpoint msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.roll = 0.0;
    msg.pitch = 0.0;
    msg.yaw = 0.0;
    msg.thrust_body = {0.0, 0.0, -0.75};
    msg.reset_integral = false;
    bodyrate_setpoint_publisher_->publish(msg);
}

void FlightControllerInterface::publish_vehicle_command(uint16_t command, float param1, float param2)
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





"""
timestamp: 1733173194914018
connected: true
voltage_v: 15.300000190734863
current_a: -1.0
current_average_a: 0.01702156290411949
discharged_mah: 0.0
remaining: 0.5000267028808594
scale: 1.0588202476501465
time_remaining_s: .nan
temperature: .nan
cell_count: 4
source: 0
priority: 0
capacity: 0
cycle_count: 0
average_time_to_empty: 0
serial_number: 0
manufacture_date: 0
state_of_health: 0
max_error: 0
id: 1
interface_error: 0
voltage_cell_v:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
max_cell_voltage_delta: 0.0
is_powering_off: false
is_required: false
faults: 0
warning: 0
full_charge_capacity_wh: 0.0
remaining_capacity_wh: 0.0
over_discharge_count: 0
nominal_voltage: 0.0
internal_resistance_estimate: 0.004999999888241291
ocv_estimate: 15.279999732971191
ocv_estimate_filtered: 16.18000030517578
volt_based_soc_estimate: 0.9888887405395508
voltage_prediction: 0.0
prediction_error: 0.0
estimation_covariance_norm: 6.0133185386657715

"""