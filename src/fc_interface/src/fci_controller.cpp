#include "fci_controller.h"
#include <cmath>
#include <iostream>
#include <algorithm>

FCI_Controller::FCI_Controller(const FCI_Transformations& transformations) 
    : transformations_(transformations), position_pid_gains_(), velocity_pid_gains_() {}

void FCI_Controller::setPositionPIDGains(const PositionControllerGains& gains) {
    position_pid_gains_ = gains;
}
void FCI_Controller::setVelocityPIDGains(const VelocityControllerGains& gains) {
    velocity_pid_gains_ = gains;
}

// Generic PID control function for a single axis
double FCI_Controller::computePID(double error, double& previous_error, double& error_integral,
                                 double sample_time, const PIDGains& gains, 
                                 bool apply_anti_windup, double min_output, double max_output) const {
    // Calculate derivative error
    double error_derivative = (error - previous_error) / sample_time;

    // Update integral error
    error_integral += error * sample_time;

    // Calculate PID output
    double output = gains.Kp * error + gains.Ki * error_integral + gains.Kd * error_derivative;

    // Apply anti-windup if output is saturated
    if (apply_anti_windup && (output >= max_output || output <= min_output)) {
        error_integral -= error * sample_time; // Undo integral update
    }

    // Constrain output
    output = std::clamp(output, min_output, max_output);

    // Update previous error
    previous_error = error;

    return output;
}

// Position controller: Generates target velocities in FRD frame
Eigen::Vector3d FCI_Controller::positionControl(double sample_time,
                                               PositionError& previous_position_error,
                                               const Stamped3DVector& position_ned_earth,
                                               const StampedQuaternion& attitude,
                                               const Stamped3DVector& target_position_ned_earth,
                                               const double max_linear_velocity) {
    // Calculate position error in NED frame
    Eigen::Vector3d position_error_ned = target_position_ned_earth.vector() - position_ned_earth.vector();

    // Transform errors to FRD frame
    Eigen::Vector3d position_error_frd = transformations_.errorGlobalToLocal(position_error_ned, attitude.quaternion());

    // Calculate derivative errors
    Eigen::Vector3d position_error_ned_d = (position_error_ned - 
                                           Eigen::Vector3d(previous_position_error.X.error,
                                                           previous_position_error.Y.error,
                                                           previous_position_error.Z.error)) / sample_time;
    Eigen::Vector3d position_error_frd_d = transformations_.errorGlobalToLocal(position_error_ned_d, attitude.quaternion());

    // Compute target velocities using PID
    double target_velocity_x = computePID(
        position_error_frd.x(), previous_position_error.X.error, previous_position_error.X.error_integral,
        sample_time, position_pid_gains_.x, false, -max_linear_velocity, max_linear_velocity); // Forward velocity
    double target_velocity_y = computePID(
        position_error_frd.y(), previous_position_error.Y.error, previous_position_error.Y.error_integral,
        sample_time, position_pid_gains_.y, false, -max_linear_velocity, max_linear_velocity);  // Lateral velocity
    double target_velocity_z = computePID(
        position_error_frd.z(), previous_position_error.Z.error, previous_position_error.Z.error_integral,
        sample_time, position_pid_gains_.z, true, -max_linear_velocity, -max_linear_velocity); // Vertical velocity

    // Return target velocities in FRD frame
    return {target_velocity_x, target_velocity_y, target_velocity_z};
}

// Velocity controller: Generates roll, pitch, thrust from velocity errors
Eigen::Vector4d FCI_Controller::velocityControl(double sample_time,
                                               VelocityError& previous_velocity_error,
                                               const Stamped3DVector& velocity_frd,
                                               const Eigen::Vector3d& target_velocity_frd,
                                               const double max_angle_radians,
                                               const double max_thrust,
                                               const double min_thrust) {
    // Calculate velocity error in FRD frame
    Eigen::Vector3d velocity_error_frd = target_velocity_frd - velocity_frd.vector();

    // Compute control outputs using PID
    double roll = computePID(
        velocity_error_frd.y(), previous_velocity_error.Y.error, previous_velocity_error.Y.error_integral,
        sample_time, velocity_pid_gains_.vel_x, true, -max_angle_radians, max_angle_radians); // Lateral velocity -> roll
    double pitch = computePID(
        velocity_error_frd.x(), previous_velocity_error.X.error, previous_velocity_error.X.error_integral,
        sample_time, velocity_pid_gains_.vel_y, true, -max_angle_radians, max_angle_radians); // Forward velocity -> pitch
    double thrust = computePID(
        velocity_error_frd.z(), previous_velocity_error.Z.error, previous_velocity_error.Z.error_integral,
        sample_time, velocity_pid_gains_.vel_z, true, max_thrust, min_thrust); // Vertical velocity -> thrust

    // Return control outputs (roll, pitch, yaw, thrust)
    return {roll, -pitch, 0.0, thrust};
} //M_PI / 18.0

double FCI_Controller::EMA_filter(double new_value, double previous_value) const {
    return ema_filter_alpha_ * previous_value + (1.0f - ema_filter_alpha_) * new_value;
}

double FCI_Controller::mapNormToAngle(double norm) const {
    constexpr double max_angle = M_PI / 18.0; // ~10 degrees
    return norm * max_angle;
}