#include "controller.h"
#include <cmath>
#include <iostream>

Controller::Controller(const Transformations& transformations) 
    : transformations_(transformations), attitude_pid_gains_() {}

void Controller::setPIDGains(const PIDControllerGains& gains) {
    attitude_pid_gains_ = gains;
}


void Controller::setPositionPIDGains(const PIDPosControllerGains& gains) {
    position_pid_gains_ = gains;
}


Eigen::Vector3d Controller::positionControl(double sample_time,
                                           PositionError& previous_position_error,
                                           const Stamped3DVector& position_ned_earth,
                                           const StampedQuaternion& attitude,
                                           const Stamped3DVector& target_position_ned_earth,
                                           const Eigen::Vector3d& /*previous_control_signal*/) 
{
    // Position error in NED
    Eigen::Vector3d position_error_ned = target_position_ned_earth.vector() - position_ned_earth.vector();

    // Derivative (guard against zero sample time)
    Eigen::Vector3d position_error_ned_d = Eigen::Vector3d::Zero();
    if (sample_time > 0.0) {
        position_error_ned_d = (position_error_ned -
                                Eigen::Vector3d(previous_position_error.X.error,
                                                previous_position_error.Y.error,
                                                previous_position_error.Z.error)) / sample_time;
    }

    // Transform errors to FRD frame
    Eigen::Vector3d position_error_frd = transformations_.errorGlobalToLocal(position_error_ned, attitude.quaternion());
    Eigen::Vector3d position_error_frd_d = transformations_.errorGlobalToLocal(position_error_ned_d, attitude.quaternion());

    // PD controller -> produce desired velocities (vx, vy, vz) in FRD frame
    double vx = position_pid_gains_.x.Kp * position_error_frd.x()
                + position_pid_gains_.x.Kd * position_error_frd_d.x();

    double vy = position_pid_gains_.y.Kp * position_error_frd.y()
                + position_pid_gains_.y.Kd * position_error_frd_d.y();

    double vz = position_pid_gains_.z.Kp * position_error_frd.z()
                + position_pid_gains_.z.Kd * position_error_frd_d.z();
  
    // Update previous position error (NED) for next derivative computation
    previous_position_error.X.error = position_error_ned.x();
    previous_position_error.Y.error = position_error_ned.y();
    previous_position_error.Z.error = position_error_ned.z();

    // Saturate velocities
    vx = std::clamp(vx, -max_horizontal_velocity_, max_horizontal_velocity_);
    vy = std::clamp(vy, -max_horizontal_velocity_, max_horizontal_velocity_);
    vz = std::clamp(vz, -max_vertical_velocity_, max_vertical_velocity_);
    
    return Eigen::Vector3d(vx, vy, vz);
}


Eigen::Vector4d Controller::velocityControl(double sample_time,
                                                VelocityError& previous_velocity_error,
                                                const Stamped3DVector& velocity_ned_earth,
                                                const StampedQuaternion& attitude,
                                                const Stamped3DVector& target_velocity_ned_earth,
                                                const Eigen::Vector4d& /*previous_control_signal*/) {

    Eigen::Vector3d velocity_error_ned = target_velocity_ned_earth.vector() - velocity_ned_earth.vector();
    Eigen::Vector3d velocity_error_frd = transformations_.errorGlobalToLocal(velocity_error_ned, attitude.quaternion());

    // Derivative computed in FRD frame using previous FRD error (prevents discontinuity during rotation)
    Eigen::Vector3d velocity_error_frd_d = Eigen::Vector3d::Zero();
    if (sample_time > 0.0) {
        velocity_error_frd_d = (velocity_error_frd - Eigen::Vector3d(previous_velocity_error.X.error,
                                                                      previous_velocity_error.Y.error,
                                                                      previous_velocity_error.Z.error)) / sample_time;
    }

    Eigen::Vector3d integral_velocity_error_frd(
        previous_velocity_error.X.error_integral,
        previous_velocity_error.Y.error_integral,
        previous_velocity_error.Z.error_integral);

    // Calculate unsaturated control outputs
    double roll_cmd = attitude_pid_gains_.roll.Kp * velocity_error_frd.y() +
                      attitude_pid_gains_.roll.Ki * integral_velocity_error_frd.y() +
                      attitude_pid_gains_.roll.Kd * velocity_error_frd_d.y();

    double pitch_cmd = attitude_pid_gains_.pitch.Kp * velocity_error_frd.x() +
                       attitude_pid_gains_.pitch.Ki * integral_velocity_error_frd.x() +
                       attitude_pid_gains_.pitch.Kd * velocity_error_frd_d.x();

    double yaw_cmd = 0.0;

    double thrust_cmd = hover_thrust_estimate_ + attitude_pid_gains_.thrust.Kp * velocity_error_frd.z() +
                        attitude_pid_gains_.thrust.Ki * integral_velocity_error_frd.z() +
                        attitude_pid_gains_.thrust.Kd * velocity_error_frd_d.z();

    // Anti-windup: integrals are in FRD so gating is direct per axis
    if (std::abs(roll_cmd)  < max_tilt_angle_)
        previous_velocity_error.Y.error_integral += velocity_error_frd.y() * sample_time;
    if (std::abs(pitch_cmd) < max_tilt_angle_)
        previous_velocity_error.X.error_integral += velocity_error_frd.x() * sample_time;
    if (thrust_cmd > -1.0 && thrust_cmd < -0.05)
        previous_velocity_error.Z.error_integral += velocity_error_frd.z() * sample_time;

    // Constrain outputs
    roll_cmd   = constrainAngle(roll_cmd);
    pitch_cmd  = constrainAngle(pitch_cmd);
    thrust_cmd = constrainThrust(thrust_cmd);
    
    //thrust_cmd = EMA_filter(thrust_cmd, previous_control_signal.w());

    // Update previous error in FRD frame (consistent with derivative computation)
    previous_velocity_error.X.error = velocity_error_frd.x();
    previous_velocity_error.Y.error = velocity_error_frd.y();
    previous_velocity_error.Z.error = velocity_error_frd.z();

    // Return control outputs (roll, pitch, yaw, thrust)
    return Eigen::Vector4d(roll_cmd, -pitch_cmd, yaw_cmd, thrust_cmd);

}

double Controller::EMA_filter(double new_value, double previous_value) const {
    return ema_filter_alpha_ * previous_value + (1.0f - ema_filter_alpha_) * new_value;
}

double Controller::mapNormToAngle(double norm) const {
    return norm * max_tilt_angle_;
}

double Controller::constrainAngle(double angle) const {
    return std::clamp(angle, -max_tilt_angle_, max_tilt_angle_);
}

double Controller::constrainThrust(double thrust) const {
    constexpr double max_thrust = -0.05;
    constexpr double min_thrust = -1.0;
    return std::clamp(thrust, min_thrust, max_thrust);
}

Eigen::Vector4d Controller::map_controls(const Stamped4DVector& input) const {
    Eigen::Vector4d output = Eigen::Vector4d::Zero();
    
    double max_velocity = 3.0; // m/s, adjust as needed
    
    // Generate velocity commands in body frame (FRD) for intuitive control
    // Forward stick → forward velocity, Right stick → right velocity
    output.x() = max_velocity * std::sin(input.y()); // pitch controls forward/backward (vx in FRD)
    output.y() = max_velocity * std::sin(input.x()); // roll controls left/right (vy in FRD)
    output.z() = input.z(); // yaw velocity command
    output.w() = input.w(); // thrust (vertical velocity in manual aided mode)
    
    return output;
}
