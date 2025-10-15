#include "fci_controller.h"
#include <cmath>
#include <iostream>

FCI_Controller::FCI_Controller(const FCI_Transformations& transformations) 
    : transformations_(transformations), attitude_pid_gains_() {}

void FCI_Controller::setPIDGains(const PIDControllerGains& gains) {
    attitude_pid_gains_ = gains;
}

Eigen::Vector4d FCI_Controller::pidControl(double sample_time,
                                           PositionError& previous_position_error,
                                           const Stamped3DVector& position_ned_earth,
                                           const StampedQuaternion& attitude,
                                           const Stamped3DVector& target_position_ned_earth,
                                           const Eigen::Vector4d& previous_control_signal) {
    // Calculate position error in NED frame
    Eigen::Vector3d position_error_ned = target_position_ned_earth.vector() - position_ned_earth.vector();
    Eigen::Vector3d position_error_ned_d = (position_error_ned - 
                                            Eigen::Vector3d(previous_position_error.X.error, 
                                                            previous_position_error.Y.error, 
                                                            previous_position_error.Z.error)) / sample_time;

    // Update integral error
    previous_position_error.X.error_integral += position_error_ned.x() * sample_time;
    previous_position_error.Y.error_integral += position_error_ned.y() * sample_time;
    previous_position_error.Z.error_integral += position_error_ned.z() * sample_time;

    // Transform errors to FRD frame
    Eigen::Vector3d position_error_frd = transformations_.errorGlobalToLocal(position_error_ned, attitude.quaternion());
    Eigen::Vector3d position_error_frd_d = transformations_.errorGlobalToLocal(position_error_ned_d, attitude.quaternion());
    Eigen::Vector3d integral_position_error_frd = transformations_.errorGlobalToLocal(
        Eigen::Vector3d(previous_position_error.X.error_integral,
                        previous_position_error.Y.error_integral,
                        previous_position_error.Z.error_integral), 
        attitude.quaternion());

    // Calculate control outputs
    double roll = attitude_pid_gains_.roll.Kp * position_error_frd.y() + 
                  attitude_pid_gains_.roll.Ki * integral_position_error_frd.y() + 
                  attitude_pid_gains_.roll.Kd * position_error_frd_d.y();

    double pitch = attitude_pid_gains_.pitch.Kp * position_error_frd.x() + 
                   attitude_pid_gains_.pitch.Ki * integral_position_error_frd.x() + 
                   attitude_pid_gains_.pitch.Kd * position_error_frd_d.x();

    double thrust = attitude_pid_gains_.thrust.Kp * position_error_frd.z() + 
                    attitude_pid_gains_.thrust.Ki * integral_position_error_frd.z() + 
                    attitude_pid_gains_.thrust.Kd * position_error_frd_d.z();

    // Constrain outputs
    roll = constrainAngle(roll);
    pitch = constrainAngle(pitch);
    thrust = constrainThrust(thrust);

    
    thrust = EMA_filter(thrust, previous_control_signal.w()); // Apply EMA filter to thrust

    // Update previous error
    previous_position_error.X.error = position_error_ned.x();
    previous_position_error.Y.error = position_error_ned.y();
    previous_position_error.Z.error = position_error_ned.z();

    // error in position
    //std::cout << "error: " << position_error_ned.transpose() << std::endl;
    //std::cout << "local error: " << position_error_frd.transpose() << std::endl;
    //std::cout << "Control: " << roll << ", " << pitch << ", " << thrust << std::endl;

    // Return control outputs (roll, pitch, yaw, thrust)
    return {roll, -pitch, 0.0, thrust};
}

Eigen::Vector4d FCI_Controller::velocityControl(double sample_time,
                                                VelocityError& previous_velocity_error,
                                                const Stamped3DVector& velocity_ned_earth,
                                                const StampedQuaternion& attitude,
                                                const Stamped3DVector& target_velocity_ned_earth,
                                                const Eigen::Vector3d& previous_control_signal) {
    // Calculate velocity error in NED frame
    Eigen::Vector3d velocity_error_ned = target_velocity_ned_earth.vector() - velocity_ned_earth.vector();
    Eigen::Vector3d velocity_error_ned_d = (velocity_error_ned -
                                            Eigen::Vector3d(previous_velocity_error.X.error,
                                                            previous_velocity_error.Y.error,
                                                            previous_velocity_error.Z.error)) / sample_time;

    std::cout << "Target velocity NED: x=" << target_velocity_ned_earth.x()
              << ", y=" << target_velocity_ned_earth.y()
              << ", z=" << target_velocity_ned_earth.z() << std::endl;
    std::cout << "Current velocity NED: x=" << velocity_ned_earth.x()
              << ", y=" << velocity_ned_earth.y()
              << ", z=" << velocity_ned_earth.z() << std::endl;

    std::cout << "Velocity error NED: x=" << velocity_error_ned.x()
              << ", y=" << velocity_error_ned.y()
              << ", z=" << velocity_error_ned.z() << std::endl;
                                                            
    // Update integral error
    previous_velocity_error.X.error_integral += velocity_error_ned.x() * sample_time;
    previous_velocity_error.Y.error_integral += velocity_error_ned.y() * sample_time;
    previous_velocity_error.Z.error_integral += velocity_error_ned.z() * sample_time;

    std::cout << "Integral Velocity error NED: x=" << previous_velocity_error.X.error_integral
              << ", y=" << previous_velocity_error.Y.error_integral
              << ", z=" << previous_velocity_error.Z.error_integral << std::endl;

    // Transform errors to FRD frame
    Eigen::Vector3d velocity_error_frd = transformations_.errorGlobalToLocal(velocity_error_ned, attitude.quaternion());
    Eigen::Vector3d velocity_error_frd_d = transformations_.errorGlobalToLocal(velocity_error_ned_d, attitude.quaternion());
    Eigen::Vector3d integral_velocity_error_frd = transformations_.errorGlobalToLocal(
        Eigen::Vector3d(previous_velocity_error.X.error_integral,
                        previous_velocity_error.Y.error_integral,
                        previous_velocity_error.Z.error_integral),
        attitude.quaternion());
    
    std::cout << "Velocity error FRD: x=" << velocity_error_frd.x()
              << ", y=" << velocity_error_frd.y()
              << ", z=" << velocity_error_frd.z() << std::endl;

    // Calculate control outputs
    double roll_cmd = attitude_pid_gains_.roll.Kp * velocity_error_frd.y() +
                      attitude_pid_gains_.roll.Ki * integral_velocity_error_frd.y() +
                      attitude_pid_gains_.roll.Kd * velocity_error_frd_d.y();

    double pitch_cmd = attitude_pid_gains_.pitch.Kp * velocity_error_frd.x() +
                       attitude_pid_gains_.pitch.Ki * integral_velocity_error_frd.x() +
                       attitude_pid_gains_.pitch.Kd * velocity_error_frd_d.x();

    double yaw_cmd = 0.0; // Yaw command

    double thrust_cmd = attitude_pid_gains_.thrust.Kp * velocity_error_frd.z() +
                        attitude_pid_gains_.thrust.Ki * integral_velocity_error_frd.z() +
                        attitude_pid_gains_.thrust.Kd * velocity_error_frd_d.z();

    std::cout << "Raw Control: thrust=" << thrust_cmd << std::endl;

    // Constrain outputs
    // roll_cmd = constrainAngle(roll_cmd);
    // pitch_cmd = constrainAngle(pitch_cmd);
    roll_cmd = 0.0;
    pitch_cmd = 0.0;
    thrust_cmd = constrainThrust(thrust_cmd);

    //thrust_cmd = EMA_filter(thrust_cmd, previous_control_signal.z());

    // Update previous error
    previous_velocity_error.X.error = velocity_error_ned.x();
    previous_velocity_error.Y.error = velocity_error_ned.y();
    previous_velocity_error.Z.error = velocity_error_ned.z();

    // Return control outputs (roll, pitch, yaw, thrust)
    return {roll_cmd, -pitch_cmd, yaw_cmd, thrust_cmd};
}


double FCI_Controller::EMA_filter(double new_value, double previous_value) const {
    return ema_filter_alpha_ * previous_value + (1.0f - ema_filter_alpha_) * new_value;
}

double FCI_Controller::mapNormToAngle(double norm) const {
    constexpr double max_angle = M_PI / 18.0; // ~10 degrees
    return norm * max_angle;
}

double FCI_Controller::constrainAngle(double angle) const {
    constexpr double max_angle = M_PI / 18.0; // ~10 degrees
    return std::clamp(angle, -max_angle, max_angle);
}

double FCI_Controller::constrainThrust(double thrust) const {
    constexpr double max_thrust = -0.05;
    constexpr double min_thrust = -1.0;
    return std::clamp(thrust, min_thrust, max_thrust);
}