#include "fci_controller.h"
#include <cmath>
#include <iostream>

FCI_Controller::FCI_Controller(const FCI_Transformations& transformations) 
    : transformations_(transformations), attitude_pid_gains_() {}

Eigen::Vector4d FCI_Controller::pidControl(double sample_time,
                                           PositionError& previous_position_error,
                                           const Stamped3DVector& position_ned_earth,
                                           const StampedQuaternion& attitude,
                                           const Stamped3DVector& target_position_ned_earth) {
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

    // Update previous error
    previous_position_error.X.error = position_error_ned.x();
    previous_position_error.Y.error = position_error_ned.y();
    previous_position_error.Z.error = position_error_ned.z();

    // Return control outputs (roll, pitch, yaw, thrust)
    return {roll, pitch, 0.0, thrust};
}

Eigen::Vector4d FCI_Controller::accelerationControl(double sample_time,
                                                    AccelerationError& previous_acceleration_error,
                                                    const Stamped3DVector& acceleration_frd,
                                                    const Stamped3DVector& target_acceleration_frd) {
    // Calculate acceleration error in FRD frame
    Eigen::Vector3d acceleration_error_frd = target_acceleration_frd.vector() - acceleration_frd.vector();
    Eigen::Vector3d acceleration_error_frd_d = (acceleration_error_frd - 
                                                Eigen::Vector3d(previous_acceleration_error.X.error,
                                                                previous_acceleration_error.Y.error,
                                                                previous_acceleration_error.Z.error)) / sample_time;

    // Update integral error
    previous_acceleration_error.X.error_integral += acceleration_error_frd.x() * sample_time;
    previous_acceleration_error.Y.error_integral += acceleration_error_frd.y() * sample_time;
    previous_acceleration_error.Z.error_integral += acceleration_error_frd.z() * sample_time;

    // Transform integral error to FRD (already in FRD, but keeping structure consistent)
    Eigen::Vector3d integral_acceleration_error_frd(previous_acceleration_error.X.error_integral,
                                                    previous_acceleration_error.Y.error_integral,
                                                    previous_acceleration_error.Z.error_integral);

    // Calculate control outputs
    double roll = attitude_pid_gains_.roll.Kp * acceleration_error_frd.y() + 
                  attitude_pid_gains_.roll.Ki * integral_acceleration_error_frd.y() + 
                  attitude_pid_gains_.roll.Kd * acceleration_error_frd_d.y();

    double pitch = attitude_pid_gains_.pitch.Kp * acceleration_error_frd.x() + 
                   attitude_pid_gains_.pitch.Ki * integral_acceleration_error_frd.x() + 
                   attitude_pid_gains_.pitch.Kd * acceleration_error_frd_d.x();

    double thrust = attitude_pid_gains_.thrust.Kp * acceleration_error_frd.z() + 
                    attitude_pid_gains_.thrust.Ki * integral_acceleration_error_frd.z() + 
                    attitude_pid_gains_.thrust.Kd * acceleration_error_frd_d.z();

    // Anti-windup for thrust
    if (thrust >= 1.0 || thrust <= -1.0) {
        previous_acceleration_error.Z.error_integral = 0.0;
    }

    // Constrain outputs
    roll = constrainAngle(roll);
    pitch = constrainAngle(pitch);
    thrust = constrainThrust(thrust);

    // Update previous error
    previous_acceleration_error.X.error = acceleration_error_frd.x();
    previous_acceleration_error.Y.error = acceleration_error_frd.y();
    previous_acceleration_error.Z.error = acceleration_error_frd.z();

    // Return control outputs (roll, pitch, yaw, thrust)
    return {roll, -pitch, 0.0, thrust};
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