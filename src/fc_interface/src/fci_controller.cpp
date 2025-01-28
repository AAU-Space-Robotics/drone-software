#include "fci_controller.h"
#include <cmath>
#include <iostream>

std::vector<double> FCI_Controller::PID_control(double &sample_time, std::vector<double> &previous_position_NEDEarth_error,
    const std::vector<double> &position_NEDEarth, const std::vector<double> &attitude, const std::vector<double> &target_position_NEDEarth
) {
    // Error calculations
    std::vector<double> position_error_NEDEarth(position_NEDEarth.size());
    std::vector<double> position_error_NEDEarth_d(position_NEDEarth.size());

    for (size_t i = 0; i < position_NEDEarth.size(); ++i) {
        position_error_NEDEarth[i] = target_position_NEDEarth[i] - position_NEDEarth[i];
        position_error_NEDEarth_d[i] = (position_error_NEDEarth[i] - previous_position_NEDEarth_error[i]) / sample_time;
    }

    // Transform errors to FRD frame
    std::vector<double> position_error_FRD = error_NEDEarth_to_FRD(position_error_NEDEarth, attitude);
    std::vector<double> position_error_FRD_d = error_NEDEarth_to_FRD(position_error_NEDEarth_d, attitude);

    // Calculate control outputs
    double roll = PIDGains.Kd_pitch * position_error_FRD[1] + PIDGains.Kd_roll * position_error_FRD_d[1];
    double pitch = PIDGains.Kp_pitch * position_error_FRD[0] + PIDGains.Kd_pitch * position_error_FRD_d[0];
    double thrust = PIDGains.Kp_thrust * position_error_FRD[2] + PIDGains.Kd_thrust * position_error_FRD_d[2];

    // Constrain outputs
    roll = constrain_angle(roll);
    pitch = constrain_angle(pitch);
    thrust = constrain_thrust(thrust);

    // Log the results
    std::cout << "Pose: " << position_NEDEarth[0] << ", " << position_NEDEarth[1] << ", " << position_NEDEarth[2] << std::endl;
    std::cout << "Error: " << position_error_NEDEarth[0] << ", " << position_error_NEDEarth[1] << ", " << position_error_NEDEarth[2] << std::endl;
    std::cout << "Control Outputs - Roll: " << roll << ", Pitch: " << pitch << ", Thrust: " << thrust << std::endl;

    // Update previous error
    previous_position_NEDEarth_error = position_error_NEDEarth;

    // Return control outputs
    return {roll, -pitch, thrust};
}

double FCI_Controller::constrain_angle(double angle) const {
    constexpr double max_angle = M_PI / 4.0;
    if (angle > max_angle) {
        angle = max_angle;
    } else if (angle < -max_angle) {
        angle = -max_angle;
    }
    return angle;
}

double FCI_Controller::constrain_thrust(double thrust) const {
    constexpr double max_thrust = 1.0;
    constexpr double min_thrust = -1.0;
    if (thrust > max_thrust) {
        thrust = max_thrust;
    } else if (thrust < min_thrust) {
        thrust = min_thrust;
    }
    return thrust;
}

std::vector<double> FCI_Controller::error_NEDEarth_to_FRD(const std::vector<double> &error_NEDEarth, const std::vector<double> &attitude_FRD_to_NED) const {
    if (attitude_FRD_to_NED.size() != 4) {
        throw std::invalid_argument("Attitude must be a quaternion (size 4).");
    }

    double q0 = attitude_FRD_to_NED[0];
    double q1 = attitude_FRD_to_NED[1];
    double q2 = attitude_FRD_to_NED[2];
    double q3 = attitude_FRD_to_NED[3];

    double norm = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm == 0) {
        throw std::invalid_argument("Quaternion norm must not be zero.");
    }
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;

    double R11 = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    double R12 = 2 * (q1 * q2 - q0 * q3);
    double R13 = 2 * (q1 * q3 + q0 * q2);
    double R21 = 2 * (q1 * q2 + q0 * q3);
    double R22 = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    double R23 = 2 * (q2 * q3 - q0 * q1);
    double R31 = 2 * (q1 * q3 - q0 * q2);
    double R32 = 2 * (q2 * q3 + q0 * q1);
    double R33 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    double error_body_N = R11 * error_NEDEarth[0] + R12 * error_NEDEarth[1] + R13 * error_NEDEarth[2];
    double error_body_E = R21 * error_NEDEarth[0] + R22 * error_NEDEarth[1] + R23 * error_NEDEarth[2];
    double error_body_D = R31 * error_NEDEarth[0] + R32 * error_NEDEarth[1] + R33 * error_NEDEarth[2];

    return {error_body_N, error_body_E, error_body_D};
}