#ifndef FCI_CONTROLLER_H
#define FCI_CONTROLLER_H

#include <vector>
#include <stdexcept>

// Controller gains
struct PIDControllerGains {
    double Kp_pitch = 0.1;
    double Kd_pitch = 0.05;
    double Kp_roll = 0.1;
    double Kd_roll = 0.05;
    double Kp_yaw = 0.1;
    double Kd_yaw = 0.05;
    double Kp_thrust = 5.0;
    double Kd_thrust = 1.0;
};

class FCI_Controller {
public:
    PIDControllerGains PIDGains;

    std::vector<double> PID_control(
        double &sample_time,
        std::vector<double> &previous_position_NEDEarth_error,
        const std::vector<double> &position_NEDEarth,
        const std::vector<double> &attitude,
        const std::vector<double> &target_position_NEDEarth
    );

private:
    double constrain_angle(double angle) const;
    double constrain_thrust(double thrust) const;
    std::vector<double> error_NEDEarth_to_FRD(const std::vector<double> &error_NEDEarth, const std::vector<double> &attitude_FRD_to_NED) const;
};

#endif // FCI_CONTROLLER_H
