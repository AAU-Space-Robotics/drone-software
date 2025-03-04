#ifndef FCI_CONTROLLER_H
#define FCI_CONTROLLER_H

#include <vector>
#include <stdexcept>
#include <fci_state_manager.h>

// Controller gains
struct PIDGains {
    double Kp;
    double Ki;
    double Kd;
};

struct PIDControllerGains {
    PIDGains pitch{0.5, 0.005, 0.02};  
    PIDGains roll{0.5, 0.005, 0.02};   
    PIDGains yaw{0.3, 0.005, 0.02};    
    PIDGains thrust{0.39, 0.00, 0.005};
};
// 0.35, 0.02, 0.12

//struct PIDControllerGains {
//    PIDGains pitch{0.1, 0.00, 0.05};
//    PIDGains roll{0.1, 0.00, 0.05};
//    PIDGains yaw{0.1, 0.00, 0.05};
//   PIDGains thrust{5.0, 0.2, 1.0};
//};


class FCI_Controller {
public:
    PIDControllerGains AttitudePIDGains;

    std::vector<double> PID_control(
        double &sample_time,
        std::vector<double> &previous_position_NEDEarth_error,
        std::vector<double> &integral_position_error_NEDEarth,
        const std::vector<double> &position_NEDEarth,
        const std::vector<double> &attitude,
        const std::vector<double> &target_position_NEDEarth
    );

    std::vector<double> Acceleration_Controller(
        double &sample_time,
        AccelerationError &previous_acceleration_error,
        Attitude &attitude,
        const std::vector<double> &acceleration_FRD,
        const std::vector<double> &target_acceleration_FRD
    );



    double map_norm_to_angle(double norm) const;

private:
    double constrain_angle(double angle) const;
    double constrain_thrust(double thrust) const;
    
    std::vector<double> error_NEDEarth_to_FRD(const std::vector<double> &error_NEDEarth, const std::vector<double> &attitude_FRD_to_NED) const;
};

#endif // FCI_CONTROLLER_H
