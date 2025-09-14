#ifndef FCI_CONTROLLER_H
#define FCI_CONTROLLER_H

#include <eigen3/Eigen/Dense>
#include "fci_state_manager.h" // For Stamped3DVector, StampedQuaternion, etc.
#include "fci_transformations.h" // For coordinate transformations

// PID gains structure
struct PIDGains {
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;

    PIDGains(double kp = 0.0, double ki = 0.0, double kd = 0.0) : Kp(kp), Ki(ki), Kd(kd) {}
};

// Controller gains for attitude and thrust
struct PIDControllerGains {
    PIDGains pitch{0.1, 0.0, 0.05};
    PIDGains roll{0.1, 0.0, 0.05};
    PIDGains yaw{0.1, 0.0, 0.05};
    PIDGains thrust{0.8, 0.0, 0.1};
};

struct AccelerationControllerGains {
    PIDGains roll{0.1, 0.0, 0.05};
    PIDGains pitch{0.1, 0.0, 0.05};
    PIDGains thrust{0.4, 0.0, 0.0};
};

class FCI_Controller {
public:
    explicit FCI_Controller(const FCI_Transformations& transformations);

    // Set PID gains for attitude and thrust
    void setPIDGains(const PIDControllerGains& gains);

    // Position PID control (returns roll, pitch, yaw, thrust)
    Eigen::Vector4d pidControl(double sample_time,
                               PositionError& previous_position_error,
                               const Stamped3DVector& position_ned_earth,
                               const StampedQuaternion& attitude,
                               const Stamped3DVector& target_position_ned_earth,
                               const Eigen::Vector4d& previous_control_signal);

    Eigen::Vector4d velocityControl(double sample_time,
                                     VelocityError& previous_velocity_error,
                                     const Stamped3DVector& velocity_ned_earth,
                                     const StampedQuaternion& attitude,
                                     const Stamped3DVector& target_velocity_ned_earth,
                                     const Eigen::Vector3d& previous_control_signal);

    // Utility function to map normalized values to angles
    double mapNormToAngle(double norm) const;
    
    float ema_filter_alpha_ = 0.01; // Alpha value for EMA filter

private:
    const FCI_Transformations& transformations_; // Reference to transformations utility
    PIDControllerGains attitude_pid_gains_;      // PID gains for attitude and thrust
    AccelerationControllerGains acceleration_pid_gains_; // PID gains for acceleration

    double EMA_filter(double current_value, double previous_value) const;

    // Constrain control outputs
    double constrainAngle(double angle) const;
    double constrainThrust(double thrust) const;
};

#endif // FCI_CONTROLLER_H