#ifndef FCI_CONTROLLER_H
#define FCI_CONTROLLER_H

#include <eigen3/Eigen/Dense>
#include "fci_state_manager.h" // For Stamped3DVector, StampedQuaternion, PositionError, VelocityError
#include "fci_transformations.h" // For coordinate transformations

// PID gains structure
struct PIDGains {
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;

    PIDGains(double kp = 0.0, double ki = 0.0, double kd = 0.0) : Kp(kp), Ki(ki), Kd(kd) {}
};

struct PositionControllerGains {
    PIDGains x;
    PIDGains y;
    PIDGains z;
};

// Controller gains for velocity control
struct VelocityControllerGains {
    PIDGains vel_x;
    PIDGains vel_y;
    PIDGains vel_z;
};

class FCI_Controller {
public:
    explicit FCI_Controller(const FCI_Transformations& transformations);

    // Set PID gains for position (attitude) control
    void setPositionPIDGains(const PositionControllerGains& gains);

    // Set PID gains for velocity control
    void setVelocityPIDGains(const VelocityControllerGains& gains);

    // Position control: Returns target velocities in FRD frame (x, y, z)
    Eigen::Vector3d positionControl(double sample_time,
                                   PositionError& previous_position_error,
                                   const Stamped3DVector& position_ned_earth,
                                   const StampedQuaternion& attitude,
                                   const Stamped3DVector& target_position_ned_earth,
                                   const double max_linear_velocity);

    // Velocity control: Returns roll, pitch, yaw, thrust
    Eigen::Vector4d velocityControl(double sample_time,
                                   VelocityError& previous_velocity_error,
                                   const Stamped3DVector& velocity_frd,
                                   const Eigen::Vector3d& target_velocity_frd,
                                   const double max_angle_radians,
                                   const double max_thrust,
                                   const double min_thrust);

    double mapNormToAngle(double norm) const;

    float max_linear_velocity_ = 0.01; // Maximum linear velocity constraint
    float ema_filter_alpha_ = 0.01; // Alpha value for EMA filter

private:
    const FCI_Transformations& transformations_; // Reference to transformations utility
    PositionControllerGains position_pid_gains_; // PID gains for position control
    VelocityControllerGains velocity_pid_gains_; // PID gains for velocity control

    // Generic PID computation for a single axis
    double computePID(double error, double& previous_error, double& error_integral,
                     double sample_time, const PIDGains& gains,
                     bool apply_anti_windup, double min_output, double max_output) const;
    double EMA_filter(double current_value, double previous_value) const;

    // Constrain control outputs
    double constrainAngle(double angle) const;
    double constrainThrust(double thrust) const;
};

#endif // FCI_CONTROLLER_H