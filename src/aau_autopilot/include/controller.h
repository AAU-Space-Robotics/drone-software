#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <eigen3/Eigen/Dense>
#include "state_manager.h" // For Stamped3DVector, StampedQuaternion, etc.
#include "transformations.h" // For coordinate transformations

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

struct PIDPosControllerGains {
    PIDGains x{0.1, 0.0, 0.05};
    PIDGains y{0.1, 0.0, 0.05};
    PIDGains z{0.1, 0.0, 0.05};
};

struct AccelerationControllerGains {
    PIDGains roll{0.1, 0.0, 0.05};
    PIDGains pitch{0.1, 0.0, 0.05};
    PIDGains thrust{0.4, 0.0, 0.0};
};

class Controller {
public:
    explicit Controller(const Transformations& transformations);

    // Set PID gains for attitude and thrust
    void setPIDGains(const PIDControllerGains& gains);

    void setPositionPIDGains(const PIDPosControllerGains& gains);

    // Position PID control (returns roll, pitch, yaw, thrust)
    Eigen::Vector4d pidControl(double sample_time,
                               PositionError& previous_position_error,
                               const Stamped3DVector& position_ned_earth,
                               const StampedQuaternion& attitude,
                               const Stamped3DVector& target_position_ned_earth,
                               const Eigen::Vector4d& previous_control_signal);

    Eigen::Vector3d positionControl(double sample_time,
                                    PositionError& previous_position_error,
                                    const Stamped3DVector& position_ned_earth,
                                    const StampedQuaternion& attitude,
                                    const Stamped3DVector& target_position_ned_earth,
                                    const Eigen::Vector3d& previous_control_signal);

    Eigen::Vector4d velocityControl(double sample_time,
                                     VelocityError& previous_velocity_error,
                                     const Stamped3DVector& velocity_ned_earth,
                                     const StampedQuaternion& attitude,
                                     const Stamped3DVector& target_velocity_ned_earth,
                                     const Eigen::Vector4d& previous_control_signal);

    // Utility function to map normalized values to angles
    double mapNormToAngle(double norm) const;
    
    float ema_filter_alpha_ = 0.01; // Alpha value for EMA filter

    double hover_thrust_estimate_ = -0.5; // Initial estimated hover thrust for most drones
    double hover_learning_rate_ = 0.0001;

    Eigen::Vector4d map_controls(const Stamped4DVector& input) const;

private:
    const Transformations& transformations_; // Reference to transformations utility
    PIDControllerGains attitude_pid_gains_;      // PID gains for attitude and thrust
    AccelerationControllerGains acceleration_pid_gains_; // PID gains for acceleration
    PIDPosControllerGains position_pid_gains_;   // PID gains for position control

    double EMA_filter(double current_value, double previous_value) const;
    
    // Constrain control outputs
    double constrainAngle(double angle) const;
    double constrainThrust(double thrust) const;
};

#endif // CONTROLLER_H
