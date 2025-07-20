#include "fci_transformations.h"

Stamped3DVector FCI_Transformations::accelerationLocalToGlobal(const rclcpp::Time& timestamp, 
                                                              const Eigen::Quaterniond& attitude, 
                                                              const Eigen::Vector3d& acceleration_local) const {
    Eigen::Vector3d acceleration_global = attitude * acceleration_local;

    Stamped3DVector result;
    result.setTime(timestamp);
    result.vector() = acceleration_global;

    return result;
}

Eigen::Quaterniond FCI_Transformations::eulerToQuaternion(double roll, double pitch, double yaw) const {
    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return q.normalized();
}

Eigen::Vector3d FCI_Transformations::quaternionToEuler(const Eigen::Quaterniond& q) const {
    // Get Euler angles in ZYX convention (yaw, pitch, roll)
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    
    // Extract yaw, pitch, roll
    double yaw = euler.x();
    double pitch = euler.y();
    double roll = euler.z();

    // Normalize pitch to [-π/2, π/2] to avoid gimbal lock ambiguities
    if (std::abs(pitch) > M_PI / 2) {
        // Adjust yaw and flip pitch and roll to maintain equivalent rotation
        yaw += M_PI;
        pitch = M_PI - pitch; // Reflect pitch around π
        roll += M_PI;
    }

    // Unwrap angles to [0, 2π]
    yaw = unwrapAngle(yaw, 2 * M_PI, 0);
    pitch = unwrapAngle(pitch, 2 * M_PI, 0);
    roll = unwrapAngle(roll, 2 * M_PI, 0);

    // Ensure yaw is in [0, 2π] and consistent with input
    return Eigen::Vector3d(yaw, pitch, roll);
}

Eigen::Vector3d FCI_Transformations::errorGlobalToLocal(const Eigen::Vector3d& error_ned_earth, 
                                                       const Eigen::Quaterniond& attitude_frd_to_ned) const {
    return attitude_frd_to_ned.conjugate() * error_ned_earth;
}

double FCI_Transformations::degToRad(double degrees) const {
    return degrees * M_PI / 180.0;
}

double FCI_Transformations::radToDeg(double radians) const {
    return radians * 180.0 / M_PI;
}

double FCI_Transformations::unwrapAngle(double angle, double max, double min) const {
    // Unwrap the angle to be within the range [min, max]    
    while (angle > max) angle -= 2.0 * M_PI;
    while (angle < min) angle += 2.0 * M_PI;
    return angle;
    }
