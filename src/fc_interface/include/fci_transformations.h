#ifndef FCI_TRANSFORMATIONS_H
#define FCI_TRANSFORMATIONS_H

#include <mutex>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "fci_state_manager.h" 

class FCI_Transformations {
public:
    FCI_Transformations(){}

    // GPS Origin Management
    void setGPSOrigin(const rclcpp::Time& timestamp, double latitude, double longitude, double altitude);
    bool isGPSOriginSet() const;
    Stamped3DVector getGPSOrigin() const;

    // Coordinate Transformations
    Stamped3DVector convertGPSToGlobalPosition(const rclcpp::Time& timestamp, double latitude, double longitude, double altitude) const;
    Stamped3DVector accelerationLocalToGlobal(const rclcpp::Time& timestamp, 
                                             const Eigen::Quaterniond& attitude, 
                                             const Eigen::Vector3d& acceleration_local) const;

    // Attitude Conversions
    Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw) const;
    Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q) const;

    // Geodetic Transformations
    Eigen::Vector3d errorGlobalToLocal(const Eigen::Vector3d& error_ned_earth, 
                                      const Eigen::Quaterniond& attitude_frd_to_ned) const;

    // Utility Functions
    double degToRad(double degrees) const;
    double radToDeg(double radians) const;
    double unwrapAngle(double angle, double max = M_PI, double min = -M_PI) const;

private:

};

#endif // FCI_TRANSFORMATIONS_H