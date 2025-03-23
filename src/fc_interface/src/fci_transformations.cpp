#include "fci_transformations.h"

void FCI_Transformations::setGPSOrigin(const rclcpp::Time& timestamp, double latitude, double longitude, double altitude) {
    std::lock_guard<std::mutex> lock(gps_data_mutex_);
    gps_origin_data_.setTime(timestamp);
    gps_origin_data_.setX(latitude);
    gps_origin_data_.setY(longitude);
    gps_origin_data_.setZ(altitude);
    gps_origin_set_ = true;
    geographic_converter_.Reset(latitude, longitude, altitude);
}

bool FCI_Transformations::isGPSOriginSet() const {
    std::lock_guard<std::mutex> lock(gps_data_mutex_);
    return gps_origin_set_;
}

Stamped3DVector FCI_Transformations::getGPSOrigin() const {
    std::lock_guard<std::mutex> lock(gps_data_mutex_);
    return gps_origin_data_;
}

Stamped3DVector FCI_Transformations::convertGPSToGlobalPosition(const rclcpp::Time& timestamp, double latitude, double longitude, double altitude) const {
    double x_enu, y_enu, z_enu;
    geographic_converter_.Forward(latitude, longitude, altitude, x_enu, y_enu, z_enu);

    Stamped3DVector global_data;
    global_data.setTime(timestamp);
    global_data.setX(y_enu);  // East → North
    global_data.setY(x_enu);  // North → East
    global_data.setZ(-z_enu); // Up → Down

    return global_data;
}

Stamped3DVector FCI_Transformations::accelerationLocalToGlobal(const rclcpp::Time& timestamp, 
                                                              const Eigen::Quaterniond& attitude, 
                                                              const Eigen::Vector3d& acceleration_local) const {
    Eigen::Vector3d acceleration_global = attitude.conjugate() * acceleration_local;

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
    return q.normalized().toRotationMatrix().eulerAngles(0, 1, 2); // Roll, Pitch, Yaw
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