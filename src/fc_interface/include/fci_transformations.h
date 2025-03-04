#ifndef FCI_TRANSFORMATIONS_H
#define FCI_TRANSFORMATIONS_H

#include <atomic>
#include <stdint.h>
#include <cmath>
#include <vector>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include "fci_state_manager.h"

// FCI_Transformation class definition
class FCI_Transformations {
public:

    // GPS functions
    void setGPSOrigin(rclcpp::Time timestamp, double lat, double lon, double alt);
    GPSData getGPSOrigin();
    bool isGPSOriginSet();
    PositionNED convertGPSToNED(rclcpp::Time timestamp, double lat, double lon, double alt);

    static std::vector<double> euler_to_quaternion(double roll, double pitch, double yaw);
    static std::vector<double> quaternion_to_euler(double q0, double q1, double q2, double q3);
    static std::vector<double> geodetic_to_ECEF(double lat, double lon, double alt);
    static std::vector<double> error_NEDEarth_to_FRD(const std::vector<double>& error_NEDEarth, const std::vector<double>& attitude_FRD_to_NED);
    AccelerationNED AccelFRDToNED(rclcpp::Time timestamp, const Attitude& attitude, const double &x, const double &y, const double &z);
    static double deg_to_rad(double deg);
    static double rad_to_deg(double rad);

private:
    std::mutex gps_data_mutex_;
    GPSData gps_origin_data_;
    bool gps_origin_set_ = false;
    GeographicLib::LocalCartesian geographic_converter_;
};

#endif // FCI_TRANSFORMATIONS_H
