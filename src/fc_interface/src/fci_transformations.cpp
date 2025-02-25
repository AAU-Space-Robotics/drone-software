#include "fci_transformations.h"

void FCI_Transformations::setGPSOrigin(rclcpp::Time timestamp, double lat, double lon, double alt) {
    std::lock_guard<std::mutex> lock(gps_data_mutex_);
    gps_origin_data_ = {timestamp, lat, lon, alt};
    gps_origin_set_ = true;
    geographic_converter_.Reset(lat, lon, alt); // Reset GeographicLib LocalCartesian
}

bool FCI_Transformations::isGPSOriginSet() {
    std::lock_guard<std::mutex> lock(gps_data_mutex_);
    return gps_origin_set_;
}

GPSData FCI_Transformations::getGPSOrigin() {
    std::lock_guard<std::mutex> lock(gps_data_mutex_);
    return gps_origin_data_;
}



PositionNED FCI_Transformations::convertGPSToNED(rclcpp::Time timestamp, double lat, double lon, double alt) {
    double x_enu, y_enu, z_enu;
    geographic_converter_.Forward(lat, lon, alt, x_enu, y_enu, z_enu);

    // Convert ENU → NED
    double x_ned = y_enu;
    double y_ned = x_enu;
    double z_ned = -z_enu;

    PositionNED ned_data = {timestamp, x_ned, y_ned, z_ned};

    return ned_data;
}

std::vector<double> FCI_Transformations::euler_to_quaternion(double roll, double pitch, double yaw) {
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    double q0 = cr * cp * cy + sr * sp * sy;
    double q1 = sr * cp * cy - cr * sp * sy;
    double q2 = cr * sp * cy + sr * cp * sy;
    double q3 = cr * cp * sy - sr * sp * cy;

    return {q0, q1, q2, q3};
}

std::vector<double> FCI_Transformations::quaternion_to_euler(double q0, double q1, double q2, double q3) {
    double norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;

    double sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    double cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
    double roll = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (q0 * q2 - q3 * q1);
    double pitch = fabs(sinp) >= 1 ? copysign(M_PI / 2, sinp) : asin(sinp);

    double siny_cosp = 2 * (q0 * q3 + q1 * q2);
    double cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    double yaw = atan2(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
}

std::vector<double> FCI_Transformations::geodetic_to_ECEF(double lat, double lon, double alt) {
    lat = deg_to_rad(lat);
    lon = deg_to_rad(lon);

    double a = 6378137.0;
    double f = 1 / 298.257223563;
    double b = a * (1 - f);
    double e = sqrt(1 - (b * b) / (a * a));

    double N = a / sqrt(1 - e * e * sin(lat) * sin(lat));

    double x = (N + alt) * cos(lat) * cos(lon);
    double y = (N + alt) * cos(lat) * sin(lon);
    double z = (N * (1 - e * e) + alt) * sin(lat);

    return {x, y, z};
}

std::vector<double> FCI_Transformations::error_NEDEarth_to_FRD(const std::vector<double>& error_NEDEarth, const std::vector<double>& attitude_FRD_to_NED) {
    double q0 = attitude_FRD_to_NED[0];
    double q1 = attitude_FRD_to_NED[1];
    double q2 = attitude_FRD_to_NED[2];
    double q3 = attitude_FRD_to_NED[3];

    double norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm == 0) {
        throw std::invalid_argument("Quaternion must not be zero.");
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

double FCI_Transformations::deg_to_rad(double deg) {
    return deg * M_PI / 180.0;
}

double FCI_Transformations::rad_to_deg(double rad) {
    return rad * 180.0 / M_PI;
}