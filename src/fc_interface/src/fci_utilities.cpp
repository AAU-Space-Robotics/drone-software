#include "fci_utilities.h"

// Setters & Getters
void FCI_Utilities::setPositionNED(const PositionNED& new_data) {
    std::lock_guard<std::mutex> lock(position_NED_mutex_);
    position_NED_ = new_data;  // Thread-safe copy
}

PositionNED FCI_Utilities::getPositionNED() {
    std::lock_guard<std::mutex> lock(position_NED_mutex_);
    return position_NED_;  // Thread-safe copy
}

void FCI_Utilities::setAttitude(const Attitude& new_data) {
    std::lock_guard<std::mutex> lock(attitude_data_mutex_);
    attitude_ = new_data;  // Thread-safe copy
}

Attitude FCI_Utilities::getAttitude() {
    std::lock_guard<std::mutex> lock(attitude_data_mutex_);
    return attitude_;  // Thread-safe copy
}

void FCI_Utilities::setTargetPositionProfile(const TargetPositionProfile& new_data) {
    std::lock_guard<std::mutex> lock(target_position_profile_mutex_);
    target_position_profile_ = new_data;  // Thread-safe copy
}

TargetPositionProfile FCI_Utilities::getTargetPositionProfile() {
    std::lock_guard<std::mutex> lock(target_position_profile_mutex_);
    return target_position_profile_;  // Thread-safe copy
}

void FCI_Utilities::setDroneCmdAck(const DroneCmdAck& new_data) {
    std::lock_guard<std::mutex> lock(drone_cmd_ack_mutex_);
    drone_cmd_ack_ = new_data;  // Thread-safe copy
}

DroneCmdAck FCI_Utilities::getDroneCmdAck() {
    std::lock_guard<std::mutex> lock(drone_cmd_ack_mutex_);
    return drone_cmd_ack_;  // Thread-safe copy
}

void FCI_Utilities::setDroneState(const DroneState& new_data) {
    std::lock_guard<std::mutex> lock(drone_state_mutex_);
    drone_state_ = new_data;  // Thread-safe copy
}

DroneState FCI_Utilities::getDroneState() {
    std::lock_guard<std::mutex> lock(drone_state_mutex_);
    return drone_state_;  // Thread-safe copy
}


// Converter functions:
std::vector<double> FCI_Utilities::euler_to_quaternion(double roll, double pitch, double yaw) {
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

std::vector<double> FCI_Utilities::quaternion_to_euler(double q0, double q1, double q2, double q3) {
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

std::vector<double> FCI_Utilities::geodetic_to_ECEF(double lat, double lon, double alt) {
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

std::vector<double> FCI_Utilities::error_NEDEarth_to_FRD(const std::vector<double>& error_NEDEarth, const std::vector<double>& attitude_FRD_to_NED) {
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

double FCI_Utilities::deg_to_rad(double deg) {
    return deg * M_PI / 180.0;
}

double FCI_Utilities::rad_to_deg(double rad) {
    return rad * 180.0 / M_PI;
}