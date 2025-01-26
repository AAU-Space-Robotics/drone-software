// fci_utilities.h
#ifndef FCI_UTILITIES_H
#define FCI_UTILITIES_H

#include <vector>
#include <cmath>
#include <stdexcept>

class FCI_Utilities {
public:
    static std::vector<double> euler_to_quaternion(double roll, double pitch, double yaw);
    static std::vector<double> quaternion_to_euler(double q0, double q1, double q2, double q3);
    static std::vector<double> geodetic_to_ECEF(double lat, double lon, double alt);
    static std::vector<double> error_NEDEarth_to_FRD(const std::vector<double>& error_NEDEarth, const std::vector<double>& attitude_FRD_to_NED);
    static double deg_to_rad(double deg);
    static double rad_to_deg(double rad);
};

#endif // FCI_UTILITIES_H