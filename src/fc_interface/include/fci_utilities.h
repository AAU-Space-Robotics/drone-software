#ifndef FCI_UTILITIES_H
#define FCI_UTILITIES_H

#include <atomic>
#include <stdint.h>
#include <cmath>
#include <vector>
#include <mutex>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

// PositionNED structure
struct PositionNED {
    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    
    //Position data in NED frame 
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    
};

// Attitude structure
struct Attitude {
    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    
    // Attitude data that rotates from NED to FRD. The quaternion is in the form [q0, q1, q2, q3] based on the hamiltonian convention
    float qw = 1.0;
    float qx = 0.0;
    float qy = 0.0;
    float qz = 0.0;
};

// target position profile structure
struct TargetPositionProfile {
    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    
    //Position data in NED frame 
    float x = 0.0;
    float y = 0.0;
    float z = -1.5;
    
    //Yaw data in radians
    float yaw = 0.0;

};

// DroneCmdAck structure
struct DroneCmdAck {
    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    
    //Command data
    uint16_t command = 0;
    uint8_t result = 0;
};

// Drone state
struct DroneState {
    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    
    //Drone state data
    uint8_t armed = 1;
    uint8_t control_loop_running = 0;
};

// FCI_Utilities class definition
class FCI_Utilities {
public:
    // Thread-safe setters and getters for NED_Data
    void setPositionNED(const PositionNED& new_data);
    PositionNED getPositionNED();

    void setAttitude(const Attitude& new_data);
    Attitude getAttitude();

    void setTargetPositionProfile(const TargetPositionProfile& new_data);
    TargetPositionProfile getTargetPositionProfile();

    void setDroneCmdAck(const DroneCmdAck& new_data);
    DroneCmdAck getDroneCmdAck();

    void setDroneState(const DroneState& new_data);
    DroneState getDroneState();

    // Utility functions
    static std::vector<double> euler_to_quaternion(double roll, double pitch, double yaw);
    static std::vector<double> quaternion_to_euler(double q0, double q1, double q2, double q3);
    static std::vector<double> geodetic_to_ECEF(double lat, double lon, double alt);
    static std::vector<double> error_NEDEarth_to_FRD(const std::vector<double>& error_NEDEarth, const std::vector<double>& attitude_FRD_to_NED);
    static double deg_to_rad(double deg);
    static double rad_to_deg(double rad);

private:
    // position_NED_ instance and mutex for thread safety
    PositionNED position_NED_;
    std::mutex position_NED_mutex_;
    Attitude attitude_;
    std::mutex attitude_data_mutex_;
    TargetPositionProfile target_position_profile_;
    std::mutex target_position_profile_mutex_;
    DroneCmdAck drone_cmd_ack_;
    std::mutex drone_cmd_ack_mutex_;
    DroneState drone_state_;
    std::mutex drone_state_mutex_;

};

#endif // FCI_UTILITIES_H
