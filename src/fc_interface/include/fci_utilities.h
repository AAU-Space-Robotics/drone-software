#ifndef FCI_UTILITIES_H
#define FCI_UTILITIES_H

#include <atomic>
#include <stdint.h>
#include <cmath>
#include <vector>
#include <mutex>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <GeographicLib/LocalCartesian.hpp>


enum class Command {
    DISARM = 0,
    ARM = 1,
    TAKEOFF = 2,
    GOTO = 3,
    LAND = 4,
    ESTOP = 5,
    MANUAL = 6,
    MANUAL_AIDED = 7
};

enum class ArmingState {
    DISARMED = 0,
    ARMED = 1,
    ARMING = 2,
    DISARMING = 3
};

enum class FlightMode {
    STANDBY = -1,
    MANUAL = 0,
    MANUAL_AIDED = 1,
    POSITION = 2
};


// PositionNED structure
struct PositionNED {
    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    
    //Position data in NED frame 
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    
};

// GPSData structure
struct GPSData {
    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    
    //GPS data
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
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

// ManualControlInput structure
struct ManualControlInput {
    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    
    //Manual control data
    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0;
    float thrust = 0.0;
};

// Drone state
struct DroneState {
    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    
    //Drone state data
    Command command = Command::DISARM;
    ArmingState arming_state = ArmingState::DISARMED;
    FlightMode flight_mode = FlightMode::STANDBY;
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

    void setManualControlInput(const ManualControlInput& new_data);
    ManualControlInput getManualControlInput();

    // GPS data functions
    void setGPSOrigin(rclcpp::Time timestamp, double lat, double lon, double alt);
    GPSData getGPSOrigin();
    PositionNED convertGPSToNED(rclcpp::Time timestamp, double lat, double lon, double alt);
    bool isGPSOriginSet();

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
    GPSData gps_origin_data_;
    bool gps_origin_set_ = false;
    std::mutex gps_data_mutex_;
    GeographicLib::LocalCartesian geographic_converter_;


    Attitude attitude_;
    std::mutex attitude_data_mutex_;
    TargetPositionProfile target_position_profile_;
    std::mutex target_position_profile_mutex_;
    DroneCmdAck drone_cmd_ack_;
    std::mutex drone_cmd_ack_mutex_;
    DroneState drone_state_;
    std::mutex drone_state_mutex_;
    ManualControlInput manual_control_input_;
    std::mutex manual_control_input_mutex_;

};

#endif // FCI_UTILITIES_H
