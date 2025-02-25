#ifndef FCI_UTILITIES_H
#define FCI_UTILITIES_H

#include <atomic>
#include <stdint.h>
#include <cmath>
#include <vector>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

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
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    
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
    double x = 0.0;
    double y = 0.0;
    double z = -1.5;
    
    //Yaw data in radians
    double yaw = 0.0;
};

struct ControlInput {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    double thrust = 0.0;
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

// FCI_StateManager class definition
class FCI_StateManager {
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

private:
    
    // Mutexes for thread safety
    std::mutex position_NED_mutex_;
    std::mutex attitude_data_mutex_;
    std::mutex target_position_profile_mutex_;
    std::mutex drone_cmd_ack_mutex_;
    std::mutex drone_state_mutex_;
    std::mutex manual_control_input_mutex_;

    // Data structures to store state information
    PositionNED position_NED_;
    Attitude attitude_;
    TargetPositionProfile target_position_profile_;
    DroneCmdAck drone_cmd_ack_;
    DroneState drone_state_;
    ManualControlInput manual_control_input_;
    
};

#endif // FCI_STATEMANAGER_H
