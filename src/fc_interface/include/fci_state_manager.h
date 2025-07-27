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
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
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
    LANDED = -2,
    STANDBY = -1,
    MANUAL = 0,
    MANUAL_AIDED = 1,
    POSITION = 2,
    SAFETYLAND_BLIND = 3,
    BEGIN_LAND_POSITION = 4,
    LAND_POSITION = 5
};

enum class TrajectoryMode {
    UNINITIALIZED = -1,
    INACTIVE = 0,
    ACTIVE = 1,
    COMPLETED = 2
};

// Used to define 3D(~4D), where the last dimension is time 
struct Stamped3DVector {
    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    Eigen::Vector3d data = Eigen::Vector3d::Zero();

    // Default constructor
    Stamped3DVector() = default;

    // Constructor with timestamp and components (added for controlMode and manualAidedMode)
    Stamped3DVector(const rclcpp::Time& ts, double x, double y, double z)
        : timestamp(ts), data(x, y, z) {}

    // Get individual components
    double x() const { return data.x(); }
    double y() const { return data.y(); }
    double z() const { return data.z(); }

    // Set individual components
    void setX(double value) { data.x() = value; }
    void setY(double value) { data.y() = value; }
    void setZ(double value) { data.z() = value; }

    // Get the full vector
    const Eigen::Vector3d& vector() const { return data; }
    Eigen::Vector3d& vector() { return data; }

    // Get and set timestamp
    rclcpp::Time getTime() const { return timestamp; }
    void setTime(const rclcpp::Time& new_time) { timestamp = new_time; }
};

struct Stamped4DVector {
    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    Eigen::Vector4d data = Eigen::Vector4d::Zero();

    // Default constructor
    Stamped4DVector() = default;

    // Constructor with timestamp and components (added for execute function)
    Stamped4DVector(const rclcpp::Time& ts, double x, double y, double z, double w)
        : timestamp(ts), data(x, y, z, w) {}

    // Get individual components
    double x() const { return data.x(); }
    double y() const { return data.y(); }
    double z() const { return data.z(); }
    double w() const { return data.w(); }

    // Set individual components
    void setX(double value) { data.x() = value; }
    void setY(double value) { data.y() = value; }
    void setZ(double value) { data.z() = value; }
    void setW(double value) { data.w() = value; }

    // Get the full vector
    const Eigen::Vector4d& vector() const { return data; }
    Eigen::Vector4d& vector() { return data; }

    // Get and set timestamp
    rclcpp::Time getTime() const { return timestamp; }
    void setTime(const rclcpp::Time& new_time) { timestamp = new_time; }
};

struct StampedQuaternion {
    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    Eigen::Quaterniond data = Eigen::Quaterniond::Identity();

    // Default constructor
    StampedQuaternion() = default;

    // Constructor with timestamp and quaternion (added for attitudeCallback)
    StampedQuaternion(const rclcpp::Time& ts, const Eigen::Quaterniond& quat)
        : timestamp(ts), data(quat) {}

    // Get individual components
    double x() const { return data.x(); }
    double y() const { return data.y(); }
    double z() const { return data.z(); }
    double w() const { return data.w(); }

    // Set individual components
    void setX(double value) { data.x() = value; }
    void setY(double value) { data.y() = value; }
    void setZ(double value) { data.z() = value; }
    void setW(double value) { data.w() = value; }

    // Get the full quaternion
    const Eigen::Quaterniond& quaternion() const { return data; }
    Eigen::Quaterniond& quaternion() { return data; } // Allows modification

    // Get and set timestamp
    rclcpp::Time getTime() const { return timestamp; }
    void setTime(const rclcpp::Time& new_time) { timestamp = new_time; }
};

// DroneCmdAck structure
struct DroneCmdAck {
    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    
    //Command data
    uint16_t command = 0;
    uint8_t result = 0;
};

struct GCSHeartbeat {
    // default constructor
    GCSHeartbeat() = default;

    // constructor with timestamp and nominal state
    GCSHeartbeat(const rclcpp::Time& ts, int8_t nominal)
        : timestamp(ts), gcs_nominal(nominal) {}

    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    int8_t gcs_nominal = 0; // 0: not nominal, 1: nominal
};

// Drone state
struct DroneState {
    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    
    //Drone state data
    Command command = Command::DISARM;
    ArmingState arming_state = ArmingState::DISARMED;
    FlightMode flight_mode = FlightMode::STANDBY;
    TrajectoryMode trajectory_mode = TrajectoryMode::INACTIVE;
    rclcpp::Time trajectory_start_time_;
    rclcpp::Duration trajectory_duration = rclcpp::Duration(0, 0);
    rclcpp::Duration flight_time = rclcpp::Duration(0, 0); // Total flight time since takeoff, resets on landing/disarming
};

// Battery state
struct BatteryState {
    rclcpp::Time timestamp = rclcpp::Time(0, 0);
    
    //Battery state data
    int cell_count = 0;
    float voltage = 0.0f;
    float charge_remaining = 0.0f;
    float discharged_mah = 0.0f;
    float average_current = 0.0f;
};

// Controller errors
struct PIDError {
    double error;
    double error_d;
    double error_integral;
};

struct AccelerationError {
    PIDError X;
    PIDError Y;
    PIDError Z;
};

struct PositionError {
    PIDError X;
    PIDError Y;
    PIDError Z;
    PIDError Yaw;
};

// FCI_StateManager class definition
class FCI_StateManager {
public:
    // Thread-safe setters and getters for NED_Data
    void setGlobalPosition(const Stamped3DVector& new_data);
    Stamped3DVector getGlobalPosition();

    void setOrigin(const Stamped3DVector& new_data);
    Stamped3DVector getOrigin();

    void setGlobalVelocity(const Stamped3DVector& new_data);
    Stamped3DVector getGlobalVelocity();

    void setGlobalAcceleration(const Stamped3DVector& new_data);
    Stamped3DVector getGlobalAcceleration();

    void setAttitude(const StampedQuaternion& new_data);
    StampedQuaternion getAttitude();

    void setTargetPositionProfile(const Stamped4DVector& new_data);
    Stamped4DVector getTargetPositionProfile();

    void setTargetAttitude(const StampedQuaternion& new_data);
    StampedQuaternion getTargetAttitude();

    void setHeartbeat(const GCSHeartbeat& new_data);
    GCSHeartbeat getHeartbeat();

    void setDroneCmdAck(const DroneCmdAck& new_data);
    DroneCmdAck getDroneCmdAck();

    void setDroneState(const DroneState& new_data);
    DroneState getDroneState();

    void setBatteryState(const BatteryState& new_data);
    BatteryState getBatteryState();

    void setManualControlInput(const Stamped4DVector& new_data);
    Stamped4DVector getManualControlInput();

    void setPositionError(const PositionError& new_data);
    PositionError getPositionError();

    void setAccelerationError(const AccelerationError& new_data);
    AccelerationError getAccelerationError();

    void setLatestControlSignal(const Eigen::Vector4d& new_data);
    Eigen::Vector4d getLatestControlSignal();

    void setGroundDistanceState(const Stamped3DVector& new_data);
    Stamped3DVector getGroundDistanceState();

    void setActuatorSpeeds(const Stamped4DVector& new_data);
    Stamped4DVector getActuatorSpeeds();

private:
    
    // Mutexes for thread safety
    std::mutex heartbeat_mutex_;

    std::mutex position_global_mutex_;
    std::mutex origin_mutex_;
    std::mutex velocity_global_mutex_;
    std::mutex acceleration_global_mutex_;
    
    std::mutex ground_distance_state_mutex_;
    std::mutex attitude_data_mutex_;
    std::mutex target_position_profile_mutex_;
    std::mutex target_attitude_mutex_;
    std::mutex drone_cmd_ack_mutex_;
    std::mutex drone_state_mutex_;
    std::mutex manual_control_input_mutex_;
    std::mutex acceleration_error_mutex_;
    std::mutex position_error_mutex_;
    std::mutex latest_control_signal_mutex_;
    std::mutex battery_state_mutex_;
    std::mutex actuator_speeds_mutex_;

    // Data structures to store state information
    GCSHeartbeat gcs_heartbeat_;

    Stamped3DVector position_global_;
    Stamped3DVector origin_;
    Stamped3DVector velocity_global_;
    Stamped3DVector acceleration_global_;
    
    Stamped3DVector ground_distance_state_;
    StampedQuaternion attitude_;
    Stamped4DVector target_position_profile_;
    StampedQuaternion target_attitude_;
    DroneCmdAck drone_cmd_ack_;
    DroneState drone_state_;
    BatteryState battery_state_;
    Stamped4DVector manual_control_input_;
    AccelerationError acceleration_error_;
    PositionError position_error_;
    Eigen::Vector4d latest_control_signal_ = Eigen::Vector4d::Zero(); // Initialize to zero
    Stamped4DVector actuator_speeds_ = Stamped4DVector(rclcpp::Time(0, 0), 0.0, 0.0, 0.0, 0.0);
    
};

#endif // FCI_STATEMANAGER_H
