#include "fci_state_manager.h"

// Setters & Getters
void FCI_StateManager::setGlobalPosition(const Stamped3DVector& new_data) {
    std::lock_guard<std::mutex> lock(position_global_mutex_);
    position_global_ = new_data;   
}

Stamped3DVector FCI_StateManager::getGlobalPosition() {
    std::lock_guard<std::mutex> lock(position_global_mutex_);
    return position_global_;   
}

// ---

void FCI_StateManager::setOrigin(const Stamped3DVector& new_data) {
    std::lock_guard<std::mutex> lock(origin_mutex_);
    origin_ = new_data;   
}

Stamped3DVector FCI_StateManager::getOrigin() {
    std::lock_guard<std::mutex> lock(origin_mutex_);
    return origin_;   
}

// ---

void FCI_StateManager::setGlobalVelocity(const Stamped3DVector& new_data) {
    std::lock_guard<std::mutex> lock(velocity_global_mutex_);
    velocity_global_ = new_data;
}

Stamped3DVector FCI_StateManager::getGlobalVelocity() {
    std::lock_guard<std::mutex> lock(velocity_global_mutex_);
    return velocity_global_;  
}

// ---

void FCI_StateManager::setGlobalAcceleration(const Stamped3DVector& new_data) {
    std::lock_guard<std::mutex> lock(acceleration_global_mutex_);
    acceleration_global_ = new_data;   
}

Stamped3DVector FCI_StateManager::getGlobalAcceleration() {
    std::lock_guard<std::mutex> lock(acceleration_global_mutex_);
    return acceleration_global_;   
}

// ---

void FCI_StateManager::setAttitude(const StampedQuaternion& new_data) {
    std::lock_guard<std::mutex> lock(attitude_data_mutex_);
    attitude_ = new_data;   
}

StampedQuaternion FCI_StateManager::getAttitude() {
    std::lock_guard<std::mutex> lock(attitude_data_mutex_);
    return attitude_;   
}

// ---

void FCI_StateManager::setTargetPositionProfile(const Stamped4DVector& new_data) {
    std::lock_guard<std::mutex> lock(target_position_profile_mutex_);
    target_position_profile_ = new_data;   
}

Stamped4DVector FCI_StateManager::getTargetPositionProfile() {
    std::lock_guard<std::mutex> lock(target_position_profile_mutex_);
    return target_position_profile_;   
}

void FCI_StateManager::setTargetAttitude(const StampedQuaternion& new_data) {
    std::lock_guard<std::mutex> lock(target_attitude_mutex_);
    target_attitude_ = new_data;   
}

StampedQuaternion FCI_StateManager::getTargetAttitude() {
    std::lock_guard<std::mutex> lock(target_attitude_mutex_);
    return target_attitude_;   
}

// ---

void FCI_StateManager::setHeartbeat(const GCSHeartbeat& new_data) {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    gcs_heartbeat_ = new_data;   
}

GCSHeartbeat FCI_StateManager::getHeartbeat() {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    return gcs_heartbeat_;   
}

// ---

void FCI_StateManager::setDroneCmdAck(const DroneCmdAck& new_data) {
    std::lock_guard<std::mutex> lock(drone_cmd_ack_mutex_);
    drone_cmd_ack_ = new_data;   
}

DroneCmdAck FCI_StateManager::getDroneCmdAck() {
    std::lock_guard<std::mutex> lock(drone_cmd_ack_mutex_);
    return drone_cmd_ack_;   
}

// ---

void FCI_StateManager::setDroneState(const DroneState& new_data) {
    std::lock_guard<std::mutex> lock(drone_state_mutex_);
    drone_state_ = new_data;   
}

DroneState FCI_StateManager::getDroneState() {
    std::lock_guard<std::mutex> lock(drone_state_mutex_);
    return drone_state_;   
}

// ---

void FCI_StateManager::setBatteryState(const BatteryState& new_data) {
    std::lock_guard<std::mutex> lock(battery_state_mutex_);
    battery_state_ = new_data;   
}

BatteryState FCI_StateManager::getBatteryState() {
    std::lock_guard<std::mutex> lock(battery_state_mutex_);
    return battery_state_;   
}

// ---

void FCI_StateManager::setManualControlInput(const Stamped4DVector& new_data) {
    std::lock_guard<std::mutex> lock(manual_control_input_mutex_);
    manual_control_input_ = new_data;   
}

Stamped4DVector FCI_StateManager::getManualControlInput() {
    std::lock_guard<std::mutex> lock(manual_control_input_mutex_);
    return manual_control_input_; 
}

// ---

void FCI_StateManager::setAccelerationError(const AccelerationError& new_data) {
    std::lock_guard<std::mutex> lock(acceleration_error_mutex_);
    acceleration_error_ = new_data;   
}

AccelerationError FCI_StateManager::getAccelerationError() {
    std::lock_guard<std::mutex> lock(acceleration_error_mutex_);
    return acceleration_error_;   
}

// ---

void FCI_StateManager::setPositionError(const PositionError& new_data) {
    std::lock_guard<std::mutex> lock(position_error_mutex_);
    position_error_ = new_data;   
}

PositionError FCI_StateManager::getPositionError() {
    std::lock_guard<std::mutex> lock(position_error_mutex_);
    return position_error_;   
}

// ---

void FCI_StateManager::setLatestControlSignal(const Eigen::Vector4d& new_data) {
    std::lock_guard<std::mutex> lock(latest_control_signal_mutex_);
    latest_control_signal_ = new_data;   
}
Eigen::Vector4d FCI_StateManager::getLatestControlSignal() {
    std::lock_guard<std::mutex> lock(latest_control_signal_mutex_);
    return latest_control_signal_;   
}

// ---


void FCI_StateManager::setGroundDistanceState(const Stamped3DVector& new_data) {
    std::lock_guard<std::mutex> lock(ground_distance_state_mutex_);
    ground_distance_state_ = new_data;   
}
Stamped3DVector FCI_StateManager::getGroundDistanceState() {
    std::lock_guard<std::mutex> lock(ground_distance_state_mutex_);
    return ground_distance_state_;   
}

// ---

void FCI_StateManager::setActuatorSpeeds(const Stamped4DVector& new_data) {
    std::lock_guard<std::mutex> lock(actuator_speeds_mutex_);
    actuator_speeds_ = new_data;   
}
Stamped4DVector FCI_StateManager::getActuatorSpeeds() {
    std::lock_guard<std::mutex> lock(actuator_speeds_mutex_);
    return actuator_speeds_;
}

// End of Setters & Getters

