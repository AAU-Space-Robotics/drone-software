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

// End of Setters & Getters

