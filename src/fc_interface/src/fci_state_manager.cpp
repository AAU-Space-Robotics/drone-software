#include "fci_state_manager.h"

// Setters & Getters
void FCI_StateManager::setPositionNED(const PositionNED& new_data) {
    std::lock_guard<std::mutex> lock(position_NED_mutex_);
    position_NED_ = new_data;   
}

PositionNED FCI_StateManager::getPositionNED() {
    std::lock_guard<std::mutex> lock(position_NED_mutex_);
    return position_NED_;   
}

void FCI_StateManager::setAccelerationNED(const AccelerationNED& new_data) {
    std::lock_guard<std::mutex> lock(acceleration_NED_mutex_);
    acceleration_NED_ = new_data;   
}

AccelerationNED FCI_StateManager::getAccelerationNED() {
    std::lock_guard<std::mutex> lock(acceleration_NED_mutex_);
    return acceleration_NED_;   
}

void FCI_StateManager::setAttitude(const Attitude& new_data) {
    std::lock_guard<std::mutex> lock(attitude_data_mutex_);
    attitude_ = new_data;   
}

Attitude FCI_StateManager::getAttitude() {
    std::lock_guard<std::mutex> lock(attitude_data_mutex_);
    return attitude_;   
}

void FCI_StateManager::setTargetPositionProfile(const TargetPositionProfile& new_data) {
    std::lock_guard<std::mutex> lock(target_position_profile_mutex_);
    target_position_profile_ = new_data;   
}

TargetPositionProfile FCI_StateManager::getTargetPositionProfile() {
    std::lock_guard<std::mutex> lock(target_position_profile_mutex_);
    return target_position_profile_;   
}

void FCI_StateManager::setDroneCmdAck(const DroneCmdAck& new_data) {
    std::lock_guard<std::mutex> lock(drone_cmd_ack_mutex_);
    drone_cmd_ack_ = new_data;   
}

DroneCmdAck FCI_StateManager::getDroneCmdAck() {
    std::lock_guard<std::mutex> lock(drone_cmd_ack_mutex_);
    return drone_cmd_ack_;   
}

void FCI_StateManager::setDroneState(const DroneState& new_data) {
    std::lock_guard<std::mutex> lock(drone_state_mutex_);
    drone_state_ = new_data;   
}

DroneState FCI_StateManager::getDroneState() {
    std::lock_guard<std::mutex> lock(drone_state_mutex_);
    return drone_state_;   
}

void FCI_StateManager::setManualControlInput(const ManualControlInput& new_data) {
    std::lock_guard<std::mutex> lock(manual_control_input_mutex_);
    manual_control_input_ = new_data;   
}

ManualControlInput FCI_StateManager::getManualControlInput() {
    std::lock_guard<std::mutex> lock(manual_control_input_mutex_);
    return manual_control_input_; 
}

void FCI_StateManager::setAccelerationError(const AccelerationError& new_data) {
    std::lock_guard<std::mutex> lock(acceleration_error_mutex_);
    acceleration_error_ = new_data;   
}

AccelerationError FCI_StateManager::getAccelerationError() {
    std::lock_guard<std::mutex> lock(acceleration_error_mutex_);
    return acceleration_error_;   
}

void FCI_StateManager::setPositionError(const PositionError& new_data) {
    std::lock_guard<std::mutex> lock(position_error_mutex_);
    position_error_ = new_data;   
}

PositionError FCI_StateManager::getPositionError() {
    std::lock_guard<std::mutex> lock(position_error_mutex_);
    return position_error_;   
}

// End of Setters & Getters

