#include "fci_state_manager.h"

// Setters & Getters
void FCI_StateManager::setPositionNED(const PositionNED& new_data) {
    std::lock_guard<std::mutex> lock(position_NED_mutex_);
    position_NED_ = new_data;  // Thread-safe copy
}

PositionNED FCI_StateManager::getPositionNED() {
    std::lock_guard<std::mutex> lock(position_NED_mutex_);
    return position_NED_;  // Thread-safe copy
}

void FCI_StateManager::setAttitude(const Attitude& new_data) {
    std::lock_guard<std::mutex> lock(attitude_data_mutex_);
    attitude_ = new_data;  // Thread-safe copy
}

Attitude FCI_StateManager::getAttitude() {
    std::lock_guard<std::mutex> lock(attitude_data_mutex_);
    return attitude_;  // Thread-safe copy
}

void FCI_StateManager::setTargetPositionProfile(const TargetPositionProfile& new_data) {
    std::lock_guard<std::mutex> lock(target_position_profile_mutex_);
    target_position_profile_ = new_data;  // Thread-safe copy
}

TargetPositionProfile FCI_StateManager::getTargetPositionProfile() {
    std::lock_guard<std::mutex> lock(target_position_profile_mutex_);
    return target_position_profile_;  // Thread-safe copy
}

void FCI_StateManager::setDroneCmdAck(const DroneCmdAck& new_data) {
    std::lock_guard<std::mutex> lock(drone_cmd_ack_mutex_);
    drone_cmd_ack_ = new_data;  // Thread-safe copy
}

DroneCmdAck FCI_StateManager::getDroneCmdAck() {
    std::lock_guard<std::mutex> lock(drone_cmd_ack_mutex_);
    return drone_cmd_ack_;  // Thread-safe copy
}

void FCI_StateManager::setDroneState(const DroneState& new_data) {
    std::lock_guard<std::mutex> lock(drone_state_mutex_);
    drone_state_ = new_data;  // Thread-safe copy
}

DroneState FCI_StateManager::getDroneState() {
    std::lock_guard<std::mutex> lock(drone_state_mutex_);
    return drone_state_;  // Thread-safe copy
}

void FCI_StateManager::setManualControlInput(const ManualControlInput& new_data) {
    std::lock_guard<std::mutex> lock(manual_control_input_mutex_);
    manual_control_input_ = new_data;  // Thread-safe copy
}

ManualControlInput FCI_StateManager::getManualControlInput() {
    std::lock_guard<std::mutex> lock(manual_control_input_mutex_);
    return manual_control_input_;  // Thread-safe copy
}

