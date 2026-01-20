#include "path_planner.h"
#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <iostream>

PathPlanner::PathPlanner() {
    // start_vel and start_acc are initialized to zero by Vector3d's default constructor
}

double PathPlanner::getTotalTime() const {
    return total_time;
}

float PathPlanner::calculateDuration(float distance, float velocity, float min_velocity, float max_velocity) const {
    // Clamp velocity to valid range
    float safe_velocity = std::clamp(velocity, min_velocity, max_velocity);

    // Calculate and return duration
    return distance / safe_velocity;
}

bool PathPlanner::GenerateTrajectory(
    const Eigen::Vector3d& start_pos,
    const Eigen::Vector3d& end_pos,
    const Eigen::Quaterniond& start_quat,
    const Eigen::Quaterniond& end_quat,
    const Eigen::Vector3d& current_velocity,
    const Eigen::Vector3d& current_acceleration,
    trajectoryMethod method) {
    use_yaw_polynomial = false;
    is_multi_waypoint_ = false;  // Single segment trajectory

    // Calculate the time required for the trajectory based on distance and velocity
    float distance = (end_pos - start_pos).norm();
    float current_yaw = transformations_.unwrapAngle(transformations_.quaternionToEuler(start_quat).x(), 2 * M_PI, 0);
    float target_yaw = transformations_.unwrapAngle(transformations_.quaternionToEuler(end_quat).x(), 2 * M_PI, 0);
    float distance_angular = std::fabs(std::atan2(std::sin(target_yaw - current_yaw), std::cos(target_yaw - current_yaw)));
    float trajectory_duration_cartesian = calculateDuration(distance, current_linear_velocity_, min_linear_velocity_, max_linear_velocity_);
    float trajectory_duration_angular = calculateDuration(distance_angular, current_angular_velocity_, min_angular_velocity_, max_angular_velocity_);
    total_time = std::max(trajectory_duration_cartesian, trajectory_duration_angular);
    start_vel = current_velocity;
    start_acc = current_acceleration;
    this->start_quat = start_quat.normalized();
    this->end_quat = end_quat.normalized();
   
    if (method == MIN_SNAP) {
        for (int i = 0; i < 3; ++i) {
            segments[i].coefficient = generatePolynomialCoefficients(
                start_pos(i), end_pos(i), start_vel(i), start_acc(i), total_time, method);
        }
    }
    return true;
}

bool PathPlanner::GenerateSpinTrajectory(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& start_quat,
    double target_yaw,
    double num_rotations,
    bool use_longest_path,
    trajectoryMethod method) {
    // Position remains constant (no translation during spin)
    // Get current yaw from start quaternion
    float current_yaw = transformations_.unwrapAngle(transformations_.quaternionToEuler(start_quat).x(), 2 * M_PI, 0);
    
    // Normalize target yaw to [0, 2π)
    float target_yaw_normalized = transformations_.unwrapAngle(target_yaw, 2 * M_PI, 0);
    
    // Calculate angular distance for the final yaw adjustment
    float delta_yaw = target_yaw_normalized - current_yaw;
    if (use_longest_path) {
        // Choose the longer path by adjusting delta_yaw
        if (delta_yaw > 0) {
            delta_yaw -= 2 * M_PI;
        } else {
            delta_yaw += 2 * M_PI;
        }
    } else {
        // Ensure shortest path by wrapping delta_yaw to [-π, π]
        delta_yaw = std::atan2(std::sin(delta_yaw), std::cos(delta_yaw));
    }
    
    // Add additional rotations (each rotation is 2π)
    float total_angular_distance = std::fabs(delta_yaw) + num_rotations * 2 * M_PI;
    
    // Calculate duration based on angular velocity
    total_time = calculateDuration(total_angular_distance, current_angular_velocity_, min_angular_velocity_, max_angular_velocity_);
    
    // Compute the effective target yaw including rotations
    double effective_target_yaw = current_yaw + delta_yaw + num_rotations * 2 * M_PI * (delta_yaw >= 0 ? 1 : -1);
    
    // Set yaw polynomial (unwrapped)
    double start_yaw = current_yaw;
    yaw_segment.coefficient = generatePolynomialCoefficients(start_yaw, effective_target_yaw, 0.0, 0.0, total_time, method);
    use_yaw_polynomial = true;
    
    // Store normalized start quaternion (end not needed for spin)
    this->start_quat = start_quat.normalized();
    
    // Set zero velocity and acceleration for position
    start_vel = Eigen::Vector3d::Zero();
    start_acc = Eigen::Vector3d::Zero();
    
    // Generate constant position polynomials (no movement)
    if (method == MIN_SNAP) {
        for (int i = 0; i < 3; ++i) {
            segments[i].coefficient = std::vector<double>{position(i), 0, 0, 0, 0, 0, 0, 0};
        }
    }
    
    return true;
}

TrajectoryPoint PathPlanner::evaluatePolynomial(const std::vector<double>& coefficients, double t) {
    TrajectoryPoint point{0.0, 0.0, 0.0};
    for (size_t i = 0; i < coefficients.size(); ++i) {
        point.position += coefficients[i] * std::pow(t, i);
    }
    for (size_t i = 1; i < coefficients.size(); ++i) {
        point.velocity += i * coefficients[i] * std::pow(t, i - 1);
    }
    for (size_t i = 2; i < coefficients.size(); ++i) {
        point.acceleration += i * (i - 1) * coefficients[i] * std::pow(t, i - 2);
    }
    return point;
}

FullTrajectoryPoint PathPlanner::getTrajectoryPoint(double t, trajectoryMethod method) {
    FullTrajectoryPoint point;
    
    if (is_multi_waypoint_) {
        // Find which segment this time belongs to
        int segment_idx = -1;
        double local_t = t;
        
        for (size_t i = 0; i < segment_info_.size(); ++i) {
            if (t >= segment_info_[i].start_time && 
                t <= segment_info_[i].start_time + segment_info_[i].duration) {
                segment_idx = i;
                local_t = t - segment_info_[i].start_time;
                break;
            }
        }
        
        // If past the end, use the last segment at its final time
        if (segment_idx == -1 && !segment_info_.empty()) {
            segment_idx = segment_info_.size() - 1;
            local_t = segment_info_[segment_idx].duration;
        }
        
        if (segment_idx >= 0 && segment_idx < static_cast<int>(multi_segments_x_.size())) {
            TrajectoryPoint x = evaluatePolynomial(multi_segments_x_[segment_idx].coefficient, local_t);
            TrajectoryPoint y = evaluatePolynomial(multi_segments_y_[segment_idx].coefficient, local_t);
            TrajectoryPoint z = evaluatePolynomial(multi_segments_z_[segment_idx].coefficient, local_t);
            TrajectoryPoint yaw = evaluatePolynomial(multi_segments_yaw_[segment_idx].coefficient, local_t);
            
            point.position = Eigen::Vector3d(x.position, y.position, z.position);
            point.velocity = Eigen::Vector3d(x.velocity, y.velocity, z.velocity);
            point.acceleration = Eigen::Vector3d(x.acceleration, y.acceleration, z.acceleration);
            point.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw.position, Eigen::Vector3d::UnitZ())).normalized();
        }
    } else if (method == MIN_SNAP) {
        TrajectoryPoint x = evaluatePolynomial(segments[0].coefficient, t);
        TrajectoryPoint y = evaluatePolynomial(segments[1].coefficient, t);
        TrajectoryPoint z = evaluatePolynomial(segments[2].coefficient, t);
        point.position = Eigen::Vector3d(x.position, y.position, z.position);
        point.velocity = Eigen::Vector3d(x.velocity, y.velocity, z.velocity);
        point.acceleration = Eigen::Vector3d(x.acceleration, y.acceleration, z.acceleration);

        if (use_yaw_polynomial) {
            // Use unwrapped yaw polynomial for orientation (for spin)
            TrajectoryPoint yaw_pt = evaluatePolynomial(yaw_segment.coefficient, t);
            point.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw_pt.position, Eigen::Vector3d::UnitZ())).normalized();
        } else if (start_quat.normalized().isApprox(end_quat.normalized(), 1e-6)) {
            // No rotation: use start quaternion directly
            point.orientation = start_quat.normalized();
        } else {
            // SLERP for non-spin, non-identical quaternions
            Eigen::Quaterniond slerp_quat = end_quat;
            if (start_quat.dot(end_quat) < 0.0) {
                slerp_quat.coeffs() *= -1.0;
            }
            point.orientation = start_quat.slerp(t / total_time, slerp_quat).normalized();
        }
    }
    return point;
}


std::vector<double> PathPlanner::generatePolynomialCoefficients(
    double start, double end, double start_vel, double start_acc, double time, trajectoryMethod /*method*/) {

    Eigen::MatrixXd A(8, 8);
    Eigen::VectorXd b(8);

    A << 1, 0, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0, 0,
         0, 0, 2, 0, 0, 0, 0, 0,
         0, 0, 0, 6, 0, 0, 0, 0,
         1, time, pow(time, 2), pow(time, 3), pow(time, 4), pow(time, 5), pow(time, 6), pow(time, 7),
         0, 1, 2 * time, 3 * pow(time, 2), 4 * pow(time, 3), 5 * pow(time, 4), 6 * pow(time, 5), 7 * pow(time, 6),
         0, 0, 2, 6 * time, 12 * pow(time, 2), 20 * pow(time, 3), 30 * pow(time, 4), 42 * pow(time, 5),
         0, 0, 0, 6, 24 * time, 60 * pow(time, 2), 120 * pow(time, 3), 210 * pow(time, 4);

    b << start, start_vel, start_acc, 0, end, 0, 0, 0;

    Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);

    return std::vector<double>{coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7]};
}

bool PathPlanner::setLinearVelocity(float linear_velocity) {
    if (linear_velocity < min_linear_velocity_ || linear_velocity > max_linear_velocity_) {
        return false; // Invalid velocity
    }
    current_linear_velocity_ = linear_velocity;
    return true;
}

bool PathPlanner::setAngularVelocity(float angular_velocity) {
    if (angular_velocity < min_angular_velocity_ || angular_velocity > max_angular_velocity_) {
        return false; // Invalid velocity
    }
    current_angular_velocity_ = angular_velocity;
    return true;
}

bool PathPlanner::GenerateMultiWaypointTrajectory(
    const std::vector<Waypoint>& waypoints, 
    const Eigen::Vector3d& start_velocity,
    const Eigen::Vector3d& start_acceleration,
    double /*start_yaw*/,
    trajectoryMethod method) {    if (waypoints.empty()) {
        return false;
    }
    
    is_multi_waypoint_ = true;
    use_yaw_polynomial = false;
    
    // Clear previous multi-waypoint data
    multi_segments_x_.clear();
    multi_segments_y_.clear();
    multi_segments_z_.clear();
    multi_segments_yaw_.clear();
    segment_info_.clear();
    segment_start_quats_.clear();
    segment_end_quats_.clear();
    
    // Current state for trajectory generation
    Eigen::Vector3d current_vel = start_velocity;
    Eigen::Vector3d current_acc = start_acceleration;
    double cumulative_time = 0.0;
    
    // Generate trajectory segments between consecutive waypoints
    for (size_t i = 1; i < waypoints.size(); ++i) {
        const Waypoint& start_wp = waypoints[i - 1];
        const Waypoint& end_wp = waypoints[i];
        
        // Determine velocities for this segment
        float linear_vel = (start_wp.linear_velocity > 0) ? start_wp.linear_velocity : current_linear_velocity_;
        float angular_vel = (start_wp.angular_velocity > 0) ? start_wp.angular_velocity : current_angular_velocity_;
        
        // Calculate segment duration
        float distance = (end_wp.position - start_wp.position).norm();
        float yaw_diff = std::fabs(std::atan2(std::sin(end_wp.yaw - start_wp.yaw), 
                                               std::cos(end_wp.yaw - start_wp.yaw)));
        float duration_linear = calculateDuration(distance, linear_vel, min_linear_velocity_, max_linear_velocity_);
        float duration_angular = calculateDuration(yaw_diff, angular_vel, min_angular_velocity_, max_angular_velocity_);
        float segment_duration = std::max(duration_linear, duration_angular);
        
        // Generate polynomial coefficients for x, y, z
        trajectorySegment seg_x, seg_y, seg_z, seg_yaw;
        if (method == MIN_SNAP) {
            seg_x.coefficient = generatePolynomialCoefficients(
                start_wp.position(0), end_wp.position(0), 
                current_vel(0), current_acc(0), segment_duration, method);
            seg_y.coefficient = generatePolynomialCoefficients(
                start_wp.position(1), end_wp.position(1), 
                current_vel(1), current_acc(1), segment_duration, method);
            seg_z.coefficient = generatePolynomialCoefficients(
                start_wp.position(2), end_wp.position(2), 
                current_vel(2), current_acc(2), segment_duration, method);
            seg_yaw.coefficient = generatePolynomialCoefficients(
                start_wp.yaw, end_wp.yaw, 0.0, 0.0, segment_duration, method);
        }
        
        multi_segments_x_.push_back(seg_x);
        multi_segments_y_.push_back(seg_y);
        multi_segments_z_.push_back(seg_z);
        multi_segments_yaw_.push_back(seg_yaw);
        
        // Store quaternions for this segment
        Eigen::Quaterniond start_q(Eigen::AngleAxisd(start_wp.yaw, Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond end_q(Eigen::AngleAxisd(end_wp.yaw, Eigen::Vector3d::UnitZ()));
        segment_start_quats_.push_back(start_q.normalized());
        segment_end_quats_.push_back(end_q.normalized());
        
        // Store segment info
        TrajectorySegmentInfo info;
        info.start_time = cumulative_time;
        info.duration = segment_duration;
        info.segment_index = i - 1;
        segment_info_.push_back(info);
        
        // Update state for next iteration (velocity and acceleration at end of segment)
        TrajectoryPoint x_end = evaluatePolynomial(seg_x.coefficient, segment_duration);
        TrajectoryPoint y_end = evaluatePolynomial(seg_y.coefficient, segment_duration);
        TrajectoryPoint z_end = evaluatePolynomial(seg_z.coefficient, segment_duration);
        current_vel = Eigen::Vector3d(x_end.velocity, y_end.velocity, z_end.velocity);
        current_acc = Eigen::Vector3d(x_end.acceleration, y_end.acceleration, z_end.acceleration);
        cumulative_time += segment_duration;
    }
    
    total_time = cumulative_time;
    return true;
}

std::vector<TrajectorySegmentInfo> PathPlanner::getSegmentInfo() const {
    return segment_info_;
}

ConstraintCheckResult PathPlanner::checkConstraints(int num_samples) const {
    ConstraintCheckResult result;
    result.satisfied = true;
    result.max_velocity = 0.0;
    result.max_acceleration = 0.0;
    result.time_at_max_velocity = 0.0;
    result.time_at_max_acceleration = 0.0;
    
    if (total_time <= 0) {
        return result;
    }
    
    double dt = total_time / num_samples;
    
    for (int i = 0; i <= num_samples; ++i) {
        double t = i * dt;
        if (t > total_time) t = total_time;
        
        FullTrajectoryPoint point = const_cast<PathPlanner*>(this)->getTrajectoryPoint(t, MIN_SNAP);
        
        double vel_mag = point.velocity.norm();
        double acc_mag = point.acceleration.norm();
        
        if (vel_mag > result.max_velocity) {
            result.max_velocity = vel_mag;
            result.time_at_max_velocity = t;
        }
        
        if (acc_mag > result.max_acceleration) {
            result.max_acceleration = acc_mag;
            result.time_at_max_acceleration = t;
        }
        
        // Check constraints
        if (vel_mag > max_linear_velocity_) {
            result.satisfied = false;
        }
    }
    
    return result;
}