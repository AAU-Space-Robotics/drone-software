#include "fci_path_planner.h"
#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <iostream>

FCI_PathPlanner::FCI_PathPlanner() {
    // start_vel and start_acc are initialized to zero by Vector3d's default constructor
}

double FCI_PathPlanner::getTotalTime() const {
    return total_time;
}

float FCI_PathPlanner::calculateDuration(float distance, float velocity, float min_velocity, float max_velocity) const {
    // Clamp velocity to valid range
    float safe_velocity = std::clamp(velocity, min_velocity, max_velocity);

    // Calculate and return duration
    return distance / safe_velocity;
}

bool FCI_PathPlanner::GenerateTrajectory(
    const Eigen::Vector3d& start_pos,
    const Eigen::Vector3d& end_pos,
    const Eigen::Quaterniond& start_quat,
    const Eigen::Quaterniond& end_quat,
    const Eigen::Vector3d& current_velocity,
    const Eigen::Vector3d& current_acceleration,
    trajectoryMethod method) {

    // Calculate the time required for the trajectory based on distance and velocity
    float distance = (end_pos - start_pos).norm();
    float current_yaw = transformations_.unwrapAngle(transformations_.quaternionToEuler(start_quat).x(), 2*M_PI, 0);
    float target_yaw = transformations_.unwrapAngle(transformations_.quaternionToEuler(end_quat).x(), 2*M_PI, 0);
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

TrajectoryPoint FCI_PathPlanner::evaluatePolynomial(const std::vector<double>& coefficients, double t) {
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

std::vector<FullTrajectoryPoint> FCI_PathPlanner::getTrajectoryPoints(double dt, trajectoryMethod method) {
    std::vector<FullTrajectoryPoint> points;
    for (double t = 0; t <= total_time; t += dt) {
        FullTrajectoryPoint point;
        if (method == MIN_SNAP) {
            TrajectoryPoint x = evaluatePolynomial(segments[0].coefficient, t);
            TrajectoryPoint y = evaluatePolynomial(segments[1].coefficient, t);
            TrajectoryPoint z = evaluatePolynomial(segments[2].coefficient, t);
            point.position = Eigen::Vector3d(x.position, y.position, z.position);
            point.velocity = Eigen::Vector3d(x.velocity, y.velocity, z.velocity);
            point.acceleration = Eigen::Vector3d(x.acceleration, y.acceleration, z.acceleration);
            // Slerp for quaternion interpolation
            Eigen::Quaterniond slerp_quat = end_quat;

            // Ensure SLERP takes the shortest path
            if (start_quat.dot(end_quat) < 0.0) {
                slerp_quat.coeffs() *= -1.0;
            }

            point.orientation = start_quat.slerp(t / total_time, slerp_quat).normalized();
        }
        points.push_back(point);
    }
    return points;
}

FullTrajectoryPoint FCI_PathPlanner::getTrajectoryPoint(double t, trajectoryMethod method) {
    FullTrajectoryPoint point;
    if (method == MIN_SNAP) {
        TrajectoryPoint x = evaluatePolynomial(segments[0].coefficient, t);
        TrajectoryPoint y = evaluatePolynomial(segments[1].coefficient, t);
        TrajectoryPoint z = evaluatePolynomial(segments[2].coefficient, t);
        point.position = Eigen::Vector3d(x.position, y.position, z.position);
        point.velocity = Eigen::Vector3d(x.velocity, y.velocity, z.velocity);
        point.acceleration = Eigen::Vector3d(x.acceleration, y.acceleration, z.acceleration);
        // Slerp for quaternion interpolation
        Eigen::Quaterniond slerp_quat = end_quat;
        
        // Ensure SLERP takes the shortest path
        if (start_quat.dot(end_quat) < 0.0) {
            slerp_quat.coeffs() *= -1.0;
        }

        point.orientation = start_quat.slerp(t / total_time, slerp_quat).normalized();
    }
    return point;
}

std::vector<double> FCI_PathPlanner::generatePolynomialCoefficients(
    double start, double end, double start_vel, double start_acc, double time, trajectoryMethod method) {

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


bool FCI_PathPlanner::setLinearVelocity(float linear_velocity) {
    if (linear_velocity < min_linear_velocity_ || linear_velocity > max_linear_velocity_) {
        return false; // Invalid velocity
    }
    current_linear_velocity_ = linear_velocity;
    return true;
}

bool FCI_PathPlanner::setAngularVelocity(float angular_velocity){
    if (angular_velocity < min_angular_velocity_ || angular_velocity > max_angular_velocity_) {
        return false; // Invalid velocity
    }
    current_angular_velocity_ = angular_velocity;
    return true;
}