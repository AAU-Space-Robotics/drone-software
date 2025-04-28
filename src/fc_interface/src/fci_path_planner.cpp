#include "fci_path_planner.h"
#include <cmath>

FCI_PathPlanner::FCI_PathPlanner() : total_time(0.0) {
    // start_vel and start_acc are initialized to zero by Vector3d's default constructor
}

float FCI_PathPlanner::getTotalTime() const {
    return total_time;
}

float FCI_PathPlanner::calculateDuration(float distance, float velocity) const {
    const float MAX_VELOCITY = 1.5f; // Maximum allowed velocity (m/s)
    const float MIN_VELOCITY = 0.1f;  // Minimum allowed velocity (m/s)

    // Clamp velocity to valid range
    float safe_velocity = std::clamp(velocity, MIN_VELOCITY, MAX_VELOCITY);

    // Calculate and return duration
    return distance / safe_velocity;
}


bool FCI_PathPlanner::GenerateTrajectory(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const Eigen::Vector3d& current_velocity,
    const Eigen::Vector3d& current_acceleration,
    double time,
    trajectoryMethod method) {

    total_time = time;

    start_vel = current_velocity;
    start_acc = current_acceleration;

    if (method == MIN_SNAP) {
        segments[0].coefficient = generatePolynomialCoefficients(start(0), end(0), start_vel(0), start_acc(0), time, method);
        segments[1].coefficient = generatePolynomialCoefficients(start(1), end(1), start_vel(1), start_acc(1), time, method);
        segments[2].coefficient = generatePolynomialCoefficients(start(2), end(2), start_vel(2), start_acc(2), time, method);
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
        point.velocity Eigen::Vector3d(x.velocity, y.velocity, z.velocity);
        point.acceleration Eigen::Vector3d(x.acceleration, y.acceleration, z.acceleration);
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