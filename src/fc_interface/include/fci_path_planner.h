#ifndef FCI_PATHPLANNER_H
#define FCI_PATHPLANNER_H

#include <eigen3/Eigen/Dense>
#include <vector>

struct TrajectoryPoint {
    double position;
    double velocity;
    double acceleration;
};

struct FullTrajectoryPoint {
    Eigen::Vector3d position;        // x, y, z
    Eigen::Quaterniond orientation;  // Quaternion for yaw
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
};

struct trajectorySegment {
    std::vector<double> coefficient;
};

enum trajectoryMethod {
    MIN_SNAP
};

class FCI_PathPlanner {
public:
    FCI_PathPlanner();

    float getTotalTime() const;

    float calculateDuration(float distance, float velocity) const;

    bool GenerateTrajectory(
        const Eigen::Vector3d& start_pos,
        const Eigen::Vector3d& end_pos,
        const Eigen::Quaterniond& start_quat,
        const Eigen::Quaterniond& end_quat,
        const Eigen::Vector3d& vel,
        const Eigen::Vector3d& acc,
        double time,
        trajectoryMethod method
    );

    TrajectoryPoint evaluatePolynomial(
        const std::vector<double>& coefficients,
        double t
    );

    std::vector<FullTrajectoryPoint> getTrajectoryPoints(
        double dt,
        trajectoryMethod method
    );

    FullTrajectoryPoint getTrajectoryPoint(
        double t,
        trajectoryMethod method
    );

private:
    double total_time;
    Eigen::Vector3d start_vel;
    Eigen::Vector3d start_acc;
    trajectorySegment segments[3]; // x, y, z
    Eigen::Quaterniond start_quat;
    Eigen::Quaterniond end_quat;

    std::vector<double> generatePolynomialCoefficients(
        double start,
        double end,
        double start_vel,
        double start_acc,
        double time,
        trajectoryMethod method
    );
};

#endif // FCI_PATHPLANNER_H