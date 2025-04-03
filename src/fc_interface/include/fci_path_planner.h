#ifndef FCI_PATHPLANNER_H
#define FCI_PATHPLANNER_H

#include <eigen3/Eigen/Dense>
#include <vector>

class Vector3d {
public:
    Eigen::Vector3d data = Eigen::Vector3d::Zero();

    Vector3d() = default;
    Vector3d(double x, double y, double z) : data(x, y, z) {}

    double x() const { return data.x(); }
    double y() const { return data.y(); }
    double z() const { return data.z(); }

    void setX(double value) { data.x() = value; }
    void setY(double value) { data.y() = value; }
    void setZ(double value) { data.z() = value; }

    const Eigen::Vector3d& vector() const { return data; }
    Eigen::Vector3d& vector() { return data; }
};

struct TrajectoryPoint {
    double position;
    double velocity;
    double acceleration;
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

    bool GenerateTrajectory(
        const Vector3d& start,
        const Vector3d& end,
        const Vector3d& vel,
        const Vector3d& acc,
        double time,
        trajectoryMethod method
    );

    TrajectoryPoint evaluatePolynomial(
        const std::vector<double>& coefficients,
        double t
    );

    std::vector<Vector3d> getTrajectoryPoints(  // Changed to Vector3d
        double dt,
        trajectoryMethod method
    );

private:
    double total_time;
    Vector3d start_vel;
    Vector3d start_acc;
    trajectorySegment segments[3];

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