#ifndef FCI_PATHPLANNER_H
#define FCI_PATHPLANNER_H

#include <eigen3/Eigen/Dense>
#include "fci_transformations.h"
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

    float current_linear_velocity_ = 0.15;    
    float current_angular_velocity_ = 0.15;   
    float min_linear_velocity_ = 0.1;        
    float min_angular_velocity_ = 0.1;       
    float max_linear_velocity_ = 0.2;        
    float max_angular_velocity_ = 0.2;       
      
    double getTotalTime() const;

    bool GenerateTrajectory(
        const Eigen::Vector3d& start_pos,
        const Eigen::Vector3d& end_pos,
        const Eigen::Quaterniond& start_quat,
        const Eigen::Quaterniond& end_quat,
        const Eigen::Vector3d& vel,
        const Eigen::Vector3d& acc,
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

    float calculateDuration(float distance, float velocity, float min_velocity, float max_velocity)  const;

    FCI_Transformations transformations_;
};

#endif // FCI_PATHPLANNER_H