#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <eigen3/Eigen/Dense>
#include "transformations.h"
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

struct Waypoint {
    Eigen::Vector3d position;
    double yaw;
    double linear_velocity;   // 0 = use default
    double angular_velocity;  // 0 = use default
};

struct TrajectorySegmentInfo {
    double start_time;
    double duration;
    int segment_index;  // Which waypoint segment this belongs to
};

struct ConstraintCheckResult {
    bool satisfied;
    double max_velocity;
    double max_acceleration;
    double time_at_max_velocity;
    double time_at_max_acceleration;
};

enum trajectoryMethod {
    MIN_SNAP
};

class PathPlanner {
public:
    PathPlanner();

    float current_linear_velocity_ = 0.15;    
    float current_angular_velocity_ = 0.15;   
    float min_linear_velocity_ = 0.1;        
    float min_angular_velocity_ = 0.1;       
    float max_linear_velocity_ = 0.2;        
    float max_angular_velocity_ = 0.2;       
      
    double getTotalTime() const;
    
    // Multi-waypoint trajectory generation
    bool GenerateMultiWaypointTrajectory(
        const std::vector<Waypoint>& waypoints,
        const Eigen::Vector3d& start_velocity,
        const Eigen::Vector3d& start_acceleration,
        double start_yaw,
        trajectoryMethod method
    );
    
    // Get segment info for multi-waypoint trajectories
    std::vector<TrajectorySegmentInfo> getSegmentInfo() const;
    
    // Check if trajectory satisfies velocity/acceleration constraints
    ConstraintCheckResult checkConstraints(int num_samples = 100) const;

    bool GenerateTrajectory(
        const Eigen::Vector3d& start_pos,
        const Eigen::Vector3d& end_pos,
        const Eigen::Quaterniond& start_quat,
        const Eigen::Quaterniond& end_quat,
        const Eigen::Vector3d& vel,
        const Eigen::Vector3d& acc,
        trajectoryMethod method
    );

    bool GenerateSpinTrajectory(
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& start_quat,
        double target_yaw,
        double num_rotations,
        bool use_longest_path,
        trajectoryMethod method
    );

    TrajectoryPoint evaluatePolynomial(
        const std::vector<double>& coefficients,
        double t
    );

    FullTrajectoryPoint getTrajectoryPoint(
        double t,
        trajectoryMethod method
    );

    bool setLinearVelocity(float linear_velocity);

    bool setAngularVelocity(float angular_velocity);

private:
    double total_time;
    Eigen::Vector3d start_vel;
    Eigen::Vector3d start_acc;
    trajectorySegment segments[3]; 
    Eigen::Quaterniond start_quat;
    Eigen::Quaterniond end_quat;
    trajectorySegment yaw_segment;
    bool use_yaw_polynomial = false;
    
    // Multi-waypoint support
    std::vector<TrajectorySegmentInfo> segment_info_;
    std::vector<trajectorySegment> multi_segments_x_;
    std::vector<trajectorySegment> multi_segments_y_;
    std::vector<trajectorySegment> multi_segments_z_;
    std::vector<trajectorySegment> multi_segments_yaw_;
    std::vector<Eigen::Quaterniond> segment_start_quats_;
    std::vector<Eigen::Quaterniond> segment_end_quats_;
    bool is_multi_waypoint_ = false;

    std::vector<double> generatePolynomialCoefficients(
        double start,
        double end,
        double start_vel,
        double start_acc,
        double time,
        trajectoryMethod method
    );

    float calculateDuration(float distance, float velocity, float min_velocity, float max_velocity) const;

    Transformations transformations_;
};

#endif // PATHPLANNER_H