#include "utils.hpp"



Eigen::Matrix4d poseMsgToMatrix(const PoseMsg& poseMsg)
{
    // Extract position
    double px = poseMsg.pose.position.x;
    double py = poseMsg.pose.position.y;
    double pz = poseMsg.pose.position.z;

    // Extract quaternion
    double qx = poseMsg.pose.orientation.x;
    double qy = poseMsg.pose.orientation.y;
    double qz = poseMsg.pose.orientation.z;
    double qw = poseMsg.pose.orientation.w;

    // Quaternion → rotation matrix
    Eigen::Quaterniond q(qw, qx, qy, qz);  // Eigen expects (w,x,y,z)
    Eigen::Matrix3d R = q.toRotationMatrix();

    // Build homogeneous transform
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T(0,3) = px;
    T(1,3) = py;
    T(2,3) = pz;

    return T;
}

