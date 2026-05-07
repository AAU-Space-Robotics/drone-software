#pragma once
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <asr_comms/msg/telemetry_status.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>




struct PoseMsg {
    struct {
        struct { double x,y,z; } position;
        struct { double x,y,z,w; } orientation;
    } pose;
};

Eigen::Matrix4d poseMsgToMatrix(const PoseMsg& poseMsg);
