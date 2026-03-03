#include <iostream>
#include <vector>
#include <eigen3/Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcutils/logging.h>


#include <interfaces/action/drone_command.hpp>
#include <interfaces/msg/manual_control_input.hpp>
#include <interfaces/msg/motion_capture_pose.hpp>
#include <interfaces/msg/drone_state.hpp>
#include <interfaces/msg/drone_scope.hpp>
#include <interfaces/msg/gcs_heartbeat.hpp>
#include <interfaces/msg/probe_global_locations.hpp>
#include <interfaces/msg/attitude_setpoint_rpy.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"


#include "utils.hpp"


class ThyraMissionExecutor : public rclcpp::Node
{
public:
    using DroneCommand = interfaces::action::DroneCommand;
    using GoalHandleDroneCommand = rclcpp_action::ServerGoalHandle<DroneCommand>;



    ThyraMissionExecutor()
    : Node("thyra_mission_node")
    {
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos.durability(rclcpp::DurabilityPolicy::TransientLocal);


        std::cout << "\n"
                  << "=============================\n"
                  << "    * THYRA MISSION NODE INITIALIZED *\n"
                  << "=============================\n"
                  << std::endl;

        // Declare parameters
        this->declare_parameter("camera.focal_length", std::vector<double>{0.0,0.0});
        this->declare_parameter("camera.principal_point", std::vector<double>{0.0,0.0});
        this->declare_parameter("camera.image_size", std::vector<int64_t>{0,0}); 
        this->declare_parameter("camera.Radial_distortion", std::vector<int64_t>{0,0,0}); 
        this->declare_parameter("camera.Tangiential_distortion", std::vector<int64_t>{0,0});
        this->declare_parameter("camera.Skew", 0.0);
        this->declare_parameter("camera.k", std::vector<double>{1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0});

        // Get parameters
        std::vector<double> focal_length = this->get_parameter("camera.focal_length").as_double_array();
        std::vector<double> principal_point = this->get_parameter("camera.principal_point").as_double_array();
        std::vector<int64_t> image_size = this->get_parameter("camera.image_size").as_integer_array();
        std::vector<int64_t> radial_distortion = this->get_parameter("camera.Radial_distortion").as_integer_array();
        std::vector<int64_t> tangential_distortion = this->get_parameter("camera.Tangiential_distortion").as_integer_array();
        double skew = this->get_parameter("camera.Skew").as_double();
        std::vector<double> k = this->get_parameter("camera.k").as_double_array();
        Eigen::Matrix3d K;
        for (int i = 0; i < 9; ++i)
            K(i/3, i%3) = k[i];


        
        // Subscribers
        sub_state = create_subscription<interfaces::msg::DroneState>(
            "/asr/thyra/out/drone_state", qos,
            [this](const interfaces::msg::DroneState::SharedPtr msg)
            { droneStateCallback(msg); }
        );
        sub_RGB_image = create_subscription<sensor_msgs::msg::CompressedImage>(
            "/asr/thyra/out/color_image/compressed", qos,
            [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
            { RGBCallback(msg); }
        );

        sub_pose_synced = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/thyra/out/pose/synced_with_RGBD", qos,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            { poseCallback(msg); }
        );

        // Action server
        drone_command_server_ = rclcpp_action::create_server<DroneCommand>(
            this, "in/mission_command",
            [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const DroneCommand::Goal> goal)
            {                return handleDroneCommand(uuid, goal);
            },
            [this](const std::shared_ptr<GoalHandleDroneCommand> goal_handle)
            {                return handleCancel(goal_handle);
            },
            [this](const std::shared_ptr<GoalHandleDroneCommand> goal_handle)
            {                handleAccepted(goal_handle);
            });

        
        
    }
    rclcpp::Time get_time() const {
            return clock_->now();
        }
private:

    void droneStateCallback(const interfaces::msg::DroneState::SharedPtr msg)
    {
        // Process drone state message
        last_drone_state = msg;
        RCLCPP_INFO(get_logger(), "Received drone state message: arming_state=%d, flight_mode=%d", msg->arming_state, msg->flight_mode);
    }
    void RGBCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
    {
        // Process RGB image message
        last_RGB_image = msg;


    }
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Process pose message
        last_pose_synced = msg;
        RCLCPP_INFO(get_logger(), "Received synced pose message.");
    }


     //-----------------------Action Server Handlers-----------------------
    rclcpp_action::GoalResponse handleDroneCommand(const rclcpp_action::GoalUUID & /*uuid*/,
                                                std::shared_ptr<const DroneCommand::Goal> goal)
    {   
        RCLCPP_INFO(get_logger(), "Received goal request: command_type=%s", goal->command_type.c_str());
        
         return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleDroneCommand> /*goal_handle*/)
    {
        RCLCPP_INFO(get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void handleAccepted(const std::shared_ptr<GoalHandleDroneCommand> goal_handle)
    {
        std::thread([this, goal_handle]()
                    { execute(goal_handle); })
            .detach();
    }

    std::shared_ptr<rclcpp::Clock> clock_;

    rclcpp::Subscription<interfaces::msg::DroneState>::SharedPtr sub_state;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_RGB_image;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_synced;

    rclcpp_action::Server<DroneCommand>::SharedPtr drone_command_server_;


    interfaces::msg::DroneState::SharedPtr last_drone_state;
    sensor_msgs::msg::CompressedImage::ConstSharedPtr last_RGB_image;
    geometry_msgs::msg::PoseStamped::SharedPtr last_pose_synced;

    void execute(const std::shared_ptr<GoalHandleDroneCommand> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Executing goal...");
        auto result = std::make_shared<DroneCommand::Result>();

        if (rclcpp::ok())
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(get_logger(), "Goal execution completed.");
        }
    }


};

//void executeMission(){
//
//
//    //Mission Parameters
//    int takeoff_marker_id = 101;
//    int landing_marker_id = 102;
//    bool all_probes_found = false;
//    int probe_search_amount = 3;
//    double search_height = -2.0; // in meters
//    int landing_aruco_detection_threshold = 5;
//    int landing_aruco_detection_count = 0; 
//
//    std::vector<int> landing_aruco_detection_list;
//    std::vector<double> landing_marker_pose_estimate;
//    std::vector<double> landing_marker_pose_estimate_used;
//
//    double yaw_to_marker_estimate = 0.0; // Initial yaw to look at marker
//    double yaw_to_marker_estimate_used = 0.0;
//    bool landing_marker_detected = false;
//    int search_attempts = 0;
//    int search_attempts_max = 3;
//    int arUco_landing_tries = 0;
//    int arUco_marker_landing_correction_threshold = 5;
//
//    // Define CAMERA_TO_DRONE_TRANSFORM (4x4 homogeneous matrix) in meters
//    std::vector<std::vector<double>> camera_to_drone = {
//        {0.0, -0.70710678, 0.70710678, 0.137751},
//        {1.0, 0.0, 0.0, -0.018467},
//        {0.0, 0.70710678, 0.70710678, 0.12126}, // Camera is 0.1m above drone center
//        {0.0, 0.0, 0.0, 1.0}
//    };
//
//    std::vector<int> last_drone_state;
//    std::vector<int> last_RGB_image;
//    std::vector<int> last_pose_synced;
//
//    // Mission Code:
//    std::cout << "Mission execution is now beginning...." << std::endl;
//
//    // Arm
//
//
//    // Takeoff
//
//
//    // Search for probes and landing marker
//
//
//    // If probes and landing marker is not detected, search in another pattern
//
//
//    // Determine landing marker pose estimate
//
//    
//    // Goto landing marker location
//
//    goToArUcoMarker(0.1);
//
//}
//
//bool check_if_all_probes_found(){
//    if(is.empty(last_drone_state) || is.empty(lan))
//}
//
//double estimate_yaw_to_marker(){
//    // Estimate orientation to look at marker
//    double yaw;
//    if (!is.empty(last_drone_state) || !is.empty(landing_marker_pose_estimate)){
//        // Process data to estimate yaw
//        std::cout << "Cannot estimate yaw: missing pose or marker estimate." << std::endl;
//        return yaw = 0.0;
//    }
//    T_world_drone = poseMsgToMatrix(last_pose_synced);
//   
//
//    
//    return yaw;
//}
//
//
//void goToArUcoMarker(double pause_timer){
//
//    double marker_threshold = 0.2; // meters
//
//    
//
//}
//
//
int main(){
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<ThyraMissionExecutor>();
    rclcpp::spin(node);
    return 0;
}