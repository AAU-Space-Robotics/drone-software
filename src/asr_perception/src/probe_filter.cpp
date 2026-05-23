#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <asr_comms/msg/probe_detections.hpp>
#include <asr_comms/msg/probe_locations.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <cstdint>

using ProbeDetections = asr_comms::msg::ProbeDetections;
using ProbeLocations  = asr_comms::msg::ProbeLocations;
using Image           = sensor_msgs::msg::Image;
using PoseStamped     = geometry_msgs::msg::PoseStamped;

// ---------------------------------------------------------------------------
// KalmanTrack — one tracked probe
// State:       x = [px, py, pz, bx, by, bz]^T   (6x1)
// Observation: z = [px, py, pz]^T                (3x1)
// ---------------------------------------------------------------------------
struct KalmanTrack {
    uint32_t id;
    int      observations{0};
    float    confidence{0.0f};

    Eigen::Matrix<double, 6, 1> x;   // state
    Eigen::Matrix<double, 6, 6> P;   // covariance

    // Observation matrix: we measure position directly, and add bias terms
    static const Eigen::Matrix<double, 3, 6> H() {
        Eigen::Matrix<double, 3, 6> mat = Eigen::Matrix<double, 3, 6>::Zero();
        mat.leftCols<3>() = Eigen::Matrix3d::Identity();
        mat.rightCols<3>() = Eigen::Matrix3d::Identity();
        return mat;
    }

    // Process noise. Not more uncertain over time
    static Eigen::Matrix<double, 6, 6> Q()
    {
        double sigma_pos = 0.01; // meters
        double sigma_bias = 0.001; // meters

        Eigen::Matrix<double, 6, 6> mat = Eigen::Matrix<double, 6, 6>::Zero();
        mat.topLeftCorner<3, 3>()     = sigma_pos * sigma_pos * Eigen::Matrix3d::Identity();
        mat.bottomRightCorner<3, 3>() = sigma_bias * sigma_bias * Eigen::Matrix3d::Identity();
        return mat;
    }

    // Measurement noise
    static Eigen::Matrix<double, 3, 3> R(double confidence = 1.0f) {
        double sigma_meas = 0.05; // meters
        return sigma_meas * sigma_meas * (1.0 / confidence) *  Eigen::Matrix3d::Identity();
    }

    // Predict forward by dt seconds.
    void predict() {
        P = P + Q();
    }

    // Update with a new 3-D measurement.
    void update(const Eigen::Vector3d& z_meas, double confidence) {
        const auto  Hk = H();
        const auto  Rk = R(confidence); // Measurement noise scaled by confidence
        const auto  S  = Hk * P * Hk.transpose() + Rk;
        const auto K = S.transpose().llt().solve((Hk * P).transpose()).transpose(); // Kalman gain
        auto IKH = Eigen::Matrix<double,6,6>::Identity() - K * Hk;
        x = x + K * (z_meas - Hk * x);
        P = IKH * P * IKH.transpose() + K * Rk * K.transpose();
        ++observations;
    }

    Eigen::Vector3d position() const { return x.head<3>(); }
};

// ---------------------------------------------------------------------------
// ProbeFilter node
// ---------------------------------------------------------------------------
class ProbeFilter : public rclcpp::Node {
public:
    ProbeFilter() : Node("probe_filter")
    {
        // --- Parameters ---
        declare_parameter("fx", 425.88);
        declare_parameter("fy", 425.88);
        declare_parameter("cx", 430.51);
        declare_parameter("cy", 238.53);
        declare_parameter("camera_to_drone_transform", std::vector<double>(16, 0.0));
        declare_parameter("min_observations",   2);
        declare_parameter("merge_distance_m",   0.6);
        declare_parameter("max_distance_m",     4.0);
        declare_parameter("track_timeout_s",   10.0);

        fx_ = get_parameter("fx").as_double();
        fy_ = get_parameter("fy").as_double();
        cx_ = get_parameter("cx").as_double();
        cy_ = get_parameter("cy").as_double();

        const auto T_flat = get_parameter("camera_to_drone_transform").as_double_array();
        if (T_flat.size() == 16) {
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    T_cam_to_drone_(r, c) = T_flat[r * 4 + c];
        } else {
            RCLCPP_WARN(get_logger(), "camera_to_drone_transform param missing/wrong size — using identity");
            T_cam_to_drone_ = Eigen::Matrix4d::Identity();
        }

        min_observations_ = get_parameter("min_observations").as_int();
        merge_distance_   = get_parameter("merge_distance_m").as_double();
        max_distance_     = get_parameter("max_distance_m").as_double();
        track_timeout_    = get_parameter("track_timeout_s").as_double();

        // --- Sync subscribers ---
        auto qos = rclcpp::QoS(5).best_effort();

        det_sub_.subscribe(this,   "/probe_detector/detections",    qos.get_rmw_qos_profile());
        depth_sub_.subscribe(this, "/thyra/out/cam/synced/depth",   qos.get_rmw_qos_profile());
        pose_sub_.subscribe(this,  "/thyra/out/cam/synced/pose",    qos.get_rmw_qos_profile());

        sync_ = std::make_shared<Synchronizer>(
            SyncPolicy(10), det_sub_, depth_sub_, pose_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.15));
        sync_->registerCallback(
            std::bind(&ProbeFilter::on_data, this,
                      std::placeholders::_1,
                      std::placeholders::_2,
                      std::placeholders::_3));

        probe_pub_ = create_publisher<ProbeLocations>("/probe_detector/locations", 10);

        RCLCPP_INFO(get_logger(), "Probe filter ready");
    }

private:
    // --- Sync callback ---
    void on_data(
        const ProbeDetections::ConstSharedPtr& detections,
        const Image::ConstSharedPtr&           depth,
        const PoseStamped::ConstSharedPtr&     pose)
    {
        const double now_s = rclcpp::Time(detections->header.stamp).seconds();

        std::vector<Eigen::Vector3d> measurements;
        std::vector<float>           confidences;

        for (uint32_t i = 0; i < detections->num_detections; ++i) {
            const float cx_px = detections->centroid_x[i];
            const float cy_px = detections->centroid_y[i];

            // 1. Project pixel + depth → 3-D in camera frame
            const Eigen::Vector3d p_cam = project_to_3d(cx_px, cy_px, *depth);
            if (p_cam.z() <= 0.0 || p_cam.norm() > max_distance_) continue;

            // 2. Transform camera frame → drone body frame
            const Eigen::Vector4d p_h(p_cam.x(), p_cam.y(), p_cam.z(), 1.0);
            const Eigen::Vector3d p_body = (T_cam_to_drone_ * p_h).head<3>();

            // 3. Transform drone body frame → global frame
            const Eigen::Vector3d p_global = to_global(p_body, *pose);

            measurements.push_back(p_global);
            confidences.push_back(detections->confidence[i]);
        }

        // 4. TODO: Data association — match measurements to existing tracks.
        //    Hungarian algorithm or nearest-neighbour within merge_distance_.
        //    For each matched pair: track.predict(dt); track.update(measurement);
        //    For unmatched measurements: initialise new KalmanTrack.
        //    For unmatched tracks: track.predict(dt) with no update (coasting).
        update_tracks(measurements, confidences, now_s);

        // 5. Publish confirmed probes
        publish_locations(detections->header);
    }

    // --- 3-D projection ---
    // Reads a single depth pixel (16UC1, millimetres) and back-projects.
    Eigen::Vector3d project_to_3d(float u, float v, const Image& depth_img) const
    {
        const int iu = static_cast<int>(std::round(u));
        const int iv = static_cast<int>(std::round(v));

        if (iu < 0 || iv < 0 ||
            iu >= static_cast<int>(depth_img.width) ||
            iv >= static_cast<int>(depth_img.height))
            return Eigen::Vector3d::Zero();

        // depth_img.encoding should be "16UC1" — millimetres
        const uint16_t d_mm = *reinterpret_cast<const uint16_t*>(
            depth_img.data.data() + iv * depth_img.step + iu * sizeof(uint16_t));

        if (d_mm == 0) return Eigen::Vector3d::Zero();

        const double z = d_mm / 1000.0;
        const double x = (u - cx_) * z / fx_;
        const double y = (v - cy_) * z / fy_;
        return {x, y, z};
    }

    // --- Global frame transform ---
    Eigen::Vector3d to_global(const Eigen::Vector3d& p_body,
                               const PoseStamped&      pose) const
    {
        const auto& o = pose.pose.orientation;
        const auto& t = pose.pose.position;
        const Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
        return q * p_body + Eigen::Vector3d(t.x, t.y, t.z);
    }

    // --- Track management ---
    void update_tracks(const std::vector<Eigen::Vector3d>& measurements,
                       const std::vector<float>&            confidences,
                       double                               now_s)
    {
        // TODO: implement proper data association here.
        //
        // Suggested approach:
        //   - For each measurement, find the nearest track within merge_distance_.
        //   - Compute dt = now_s - track.last_update_s, then:
        //       track.predict(dt);
        //       track.update(measurement);
        //       track.confidence = 0.8f * track.confidence + 0.2f * new_confidence;
        //       track.last_update_s = now_s;
        //   - For unmatched measurements: create a new track (see init_track below).
        //   - Expire tracks where (now_s - last_update_s) > track_timeout_.
        (void)measurements; (void)confidences; (void)now_s;
    }

    KalmanTrack init_track(const Eigen::Vector3d& pos, float confidence)
    {
        KalmanTrack t;
        t.id           = next_id_++;
        t.observations = 1;
        t.confidence   = confidence;
        t.x            = Eigen::Matrix<double, 6, 1>::Zero();
        t.x.head<3>()  = pos;
        t.P            = Eigen::Matrix<double, 6, 6>::Identity() * 1.0;
        return t;
    }

    // --- Publish ---
    void publish_locations(const std_msgs::msg::Header& header)
    {
        ProbeLocations msg;
        msg.header = header;

        for (const auto& track : tracks_) {
            if (track.observations < min_observations_) continue;
            const auto pos = track.position();
            msg.positions.push_back(static_cast<float>(pos.x()));
            msg.positions.push_back(static_cast<float>(pos.y()));
            msg.positions.push_back(static_cast<float>(pos.z()));
            msg.confidence.push_back(track.confidence);
            ++msg.num_probes;
        }

        probe_pub_->publish(msg);
    }

    // --- Types ---
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        ProbeDetections, Image, PoseStamped>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

    // --- Members ---
    double fx_, fy_, cx_, cy_;
    Eigen::Matrix4d T_cam_to_drone_;
    int    min_observations_;
    double merge_distance_;
    double max_distance_;
    double track_timeout_;

    std::vector<KalmanTrack> tracks_;
    uint32_t                 next_id_{0};

    message_filters::Subscriber<ProbeDetections> det_sub_;
    message_filters::Subscriber<Image>           depth_sub_;
    message_filters::Subscriber<PoseStamped>     pose_sub_;
    std::shared_ptr<Synchronizer>                sync_;

    rclcpp::Publisher<ProbeLocations>::SharedPtr probe_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProbeFilter>());
    rclcpp::shutdown();
    return 0;
}
