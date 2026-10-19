#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <limits>
#include <string>
#include <vector>

class CorridorParallelRightSelector
{
public:
    CorridorParallelRightSelector(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh)
    {
        loadParams();

        scan_sub_ = nh_.subscribe(scan_topic_, 1, &CorridorParallelRightSelector::scanCallback, this);
        start_capture_sub_ = nh_.subscribe(start_capture_topic_, 1, &CorridorParallelRightSelector::startCaptureCallback, this);
        marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_array_topic_, 1, true);
        path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
        start_capture_feedback_pub_ = nh_.advertise<std_msgs::UInt8>(start_capture_feedback_topic_, 1, true);
        fence_status_pub_ = nh_.advertise<std_msgs::Int32>(fence_status_topic_, 1, true);
        heartbeat_pub_ = nh_.advertise<std_msgs::Bool>(heartbeat_topic_, 1);
        heartbeat_timer_ = nh_.createTimer(ros::Duration(1.0), &CorridorParallelRightSelector::heartbeatTimerCallback, this);

        updateFenceStatus(true);

        ROS_INFO_STREAM("[corridor_parallel_right_selector_dynamic_gate_robust] started in standby"
                        << " scan_topic=" << scan_topic_
                        << " start_capture_topic=" << start_capture_topic_
                        << " start_capture_feedback_topic=" << start_capture_feedback_topic_
                        << " marker_array_topic=" << marker_array_topic_
                        << " path_topic=" << path_topic_
                        << " cmd_vel_topic=" << cmd_vel_topic_
                        << " fence_status_topic=" << fence_status_topic_
                        << " heartbeat_topic=" << heartbeat_topic_
                        << " enable_right_local_path=" << (enable_right_local_path_ ? "true" : "false")
                        << " enable_path_tracker=" << (enable_path_tracker_ ? "true" : "false")
                        << " tracking_controller_type=" << tracking_controller_type_
                        << " right_path_offset_m=" << right_path_offset_m_
                        << " waiting_start_capture_command=true");
    }

private:
    struct Point2D
    {
        double x{0.0};
        double y{0.0};
    };

    struct ScanPoint
    {
        Point2D p;
        int scan_index{0};
        double angle_rad{0.0};
        double range_m{0.0};
    };

    struct Cluster
    {
        std::vector<ScanPoint> pts;
    };

    struct SideAccumulationFrame
    {
        ros::Time stamp;
        std::vector<ScanPoint> left_pts;
        std::vector<ScanPoint> right_pts;
    };

    struct FittedLine
    {
        bool valid{false};
        bool left_side{false};

        double tx{1.0};
        double ty{0.0};
        double nx{0.0};
        double ny{1.0};
        double rho{0.0};
        double theta{0.0};

        Point2D centroid;
        Point2D p0;
        Point2D p1;

        int point_count{0};
        double span_length{0.0};
        double residual_rms{0.0};
        double linearity_ratio{0.0};
        double tangent_angle_deg{0.0};
        double score{0.0};

        std::vector<Point2D> support_pts;
    };

    struct StablePathState
    {
        bool valid{false};
        bool transition_active{false};

        Point2D centroid;
        double tx{1.0};
        double ty{0.0};
        double span_length{0.0};

        int over_trigger_count{0};
        double last_lateral_diff_m{0.0};
        double last_angle_diff_deg{0.0};
        std::string mode{"idle"};

        ros::Time last_state_stamp;
        ros::Time last_valid_measurement_stamp;
    };

    struct PathProjection
    {
        bool valid{false};
        Point2D closest_point;
        int segment_index{-1};
        double segment_t{0.0};
        double distance{0.0};
        double path_yaw{0.0};
        double cumulative_s_to_projection{0.0};
    };

    struct TrackingCommand
    {
        bool valid{false};
        std::string controller_name{"none"};
        std::string status{"idle"};

        double linear_x{0.0};
        double angular_z{0.0};
        double heading_error_deg{0.0};
        double cross_track_error_m{0.0};
        double lookahead_distance_m{0.0};
        double curvature{0.0};
        int target_index{-1};

        bool has_nearest_point{false};
        Point2D nearest_point;
        bool has_target_point{false};
        Point2D target_point;
        bool has_reference_point{false};
        Point2D reference_point;
    };

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber scan_sub_;
    ros::Subscriber start_capture_sub_;
    ros::Publisher marker_array_pub_;
    ros::Publisher path_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher start_capture_feedback_pub_;
    ros::Publisher fence_status_pub_;
    ros::Publisher heartbeat_pub_;
    ros::Timer heartbeat_timer_;

    std::string scan_topic_;
    std::string marker_array_topic_;
    std::string path_topic_;
    std::string cmd_vel_topic_;
    std::string start_capture_topic_;
    std::string start_capture_feedback_topic_;
    std::string fence_status_topic_;
    std::string heartbeat_topic_;

    enum FenceStatus
    {
        FENCE_DISABLED = 0,
        FENCE_RUNNING = 1,
        FENCE_PAUSED = 2,
        FENCE_LOST = 3,
        FENCE_FAILED = 4,
        FENCE_CANCELED = 5
    };

    uint8_t start_capture_value_{0};
    bool start_capture_cmd_received_{false};
    bool zero_cmd_sent_on_stop_{false};
    int fence_status_{FENCE_PAUSED};

    double range_min_use_{0.10};
    double range_max_use_{8.00};

    // 方案一：左右角域筛点
    double left_sector_angle_min_deg_{5.0};
    double left_sector_angle_max_deg_{85.0};
    double right_sector_angle_min_deg_{-85.0};
    double right_sector_angle_max_deg_{-5.0};

    // 可选的宽松横向带，避免把太近/太远的点带进来；不是矩形 ROI，只是侧向距离门限
    bool enable_lateral_band_filter_{false};
    double left_min_abs_y_m_{0.10};
    double left_max_abs_y_m_{3.00};
    double right_min_abs_y_m_{0.10};
    double right_max_abs_y_m_{3.00};

    // 原始激光坏点滤波：优先清掉灰尘点、孤立反射点、局部突刺点
    bool enable_scan_outlier_filter_{true};
    bool filter_only_right_sector_{true};
    int scan_median_half_window_{2};
    double scan_hampel_k_{2.8};
    double scan_hampel_min_dev_m_{0.03};
    int scan_continuity_half_window_{2};
    double scan_continuity_max_range_jump_m_{0.18};
    int scan_min_valid_neighbors_{1};

    // 动态墙跟踪门控：不是固定横向带，而是围绕“上一帧右墙”构造自适应带状区域
    bool enable_dynamic_right_wall_gate_{true};
    double dynamic_gate_lateral_margin_m_{0.35};
    double dynamic_gate_projection_back_m_{0.80};
    double dynamic_gate_projection_forward_m_{6.00};
    int dynamic_gate_min_points_{6};
    double dynamic_gate_max_time_gap_sec_{0.80};

    // 聚类参数
    double cluster_break_distance_m_{0.15};
    int cluster_max_index_gap_{3};
    int min_cluster_points_{8};

    // 自适应聚类 + 二次合并 + 短时累积
    bool enable_adaptive_clustering_{true};
    double adaptive_cluster_base_m_{0.03};
    double adaptive_cluster_angle_scale_{2.20};
    double adaptive_cluster_max_distance_m_{0.60};
    double adaptive_cluster_max_angle_gap_deg_{2.00};

    bool enable_secondary_merge_{true};
    double secondary_merge_angle_thresh_deg_{8.0};
    double secondary_merge_rho_thresh_m_{0.20};
    double secondary_merge_projection_gap_m_{0.80};
    double secondary_merge_max_combined_residual_m_{0.06};

    bool enable_short_time_accumulation_{true};
    int accumulation_history_frames_{4};
    double accumulation_max_age_sec_{0.35};

    // 方案一：最近右墙局部直线拟合
    double nearest_wall_cluster_min_arc_m_{0.35};
    double nearest_wall_max_seed_range_m_{4.50};
    double local_support_symmetric_half_arc_m_{0.90};
    double local_support_one_side_arc_m_{1.20};
    double local_support_max_step_m_{0.35};
    double local_support_weight_sigma_m_{0.45};
    int local_support_min_points_{5};
    double local_support_min_span_m_{0.25};
    double local_fit_max_residual_m_{0.10};
    double local_fit_min_linearity_ratio_{6.0};
    double local_fit_seed_to_line_max_m_{0.12};

    // 鲁棒拟合：对残差大的点做截断重拟合，降低坏点/碎点对方向的拖拽
    bool enable_robust_refit_{true};
    int robust_refit_iterations_{2};
    double robust_refit_outlier_dist_m_{0.08};
    double robust_refit_min_keep_ratio_{0.45};

    // 右墙跳空桥接参数：允许沿当前墙切向跨过短空档，继续吸附下一段同墙点
    bool enable_gap_bridge_{true};
    double gap_bridge_max_projection_gap_m_{1.20};
    double gap_bridge_max_lateral_error_m_{0.18};
    double gap_bridge_max_endpoint_distance_m_{1.60};
    double gap_bridge_min_segment_arc_m_{0.18};
    int gap_bridge_min_points_{3};
    int gap_bridge_max_hops_each_side_{2};
    double gap_bridge_refit_max_residual_m_{0.12};

    // 线性度筛选参数
    double min_cluster_span_m_{0.40};
    double max_pca_residual_m_{0.05};
    double min_linearity_ratio_{18.0};
    double max_abs_tangent_angle_deg_{18.0};

    // 候选打分参数
    double score_weight_span_{5.0};
    double score_weight_points_{0.05};
    double score_weight_linearity_{0.10};
    double score_weight_residual_{20.0};
    double score_weight_angle_deg_{0.08};
    double linearity_ratio_saturate_{80.0};

    // 右侧局部路径参数
    bool enable_right_local_path_{true};
    double right_path_offset_m_{1.0};
    double right_path_offset_min_m_{0.80};
    double right_path_offset_max_m_{1.50};
    double path_forward_length_m_{4.00};
    double path_backward_length_m_{0.20};
    double path_resolution_m_{0.10};
    double path_min_x_m_{-0.20};
    int min_path_points_{2};

    // 路径抗跳组合方案参数：冻结 + 连续超阈值确认 + 低通过渡
    bool enable_path_stabilizer_{true};
    double path_freeze_lateral_thresh_m_{0.03};
    double path_freeze_angle_thresh_deg_{1.0};
    double path_update_trigger_lateral_thresh_m_{0.08};
    double path_update_trigger_angle_thresh_deg_{2.5};
    int path_update_required_consecutive_frames_{8};
    double path_transition_alpha_{0.92};
    double path_state_hold_sec_{5.0};
    double path_reinitialize_after_lost_sec_{1.0};

    // 可视化参数
    bool publish_sector_rays_{true};
    bool publish_lateral_band_markers_{true};
    bool publish_accumulated_points_markers_{true};
    double sector_ray_length_m_{5.0};
    double lateral_band_x_min_m_{-0.20};
    double lateral_band_x_max_m_{8.00};
    double point_marker_scale_{0.05};
    double candidate_line_scale_{0.035};
    double best_line_scale_{0.085};
    double line_display_extend_m_{0.25};
    double path_marker_scale_{0.10};
    double path_point_marker_scale_{0.07};
    double path_offset_line_scale_{0.05};

    // 局部路径跟踪控制器（Stanley / Pure Pursuit）
    bool enable_path_tracker_{true};
    std::string tracking_controller_type_{"stanley"};
    bool publish_zero_cmd_when_invalid_{true};
    double tracking_nominal_speed_{0.30};
    double tracking_min_speed_{0.06};
    double tracking_max_speed_{0.50};
    double tracking_max_angular_speed_{0.80};
    double tracking_heading_slowdown_gain_{1.2};
    double tracking_lateral_slowdown_gain_{1.0};
    double tracking_min_speed_ratio_{0.20};
    double tracking_stop_heading_error_deg_{60.0};
    double tracking_stop_cross_track_error_m_{1.20};
    double tracking_stop_path_forward_x_thresh_m_{0.25};
    double tracking_reference_forward_offset_m_{0.0};

    double stanley_gain_{1.20};
    double stanley_softening_speed_{0.25};
    double stanley_heading_gain_{1.00};
    double stanley_cross_track_gain_{1.00};

    double pure_pursuit_lookahead_base_m_{0.90};
    double pure_pursuit_lookahead_speed_gain_{1.20};
    double pure_pursuit_min_lookahead_m_{0.70};
    double pure_pursuit_max_lookahead_m_{2.20};
    double pure_pursuit_curvature_gain_{1.00};

    StablePathState stable_path_state_;
    std::deque<SideAccumulationFrame> side_accumulation_history_;

private:
    static double deg2rad(double deg)
    {
        return deg * M_PI / 180.0;
    }

    static double rad2deg(double rad)
    {
        return rad * 180.0 / M_PI;
    }

    static double norm2D(double x, double y)
    {
        return std::sqrt(x * x + y * y);
    }

    static double pointDistance(const Point2D& a, const Point2D& b)
    {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    static geometry_msgs::Point toRosPoint(double x, double y, double z = 0.0)
    {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }

    static double clampValue(double v, double lo, double hi)
    {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    static geometry_msgs::Quaternion yawToQuaternion(double yaw)
    {
        geometry_msgs::Quaternion q;
        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(yaw * 0.5);
        q.w = std::cos(yaw * 0.5);
        return q;
    }

    static double shortestAngleDiffRad(double target, double source)
    {
        double d = target - source;
        while (d > M_PI) d -= 2.0 * M_PI;
        while (d < -M_PI) d += 2.0 * M_PI;
        return d;
    }

    static double angleDiffDegFromDirections(double tx_a, double ty_a, double tx_b, double ty_b)
    {
        const double yaw_a = std::atan2(ty_a, tx_a);
        const double yaw_b = std::atan2(ty_b, tx_b);
        return std::fabs(rad2deg(shortestAngleDiffRad(yaw_a, yaw_b)));
    }

    static Point2D directionToNormal(double tx, double ty)
    {
        return Point2D{-ty, tx};
    }

    void loadParams()
    {
        pnh_.param<std::string>("scan_topic", scan_topic_, "/scan");
        pnh_.param<std::string>("marker_array_topic", marker_array_topic_, "/corridor_parallel_right_selector/markers");
        pnh_.param<std::string>("path_topic", path_topic_, "/corridor_parallel_right_selector/right_local_path");
        pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
        pnh_.param<std::string>("start_capture_topic", start_capture_topic_, "/start_capture");
        pnh_.param<std::string>("start_capture_feedback_topic", start_capture_feedback_topic_, "/start_capture_feedback");
        pnh_.param<std::string>("fence_status_topic", fence_status_topic_, "/fence_follow/status");
        pnh_.param<std::string>("heartbeat_topic", heartbeat_topic_, "/heartbeat/fence_follow");

        pnh_.param("range_min_use", range_min_use_, 0.10);
        pnh_.param("range_max_use", range_max_use_, 8.00);

        pnh_.param("left_sector_angle_min_deg", left_sector_angle_min_deg_, 5.0);
        pnh_.param("left_sector_angle_max_deg", left_sector_angle_max_deg_, 85.0);
        pnh_.param("right_sector_angle_min_deg", right_sector_angle_min_deg_, -85.0);
        pnh_.param("right_sector_angle_max_deg", right_sector_angle_max_deg_, -5.0);

        pnh_.param("enable_lateral_band_filter", enable_lateral_band_filter_, false);
        pnh_.param("left_min_abs_y_m", left_min_abs_y_m_, 0.10);
        pnh_.param("left_max_abs_y_m", left_max_abs_y_m_, 3.00);
        pnh_.param("right_min_abs_y_m", right_min_abs_y_m_, 0.10);
        pnh_.param("right_max_abs_y_m", right_max_abs_y_m_, 3.00);

        pnh_.param("enable_scan_outlier_filter", enable_scan_outlier_filter_, true);
        pnh_.param("filter_only_right_sector", filter_only_right_sector_, true);
        pnh_.param("scan_median_half_window", scan_median_half_window_, 2);
        pnh_.param("scan_hampel_k", scan_hampel_k_, 2.8);
        pnh_.param("scan_hampel_min_dev_m", scan_hampel_min_dev_m_, 0.03);
        pnh_.param("scan_continuity_half_window", scan_continuity_half_window_, 2);
        pnh_.param("scan_continuity_max_range_jump_m", scan_continuity_max_range_jump_m_, 0.18);
        pnh_.param("scan_min_valid_neighbors", scan_min_valid_neighbors_, 1);

        pnh_.param("enable_dynamic_right_wall_gate", enable_dynamic_right_wall_gate_, true);
        pnh_.param("dynamic_gate_lateral_margin_m", dynamic_gate_lateral_margin_m_, 0.35);
        pnh_.param("dynamic_gate_projection_back_m", dynamic_gate_projection_back_m_, 0.80);
        pnh_.param("dynamic_gate_projection_forward_m", dynamic_gate_projection_forward_m_, 6.00);
        pnh_.param("dynamic_gate_min_points", dynamic_gate_min_points_, 6);
        pnh_.param("dynamic_gate_max_time_gap_sec", dynamic_gate_max_time_gap_sec_, 0.80);

        pnh_.param("cluster_break_distance_m", cluster_break_distance_m_, 0.15);
        pnh_.param("cluster_max_index_gap", cluster_max_index_gap_, 3);
        pnh_.param("min_cluster_points", min_cluster_points_, 8);

        pnh_.param("enable_adaptive_clustering", enable_adaptive_clustering_, true);
        pnh_.param("adaptive_cluster_base_m", adaptive_cluster_base_m_, 0.03);
        pnh_.param("adaptive_cluster_angle_scale", adaptive_cluster_angle_scale_, 2.20);
        pnh_.param("adaptive_cluster_max_distance_m", adaptive_cluster_max_distance_m_, 0.60);
        pnh_.param("adaptive_cluster_max_angle_gap_deg", adaptive_cluster_max_angle_gap_deg_, 2.00);

        pnh_.param("enable_secondary_merge", enable_secondary_merge_, true);
        pnh_.param("secondary_merge_angle_thresh_deg", secondary_merge_angle_thresh_deg_, 8.0);
        pnh_.param("secondary_merge_rho_thresh_m", secondary_merge_rho_thresh_m_, 0.20);
        pnh_.param("secondary_merge_projection_gap_m", secondary_merge_projection_gap_m_, 0.80);
        pnh_.param("secondary_merge_max_combined_residual_m", secondary_merge_max_combined_residual_m_, 0.06);

        pnh_.param("enable_short_time_accumulation", enable_short_time_accumulation_, true);
        pnh_.param("accumulation_history_frames", accumulation_history_frames_, 4);
        pnh_.param("accumulation_max_age_sec", accumulation_max_age_sec_, 0.35);

        pnh_.param("nearest_wall_cluster_min_arc_m", nearest_wall_cluster_min_arc_m_, 0.35);
        pnh_.param("nearest_wall_max_seed_range_m", nearest_wall_max_seed_range_m_, 4.50);
        pnh_.param("local_support_symmetric_half_arc_m", local_support_symmetric_half_arc_m_, 0.90);
        pnh_.param("local_support_one_side_arc_m", local_support_one_side_arc_m_, 1.20);
        pnh_.param("local_support_max_step_m", local_support_max_step_m_, 0.35);
        pnh_.param("local_support_weight_sigma_m", local_support_weight_sigma_m_, 0.45);
        pnh_.param("local_support_min_points", local_support_min_points_, 5);
        pnh_.param("local_support_min_span_m", local_support_min_span_m_, 0.25);
        pnh_.param("local_fit_max_residual_m", local_fit_max_residual_m_, 0.10);
        pnh_.param("local_fit_min_linearity_ratio", local_fit_min_linearity_ratio_, 6.0);
        pnh_.param("local_fit_seed_to_line_max_m", local_fit_seed_to_line_max_m_, 0.12);

        pnh_.param("enable_robust_refit", enable_robust_refit_, true);
        pnh_.param("robust_refit_iterations", robust_refit_iterations_, 2);
        pnh_.param("robust_refit_outlier_dist_m", robust_refit_outlier_dist_m_, 0.08);
        pnh_.param("robust_refit_min_keep_ratio", robust_refit_min_keep_ratio_, 0.45);

        pnh_.param("enable_gap_bridge", enable_gap_bridge_, true);
        pnh_.param("gap_bridge_max_projection_gap_m", gap_bridge_max_projection_gap_m_, 1.20);
        pnh_.param("gap_bridge_max_lateral_error_m", gap_bridge_max_lateral_error_m_, 0.18);
        pnh_.param("gap_bridge_max_endpoint_distance_m", gap_bridge_max_endpoint_distance_m_, 1.60);
        pnh_.param("gap_bridge_min_segment_arc_m", gap_bridge_min_segment_arc_m_, 0.18);
        pnh_.param("gap_bridge_min_points", gap_bridge_min_points_, 3);
        pnh_.param("gap_bridge_max_hops_each_side", gap_bridge_max_hops_each_side_, 2);
        pnh_.param("gap_bridge_refit_max_residual_m", gap_bridge_refit_max_residual_m_, 0.12);

        pnh_.param("min_cluster_span_m", min_cluster_span_m_, 0.40);
        pnh_.param("max_pca_residual_m", max_pca_residual_m_, 0.05);
        pnh_.param("min_linearity_ratio", min_linearity_ratio_, 18.0);
        pnh_.param("max_abs_tangent_angle_deg", max_abs_tangent_angle_deg_, 18.0);

        pnh_.param("score_weight_span", score_weight_span_, 5.0);
        pnh_.param("score_weight_points", score_weight_points_, 0.05);
        pnh_.param("score_weight_linearity", score_weight_linearity_, 0.10);
        pnh_.param("score_weight_residual", score_weight_residual_, 20.0);
        pnh_.param("score_weight_angle_deg", score_weight_angle_deg_, 0.08);
        pnh_.param("linearity_ratio_saturate", linearity_ratio_saturate_, 80.0);

        pnh_.param("enable_right_local_path", enable_right_local_path_, true);
        pnh_.param("right_path_offset_m", right_path_offset_m_, 1.0);
        pnh_.param("right_path_offset_min_m", right_path_offset_min_m_, 0.80);
        pnh_.param("right_path_offset_max_m", right_path_offset_max_m_, 1.50);
        pnh_.param("path_forward_length_m", path_forward_length_m_, 4.00);
        pnh_.param("path_backward_length_m", path_backward_length_m_, 0.20);
        pnh_.param("path_resolution_m", path_resolution_m_, 0.10);
        pnh_.param("path_min_x_m", path_min_x_m_, -0.20);
        pnh_.param("min_path_points", min_path_points_, 2);

        pnh_.param("enable_path_stabilizer", enable_path_stabilizer_, true);
        pnh_.param("path_freeze_lateral_thresh_m", path_freeze_lateral_thresh_m_, 0.03);
        pnh_.param("path_freeze_angle_thresh_deg", path_freeze_angle_thresh_deg_, 1.0);
        pnh_.param("path_update_trigger_lateral_thresh_m", path_update_trigger_lateral_thresh_m_, 0.08);
        pnh_.param("path_update_trigger_angle_thresh_deg", path_update_trigger_angle_thresh_deg_, 2.5);
        pnh_.param("path_update_required_consecutive_frames", path_update_required_consecutive_frames_, 8);
        pnh_.param("path_transition_alpha", path_transition_alpha_, 0.92);
        pnh_.param("path_state_hold_sec", path_state_hold_sec_, 5.0);
        pnh_.param("path_reinitialize_after_lost_sec", path_reinitialize_after_lost_sec_, 1.0);

        pnh_.param("publish_sector_rays", publish_sector_rays_, true);
        pnh_.param("publish_lateral_band_markers", publish_lateral_band_markers_, true);
        pnh_.param("publish_accumulated_points_markers", publish_accumulated_points_markers_, true);
        pnh_.param("sector_ray_length_m", sector_ray_length_m_, 5.0);
        pnh_.param("lateral_band_x_min_m", lateral_band_x_min_m_, -0.20);
        pnh_.param("lateral_band_x_max_m", lateral_band_x_max_m_, 8.00);
        pnh_.param("point_marker_scale", point_marker_scale_, 0.05);
        pnh_.param("candidate_line_scale", candidate_line_scale_, 0.035);
        pnh_.param("best_line_scale", best_line_scale_, 0.085);
        pnh_.param("line_display_extend_m", line_display_extend_m_, 0.25);
        pnh_.param("path_marker_scale", path_marker_scale_, 0.10);
        pnh_.param("path_point_marker_scale", path_point_marker_scale_, 0.07);
        pnh_.param("path_offset_line_scale", path_offset_line_scale_, 0.05);

        pnh_.param("enable_path_tracker", enable_path_tracker_, true);
        pnh_.param<std::string>("tracking_controller_type", tracking_controller_type_, "stanley");
        pnh_.param("publish_zero_cmd_when_invalid", publish_zero_cmd_when_invalid_, true);
        pnh_.param("tracking_nominal_speed", tracking_nominal_speed_, 0.30);
        pnh_.param("tracking_min_speed", tracking_min_speed_, 0.06);
        pnh_.param("tracking_max_speed", tracking_max_speed_, 0.50);
        pnh_.param("tracking_max_angular_speed", tracking_max_angular_speed_, 0.80);
        pnh_.param("tracking_heading_slowdown_gain", tracking_heading_slowdown_gain_, 1.20);
        pnh_.param("tracking_lateral_slowdown_gain", tracking_lateral_slowdown_gain_, 1.00);
        pnh_.param("tracking_min_speed_ratio", tracking_min_speed_ratio_, 0.20);
        pnh_.param("tracking_stop_heading_error_deg", tracking_stop_heading_error_deg_, 60.0);
        pnh_.param("tracking_stop_cross_track_error_m", tracking_stop_cross_track_error_m_, 1.20);
        pnh_.param("tracking_stop_path_forward_x_thresh_m", tracking_stop_path_forward_x_thresh_m_, 0.25);
        pnh_.param("tracking_reference_forward_offset_m", tracking_reference_forward_offset_m_, 0.0);

        pnh_.param("stanley_gain", stanley_gain_, 1.20);
        pnh_.param("stanley_softening_speed", stanley_softening_speed_, 0.25);
        pnh_.param("stanley_heading_gain", stanley_heading_gain_, 1.00);
        pnh_.param("stanley_cross_track_gain", stanley_cross_track_gain_, 1.00);

        pnh_.param("pure_pursuit_lookahead_base_m", pure_pursuit_lookahead_base_m_, 0.90);
        pnh_.param("pure_pursuit_lookahead_speed_gain", pure_pursuit_lookahead_speed_gain_, 1.20);
        pnh_.param("pure_pursuit_min_lookahead_m", pure_pursuit_min_lookahead_m_, 0.70);
        pnh_.param("pure_pursuit_max_lookahead_m", pure_pursuit_max_lookahead_m_, 2.20);
        pnh_.param("pure_pursuit_curvature_gain", pure_pursuit_curvature_gain_, 1.00);

        cluster_break_distance_m_ = std::max(0.0, cluster_break_distance_m_);
        cluster_max_index_gap_ = std::max(1, cluster_max_index_gap_);
        min_cluster_points_ = std::max(2, min_cluster_points_);
        adaptive_cluster_base_m_ = std::max(0.0, adaptive_cluster_base_m_);
        adaptive_cluster_angle_scale_ = std::max(0.0, adaptive_cluster_angle_scale_);
        adaptive_cluster_max_distance_m_ = std::max(cluster_break_distance_m_, adaptive_cluster_max_distance_m_);
        adaptive_cluster_max_angle_gap_deg_ = std::max(0.0, adaptive_cluster_max_angle_gap_deg_);
        secondary_merge_angle_thresh_deg_ = std::max(0.0, secondary_merge_angle_thresh_deg_);
        secondary_merge_rho_thresh_m_ = std::max(0.0, secondary_merge_rho_thresh_m_);
        secondary_merge_projection_gap_m_ = std::max(0.0, secondary_merge_projection_gap_m_);
        secondary_merge_max_combined_residual_m_ = std::max(0.0, secondary_merge_max_combined_residual_m_);
        accumulation_history_frames_ = std::max(1, accumulation_history_frames_);
        accumulation_max_age_sec_ = std::max(0.0, accumulation_max_age_sec_);
        nearest_wall_cluster_min_arc_m_ = std::max(0.05, nearest_wall_cluster_min_arc_m_);
        nearest_wall_max_seed_range_m_ = std::max(range_min_use_, nearest_wall_max_seed_range_m_);
        local_support_symmetric_half_arc_m_ = std::max(0.10, local_support_symmetric_half_arc_m_);
        local_support_one_side_arc_m_ = std::max(local_support_symmetric_half_arc_m_, local_support_one_side_arc_m_);
        local_support_max_step_m_ = std::max(0.02, local_support_max_step_m_);
        local_support_weight_sigma_m_ = std::max(0.05, local_support_weight_sigma_m_);
        local_support_min_points_ = std::max(2, local_support_min_points_);
        local_support_min_span_m_ = std::max(0.05, local_support_min_span_m_);
        local_fit_max_residual_m_ = std::max(0.0, local_fit_max_residual_m_);
        local_fit_min_linearity_ratio_ = std::max(1.0, local_fit_min_linearity_ratio_);
        local_fit_seed_to_line_max_m_ = std::max(0.0, local_fit_seed_to_line_max_m_);
        gap_bridge_max_projection_gap_m_ = std::max(0.0, gap_bridge_max_projection_gap_m_);
        gap_bridge_max_lateral_error_m_ = std::max(0.0, gap_bridge_max_lateral_error_m_);
        gap_bridge_max_endpoint_distance_m_ = std::max(0.0, gap_bridge_max_endpoint_distance_m_);
        gap_bridge_min_segment_arc_m_ = std::max(0.0, gap_bridge_min_segment_arc_m_);
        gap_bridge_min_points_ = std::max(1, gap_bridge_min_points_);
        gap_bridge_max_hops_each_side_ = std::max(0, gap_bridge_max_hops_each_side_);
        gap_bridge_refit_max_residual_m_ = std::max(0.0, gap_bridge_refit_max_residual_m_);
        lateral_band_x_max_m_ = std::max(lateral_band_x_min_m_ + 0.1, lateral_band_x_max_m_);

        right_path_offset_m_ = clampValue(right_path_offset_m_, right_path_offset_min_m_, right_path_offset_max_m_);
        path_forward_length_m_ = std::max(0.50, path_forward_length_m_);
        path_backward_length_m_ = std::max(0.0, path_backward_length_m_);
        path_resolution_m_ = std::max(0.02, path_resolution_m_);
        min_path_points_ = std::max(2, min_path_points_);
        path_freeze_lateral_thresh_m_ = std::max(0.0, path_freeze_lateral_thresh_m_);
        path_freeze_angle_thresh_deg_ = std::max(0.0, path_freeze_angle_thresh_deg_);
        path_update_trigger_lateral_thresh_m_ = std::max(path_freeze_lateral_thresh_m_, path_update_trigger_lateral_thresh_m_);
        path_update_trigger_angle_thresh_deg_ = std::max(path_freeze_angle_thresh_deg_, path_update_trigger_angle_thresh_deg_);
        path_update_required_consecutive_frames_ = std::max(1, path_update_required_consecutive_frames_);
        path_transition_alpha_ = clampValue(path_transition_alpha_, 0.0, 0.999);
        path_state_hold_sec_ = std::max(0.0, path_state_hold_sec_);
        path_reinitialize_after_lost_sec_ = std::max(0.0, path_reinitialize_after_lost_sec_);

        tracking_nominal_speed_ = std::max(0.0, tracking_nominal_speed_);
        tracking_min_speed_ = std::max(0.0, tracking_min_speed_);
        tracking_max_speed_ = std::max(tracking_min_speed_, tracking_max_speed_);
        tracking_max_angular_speed_ = std::max(0.0, tracking_max_angular_speed_);
        tracking_min_speed_ratio_ = clampValue(tracking_min_speed_ratio_, 0.0, 1.0);
        tracking_stop_heading_error_deg_ = std::max(0.0, tracking_stop_heading_error_deg_);
        tracking_stop_cross_track_error_m_ = std::max(0.0, tracking_stop_cross_track_error_m_);
        tracking_stop_path_forward_x_thresh_m_ = std::max(0.0, tracking_stop_path_forward_x_thresh_m_);
        tracking_reference_forward_offset_m_ = std::max(0.0, tracking_reference_forward_offset_m_);
        stanley_gain_ = std::max(0.0, stanley_gain_);
        stanley_softening_speed_ = std::max(0.0, stanley_softening_speed_);
        stanley_heading_gain_ = std::max(0.0, stanley_heading_gain_);
        stanley_cross_track_gain_ = std::max(0.0, stanley_cross_track_gain_);
        pure_pursuit_lookahead_base_m_ = std::max(0.05, pure_pursuit_lookahead_base_m_);
        pure_pursuit_lookahead_speed_gain_ = std::max(0.0, pure_pursuit_lookahead_speed_gain_);
        pure_pursuit_min_lookahead_m_ = std::max(0.05, pure_pursuit_min_lookahead_m_);
        pure_pursuit_max_lookahead_m_ = std::max(pure_pursuit_min_lookahead_m_, pure_pursuit_max_lookahead_m_);
        pure_pursuit_curvature_gain_ = std::max(0.0, pure_pursuit_curvature_gain_);
    }

    bool isRangeValid(float r) const
    {
        return std::isfinite(r) && r >= range_min_use_ && r <= range_max_use_;
    }

    static bool isAngleInRangeDeg(double angle_deg, double min_deg, double max_deg)
    {
        if (min_deg <= max_deg)
            return angle_deg >= min_deg && angle_deg <= max_deg;
        return angle_deg >= min_deg || angle_deg <= max_deg;
    }

    bool passLateralBand(const Point2D& p, bool left_side) const
    {
        if (!enable_lateral_band_filter_)
            return true;

        if (left_side)
        {
            if (p.y <= 0.0)
                return false;
            const double ay = std::fabs(p.y);
            return ay >= left_min_abs_y_m_ && ay <= left_max_abs_y_m_;
        }

        if (p.y >= 0.0)
            return false;
        const double ay = std::fabs(p.y);
        return ay >= right_min_abs_y_m_ && ay <= right_max_abs_y_m_;
    }

    bool shouldFilterThisAngle(double angle_deg) const
    {
        if (!filter_only_right_sector_)
            return true;
        return isAngleInRangeDeg(angle_deg, right_sector_angle_min_deg_, right_sector_angle_max_deg_);
    }

    static double medianOfVector(std::vector<double> values)
    {
        if (values.empty())
            return std::numeric_limits<double>::quiet_NaN();
        std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
        double med = values[values.size() / 2];
        if (values.size() % 2 == 0)
        {
            const auto it = std::max_element(values.begin(), values.begin() + values.size() / 2);
            med = 0.5 * (med + *it);
        }
        return med;
    }

    std::vector<float> filterScanRanges(const sensor_msgs::LaserScan::ConstPtr& scan_msg) const
    {
        std::vector<float> filtered = scan_msg->ranges;
        if (!enable_scan_outlier_filter_)
            return filtered;

        const int n = static_cast<int>(filtered.size());
        if (n <= 0)
            return filtered;

        std::vector<float> stage1 = filtered;
        for (int i = 0; i < n; ++i)
        {
            const float r = filtered[i];
            if (!isRangeValid(r))
                continue;

            const double angle_deg = rad2deg(scan_msg->angle_min + static_cast<double>(i) * scan_msg->angle_increment);
            if (!shouldFilterThisAngle(angle_deg))
                continue;

            std::vector<double> neigh;
            neigh.reserve(static_cast<size_t>(2 * scan_median_half_window_ + 1));
            for (int k = std::max(0, i - scan_median_half_window_); k <= std::min(n - 1, i + scan_median_half_window_); ++k)
            {
                if (isRangeValid(filtered[k]))
                    neigh.push_back(static_cast<double>(filtered[k]));
            }
            if (static_cast<int>(neigh.size()) < std::max(3, scan_median_half_window_ + 1))
                continue;

            const double med = medianOfVector(neigh);
            std::vector<double> abs_dev;
            abs_dev.reserve(neigh.size());
            for (double v : neigh)
                abs_dev.push_back(std::fabs(v - med));
            const double mad = medianOfVector(abs_dev);
            const double sigma = 1.4826 * mad;
            const double thresh = std::max(scan_hampel_min_dev_m_, scan_hampel_k_ * std::max(sigma, 1e-4));
            if (std::fabs(static_cast<double>(r) - med) > thresh)
                stage1[i] = std::numeric_limits<float>::quiet_NaN();
        }

        std::vector<float> stage2 = stage1;
        for (int i = 0; i < n; ++i)
        {
            const float r = stage1[i];
            if (!isRangeValid(r))
                continue;

            const double angle_deg = rad2deg(scan_msg->angle_min + static_cast<double>(i) * scan_msg->angle_increment);
            if (!shouldFilterThisAngle(angle_deg))
                continue;

            int valid_neighbors = 0;
            for (int k = std::max(0, i - scan_continuity_half_window_); k <= std::min(n - 1, i + scan_continuity_half_window_); ++k)
            {
                if (k == i || !isRangeValid(stage1[k]))
                    continue;
                if (std::fabs(static_cast<double>(stage1[k]) - static_cast<double>(r)) <= scan_continuity_max_range_jump_m_)
                    ++valid_neighbors;
            }
            if (valid_neighbors < scan_min_valid_neighbors_)
                stage2[i] = std::numeric_limits<float>::quiet_NaN();
        }

        return stage2;
    }

    std::vector<ScanPoint> scanToOrderedPoints(const sensor_msgs::LaserScan::ConstPtr& scan_msg) const
    {
        std::vector<ScanPoint> pts;
        pts.reserve(scan_msg->ranges.size());

        const std::vector<float> filtered_ranges = filterScanRanges(scan_msg);

        double angle = scan_msg->angle_min;
        for (size_t i = 0; i < filtered_ranges.size(); ++i, angle += scan_msg->angle_increment)
        {
            const float r = filtered_ranges[i];
            if (!isRangeValid(r))
                continue;

            ScanPoint sp;
            sp.scan_index = static_cast<int>(i);
            sp.angle_rad = angle;
            sp.range_m = static_cast<double>(r);
            sp.p.x = sp.range_m * std::cos(angle);
            sp.p.y = sp.range_m * std::sin(angle);
            pts.push_back(sp);
        }
        return pts;
    }

    std::vector<ScanPoint> selectSidePoints(const std::vector<ScanPoint>& all_pts, bool left_side) const
    {
        std::vector<ScanPoint> out;
        out.reserve(all_pts.size());

        for (const auto& sp : all_pts)
        {
            const double angle_deg = rad2deg(sp.angle_rad);
            const bool side_ok = left_side
                                 ? isAngleInRangeDeg(angle_deg, left_sector_angle_min_deg_, left_sector_angle_max_deg_)
                                 : isAngleInRangeDeg(angle_deg, right_sector_angle_min_deg_, right_sector_angle_max_deg_);
            if (!side_ok)
                continue;

            if (left_side && sp.p.y <= 0.0)
                continue;
            if (!left_side && sp.p.y >= 0.0)
                continue;

            if (!passLateralBand(sp.p, left_side))
                continue;

            out.push_back(sp);
        }
        return out;
    }

    bool getDynamicGateReferenceLine(FittedLine& ref_line, const ros::Time& now) const
    {
        if (!enable_dynamic_right_wall_gate_)
            return false;
        if (!stable_path_state_.valid)
            return false;
        if (dynamic_gate_max_time_gap_sec_ > 0.0 &&
            (now - stable_path_state_.last_valid_measurement_stamp).toSec() > dynamic_gate_max_time_gap_sec_)
            return false;
        ref_line = buildPathReferenceLineFromStableState();
        return ref_line.valid;
    }

    std::vector<ScanPoint> applyDynamicRightWallGate(const std::vector<ScanPoint>& pts,
                                                     const ros::Time& now) const
    {
        if (!enable_dynamic_right_wall_gate_)
            return pts;

        FittedLine ref_line;
        if (!getDynamicGateReferenceLine(ref_line, now))
            return pts;

        std::vector<ScanPoint> gated;
        gated.reserve(pts.size());
        for (const auto& sp : pts)
        {
            const double lateral = std::fabs(pointToLineDistance(sp.p, ref_line));
            if (lateral > dynamic_gate_lateral_margin_m_)
                continue;

            const double proj = (sp.p.x - ref_line.centroid.x) * ref_line.tx +
                                (sp.p.y - ref_line.centroid.y) * ref_line.ty;
            if (proj < -dynamic_gate_projection_back_m_ || proj > dynamic_gate_projection_forward_m_)
                continue;

            gated.push_back(sp);
        }

        if (static_cast<int>(gated.size()) >= dynamic_gate_min_points_)
            return gated;
        return pts;
    }

    void appendAccumulationFrame(const ros::Time& stamp,
                                const std::vector<ScanPoint>& left_selected,
                                const std::vector<ScanPoint>& right_selected)
    {
        SideAccumulationFrame frame;
        frame.stamp = stamp;
        frame.left_pts = left_selected;
        frame.right_pts = right_selected;
        side_accumulation_history_.push_back(frame);

        while (!side_accumulation_history_.empty() &&
               static_cast<int>(side_accumulation_history_.size()) > accumulation_history_frames_)
        {
            side_accumulation_history_.pop_front();
        }

        while (!side_accumulation_history_.empty() &&
               accumulation_max_age_sec_ > 0.0 &&
               (stamp - side_accumulation_history_.front().stamp).toSec() > accumulation_max_age_sec_)
        {
            side_accumulation_history_.pop_front();
        }
    }

    std::vector<ScanPoint> collectAccumulatedSidePoints(const ros::Time& stamp,
                                                        const std::vector<ScanPoint>& current_selected,
                                                        bool left_side) const
    {
        if (!enable_short_time_accumulation_)
            return current_selected;

        std::vector<ScanPoint> out;
        for (const auto& frame : side_accumulation_history_)
        {
            if (accumulation_max_age_sec_ > 0.0 &&
                (stamp - frame.stamp).toSec() > accumulation_max_age_sec_)
                continue;

            const auto& src = left_side ? frame.left_pts : frame.right_pts;
            out.insert(out.end(), src.begin(), src.end());
        }

        if (out.empty())
            out = current_selected;

        std::stable_sort(out.begin(), out.end(),
                         [](const ScanPoint& a, const ScanPoint& b)
                         {
                             if (std::fabs(a.angle_rad - b.angle_rad) > 1e-9)
                                 return a.angle_rad < b.angle_rad;
                             if (std::fabs(a.range_m - b.range_m) > 1e-9)
                                 return a.range_m < b.range_m;
                             return a.scan_index < b.scan_index;
                         });
        return out;
    }

    double computeAdaptiveClusterDistanceThreshold(const ScanPoint& prev,
                                                   const ScanPoint& curr,
                                                   double nominal_angle_increment_rad) const
    {
        if (!enable_adaptive_clustering_)
            return cluster_break_distance_m_;

        const double avg_range = 0.5 * (prev.range_m + curr.range_m);
        const double angle_gap = std::fabs(curr.angle_rad - prev.angle_rad);
        const double effective_angle_gap = std::max(angle_gap, std::fabs(nominal_angle_increment_rad));
        const double geometry_term = 2.0 * avg_range * std::sin(0.5 * effective_angle_gap);
        const double threshold = adaptive_cluster_base_m_ + adaptive_cluster_angle_scale_ * geometry_term;
        return clampValue(threshold, cluster_break_distance_m_, adaptive_cluster_max_distance_m_);
    }

    bool canAdaptiveClusterConnect(const ScanPoint& prev,
                                   const ScanPoint& curr,
                                   double nominal_angle_increment_rad) const
    {
        const double dist = pointDistance(prev.p, curr.p);

        if (!enable_adaptive_clustering_)
        {
            const int index_gap = std::abs(curr.scan_index - prev.scan_index);
            return index_gap <= cluster_max_index_gap_ && dist <= cluster_break_distance_m_;
        }

        const double angle_gap = std::fabs(curr.angle_rad - prev.angle_rad);
        const double max_angle_gap = std::max(deg2rad(adaptive_cluster_max_angle_gap_deg_),
                                              std::fabs(nominal_angle_increment_rad) * static_cast<double>(cluster_max_index_gap_));
        if (angle_gap > max_angle_gap)
            return false;

        const double dist_thresh = computeAdaptiveClusterDistanceThreshold(prev, curr, nominal_angle_increment_rad);
        return dist <= dist_thresh;
    }

    static void sortClusterPointsByAngle(std::vector<ScanPoint>& pts)
    {
        std::stable_sort(pts.begin(), pts.end(),
                         [](const ScanPoint& a, const ScanPoint& b)
                         {
                             if (std::fabs(a.angle_rad - b.angle_rad) > 1e-9)
                                 return a.angle_rad < b.angle_rad;
                             if (std::fabs(a.range_m - b.range_m) > 1e-9)
                                 return a.range_m < b.range_m;
                             return a.scan_index < b.scan_index;
                         });
    }

    std::vector<Cluster> buildClusters(const std::vector<ScanPoint>& side_pts,
                                       double nominal_angle_increment_rad) const
    {
        std::vector<Cluster> clusters;
        if (side_pts.empty())
            return clusters;

        std::vector<ScanPoint> ordered = side_pts;
        sortClusterPointsByAngle(ordered);

        Cluster cur;
        cur.pts.push_back(ordered.front());

        for (size_t i = 1; i < ordered.size(); ++i)
        {
            const ScanPoint& prev = ordered[i - 1];
            const ScanPoint& curr = ordered[i];

            if (canAdaptiveClusterConnect(prev, curr, nominal_angle_increment_rad))
            {
                cur.pts.push_back(curr);
            }
            else
            {
                if (!cur.pts.empty())
                    clusters.push_back(cur);
                cur = Cluster();
                cur.pts.push_back(curr);
            }
        }
        if (!cur.pts.empty())
            clusters.push_back(cur);
        return clusters;
    }

    static void computeProjectionIntervalOnDirection(const std::vector<ScanPoint>& pts,
                                                     const Point2D& t,
                                                     double& proj_min,
                                                     double& proj_max)
    {
        proj_min = std::numeric_limits<double>::max();
        proj_max = -std::numeric_limits<double>::max();
        for (const auto& sp : pts)
        {
            const double proj = sp.p.x * t.x + sp.p.y * t.y;
            proj_min = std::min(proj_min, proj);
            proj_max = std::max(proj_max, proj);
        }
    }

    bool shouldMergeClusters(const Cluster& a,
                             const Cluster& b,
                             bool left_side) const
    {
        if (!enable_secondary_merge_)
            return false;
        if (a.pts.size() < 2 || b.pts.size() < 2)
            return false;

        FittedLine line_a;
        FittedLine line_b;
        if (!fitLineByPCA(a.pts, line_a, left_side) || !fitLineByPCA(b.pts, line_b, left_side))
            return false;

        const double angle_diff_deg = angleDiffDegFromDirections(line_a.tx, line_a.ty, line_b.tx, line_b.ty);
        if (angle_diff_deg > secondary_merge_angle_thresh_deg_)
            return false;

        if (std::fabs(line_a.rho - line_b.rho) > secondary_merge_rho_thresh_m_)
            return false;

        Point2D t{line_a.tx + line_b.tx, line_a.ty + line_b.ty};
        const double tnorm = norm2D(t.x, t.y);
        if (tnorm < 1e-9)
            return false;
        t.x /= tnorm;
        t.y /= tnorm;
        if (t.x < 0.0)
        {
            t.x = -t.x;
            t.y = -t.y;
        }

        double a_min, a_max, b_min, b_max;
        computeProjectionIntervalOnDirection(a.pts, t, a_min, a_max);
        computeProjectionIntervalOnDirection(b.pts, t, b_min, b_max);
        if (a_min > b_min)
        {
            std::swap(a_min, b_min);
            std::swap(a_max, b_max);
        }

        const double projection_gap = std::max(0.0, b_min - a_max);
        if (projection_gap > secondary_merge_projection_gap_m_)
            return false;

        std::vector<ScanPoint> merged_pts = a.pts;
        merged_pts.insert(merged_pts.end(), b.pts.begin(), b.pts.end());
        sortClusterPointsByAngle(merged_pts);

        FittedLine merged_line;
        if (!fitLineByPCA(merged_pts, merged_line, left_side))
            return false;

        if (merged_line.residual_rms > secondary_merge_max_combined_residual_m_)
            return false;

        return true;
    }

    std::vector<Cluster> mergeClustersSecondPass(const std::vector<Cluster>& input_clusters,
                                                 bool left_side) const
    {
        if (!enable_secondary_merge_ || input_clusters.size() < 2)
            return input_clusters;

        std::vector<Cluster> clusters = input_clusters;
        bool merged_any = true;

        while (merged_any)
        {
            merged_any = false;
            for (size_t i = 0; i < clusters.size() && !merged_any; ++i)
            {
                for (size_t j = i + 1; j < clusters.size() && !merged_any; ++j)
                {
                    if (!shouldMergeClusters(clusters[i], clusters[j], left_side))
                        continue;

                    Cluster merged_cluster;
                    merged_cluster.pts = clusters[i].pts;
                    merged_cluster.pts.insert(merged_cluster.pts.end(),
                                              clusters[j].pts.begin(),
                                              clusters[j].pts.end());
                    sortClusterPointsByAngle(merged_cluster.pts);

                    clusters[i] = merged_cluster;
                    clusters.erase(clusters.begin() + static_cast<long>(j));
                    merged_any = true;
                }
            }
        }

        return clusters;
    }


    static double computePolylineArcLength(const std::vector<ScanPoint>& pts)
    {
        if (pts.size() < 2)
            return 0.0;

        double arc = 0.0;
        for (size_t i = 1; i < pts.size(); ++i)
            arc += pointDistance(pts[i - 1].p, pts[i].p);
        return arc;
    }

    static double pointToLineDistance(const Point2D& p, const FittedLine& line)
    {
        return std::fabs(line.nx * p.x + line.ny * p.y - line.rho);
    }

    void flipLineDirection(FittedLine& line) const
    {
        line.tx = -line.tx;
        line.ty = -line.ty;
        line.nx = -line.nx;
        line.ny = -line.ny;
        line.rho = -line.rho;
        line.theta = std::atan2(line.ny, line.nx);
        if (line.theta < 0.0)
            line.theta += M_PI;
        std::swap(line.p0, line.p1);
    }

    void alignLineDirectionWithHistory(FittedLine& line) const
    {
        if (!line.valid)
            return;

        if (stable_path_state_.valid)
        {
            const double dot = line.tx * stable_path_state_.tx + line.ty * stable_path_state_.ty;
            if (dot < 0.0)
                flipLineDirection(line);
            return;
        }

        if (line.tx < 0.0)
            flipLineDirection(line);
    }


    static double projectPointOnDirection(const Point2D& p, double tx, double ty)
    {
        return p.x * tx + p.y * ty;
    }

    struct BridgeSegmentCandidate
    {
        bool valid{false};
        size_t cluster_idx{0};
        std::vector<ScanPoint> pts;
        double projection_gap{0.0};
        double lateral_mean{0.0};
        double endpoint_distance{0.0};
        double arc_length{0.0};
        double score{0.0};
    };

    std::vector<std::vector<ScanPoint>> splitClusterIntoBridgeRuns(const Cluster& cluster,
                                                                   const FittedLine& line,
                                                                   bool forward,
                                                                   double reference_proj) const
    {
        std::vector<std::vector<ScanPoint>> runs;
        std::vector<ScanPoint> cur;

        auto flush_run = [&]()
        {
            if (!cur.empty())
            {
                runs.push_back(cur);
                cur.clear();
            }
        };

        for (const auto& sp : cluster.pts)
        {
            const double proj = projectPointOnDirection(sp.p, line.tx, line.ty);
            const double projection_gap = forward ? (proj - reference_proj) : (reference_proj - proj);
            const double lateral_error = std::fabs(line.nx * sp.p.x + line.ny * sp.p.y - line.rho);

            const bool valid = (projection_gap >= -0.08) &&
                               (projection_gap <= gap_bridge_max_projection_gap_m_) &&
                               (lateral_error <= gap_bridge_max_lateral_error_m_);
            if (valid)
            {
                if (!cur.empty())
                {
                    const double step = pointDistance(cur.back().p, sp.p);
                    if (step > std::max(local_support_max_step_m_ * 2.2, gap_bridge_max_endpoint_distance_m_))
                        flush_run();
                }
                cur.push_back(sp);
            }
            else
            {
                flush_run();
            }
        }
        flush_run();
        return runs;
    }

    bool evaluateBridgeRun(const std::vector<ScanPoint>& run,
                           const FittedLine& line,
                           bool forward,
                           double reference_proj,
                           const Point2D& reference_endpoint,
                           BridgeSegmentCandidate& cand) const
    {
        cand = BridgeSegmentCandidate();
        if (run.size() < static_cast<size_t>(gap_bridge_min_points_))
            return false;

        double arc = computePolylineArcLength(run);
        if (arc < gap_bridge_min_segment_arc_m_)
            return false;

        double min_gap = std::numeric_limits<double>::infinity();
        double lateral_sum = 0.0;
        double endpoint_distance = std::numeric_limits<double>::infinity();
        for (const auto& sp : run)
        {
            const double proj = projectPointOnDirection(sp.p, line.tx, line.ty);
            const double gap = forward ? (proj - reference_proj) : (reference_proj - proj);
            if (gap >= -0.08)
                min_gap = std::min(min_gap, gap);
            lateral_sum += std::fabs(line.nx * sp.p.x + line.ny * sp.p.y - line.rho);
            endpoint_distance = std::min(endpoint_distance, pointDistance(reference_endpoint, sp.p));
        }
        if (!std::isfinite(min_gap))
            return false;
        if (endpoint_distance > gap_bridge_max_endpoint_distance_m_)
            return false;

        const double lateral_mean = lateral_sum / static_cast<double>(run.size());
        const double score = 2.5 * min_gap + 4.0 * lateral_mean + 1.2 * endpoint_distance
                           - 0.8 * arc - 0.06 * static_cast<double>(run.size());

        cand.valid = true;
        cand.pts = run;
        cand.projection_gap = min_gap;
        cand.lateral_mean = lateral_mean;
        cand.endpoint_distance = endpoint_distance;
        cand.arc_length = arc;
        cand.score = score;
        return true;
    }

    bool findBestBridgeSegment(const std::vector<Cluster>& clusters,
                               const std::vector<size_t>& used_cluster_indices,
                               const FittedLine& current_line,
                               bool forward,
                               BridgeSegmentCandidate& best_candidate) const
    {
        best_candidate = BridgeSegmentCandidate();
        if (!enable_gap_bridge_)
            return false;

        const double reference_proj = forward ?
            projectPointOnDirection(current_line.p1, current_line.tx, current_line.ty) :
            projectPointOnDirection(current_line.p0, current_line.tx, current_line.ty);
        const Point2D reference_endpoint = forward ? current_line.p1 : current_line.p0;

        auto is_used = [&](size_t idx)
        {
            return std::find(used_cluster_indices.begin(), used_cluster_indices.end(), idx) != used_cluster_indices.end();
        };

        bool found = false;
        double best_score = std::numeric_limits<double>::infinity();

        for (size_t ci = 0; ci < clusters.size(); ++ci)
        {
            if (is_used(ci))
                continue;
            const auto& cluster = clusters[ci];
            if (cluster.pts.size() < static_cast<size_t>(gap_bridge_min_points_))
                continue;

            const auto runs = splitClusterIntoBridgeRuns(cluster, current_line, forward, reference_proj);
            for (const auto& run : runs)
            {
                BridgeSegmentCandidate cand;
                if (!evaluateBridgeRun(run, current_line, forward, reference_proj, reference_endpoint, cand))
                    continue;
                cand.cluster_idx = ci;
                if (!found || cand.score < best_score)
                {
                    found = true;
                    best_score = cand.score;
                    best_candidate = cand;
                }
            }
        }
        return found;
    }

    bool expandSupportAcrossGap(const std::vector<Cluster>& all_clusters,
                                const std::vector<size_t>& initial_used_cluster_indices,
                                const std::vector<ScanPoint>& initial_support,
                                const Point2D& seed_point,
                                std::vector<ScanPoint>& expanded_support,
                                FittedLine& expanded_line) const
    {
        expanded_support = initial_support;
        expanded_line = FittedLine();
        if (expanded_support.size() < 2)
            return false;

        if (!fitWeightedLineByPCA(expanded_support, seed_point, expanded_line, false))
            return false;
        alignLineDirectionWithHistory(expanded_line);

        std::vector<size_t> used_cluster_indices = initial_used_cluster_indices;

        auto try_extend_one_side = [&](bool forward)
        {
            for (int hop = 0; hop < gap_bridge_max_hops_each_side_; ++hop)
            {
                BridgeSegmentCandidate best_seg;
                if (!findBestBridgeSegment(all_clusters, used_cluster_indices, expanded_line, forward, best_seg))
                    break;

                std::vector<ScanPoint> trial_support = expanded_support;
                trial_support.insert(trial_support.end(), best_seg.pts.begin(), best_seg.pts.end());
                sortClusterPointsByAngle(trial_support);

                FittedLine trial_line;
                if (!fitWeightedLineByPCA(trial_support, seed_point, trial_line, false))
                    break;
                alignLineDirectionWithHistory(trial_line);

                if (trial_line.residual_rms > gap_bridge_refit_max_residual_m_)
                    break;
                if (trial_line.linearity_ratio < std::max(1.5, local_fit_min_linearity_ratio_ * 0.65))
                    break;
                if (trial_line.centroid.y >= 0.0)
                    break;
                if (pointToLineDistance(seed_point, trial_line) > std::max(local_fit_seed_to_line_max_m_ * 1.6, 0.20))
                    break;

                expanded_support.swap(trial_support);
                expanded_line = trial_line;
                used_cluster_indices.push_back(best_seg.cluster_idx);
            }
        };

        if (enable_gap_bridge_ && gap_bridge_max_hops_each_side_ > 0)
        {
            try_extend_one_side(false);
            try_extend_one_side(true);
        }
        return expanded_line.valid;
    }

    std::vector<ScanPoint> extractLocalSupportWindow(const Cluster& cluster,
                                                     int seed_idx,
                                                     bool use_backward,
                                                     bool use_forward,
                                                     double max_arc_length) const
    {
        std::vector<ScanPoint> support;
        if (cluster.pts.empty() || seed_idx < 0 || seed_idx >= static_cast<int>(cluster.pts.size()))
            return support;

        support.push_back(cluster.pts[seed_idx]);

        if (use_backward)
        {
            double arc = 0.0;
            for (int i = seed_idx - 1; i >= 0; --i)
            {
                const double step = pointDistance(cluster.pts[i].p, cluster.pts[i + 1].p);
                if (step > local_support_max_step_m_)
                    break;
                if (arc + step > max_arc_length)
                    break;
                arc += step;
                support.insert(support.begin(), cluster.pts[i]);
            }
        }

        if (use_forward)
        {
            double arc = 0.0;
            for (int i = seed_idx + 1; i < static_cast<int>(cluster.pts.size()); ++i)
            {
                const double step = pointDistance(cluster.pts[i].p, cluster.pts[i - 1].p);
                if (step > local_support_max_step_m_)
                    break;
                if (arc + step > max_arc_length)
                    break;
                arc += step;
                support.push_back(cluster.pts[i]);
            }
        }

        return support;
    }

    bool fitWeightedLineByPCA(const std::vector<ScanPoint>& pts,
                              const Point2D& seed_point,
                              FittedLine& line,
                              bool left_side) const
    {
        line = FittedLine();
        if (pts.size() < 2)
            return false;

        const double sigma = std::max(0.05, local_support_weight_sigma_m_);
        const double inv_two_sigma_sq = 1.0 / (2.0 * sigma * sigma);

        double w_sum = 0.0;
        double cx = 0.0;
        double cy = 0.0;
        std::vector<double> weights;
        weights.reserve(pts.size());

        for (const auto& sp : pts)
        {
            const double dx = sp.p.x - seed_point.x;
            const double dy = sp.p.y - seed_point.y;
            const double d2 = dx * dx + dy * dy;
            const double w_geo = std::exp(-d2 * inv_two_sigma_sq);
            const double w_range = 1.0 / std::max(0.15, sp.range_m);
            const double w = std::max(1e-6, w_geo * w_range);
            weights.push_back(w);
            w_sum += w;
            cx += w * sp.p.x;
            cy += w * sp.p.y;
        }

        if (w_sum < 1e-9)
            return false;

        cx /= w_sum;
        cy /= w_sum;

        double sxx = 0.0;
        double sxy = 0.0;
        double syy = 0.0;
        for (size_t i = 0; i < pts.size(); ++i)
        {
            const double dx = pts[i].p.x - cx;
            const double dy = pts[i].p.y - cy;
            const double w = weights[i];
            sxx += w * dx * dx;
            sxy += w * dx * dy;
            syy += w * dy * dy;
        }

        sxx /= w_sum;
        sxy /= w_sum;
        syy /= w_sum;

        const double trace = sxx + syy;
        const double det = sxx * syy - sxy * sxy;
        const double disc = std::max(0.0, trace * trace * 0.25 - det);
        const double root = std::sqrt(disc);
        const double lambda1 = trace * 0.5 + root;
        const double lambda2 = trace * 0.5 - root;

        double tx = 1.0;
        double ty = 0.0;
        if (std::fabs(sxy) > 1e-12)
        {
            tx = lambda1 - syy;
            ty = sxy;
        }
        else
        {
            if (sxx >= syy)
            {
                tx = 1.0;
                ty = 0.0;
            }
            else
            {
                tx = 0.0;
                ty = 1.0;
            }
        }

        const double tnorm = norm2D(tx, ty);
        if (tnorm < 1e-12)
            return false;
        tx /= tnorm;
        ty /= tnorm;
        if (tx < 0.0)
        {
            tx = -tx;
            ty = -ty;
        }

        const double nx = -ty;
        const double ny = tx;
        const double rho = nx * cx + ny * cy;
        double theta = std::atan2(ny, nx);
        if (theta < 0.0)
            theta += M_PI;

        double tmin = std::numeric_limits<double>::max();
        double tmax = -std::numeric_limits<double>::max();
        double residual_sum = 0.0;
        std::vector<Point2D> support_pts;
        support_pts.reserve(pts.size());

        for (size_t i = 0; i < pts.size(); ++i)
        {
            const auto& sp = pts[i];
            const double proj = (sp.p.x - cx) * tx + (sp.p.y - cy) * ty;
            tmin = std::min(tmin, proj);
            tmax = std::max(tmax, proj);
            const double dist = nx * sp.p.x + ny * sp.p.y - rho;
            residual_sum += weights[i] * dist * dist;
            support_pts.push_back(sp.p);
        }

        const double residual_rms = std::sqrt(residual_sum / w_sum);
        const double span = std::max(0.0, tmax - tmin);
        const double linearity_ratio = lambda1 / std::max(lambda2, 1e-9);
        const double tangent_angle_deg = std::fabs(rad2deg(std::atan2(ty, tx)));

        line.valid = true;
        line.left_side = left_side;
        line.tx = tx;
        line.ty = ty;
        line.nx = nx;
        line.ny = ny;
        line.rho = rho;
        line.theta = theta;
        line.centroid = {cx, cy};
        line.p0 = {cx + tmin * tx, cy + tmin * ty};
        line.p1 = {cx + tmax * tx, cy + tmax * ty};
        line.point_count = static_cast<int>(pts.size());
        line.span_length = span;
        line.residual_rms = residual_rms;
        line.linearity_ratio = linearity_ratio;
        line.tangent_angle_deg = tangent_angle_deg;
        line.support_pts = support_pts;
        return true;
    }

    bool isNearestLocalLineAccepted(const FittedLine& line,
                                    const Point2D& seed_point) const
    {
        if (!line.valid)
            return false;
        if (line.point_count < local_support_min_points_)
            return false;
        if (line.span_length < local_support_min_span_m_)
            return false;
        if (line.residual_rms > local_fit_max_residual_m_)
            return false;
        if (line.linearity_ratio < local_fit_min_linearity_ratio_)
            return false;
        if (line.centroid.y >= 0.0)
            return false;
        if (pointToLineDistance(seed_point, line) > local_fit_seed_to_line_max_m_)
            return false;
        return true;
    }

    double computeNearestLocalFitScore(const FittedLine& line,
                                       double seed_range_m,
                                       const Point2D& seed_point) const
    {
        const double clipped_ratio = std::min(line.linearity_ratio, linearity_ratio_saturate_);
        const double seed_to_line = pointToLineDistance(seed_point, line);
        return 20.0 / std::max(0.05, seed_range_m) +
               4.0 * line.span_length +
               0.04 * static_cast<double>(line.point_count) +
               0.04 * clipped_ratio -
               25.0 * line.residual_rms -
               6.0 * seed_to_line;
    }

    bool fitNearestRightWallLocalLine(const std::vector<Cluster>& right_clusters,
                                      std::vector<FittedLine>& candidates,
                                      FittedLine& best_line,
                                      std::vector<ScanPoint>& best_cluster_pts,
                                      Point2D& seed_point,
                                      double& seed_range_m) const
    {
        candidates.clear();
        best_line = FittedLine();
        best_cluster_pts.clear();
        seed_point = Point2D();
        seed_range_m = std::numeric_limits<double>::infinity();

        double best_score = -std::numeric_limits<double>::infinity();

        for (size_t cluster_idx = 0; cluster_idx < right_clusters.size(); ++cluster_idx)
        {
            const auto& cluster = right_clusters[cluster_idx];
            if (cluster.pts.size() < static_cast<size_t>(local_support_min_points_))
                continue;

            const double cluster_arc = computePolylineArcLength(cluster.pts);
            if (cluster_arc < nearest_wall_cluster_min_arc_m_)
                continue;

            int seed_idx = -1;
            double cluster_seed_range = std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < cluster.pts.size(); ++i)
            {
                if (cluster.pts[i].range_m < cluster_seed_range)
                {
                    cluster_seed_range = cluster.pts[i].range_m;
                    seed_idx = static_cast<int>(i);
                }
            }
            if (seed_idx < 0)
                continue;
            if (cluster_seed_range > nearest_wall_max_seed_range_m_)
                continue;

            const Point2D cluster_seed_point = cluster.pts[seed_idx].p;
            std::vector<std::vector<ScanPoint>> support_sets;
            support_sets.push_back(extractLocalSupportWindow(cluster, seed_idx, true, true,
                                                             local_support_symmetric_half_arc_m_));
            support_sets.push_back(extractLocalSupportWindow(cluster, seed_idx, true, false,
                                                             local_support_one_side_arc_m_));
            support_sets.push_back(extractLocalSupportWindow(cluster, seed_idx, false, true,
                                                             local_support_one_side_arc_m_));

            for (const auto& support : support_sets)
            {
                if (support.size() < static_cast<size_t>(local_support_min_points_))
                    continue;

                std::vector<ScanPoint> expanded_support;
                FittedLine line;
                if (!expandSupportAcrossGap(right_clusters,
                                            std::vector<size_t>{cluster_idx},
                                            support,
                                            cluster_seed_point,
                                            expanded_support,
                                            line))
                    continue;

                if (!isNearestLocalLineAccepted(line, cluster_seed_point))
                    continue;

                line.score = computeNearestLocalFitScore(line, cluster_seed_range, cluster_seed_point);
                candidates.push_back(line);

                if (line.score > best_score)
                {
                    best_score = line.score;
                    best_line = line;
                    best_cluster_pts = expanded_support;
                    seed_point = cluster_seed_point;
                    seed_range_m = cluster_seed_range;
                }
            }
        }

        std::sort(candidates.begin(), candidates.end(),
                  [](const FittedLine& a, const FittedLine& b)
                  {
                      return a.score > b.score;
                  });
        return best_line.valid;
    }

    bool fitLineByPCA(const std::vector<ScanPoint>& pts, FittedLine& line, bool left_side) const
    {
        auto computeLineFromPoints = [&](const std::vector<ScanPoint>& in_pts, FittedLine& out_line) -> bool
        {
            out_line = FittedLine();
            if (in_pts.size() < 2)
                return false;

            double cx = 0.0;
            double cy = 0.0;
            for (const auto& sp : in_pts)
            {
                cx += sp.p.x;
                cy += sp.p.y;
            }
            cx /= static_cast<double>(in_pts.size());
            cy /= static_cast<double>(in_pts.size());

            double sxx = 0.0;
            double sxy = 0.0;
            double syy = 0.0;
            for (const auto& sp : in_pts)
            {
                const double dx = sp.p.x - cx;
                const double dy = sp.p.y - cy;
                sxx += dx * dx;
                sxy += dx * dy;
                syy += dy * dy;
            }

            sxx /= static_cast<double>(in_pts.size());
            sxy /= static_cast<double>(in_pts.size());
            syy /= static_cast<double>(in_pts.size());

            const double trace = sxx + syy;
            const double det = sxx * syy - sxy * sxy;
            const double disc = std::max(0.0, trace * trace * 0.25 - det);
            const double root = std::sqrt(disc);
            const double lambda1 = trace * 0.5 + root;
            const double lambda2 = trace * 0.5 - root;

            double tx = 1.0;
            double ty = 0.0;
            if (std::fabs(sxy) > 1e-12)
            {
                tx = lambda1 - syy;
                ty = sxy;
            }
            else
            {
                if (sxx >= syy)
                {
                    tx = 1.0;
                    ty = 0.0;
                }
                else
                {
                    tx = 0.0;
                    ty = 1.0;
                }
            }

            const double tnorm = norm2D(tx, ty);
            if (tnorm < 1e-12)
                return false;
            tx /= tnorm;
            ty /= tnorm;

            if (tx < 0.0)
            {
                tx = -tx;
                ty = -ty;
            }

            const double nx = -ty;
            const double ny = tx;
            const double rho = nx * cx + ny * cy;
            double theta = std::atan2(ny, nx);
            if (theta < 0.0)
                theta += M_PI;

            double tmin = std::numeric_limits<double>::max();
            double tmax = -std::numeric_limits<double>::max();
            double residual_sum_sq = 0.0;
            std::vector<Point2D> support_pts;
            support_pts.reserve(in_pts.size());

            for (const auto& sp : in_pts)
            {
                const double proj = (sp.p.x - cx) * tx + (sp.p.y - cy) * ty;
                tmin = std::min(tmin, proj);
                tmax = std::max(tmax, proj);

                const double dist = nx * sp.p.x + ny * sp.p.y - rho;
                residual_sum_sq += dist * dist;
                support_pts.push_back(sp.p);
            }

            const double residual_rms = std::sqrt(residual_sum_sq / static_cast<double>(in_pts.size()));
            const double span = std::max(0.0, tmax - tmin);
            const double linearity_ratio = lambda1 / std::max(lambda2, 1e-9);
            const double tangent_angle_deg = std::fabs(rad2deg(std::atan2(ty, tx)));

            out_line.valid = true;
            out_line.left_side = left_side;
            out_line.tx = tx;
            out_line.ty = ty;
            out_line.nx = nx;
            out_line.ny = ny;
            out_line.rho = rho;
            out_line.theta = theta;
            out_line.centroid = {cx, cy};
            out_line.p0 = {cx + tmin * tx, cy + tmin * ty};
            out_line.p1 = {cx + tmax * tx, cy + tmax * ty};
            out_line.point_count = static_cast<int>(in_pts.size());
            out_line.span_length = span;
            out_line.residual_rms = residual_rms;
            out_line.linearity_ratio = linearity_ratio;
            out_line.tangent_angle_deg = tangent_angle_deg;
            out_line.support_pts = support_pts;
            return true;
        };

        line = FittedLine();
        if (pts.size() < 2)
            return false;

        std::vector<ScanPoint> working = pts;
        FittedLine current;
        if (!computeLineFromPoints(working, current))
            return false;

        if (!enable_robust_refit_ || robust_refit_iterations_ <= 0)
        {
            line = current;
            return true;
        }

        const size_t min_keep = std::max<size_t>(2, static_cast<size_t>(std::ceil(robust_refit_min_keep_ratio_ * static_cast<double>(pts.size()))));
        for (int iter = 0; iter < robust_refit_iterations_; ++iter)
        {
            std::vector<ScanPoint> trimmed;
            trimmed.reserve(working.size());
            for (const auto& sp : working)
            {
                const double dist = std::fabs(pointToLineDistance(sp.p, current));
                if (dist <= robust_refit_outlier_dist_m_)
                    trimmed.push_back(sp);
            }

            if (trimmed.size() < min_keep || trimmed.size() == working.size())
                break;

            FittedLine refined;
            if (!computeLineFromPoints(trimmed, refined))
                break;
            working.swap(trimmed);
            current = refined;
        }

        line = current;
        return true;
    }

    double computeCandidateScore(const FittedLine& line) const
    {
        const double clipped_ratio = std::min(line.linearity_ratio, linearity_ratio_saturate_);
        return score_weight_span_ * line.span_length +
               score_weight_points_ * static_cast<double>(line.point_count) +
               score_weight_linearity_ * clipped_ratio -
               score_weight_residual_ * line.residual_rms -
               score_weight_angle_deg_ * line.tangent_angle_deg;
    }

    bool isCandidateAccepted(const FittedLine& line, bool left_side) const
    {
        if (!line.valid)
            return false;
        if (line.point_count < min_cluster_points_)
            return false;
        if (line.span_length < min_cluster_span_m_)
            return false;
        if (line.residual_rms > max_pca_residual_m_)
            return false;
        if (line.linearity_ratio < min_linearity_ratio_)
            return false;
        if (line.tangent_angle_deg > max_abs_tangent_angle_deg_)
            return false;

        if (left_side)
        {
            if (line.centroid.y <= 0.0)
                return false;
        }
        else
        {
            if (line.centroid.y >= 0.0)
                return false;
        }
        return true;
    }

    std::vector<FittedLine> extractAcceptedCandidates(const std::vector<Cluster>& clusters, bool left_side) const
    {
        std::vector<FittedLine> candidates;
        candidates.reserve(clusters.size());

        for (const auto& cluster : clusters)
        {
            if (static_cast<int>(cluster.pts.size()) < min_cluster_points_)
                continue;

            FittedLine line;
            if (!fitLineByPCA(cluster.pts, line, left_side))
                continue;

            if (!isCandidateAccepted(line, left_side))
                continue;

            line.score = computeCandidateScore(line);
            candidates.push_back(line);
        }

        std::sort(candidates.begin(), candidates.end(),
                  [](const FittedLine& a, const FittedLine& b)
                  {
                      return a.score > b.score;
                  });
        return candidates;
    }

    bool selectBestCandidate(const std::vector<FittedLine>& candidates, FittedLine& best) const
    {
        if (candidates.empty())
        {
            best = FittedLine();
            return false;
        }
        best = candidates.front();
        return true;
    }

    void extendLineForDisplay(FittedLine& line) const
    {
        if (!line.valid)
            return;

        line.p0.x -= line_display_extend_m_ * line.tx;
        line.p0.y -= line_display_extend_m_ * line.ty;
        line.p1.x += line_display_extend_m_ * line.tx;
        line.p1.y += line_display_extend_m_ * line.ty;
    }

    Point2D computeRightPathInteriorNormalFromDirection(double tx, double ty) const
    {
        Point2D n = directionToNormal(tx, ty);
        if (n.y < 0.0)
        {
            n.x = -n.x;
            n.y = -n.y;
        }
        return n;
    }

    Point2D computeRightPathInteriorNormal(const FittedLine& right_line) const
    {
        return computeRightPathInteriorNormalFromDirection(right_line.tx, right_line.ty);
    }

    double computeStablePathLateralDiff(const StablePathState& state, const FittedLine& observed_line) const
    {
        if (!state.valid || !observed_line.valid)
            return std::numeric_limits<double>::infinity();

        const Point2D n_in = computeRightPathInteriorNormalFromDirection(state.tx, state.ty);
        const double dx = observed_line.centroid.x - state.centroid.x;
        const double dy = observed_line.centroid.y - state.centroid.y;
        return std::fabs(dx * n_in.x + dy * n_in.y);
    }

    double computeStablePathAngleDiffDeg(const StablePathState& state, const FittedLine& observed_line) const
    {
        if (!state.valid || !observed_line.valid)
            return std::numeric_limits<double>::infinity();
        return angleDiffDegFromDirections(state.tx, state.ty, observed_line.tx, observed_line.ty);
    }

    void initializeStablePathState(const ros::Time& stamp, const FittedLine& line, const std::string& mode)
    {
        stable_path_state_ = StablePathState();
        stable_path_state_.valid = line.valid;
        stable_path_state_.transition_active = false;
        stable_path_state_.centroid = line.centroid;
        stable_path_state_.tx = line.tx;
        stable_path_state_.ty = line.ty;
        stable_path_state_.span_length = std::max(line.span_length, path_forward_length_m_ + path_backward_length_m_);
        stable_path_state_.over_trigger_count = 0;
        stable_path_state_.last_lateral_diff_m = 0.0;
        stable_path_state_.last_angle_diff_deg = 0.0;
        stable_path_state_.mode = mode;
        stable_path_state_.last_state_stamp = stamp;
        stable_path_state_.last_valid_measurement_stamp = stamp;
    }

    void smoothStablePathTowardObservation(const ros::Time& stamp, const FittedLine& observed_line)
    {
        const double alpha = path_transition_alpha_;
        stable_path_state_.centroid.x = alpha * stable_path_state_.centroid.x + (1.0 - alpha) * observed_line.centroid.x;
        stable_path_state_.centroid.y = alpha * stable_path_state_.centroid.y + (1.0 - alpha) * observed_line.centroid.y;

        double sx = alpha * stable_path_state_.tx + (1.0 - alpha) * observed_line.tx;
        double sy = alpha * stable_path_state_.ty + (1.0 - alpha) * observed_line.ty;
        const double snorm = norm2D(sx, sy);
        if (snorm > 1e-12)
        {
            sx /= snorm;
            sy /= snorm;
            if (sx < 0.0)
            {
                sx = -sx;
                sy = -sy;
            }
            stable_path_state_.tx = sx;
            stable_path_state_.ty = sy;
        }

        stable_path_state_.span_length =
            alpha * stable_path_state_.span_length +
            (1.0 - alpha) * std::max(observed_line.span_length, path_forward_length_m_ + path_backward_length_m_);

        stable_path_state_.last_state_stamp = stamp;
        stable_path_state_.last_valid_measurement_stamp = stamp;
    }

    FittedLine buildPathReferenceLineFromStableState() const
    {
        FittedLine line;
        if (!stable_path_state_.valid)
            return line;

        line.valid = true;
        line.left_side = false;
        line.tx = stable_path_state_.tx;
        line.ty = stable_path_state_.ty;
        line.nx = -line.ty;
        line.ny = line.tx;
        line.rho = line.nx * stable_path_state_.centroid.x + line.ny * stable_path_state_.centroid.y;
        line.theta = std::atan2(line.ny, line.nx);
        if (line.theta < 0.0)
            line.theta += M_PI;

        line.centroid = stable_path_state_.centroid;
        line.span_length = std::max(0.0, stable_path_state_.span_length);
        line.tangent_angle_deg = std::fabs(rad2deg(std::atan2(line.ty, line.tx)));
        line.score = 0.0;
        line.point_count = 0;
        line.residual_rms = 0.0;
        line.linearity_ratio = 0.0;

        const double half_span = 0.5 * std::max(line.span_length, path_forward_length_m_ + path_backward_length_m_);
        line.p0 = {line.centroid.x - half_span * line.tx, line.centroid.y - half_span * line.ty};
        line.p1 = {line.centroid.x + half_span * line.tx, line.centroid.y + half_span * line.ty};
        return line;
    }

    bool updateStablePathReference(const ros::Time& stamp,
                                   const FittedLine& observed_right_line,
                                   FittedLine& stabilized_line) 
    {
        stabilized_line = FittedLine();

        if (!enable_right_local_path_)
            return false;

        if (!enable_path_stabilizer_)
        {
            if (!observed_right_line.valid)
                return false;
            stabilized_line = observed_right_line;
            return true;
        }

        if (!observed_right_line.valid)
        {
            if (stable_path_state_.valid &&
                (stamp - stable_path_state_.last_valid_measurement_stamp).toSec() <= path_state_hold_sec_)
            {
                stable_path_state_.mode = "hold_no_meas";
                stable_path_state_.transition_active = false;
                stable_path_state_.over_trigger_count = 0;
                stable_path_state_.last_lateral_diff_m = 0.0;
                stable_path_state_.last_angle_diff_deg = 0.0;
                stabilized_line = buildPathReferenceLineFromStableState();
                return stabilized_line.valid;
            }

            stable_path_state_ = StablePathState();
            stable_path_state_.mode = "invalid";
            return false;
        }

        if (!stable_path_state_.valid)
        {
            initializeStablePathState(stamp, observed_right_line, "init");
            stabilized_line = buildPathReferenceLineFromStableState();
            return true;
        }

        const double lost_dt = (stamp - stable_path_state_.last_valid_measurement_stamp).toSec();
        if (lost_dt > path_reinitialize_after_lost_sec_)
        {
            initializeStablePathState(stamp, observed_right_line, "reinit_after_lost");
            stabilized_line = buildPathReferenceLineFromStableState();
            return true;
        }

        stable_path_state_.last_valid_measurement_stamp = stamp;

        const double lateral_diff = computeStablePathLateralDiff(stable_path_state_, observed_right_line);
        const double angle_diff_deg = computeStablePathAngleDiffDeg(stable_path_state_, observed_right_line);

        stable_path_state_.last_lateral_diff_m = lateral_diff;
        stable_path_state_.last_angle_diff_deg = angle_diff_deg;

        const bool small_change =
            lateral_diff <= path_freeze_lateral_thresh_m_ &&
            angle_diff_deg <= path_freeze_angle_thresh_deg_;

        const bool trigger_change =
            lateral_diff >= path_update_trigger_lateral_thresh_m_ ||
            angle_diff_deg >= path_update_trigger_angle_thresh_deg_;

        if (small_change)
        {
            stable_path_state_.mode = "freeze";
            stable_path_state_.transition_active = false;
            stable_path_state_.over_trigger_count = 0;
            stabilized_line = buildPathReferenceLineFromStableState();
            return stabilized_line.valid;
        }

        if (trigger_change)
        {
            ++stable_path_state_.over_trigger_count;
        }
        else
        {
            stable_path_state_.over_trigger_count = std::max(0, stable_path_state_.over_trigger_count - 1);
        }

        if (stable_path_state_.over_trigger_count >= path_update_required_consecutive_frames_)
            stable_path_state_.transition_active = true;

        if (stable_path_state_.transition_active)
        {
            smoothStablePathTowardObservation(stamp, observed_right_line);

            const double post_lateral_diff = computeStablePathLateralDiff(stable_path_state_, observed_right_line);
            const double post_angle_diff_deg = computeStablePathAngleDiffDeg(stable_path_state_, observed_right_line);
            stable_path_state_.last_lateral_diff_m = post_lateral_diff;
            stable_path_state_.last_angle_diff_deg = post_angle_diff_deg;

            if (post_lateral_diff <= path_freeze_lateral_thresh_m_ &&
                post_angle_diff_deg <= path_freeze_angle_thresh_deg_)
            {
                stable_path_state_.mode = "freeze";
                stable_path_state_.transition_active = false;
                stable_path_state_.over_trigger_count = 0;
            }
            else
            {
                stable_path_state_.mode = "transition";
            }
        }
        else
        {
            stable_path_state_.mode = "pending";
        }

        stabilized_line = buildPathReferenceLineFromStableState();
        return stabilized_line.valid;
    }
    bool buildRightLocalPath(const std_msgs::Header& header,
                             const FittedLine& right_line,
                             nav_msgs::Path& path,
                             Point2D& offset_line_p0,
                             Point2D& offset_line_p1,
                             Point2D& offset_centroid) const
    {
        path = nav_msgs::Path();
        path.header = header;
        offset_line_p0 = Point2D();
        offset_line_p1 = Point2D();
        offset_centroid = Point2D();

        if (!enable_right_local_path_ || !right_line.valid)
            return false;

        const double offset = clampValue(right_path_offset_m_, right_path_offset_min_m_, right_path_offset_max_m_);
        const Point2D n_in = computeRightPathInteriorNormal(right_line);

        offset_centroid.x = right_line.centroid.x + offset * n_in.x;
        offset_centroid.y = right_line.centroid.y + offset * n_in.y;

        offset_line_p0.x = right_line.p0.x + offset * n_in.x;
        offset_line_p0.y = right_line.p0.y + offset * n_in.y;
        offset_line_p1.x = right_line.p1.x + offset * n_in.x;
        offset_line_p1.y = right_line.p1.y + offset * n_in.y;

        const double yaw = std::atan2(right_line.ty, right_line.tx);
        const geometry_msgs::Quaternion q = yawToQuaternion(yaw);

        const double t_robot = -(offset_centroid.x * right_line.tx + offset_centroid.y * right_line.ty);
        const double start_t = t_robot - path_backward_length_m_;
        const double end_t = t_robot + path_forward_length_m_;

        int point_counter = 0;
        for (double t = start_t; t <= end_t + 1e-9; t += path_resolution_m_)
        {
            const double px = offset_centroid.x + t * right_line.tx;
            const double py = offset_centroid.y + t * right_line.ty;

            if (px < path_min_x_m_)
                continue;

            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.position.x = px;
            pose.pose.position.y = py;
            pose.pose.position.z = 0.0;
            pose.pose.orientation = q;
            path.poses.push_back(pose);
            ++point_counter;
        }

        if (point_counter < min_path_points_)
        {
            path.poses.clear();
            return false;
        }

        return true;
    }

    visualization_msgs::Marker makeDeleteAllMarker(const std::string& frame_id, const ros::Time& stamp) const
    {
        visualization_msgs::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.action = visualization_msgs::Marker::DELETEALL;
        return m;
    }

    visualization_msgs::Marker makePointMarker(const std::string& frame_id,
                                               const ros::Time& stamp,
                                               const std::string& ns,
                                               int id,
                                               const std::vector<Point2D>& pts,
                                               float r, float g, float b,
                                               double scale) const
    {
        visualization_msgs::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::Marker::POINTS;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = scale;
        m.scale.y = scale;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 1.0;

        m.points.reserve(pts.size());
        for (const auto& p : pts)
            m.points.push_back(toRosPoint(p.x, p.y, 0.03));
        return m;
    }

    visualization_msgs::Marker makeLineStripMarker(const std::string& frame_id,
                                                   const ros::Time& stamp,
                                                   const std::string& ns,
                                                   int id,
                                                   const Point2D& p0,
                                                   const Point2D& p1,
                                                   float r, float g, float b,
                                                   double scale,
                                                   double z = 0.10) const
    {
        visualization_msgs::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = scale;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 1.0;
        m.points.push_back(toRosPoint(p0.x, p0.y, z));
        m.points.push_back(toRosPoint(p1.x, p1.y, z));
        return m;
    }

    visualization_msgs::Marker makeTextMarker(const std::string& frame_id,
                                              const ros::Time& stamp,
                                              const std::string& ns,
                                              int id,
                                              const Point2D& p,
                                              const std::string& text,
                                              float r, float g, float b) const
    {
        visualization_msgs::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position = toRosPoint(p.x, p.y, 0.30);
        m.pose.orientation.w = 1.0;
        m.scale.z = 0.16;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 1.0;
        m.text = text;
        return m;
    }

    visualization_msgs::Marker makeSectorBoundaryMarker(const std::string& frame_id,
                                                        const ros::Time& stamp,
                                                        int id,
                                                        bool left_side) const
    {
        const double a0 = deg2rad(left_side ? left_sector_angle_min_deg_ : right_sector_angle_min_deg_);
        const double a1 = deg2rad(left_side ? left_sector_angle_max_deg_ : right_sector_angle_max_deg_);

        visualization_msgs::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.ns = left_side ? "left_sector" : "right_sector";
        m.id = id;
        m.type = visualization_msgs::Marker::LINE_LIST;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.03;
        if (left_side)
        {
            m.color.r = 0.2f;
            m.color.g = 0.7f;
            m.color.b = 1.0f;
            m.color.a = 1.0f;
        }
        else
        {
            m.color.r = 1.0f;
            m.color.g = 0.6f;
            m.color.b = 0.0f;
            m.color.a = 1.0f;
        }

        auto addRay = [&](double ang)
        {
            m.points.push_back(toRosPoint(0.0, 0.0, 0.02));
            m.points.push_back(toRosPoint(sector_ray_length_m_ * std::cos(ang),
                                          sector_ray_length_m_ * std::sin(ang), 0.02));
        };

        addRay(a0);
        addRay(a1);
        return m;
    }

    visualization_msgs::Marker makeLateralBandMarker(const std::string& frame_id,
                                                     const ros::Time& stamp,
                                                     int id,
                                                     bool left_side) const
    {
        visualization_msgs::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.ns = left_side ? "left_lateral_band" : "right_lateral_band";
        m.id = id;
        m.type = visualization_msgs::Marker::LINE_LIST;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.025;
        if (left_side)
        {
            m.color.r = 0.1f;
            m.color.g = 0.9f;
            m.color.b = 0.7f;
            m.color.a = 0.9f;
        }
        else
        {
            m.color.r = 1.0f;
            m.color.g = 0.9f;
            m.color.b = 0.2f;
            m.color.a = 0.9f;
        }

        const double x0 = lateral_band_x_min_m_;
        const double x1 = lateral_band_x_max_m_;
        const double y_min = left_side ? left_min_abs_y_m_ : -right_max_abs_y_m_;
        const double y_max = left_side ? left_max_abs_y_m_ : -right_min_abs_y_m_;

        auto add_edge = [&](double xa, double ya, double xb, double yb)
        {
            m.points.push_back(toRosPoint(xa, ya, 0.025));
            m.points.push_back(toRosPoint(xb, yb, 0.025));
        };

        add_edge(x0, y_min, x1, y_min);
        add_edge(x0, y_max, x1, y_max);
        add_edge(x0, y_min, x0, y_max);
        add_edge(x1, y_min, x1, y_max);
        return m;
    }

    visualization_msgs::Marker makeCandidateLinesMarker(const std::string& frame_id,
                                                        const ros::Time& stamp,
                                                        const std::string& ns,
                                                        int id,
                                                        const std::vector<FittedLine>& lines,
                                                        float r, float g, float b,
                                                        double scale) const
    {
        visualization_msgs::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::Marker::LINE_LIST;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = scale;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 0.90;

        for (const auto& line : lines)
        {
            m.points.push_back(toRosPoint(line.p0.x, line.p0.y, 0.06));
            m.points.push_back(toRosPoint(line.p1.x, line.p1.y, 0.06));
        }
        return m;
    }

    visualization_msgs::Marker makePathLineMarker(const std::string& frame_id,
                                                  const ros::Time& stamp,
                                                  const std::string& ns,
                                                  int id,
                                                  const nav_msgs::Path& path,
                                                  float r, float g, float b,
                                                  double scale,
                                                  double z = 0.12) const
    {
        visualization_msgs::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = scale;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 1.0;

        for (const auto& pose : path.poses)
            m.points.push_back(toRosPoint(pose.pose.position.x, pose.pose.position.y, z));
        return m;
    }

    visualization_msgs::Marker makePathPointsMarker(const std::string& frame_id,
                                                    const ros::Time& stamp,
                                                    const std::string& ns,
                                                    int id,
                                                    const nav_msgs::Path& path,
                                                    float r, float g, float b,
                                                    double scale) const
    {
        visualization_msgs::Marker m;
        m.header.frame_id = frame_id;
        m.header.stamp = stamp;
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::Marker::POINTS;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = scale;
        m.scale.y = scale;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 1.0;

        for (const auto& pose : path.poses)
            m.points.push_back(toRosPoint(pose.pose.position.x, pose.pose.position.y, 0.14));
        return m;
    }

    std::string normalizedTrackingControllerType() const
    {
        if (tracking_controller_type_ == "pure_pursuit" ||
            tracking_controller_type_ == "pure-pursuit" ||
            tracking_controller_type_ == "pp")
            return "pure_pursuit";
        return "stanley";
    }

    Point2D trackingReferencePoint() const
    {
        return Point2D{tracking_reference_forward_offset_m_, 0.0};
    }

    bool extractPathPolylinePoints(const nav_msgs::Path& path, std::vector<Point2D>& pts) const
    {
        pts.clear();
        pts.reserve(path.poses.size());
        for (const auto& pose : path.poses)
            pts.push_back(Point2D{pose.pose.position.x, pose.pose.position.y});
        return pts.size() >= 2;
    }

    bool findClosestProjectionOnPath(const nav_msgs::Path& path,
                                    const Point2D& reference_point,
                                    PathProjection& proj) const
    {
        proj = PathProjection();
        std::vector<Point2D> pts;
        if (!extractPathPolylinePoints(path, pts))
            return false;

        double best_dist = std::numeric_limits<double>::max();
        double cumulative_s = 0.0;
        bool found = false;

        for (size_t i = 0; i + 1 < pts.size(); ++i)
        {
            const Point2D& p0 = pts[i];
            const Point2D& p1 = pts[i + 1];
            const double dx = p1.x - p0.x;
            const double dy = p1.y - p0.y;
            const double seg_len2 = dx * dx + dy * dy;
            const double seg_len = std::sqrt(seg_len2);
            if (seg_len < 1e-9)
                continue;

            double t = ((reference_point.x - p0.x) * dx + (reference_point.y - p0.y) * dy) / seg_len2;
            t = clampValue(t, 0.0, 1.0);

            Point2D cp;
            cp.x = p0.x + t * dx;
            cp.y = p0.y + t * dy;
            const double dist = norm2D(cp.x - reference_point.x, cp.y - reference_point.y);

            if (dist < best_dist)
            {
                best_dist = dist;
                proj.valid = true;
                proj.closest_point = cp;
                proj.segment_index = static_cast<int>(i);
                proj.segment_t = t;
                proj.distance = dist;
                proj.path_yaw = std::atan2(dy, dx);
                proj.cumulative_s_to_projection = cumulative_s + t * seg_len;
                found = true;
            }
            cumulative_s += seg_len;
        }

        if (!found)
        {
            const Point2D& p = pts.front();
            proj.valid = true;
            proj.closest_point = p;
            proj.segment_index = 0;
            proj.segment_t = 0.0;
            proj.distance = norm2D(p.x - reference_point.x, p.y - reference_point.y);
            if (pts.size() >= 2)
                proj.path_yaw = std::atan2(pts[1].y - pts[0].y, pts[1].x - pts[0].x);
            proj.cumulative_s_to_projection = 0.0;
        }
        return proj.valid;
    }

    bool findClosestProjectionOnPath(const nav_msgs::Path& path, PathProjection& proj) const
    {
        return findClosestProjectionOnPath(path, Point2D{0.0, 0.0}, proj);
    }

    bool sampleTargetPointAlongPath(const nav_msgs::Path& path,
                                    const PathProjection& proj,
                                    double lookahead_s,
                                    Point2D& target_point,
                                    int& target_index) const
    {
        target_point = Point2D();
        target_index = -1;
        std::vector<Point2D> pts;
        if (!extractPathPolylinePoints(path, pts) || !proj.valid)
            return false;

        if (lookahead_s <= 1e-9)
        {
            target_point = proj.closest_point;
            target_index = std::max(0, proj.segment_index);
            return true;
        }

        int seg_idx = std::max(0, proj.segment_index);
        Point2D current = proj.closest_point;
        double remain = lookahead_s;

        while (seg_idx + 1 < static_cast<int>(pts.size()))
        {
            const Point2D seg_end = pts[seg_idx + 1];
            const double dx = seg_end.x - current.x;
            const double dy = seg_end.y - current.y;
            const double seg_len = norm2D(dx, dy);
            if (seg_len < 1e-9)
            {
                ++seg_idx;
                current = seg_end;
                continue;
            }

            if (remain <= seg_len)
            {
                const double ratio = remain / seg_len;
                target_point.x = current.x + ratio * dx;
                target_point.y = current.y + ratio * dy;
                target_index = seg_idx + 1;
                return true;
            }

            remain -= seg_len;
            ++seg_idx;
            current = seg_end;
        }

        target_point = pts.back();
        target_index = static_cast<int>(pts.size()) - 1;
        return true;
    }

    double computePathMaxForwardX(const nav_msgs::Path& path,
                                  const Point2D& reference_point) const
    {
        double max_x = -std::numeric_limits<double>::max();
        for (const auto& pose : path.poses)
            max_x = std::max(max_x, pose.pose.position.x - reference_point.x);
        return max_x;
    }

    double computePathMaxForwardX(const nav_msgs::Path& path) const
    {
        return computePathMaxForwardX(path, Point2D{0.0, 0.0});
    }

    double computeTrackingLinearSpeed(double heading_error_rad,
                                      double cross_track_error_m,
                                      const nav_msgs::Path& path,
                                      const Point2D& reference_point) const
    {
        if (path.poses.size() < static_cast<size_t>(min_path_points_))
            return 0.0;

        const double max_forward_x = computePathMaxForwardX(path, reference_point);
        if (max_forward_x < tracking_stop_path_forward_x_thresh_m_)
            return 0.0;

        if (std::fabs(rad2deg(heading_error_rad)) >= tracking_stop_heading_error_deg_)
            return 0.0;
        if (std::fabs(cross_track_error_m) >= tracking_stop_cross_track_error_m_)
            return 0.0;

        const double heading_scale = 1.0 / (1.0 + tracking_heading_slowdown_gain_ * std::fabs(heading_error_rad));
        const double lateral_scale = 1.0 / (1.0 + tracking_lateral_slowdown_gain_ * std::fabs(cross_track_error_m));
        const double ratio = std::max(tracking_min_speed_ratio_, std::min(1.0, heading_scale * lateral_scale));

        const double speed = tracking_nominal_speed_ * ratio;
        return clampValue(speed, tracking_min_speed_, tracking_max_speed_);
    }

    bool computeStanleyTrackingCommand(const nav_msgs::Path& path, TrackingCommand& cmd) const
    {
        cmd = TrackingCommand();
        cmd.controller_name = "stanley";

        const Point2D reference_point = trackingReferencePoint();
        PathProjection proj;
        if (!findClosestProjectionOnPath(path, reference_point, proj))
        {
            cmd.status = "no_projection";
            return false;
        }

        const double path_yaw = proj.path_yaw;
        const double heading_error = shortestAngleDiffRad(path_yaw, 0.0);
        const Point2D n_left{-std::sin(path_yaw), std::cos(path_yaw)};
        const double cross_track_error =
            n_left.x * (proj.closest_point.x - reference_point.x) +
            n_left.y * (proj.closest_point.y - reference_point.y);
        const double v = computeTrackingLinearSpeed(heading_error, cross_track_error, path, reference_point);
        const double stanley_term = std::atan2(stanley_gain_ * cross_track_error,
                                               stanley_softening_speed_ + std::fabs(v));
        double omega = stanley_heading_gain_ * heading_error +
                       stanley_cross_track_gain_ * stanley_term;
        omega = clampValue(omega, -tracking_max_angular_speed_, tracking_max_angular_speed_);

        cmd.valid = true;
        cmd.status = "ok";
        cmd.linear_x = v;
        cmd.angular_z = omega;
        cmd.heading_error_deg = rad2deg(heading_error);
        cmd.cross_track_error_m = cross_track_error;
        cmd.lookahead_distance_m = 0.0;
        cmd.curvature = (std::fabs(v) > 1e-6) ? (omega / v) : 0.0;
        cmd.target_index = proj.segment_index;
        cmd.has_nearest_point = true;
        cmd.nearest_point = proj.closest_point;
        cmd.has_target_point = true;
        cmd.target_point = proj.closest_point;
        cmd.has_reference_point = true;
        cmd.reference_point = reference_point;
        return true;
    }

    bool computePurePursuitTrackingCommand(const nav_msgs::Path& path, TrackingCommand& cmd) const
    {
        cmd = TrackingCommand();
        cmd.controller_name = "pure_pursuit";

        const Point2D reference_point = trackingReferencePoint();
        PathProjection proj;
        if (!findClosestProjectionOnPath(path, reference_point, proj))
        {
            cmd.status = "no_projection";
            return false;
        }

        const double path_yaw = proj.path_yaw;
        const Point2D n_left{-std::sin(path_yaw), std::cos(path_yaw)};
        const double cross_track_error =
            n_left.x * (proj.closest_point.x - reference_point.x) +
            n_left.y * (proj.closest_point.y - reference_point.y);
        const double heading_error = shortestAngleDiffRad(path_yaw, 0.0);
        const double lookahead = clampValue(pure_pursuit_lookahead_base_m_ +
                                            pure_pursuit_lookahead_speed_gain_ * tracking_nominal_speed_,
                                            pure_pursuit_min_lookahead_m_,
                                            pure_pursuit_max_lookahead_m_);

        Point2D target_point;
        int target_index = -1;
        if (!sampleTargetPointAlongPath(path, proj, lookahead, target_point, target_index))
        {
            cmd.status = "no_target";
            return false;
        }

        const double target_rel_x = target_point.x - reference_point.x;
        const double target_rel_y = target_point.y - reference_point.y;
        const double ld = norm2D(target_rel_x, target_rel_y);
        if (ld < 1e-6)
        {
            cmd.status = "target_too_close";
            return false;
        }

        const double v = computeTrackingLinearSpeed(heading_error, cross_track_error, path, reference_point);
        const double curvature = pure_pursuit_curvature_gain_ * (2.0 * target_rel_y) / std::max(ld * ld, 1e-9);
        double omega = v * curvature;
        omega = clampValue(omega, -tracking_max_angular_speed_, tracking_max_angular_speed_);

        cmd.valid = true;
        cmd.status = "ok";
        cmd.linear_x = v;
        cmd.angular_z = omega;
        cmd.heading_error_deg = rad2deg(heading_error);
        cmd.cross_track_error_m = cross_track_error;
        cmd.lookahead_distance_m = ld;
        cmd.curvature = curvature;
        cmd.target_index = target_index;
        cmd.has_nearest_point = true;
        cmd.nearest_point = proj.closest_point;
        cmd.has_target_point = true;
        cmd.target_point = target_point;
        cmd.has_reference_point = true;
        cmd.reference_point = reference_point;
        return true;
    }

    bool computeTrackingCommand(const nav_msgs::Path& path, TrackingCommand& cmd) const
    {
        cmd = TrackingCommand();
        if (!enable_path_tracker_)
        {
            cmd.status = "tracker_disabled";
            return false;
        }
        if (path.poses.size() < static_cast<size_t>(min_path_points_))
        {
            cmd.status = "path_invalid";
            return false;
        }

        if (normalizedTrackingControllerType() == "pure_pursuit")
            return computePurePursuitTrackingCommand(path, cmd);
        return computeStanleyTrackingCommand(path, cmd);
    }

    bool shouldRunFence() const
    {
        return start_capture_cmd_received_ && (start_capture_value_ != 0);
    }

    void heartbeatTimerCallback(const ros::TimerEvent&)
    {
        std_msgs::Bool hb;
        hb.data = true;
        heartbeat_pub_.publish(hb);
    }

    void publishFenceStatusMsg() const
    {
        std_msgs::Int32 msg;
        msg.data = fence_status_;
        fence_status_pub_.publish(msg);
    }

    void updateFenceStatus(bool force_publish = false)
    {
        int new_status = FENCE_PAUSED;
        if (start_capture_cmd_received_ && start_capture_value_ != 0)
            new_status = FENCE_RUNNING;
        else
            new_status = FENCE_PAUSED;

        if (force_publish || new_status != fence_status_)
        {
            fence_status_ = new_status;
            publishFenceStatusMsg();
        }
        else
        {
            fence_status_ = new_status;
        }
    }

    void publishZeroCmdVelOnce()
    {
        if (!enable_path_tracker_)
            return;
        if (zero_cmd_sent_on_stop_)
            return;

        geometry_msgs::Twist twist;
        cmd_vel_pub_.publish(twist);
        zero_cmd_sent_on_stop_ = true;
    }

    void clearZeroStopLatch()
    {
        zero_cmd_sent_on_stop_ = false;
    }

    void setFenceLostStatusIfNeeded()
    {
        if (shouldRunFence() && fence_status_ != FENCE_LOST)
        {
            fence_status_ = FENCE_LOST;
            publishFenceStatusMsg();
        }
    }

    void setFenceRunningStatusIfNeeded()
    {
        if (shouldRunFence() && fence_status_ != FENCE_RUNNING)
        {
            fence_status_ = FENCE_RUNNING;
            publishFenceStatusMsg();
        }
    }

    void publishStartCaptureFeedback(uint8_t state) const
    {
        std_msgs::UInt8 msg;
        msg.data = state;
        start_capture_feedback_pub_.publish(msg);
    }

    void startCaptureCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        const uint8_t new_state = (msg->data == 0) ? 0 : 1;
        const bool first_command = !start_capture_cmd_received_;
        const uint8_t old_state = start_capture_value_;

        start_capture_cmd_received_ = true;
        start_capture_value_ = new_state;

        if (new_state == 0)
        {
            publishZeroCmdVelOnce();
            ROS_WARN_THROTTLE(1.0,
                              "[corridor_parallel_right_selector_dynamic_gate_robust] receive /start_capture=0, pause algorithm and publish zero cmd once.");
        }
        else
        {
            clearZeroStopLatch();
            ROS_INFO_THROTTLE(1.0,
                              "[corridor_parallel_right_selector_dynamic_gate_robust] receive /start_capture=1, start algorithm.");
        }

        updateFenceStatus(true);
        publishStartCaptureFeedback(new_state);

        if (first_command)
        {
            ROS_INFO_STREAM("[corridor_parallel_right_selector_dynamic_gate_robust] first /start_capture command received, standby finished. value="
                            << static_cast<int>(new_state));
        }
        else if (old_state != new_state)
        {
            ROS_INFO_STREAM("[corridor_parallel_right_selector_dynamic_gate_robust] /start_capture state changed from "
                            << static_cast<int>(old_state) << " to " << static_cast<int>(new_state));
        }
    }

    void publishCmdVel(const TrackingCommand& cmd)
    {
        if (!enable_path_tracker_)
            return;

        if (cmd.valid)
        {
            geometry_msgs::Twist twist;
            twist.linear.x = cmd.linear_x;
            twist.angular.z = cmd.angular_z;
            cmd_vel_pub_.publish(twist);
            clearZeroStopLatch();
            setFenceRunningStatusIfNeeded();
            return;
        }

        if (!publish_zero_cmd_when_invalid_)
            return;

        publishZeroCmdVelOnce();
        setFenceLostStatusIfNeeded();
    }

    std::vector<Point2D> toPoint2DVector(const std::vector<ScanPoint>& scan_pts) const
    {
        std::vector<Point2D> pts;
        pts.reserve(scan_pts.size());
        for (const auto& sp : scan_pts)
            pts.push_back(sp.p);
        return pts;
    }

    void publishPath(const std_msgs::Header& header, const nav_msgs::Path& path) const
    {
        if (path.header.frame_id.empty())
        {
            nav_msgs::Path empty_path;
            empty_path.header = header;
            path_pub_.publish(empty_path);
            return;
        }
        path_pub_.publish(path);
    }
    void publishMarkers(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                        const std::vector<ScanPoint>& right_selected,
                        const std::vector<ScanPoint>& right_accumulated,
                        const std::vector<ScanPoint>& best_cluster_pts,
                        bool has_seed_point,
                        const Point2D& seed_point,
                        double seed_range_m,
                        const std::vector<FittedLine>& right_candidates,
                        const FittedLine& right_best,
                        const nav_msgs::Path& right_local_path,
                        bool right_local_path_valid,
                        const Point2D& offset_line_p0,
                        const Point2D& offset_line_p1,
                        const Point2D& offset_centroid,
                        const TrackingCommand& tracking_cmd)
    {
        visualization_msgs::MarkerArray ma;
        ma.markers.push_back(makeDeleteAllMarker(scan_msg->header.frame_id, scan_msg->header.stamp));

        if (publish_sector_rays_)
            ma.markers.push_back(makeSectorBoundaryMarker(scan_msg->header.frame_id, scan_msg->header.stamp, 1, false));

        if (enable_lateral_band_filter_ && publish_lateral_band_markers_)
            ma.markers.push_back(makeLateralBandMarker(scan_msg->header.frame_id, scan_msg->header.stamp, 3, false));

        ma.markers.push_back(makePointMarker(scan_msg->header.frame_id, scan_msg->header.stamp,
                                             "right_selected_points", 11, toPoint2DVector(right_selected),
                                             1.0f, 1.0f, 0.2f, point_marker_scale_));

        if (publish_accumulated_points_markers_)
        {
            ma.markers.push_back(makePointMarker(scan_msg->header.frame_id, scan_msg->header.stamp,
                                                 "right_accumulated_points", 13, toPoint2DVector(right_accumulated),
                                                 1.0f, 0.8f, 0.0f, point_marker_scale_ * 0.8));
        }

        if (!best_cluster_pts.empty())
        {
            ma.markers.push_back(makePointMarker(scan_msg->header.frame_id, scan_msg->header.stamp,
                                                 "nearest_cluster_points", 18, toPoint2DVector(best_cluster_pts),
                                                 0.1f, 0.9f, 0.9f, point_marker_scale_ * 0.95));
        }

        ma.markers.push_back(makeCandidateLinesMarker(scan_msg->header.frame_id, scan_msg->header.stamp,
                                                      "right_candidate_lines", 21, right_candidates,
                                                      1.0f, 0.65f, 0.0f, candidate_line_scale_));

        if (has_seed_point)
        {
            ma.markers.push_back(makePointMarker(scan_msg->header.frame_id, scan_msg->header.stamp,
                                                 "nearest_seed_point", 39,
                                                 std::vector<Point2D>{seed_point},
                                                 1.0f, 1.0f, 1.0f, point_marker_scale_ * 1.9));
        }

        if (right_best.valid)
        {
            ma.markers.push_back(makePointMarker(scan_msg->header.frame_id, scan_msg->header.stamp,
                                                 "right_best_support", 40, right_best.support_pts,
                                                 1.0f, 0.5f, 0.0f, point_marker_scale_ * 1.2));
            ma.markers.push_back(makeLineStripMarker(scan_msg->header.frame_id, scan_msg->header.stamp,
                                                     "right_best_line", 41,
                                                     right_best.p0, right_best.p1,
                                                     1.0f, 0.3f, 0.0f, best_line_scale_));
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "nearest-wall R score=%.2f pts=%d span=%.2f res=%.3f ang=%.1f seed=%.2f",
                          right_best.score,
                          right_best.point_count,
                          right_best.span_length,
                          right_best.residual_rms,
                          right_best.tangent_angle_deg,
                          std::isfinite(seed_range_m) ? seed_range_m : -1.0);
            ma.markers.push_back(makeTextMarker(scan_msg->header.frame_id, scan_msg->header.stamp,
                                                "right_best_text", 42,
                                                right_best.centroid, std::string(buf),
                                                1.0f, 0.3f, 0.0f));
        }

        if (right_local_path_valid)
        {
            ma.markers.push_back(makeLineStripMarker(scan_msg->header.frame_id, scan_msg->header.stamp,
                                                     "right_offset_reference_line", 50,
                                                     offset_line_p0, offset_line_p1,
                                                     0.9f, 0.0f, 0.9f, path_offset_line_scale_, 0.11));

            ma.markers.push_back(makePathLineMarker(scan_msg->header.frame_id, scan_msg->header.stamp,
                                                    "right_local_path_line", 51,
                                                    right_local_path,
                                                    1.0f, 0.0f, 1.0f, path_marker_scale_));

            ma.markers.push_back(makePathPointsMarker(scan_msg->header.frame_id, scan_msg->header.stamp,
                                                      "right_local_path_points", 52,
                                                      right_local_path,
                                                      1.0f, 0.2f, 1.0f, path_point_marker_scale_));

            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "right_local_path offset=%.2f[m] n=%zu mode=%s cnt=%d d=%.3f da=%.2f",
                          clampValue(right_path_offset_m_, right_path_offset_min_m_, right_path_offset_max_m_),
                          right_local_path.poses.size(),
                          stable_path_state_.mode.c_str(),
                          stable_path_state_.over_trigger_count,
                          stable_path_state_.last_lateral_diff_m,
                          stable_path_state_.last_angle_diff_deg);
            ma.markers.push_back(makeTextMarker(scan_msg->header.frame_id, scan_msg->header.stamp,
                                                "right_local_path_text", 53,
                                                offset_centroid,
                                                std::string(buf),
                                                1.0f, 0.0f, 1.0f));
        }

        if (enable_path_tracker_)
        {
            if (tracking_cmd.has_reference_point)
            {
                ma.markers.push_back(makePointMarker(scan_msg->header.frame_id, scan_msg->header.stamp,
                                                     "tracker_reference_point", 59,
                                                     std::vector<Point2D>{tracking_cmd.reference_point},
                                                     0.1f, 1.0f, 0.1f, point_marker_scale_ * 1.6));
            }

            if (tracking_cmd.has_nearest_point)
            {
                ma.markers.push_back(makePointMarker(scan_msg->header.frame_id, scan_msg->header.stamp,
                                                     "tracker_nearest_point", 60,
                                                     std::vector<Point2D>{tracking_cmd.nearest_point},
                                                     0.0f, 0.6f, 1.0f, point_marker_scale_ * 1.4));
            }

            if (tracking_cmd.has_target_point)
            {
                ma.markers.push_back(makePointMarker(scan_msg->header.frame_id, scan_msg->header.stamp,
                                                     "tracker_target_point", 61,
                                                     std::vector<Point2D>{tracking_cmd.target_point},
                                                     1.0f, 0.0f, 0.0f, point_marker_scale_ * 1.6));
                const Point2D tracker_reference =
                    tracking_cmd.has_reference_point ? tracking_cmd.reference_point : Point2D{0.0, 0.0};
                ma.markers.push_back(makeLineStripMarker(scan_msg->header.frame_id, scan_msg->header.stamp,
                                                         "tracker_target_ray", 62,
                                                         tracker_reference, tracking_cmd.target_point,
                                                         1.0f, 0.0f, 0.0f, 0.025, 0.08));
            }

            char buf[320];
            std::snprintf(buf, sizeof(buf),
                          "tracker=%s status=%s v=%.3f wz=%.3f ey=%.3f eth=%.2f ld=%.2f",
                          tracking_cmd.controller_name.c_str(),
                          tracking_cmd.status.c_str(),
                          tracking_cmd.linear_x,
                          tracking_cmd.angular_z,
                          tracking_cmd.cross_track_error_m,
                          tracking_cmd.heading_error_deg,
                          tracking_cmd.lookahead_distance_m);
            ma.markers.push_back(makeTextMarker(scan_msg->header.frame_id, scan_msg->header.stamp,
                                                "tracker_text", 63,
                                                Point2D{0.80, 0.0},
                                                std::string(buf),
                                                1.0f, 1.0f, 1.0f));
        }

        marker_array_pub_.publish(ma);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {
        if (!shouldRunFence())
        {
            updateFenceStatus();
            return;
        }

        updateFenceStatus();

        const std::vector<ScanPoint> all_pts = scanToOrderedPoints(scan_msg);

        const std::vector<ScanPoint> right_selected_raw = selectSidePoints(all_pts, false);
        const std::vector<ScanPoint> right_selected = applyDynamicRightWallGate(right_selected_raw,
                                                                                scan_msg->header.stamp);
        const std::vector<ScanPoint> empty_left_selected;

        appendAccumulationFrame(scan_msg->header.stamp, empty_left_selected, right_selected);

        const std::vector<ScanPoint> right_accumulated_raw = collectAccumulatedSidePoints(scan_msg->header.stamp,
                                                                                           right_selected,
                                                                                           false);
        const std::vector<ScanPoint> right_accumulated = applyDynamicRightWallGate(right_accumulated_raw,
                                                                                    scan_msg->header.stamp);

        std::vector<Cluster> right_clusters = buildClusters(right_accumulated, scan_msg->angle_increment);
        right_clusters = mergeClustersSecondPass(right_clusters, false);

        std::vector<FittedLine> right_candidates;
        FittedLine right_best_raw;
        std::vector<ScanPoint> best_cluster_pts;
        Point2D seed_point;
        double seed_range_m = std::numeric_limits<double>::infinity();
        fitNearestRightWallLocalLine(right_clusters,
                                     right_candidates,
                                     right_best_raw,
                                     best_cluster_pts,
                                     seed_point,
                                     seed_range_m);

        FittedLine right_path_reference_line;
        updateStablePathReference(scan_msg->header.stamp,
                                  right_best_raw,
                                  right_path_reference_line);

        nav_msgs::Path right_local_path;
        Point2D offset_line_p0;
        Point2D offset_line_p1;
        Point2D offset_centroid;
        const bool right_local_path_valid = buildRightLocalPath(scan_msg->header,
                                                                right_path_reference_line,
                                                                right_local_path,
                                                                offset_line_p0,
                                                                offset_line_p1,
                                                                offset_centroid);
        publishPath(scan_msg->header, right_local_path_valid ? right_local_path : nav_msgs::Path());

        TrackingCommand tracking_cmd;
        if (right_local_path_valid)
            computeTrackingCommand(right_local_path, tracking_cmd);
        else
        {
            tracking_cmd.controller_name = normalizedTrackingControllerType();
            tracking_cmd.status = "path_invalid";
        }
        publishCmdVel(tracking_cmd);

        FittedLine right_best_display = right_best_raw;
        if (right_best_display.valid)
            extendLineForDisplay(right_best_display);

        publishMarkers(scan_msg,
                       right_selected,
                       right_accumulated,
                       best_cluster_pts,
                       std::isfinite(seed_range_m),
                       seed_point,
                       seed_range_m,
                       right_candidates,
                       right_best_display,
                       right_local_path,
                       right_local_path_valid,
                       offset_line_p0,
                       offset_line_p1,
                       offset_centroid,
                       tracking_cmd);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "corridor_parallel_right_selector_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    CorridorParallelRightSelector node(nh, pnh);
    ros::spin();
    return 0;

}
