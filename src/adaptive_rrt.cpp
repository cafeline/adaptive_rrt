#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "adaptive_rrt/rrt_planner.hpp"
#include "adaptive_rrt/types.hpp"
#include <optional>
#include <memory>
#include <chrono>
#include <vector>
#include <map>

namespace adaptive_rrt {

class AdaptiveRRTNode : public rclcpp::Node {
public:
    AdaptiveRRTNode() : Node("adaptive_rrt_node") {
        // パラメータ設定
        this->declare_parameter("wall_clearance_distance", 0.3);
        this->declare_parameter("global_frame", "map");
        this->declare_parameter("base_frame", "base_footprint");

        // RRT設定パラメータ
        this->declare_parameter("step_size", 0.5);
        this->declare_parameter("goal_tolerance", 0.2);
        this->declare_parameter("goal_bias", 0.1);
        this->declare_parameter("rewire_radius", 1.0);
        this->declare_parameter("max_iterations", 1000);
        this->declare_parameter("enable_rrt_star", true);

        // 制御点・経路出力関連パラメータ
        this->declare_parameter("path_point_min_distance", 0.1);
        this->declare_parameter("enable_path_smoothing", true);
        this->declare_parameter("smoothing_iterations", 5);
        this->declare_parameter("smoothing_weight", 0.3);
        this->declare_parameter("curvature_threshold", 0.5);

        // 動的再経路計画パラメータ
        this->declare_parameter("obstacle_blocking_timeout", 3.0);
        this->declare_parameter("path_obstacle_check_radius", 0.5);
        this->declare_parameter("path_interpolation_resolution", 0.05);
        this->declare_parameter("enable_dynamic_replanning", true);

        // TF2関連初期化
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // パブリッシャー初期化
        auto path_qos = rclcpp::QoS(10).reliable();
        path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("rrt_planned_path", path_qos);

        auto viz_qos = rclcpp::QoS(10).reliable().durability_volatile();
        path_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("rrt_path_viz", viz_qos);
        tree_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("rrt_tree_viz", viz_qos);

        // 膨張マップ可視化用のパブリッシャー
        auto map_viz_qos = rclcpp::QoS(1).reliable().transient_local();
        inflated_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("rrt_inflated_map", map_viz_qos);

        // サブスクライバー初期化
        auto map_qos = rclcpp::QoS(1).reliable().transient_local();
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", map_qos, std::bind(&AdaptiveRRTNode::map_callback, this, std::placeholders::_1));

        // clicked_pointを受け取ってゴールとして設定
        clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "clicked_point", 10, std::bind(&AdaptiveRRTNode::clicked_point_callback, this, std::placeholders::_1));

        // 障害物情報を受け取る
        static_obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "static_obstacles", 10, std::bind(&AdaptiveRRTNode::static_obstacles_callback, this, std::placeholders::_1));
        dynamic_obstacles_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "dynamic_obstacles", 10, std::bind(&AdaptiveRRTNode::dynamic_obstacles_callback, this, std::placeholders::_1));

        // 動的再経路計画の初期化
        path_is_blocked_ = false;

        // 経路監視タイマー（1Hz）
        path_monitor_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&AdaptiveRRTNode::monitor_path_obstacles, this));

        RCLCPP_INFO(this->get_logger(), "Adaptive RRT Node initialized");
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        try {
            occupancy_grid_ = *msg;

            // RRT設定を更新
            RRTConfig config;
            config.step_size = this->get_parameter("step_size").as_double();
            config.goal_tolerance = this->get_parameter("goal_tolerance").as_double();
            config.goal_bias = this->get_parameter("goal_bias").as_double();
            config.rewire_radius = this->get_parameter("rewire_radius").as_double();
            config.max_iterations = this->get_parameter("max_iterations").as_int();
            config.enable_rrt_star = this->get_parameter("enable_rrt_star").as_bool();

            // 制御点・経路出力関連設定
            config.path_point_min_distance = this->get_parameter("path_point_min_distance").as_double();
            config.enable_path_smoothing = this->get_parameter("enable_path_smoothing").as_bool();
            config.smoothing_iterations = this->get_parameter("smoothing_iterations").as_int();
            config.smoothing_weight = this->get_parameter("smoothing_weight").as_double();
            config.curvature_threshold = this->get_parameter("curvature_threshold").as_double();

            // RRT経路計画器を初期化
            Position origin(msg->info.origin.position.x, msg->info.origin.position.y);
            double wall_clearance = this->get_parameter("wall_clearance_distance").as_double();

            path_planner_ = std::make_unique<RRTPathPlanner>(
                *msg, msg->info.resolution, origin, wall_clearance, config);

            RCLCPP_DEBUG(this->get_logger(), "Map received: %dx%d, resolution: %.3f m/cell",
                       msg->info.width, msg->info.height, msg->info.resolution);

            // 統合膨張マップをパブリッシュ
            publish_integrated_inflated_map();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Map callback error: %s", e.what());
        }
    }

    void clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        try {
            goal_position_ = Position(msg->point.x, msg->point.y);
            RCLCPP_DEBUG(this->get_logger(), "Goal set from clicked_point: (%.2f, %.2f)",
                       goal_position_->x, goal_position_->y);

            // ロボットの現在位置をTFから取得
            auto robot_pos = get_robot_position();
            if (robot_pos.has_value()) {
                start_position_ = robot_pos.value();
                RCLCPP_DEBUG(this->get_logger(), "Start position from TF: (%.2f, %.2f)",
                           start_position_->x, start_position_->y);
            } else {
                // TF取得に失敗した場合は原点を使用
                start_position_ = Position(0.0, 0.0);
                RCLCPP_WARN(this->get_logger(), "Failed to get robot position from TF, using origin");
            }

            plan_path();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Clicked point callback error: %s", e.what());
        }
    }

    void plan_path() {
        if (!path_planner_ || !start_position_.has_value() || !goal_position_.has_value()) {
            RCLCPP_WARN(this->get_logger(), "Path planning: missing requirements");
            return;
        }

        try {
            auto start_time = std::chrono::high_resolution_clock::now();
            auto planned_path = path_planner_->plan_path(start_position_.value(), goal_position_.value());
            auto end_time = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

            if (planned_path.has_value()) {
                current_path_ = planned_path.value();
                const auto& stats = path_planner_->get_statistics();
                
                RCLCPP_INFO(this->get_logger(), 
                    "RRT Path Planning - Points: %zu, Nodes: %d, Time: %ld ms, Length: %.2f m",
                    current_path_.size(), stats.nodes_generated, duration.count(), stats.path_length);

                publish_path();
                publish_path_visualization();
                publish_tree_visualization();
            } else {
                RCLCPP_WARN(this->get_logger(), "RRT Path planning failed");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Path planning error: %s", e.what());
        }
    }

    void publish_path() {
        if (current_path_.empty()) {
            return;
        }

        auto path_msg = geometry_msgs::msg::PoseArray();
        path_msg.header.frame_id = this->get_parameter("global_frame").as_string();
        path_msg.header.stamp = this->get_clock()->now();

        for (const auto& path_point : current_path_.points) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = path_point.position.x;
            pose.position.y = path_point.position.y;
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
    }

    void publish_path_visualization() {
        if (current_path_.empty()) {
            return;
        }

        auto marker_array = visualization_msgs::msg::MarkerArray();
        std::string global_frame = this->get_parameter("global_frame").as_string();

        // 古いマーカーを削除
        auto delete_marker = visualization_msgs::msg::Marker();
        delete_marker.header.frame_id = global_frame;
        delete_marker.header.stamp = this->get_clock()->now();
        delete_marker.id = 0;
        delete_marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(delete_marker);

        // 線分マーカー（経路全体）
        auto line_marker = visualization_msgs::msg::Marker();
        line_marker.header.frame_id = global_frame;
        line_marker.header.stamp = this->get_clock()->now();
        line_marker.id = 0;
        line_marker.ns = "rrt_path";
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;

        for (const auto& path_point : current_path_.points) {
            geometry_msgs::msg::Point point;
            point.x = path_point.position.x;
            point.y = path_point.position.y;
            point.z = 0.1;
            line_marker.points.push_back(point);
        }

        line_marker.scale.x = 0.08;  // RRTは少し太くして区別

        // 再計画された経路は緑色、通常経路は青色
        if (path_is_blocked_) {
            line_marker.color.r = 0.0;  // 緑色（再計画経路）
            line_marker.color.g = 1.0;
            line_marker.color.b = 0.0;
        } else {
            line_marker.color.r = 0.0;  // 青色（通常経路）
            line_marker.color.g = 0.0;
            line_marker.color.b = 1.0;
        }
        line_marker.color.a = 0.8;
        line_marker.lifetime = rclcpp::Duration::from_seconds(0);

        marker_array.markers.push_back(line_marker);
        path_viz_pub_->publish(marker_array);

        RCLCPP_DEBUG(this->get_logger(), "Published RRT path visualization with %zu points",
                   line_marker.points.size());
    }

    void publish_tree_visualization() {
        // RRTツリーの可視化は省略（必要に応じて実装）
        // 現在は簡単なパス可視化のみ実装
    }

    void publish_integrated_inflated_map() {
        if (!path_planner_) {
            return;
        }
        
        try {
            // 基本の膨張マップを取得
            const auto& base_inflated_grid = path_planner_->get_inflated_grid();
            
            // 統合マップを作成（基本膨張マップをコピー）
            auto integrated_grid = base_inflated_grid;
            
            // static_obstaclesのマスクを統合
            add_obstacles_to_grid(static_obstacles_, integrated_grid);
            
            // dynamic_obstaclesのマスクを統合  
            add_obstacles_to_grid(dynamic_obstacles_, integrated_grid);
            
            // OccupancyGridメッセージを作成
            auto inflated_map_msg = nav_msgs::msg::OccupancyGrid();
            inflated_map_msg.header.frame_id = this->get_parameter("global_frame").as_string();
            inflated_map_msg.header.stamp = this->get_clock()->now();
            
            // 元のマップ情報をコピー
            inflated_map_msg.info = occupancy_grid_.info;
            
            // 統合マップデータを変換
            inflated_map_msg.data.resize(integrated_grid.size() * integrated_grid[0].size());
            for (size_t y = 0; y < integrated_grid.size(); ++y) {
                for (size_t x = 0; x < integrated_grid[y].size(); ++x) {
                    int index = y * integrated_grid[y].size() + x;
                    inflated_map_msg.data[index] = static_cast<int8_t>(integrated_grid[y][x]);
                }
            }
            
            // パブリッシュ
            inflated_map_pub_->publish(inflated_map_msg);
            
            RCLCPP_DEBUG(this->get_logger(), "Published RRT integrated inflated map: %zux%zu cells",
                       integrated_grid[0].size(), integrated_grid.size());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to publish integrated inflated map: %s", e.what());
        }
    }
    
    void add_obstacles_to_grid(const std::vector<visualization_msgs::msg::Marker>& obstacles,
                              std::vector<std::vector<int>>& grid) {
        if (!path_planner_ || occupancy_grid_.info.width == 0 || occupancy_grid_.info.height == 0) {
            return;
        }
        
        double resolution = occupancy_grid_.info.resolution;
        double origin_x = occupancy_grid_.info.origin.position.x;
        double origin_y = occupancy_grid_.info.origin.position.y;
        
        for (const auto& obstacle : obstacles) {
            // 障害物の位置をグリッド座標に変換
            int center_x = static_cast<int>((obstacle.pose.position.x - origin_x) / resolution);
            int center_y = static_cast<int>((obstacle.pose.position.y - origin_y) / resolution);
            
            // 障害物のサイズから膨張半径を計算
            double obstacle_radius = std::max(obstacle.scale.x, obstacle.scale.y) / 2.0;
            double clearance_distance = this->get_parameter("wall_clearance_distance").as_double();
            double total_radius = obstacle_radius + clearance_distance;
            int inflation_cells = static_cast<int>(total_radius / resolution) + 1;
            
            // 障害物周囲を膨張
            for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                    int grid_x = center_x + dx;
                    int grid_y = center_y + dy;
                    
                    // 境界チェック
                    if (grid_x >= 0 && grid_x < static_cast<int>(grid[0].size()) && 
                        grid_y >= 0 && grid_y < static_cast<int>(grid.size())) {
                        
                        // ユークリッド距離で膨張範囲を決定
                        double distance = std::sqrt(dx * dx + dy * dy) * resolution;
                        if (distance <= total_radius) {
                            // 占有値を設定（100は完全占有）
                            grid[grid_y][grid_x] = 100;
                        }
                    }
                }
            }
        }
    }

    std::optional<Position> get_robot_position() {
        try {
            std::string base_frame = this->get_parameter("base_frame").as_string();
            std::string global_frame = this->get_parameter("global_frame").as_string();

            // TF取得
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(
                global_frame, base_frame, tf2::TimePointZero, tf2::durationFromSec(0.1));

            // 位置の取得
            Position position(transform_stamped.transform.translation.x,
                            transform_stamped.transform.translation.y);

            return position;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_DEBUG(this->get_logger(), "TF lookup failed: %s", ex.what());
            return std::nullopt;
        }
    }

    void static_obstacles_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        static_obstacles_.clear();
        for (const auto& marker : msg->markers) {
            if (marker.action == visualization_msgs::msg::Marker::ADD) {
                static_obstacles_.push_back(marker);
            }
        }
        
        // static_obstaclesが更新されたら統合膨張マップを再パブリッシュ
        publish_integrated_inflated_map();
    }

    void dynamic_obstacles_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        dynamic_obstacles_.clear();
        for (const auto& marker : msg->markers) {
            if (marker.action == visualization_msgs::msg::Marker::ADD) {
                dynamic_obstacles_.push_back(marker);
            }
        }
        
        // dynamic_obstaclesが更新されたら統合膨張マップを再パブリッシュ
        publish_integrated_inflated_map();
    }

    void monitor_path_obstacles() {
        if (!this->get_parameter("enable_dynamic_replanning").as_bool()) {
            return;
        }

        if (current_path_.empty() || !start_position_.has_value() || !goal_position_.has_value()) {
            return;
        }

        double check_radius = this->get_parameter("path_obstacle_check_radius").as_double();
        double blocking_timeout = this->get_parameter("obstacle_blocking_timeout").as_double();
        auto now = std::chrono::steady_clock::now();

        // 経路上の障害物をチェック
        bool current_blocking = check_path_blocked(check_radius);

        RCLCPP_DEBUG(this->get_logger(), "monitor_path_obstacles: current_blocking=%s, path_is_blocked_=%s",
                    current_blocking ? "true" : "false", path_is_blocked_ ? "true" : "false");

        if (current_blocking) {
            if (!path_is_blocked_) {
                // 新しい阻害開始
                path_blocking_start_times_["current"] = now;
                path_is_blocked_ = true;
                RCLCPP_INFO(this->get_logger(), "[DEBUG] RRT Path blocked by obstacles, monitoring... (timeout=%.1fs)", blocking_timeout);
            } else {
                // 阻害継続中
                auto blocked_duration = std::chrono::duration_cast<std::chrono::duration<double>>(
                    now - path_blocking_start_times_["current"]).count();

                RCLCPP_INFO(this->get_logger(), "[DEBUG] RRT Path blocked for %.1f/%.1f seconds", blocked_duration, blocking_timeout);

                if (blocked_duration >= blocking_timeout) {
                    RCLCPP_INFO(this->get_logger(), "[DEBUG] Triggering RRT replanning after %.1f seconds", blocked_duration);
                    replan_path_with_obstacles();
                    path_blocking_start_times_.clear();
                    path_is_blocked_ = false;
                }
            }
        } else {
            // 阻害解除
            if (path_is_blocked_) {
                RCLCPP_INFO(this->get_logger(), "[DEBUG] RRT Path no longer blocked");
                path_is_blocked_ = false;
                path_blocking_start_times_.clear();
            }
        }
    }

    bool check_path_blocked(double check_radius) {
        if (current_path_.empty()) {
            return false;
        }

        // 経路を補間して密な点列を生成
        double interpolation_resolution = this->get_parameter("path_interpolation_resolution").as_double();
        auto interpolated_points = interpolate_path(current_path_, interpolation_resolution);

        for (const auto& path_point : interpolated_points) {
            // 各経路点の周囲に障害物があるかチェック
            for (const auto& static_obstacle : static_obstacles_) {
                if (is_obstacle_near_point(path_point, static_obstacle, check_radius)) {
                    return true;
                }
            }
            for (const auto& dynamic_obstacle : dynamic_obstacles_) {
                if (is_obstacle_near_point(path_point, dynamic_obstacle, check_radius)) {
                    return true;
                }
            }
        }
        
        return false;
    }

    std::vector<Position> interpolate_path(const Path& path, double resolution) {
        std::vector<Position> interpolated_points;

        if (path.empty()) {
            return interpolated_points;
        }

        // 最初の点を追加
        interpolated_points.push_back(path.points[0].position);

        for (size_t i = 1; i < path.points.size(); ++i) {
            const auto& prev_point = path.points[i-1].position;
            const auto& curr_point = path.points[i].position;

            // 2点間の距離を計算
            double distance = prev_point.distance_to(curr_point);

            // 補間が必要かチェック
            if (distance > resolution) {
                // 必要な分割数を計算
                int num_divisions = static_cast<int>(std::ceil(distance / resolution));

                // 分割点を生成
                for (int j = 1; j < num_divisions; ++j) {
                    double t = static_cast<double>(j) / num_divisions;
                    Position interpolated_point(
                        prev_point.x + t * (curr_point.x - prev_point.x),
                        prev_point.y + t * (curr_point.y - prev_point.y)
                    );
                    interpolated_points.push_back(interpolated_point);
                }
            }

            // 現在の点を追加
            interpolated_points.push_back(curr_point);
        }

        return interpolated_points;
    }

    bool is_obstacle_near_point(const Position& point,
                               const visualization_msgs::msg::Marker& obstacle,
                               double radius) {
        double dx = point.x - obstacle.pose.position.x;
        double dy = point.y - obstacle.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // 障害物のサイズも考慮
        double obstacle_radius = std::max(obstacle.scale.x, obstacle.scale.y) / 2.0;
        return distance <= (radius + obstacle_radius);
    }

    void replan_path_with_obstacles() {
        if (!path_planner_ || !start_position_.has_value() || !goal_position_.has_value()) {
            return;
        }

        try {
            RCLCPP_INFO(this->get_logger(), "Starting RRT dynamic replanning with obstacle avoidance");

            // 現在のロボット位置を取得
            auto robot_pos = get_robot_position();
            if (robot_pos.has_value()) {
                start_position_ = robot_pos.value();
            }

            // 障害物情報を変換
            std::vector<std::pair<Position, double>> static_obstacles_converted;
            std::vector<std::pair<Position, double>> dynamic_obstacles_converted;

            // 静的障害物を変換
            for (const auto& static_obstacle : static_obstacles_) {
                Position obs_pos(static_obstacle.pose.position.x, static_obstacle.pose.position.y);
                double obs_radius = std::sqrt(static_obstacle.scale.x * static_obstacle.scale.x +
                                            static_obstacle.scale.y * static_obstacle.scale.y) / 2.0 * 1.1;
                static_obstacles_converted.emplace_back(obs_pos, obs_radius);
            }

            // 動的障害物を変換
            for (const auto& dynamic_obstacle : dynamic_obstacles_) {
                Position obs_pos(dynamic_obstacle.pose.position.x, dynamic_obstacle.pose.position.y);
                double obs_radius = std::sqrt(dynamic_obstacle.scale.x * dynamic_obstacle.scale.x +
                                            dynamic_obstacle.scale.y * dynamic_obstacle.scale.y) / 2.0 * 1.1;
                dynamic_obstacles_converted.emplace_back(obs_pos, obs_radius);
            }

            RCLCPP_INFO(this->get_logger(), "[DEBUG] RRT Replanning with %zu static and %zu dynamic obstacles",
                       static_obstacles_converted.size(), dynamic_obstacles_converted.size());

            // 障害物情報を考慮した再計画
            auto start_time = std::chrono::high_resolution_clock::now();
            auto planned_path = path_planner_->plan_path_with_dynamic_obstacles(
                start_position_.value(), goal_position_.value(), dynamic_obstacles_converted, static_obstacles_converted);
            auto end_time = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

            if (planned_path.has_value()) {
                current_path_ = planned_path.value();
                const auto& stats = path_planner_->get_statistics();
                
                RCLCPP_INFO(this->get_logger(), 
                    "[DEBUG] RRT Dynamic replanning successful - Points: %zu, Nodes: %d, Time: %ld ms",
                    current_path_.size(), stats.nodes_generated, duration.count());

                publish_path();
                publish_path_visualization();
                publish_tree_visualization();

                RCLCPP_INFO(this->get_logger(), "[DEBUG] RRT Path and visualization published");
            } else {
                RCLCPP_INFO(this->get_logger(), "[DEBUG] RRT Dynamic replanning failed after %ld ms", duration.count());
            }
        } catch (const std::exception& e) {
            RCLCPP_INFO(this->get_logger(), "[DEBUG] RRT Dynamic replanning exception: %s", e.what());
        }
    }

    // メンバ変数
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_viz_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_viz_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_map_pub_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr static_obstacles_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_obstacles_sub_;

    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    std::unique_ptr<RRTPathPlanner> path_planner_;

    // TF2関連
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::optional<Position> start_position_;
    std::optional<Position> goal_position_;
    Path current_path_;

    // 動的再経路計画用
    std::vector<visualization_msgs::msg::Marker> static_obstacles_;
    std::vector<visualization_msgs::msg::Marker> dynamic_obstacles_;
    std::map<std::string, std::chrono::steady_clock::time_point> path_blocking_start_times_;
    bool path_is_blocked_;
    rclcpp::TimerBase::SharedPtr path_monitor_timer_;
};

} // namespace adaptive_rrt

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<adaptive_rrt::AdaptiveRRTNode>());
    rclcpp::shutdown();
    return 0;
}
