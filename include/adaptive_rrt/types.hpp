#ifndef ADAPTIVE_RRT_TYPES_HPP_
#define ADAPTIVE_RRT_TYPES_HPP_

#include <vector>
#include <optional>
#include <cmath>
#include <memory>
#include <random>
#include <algorithm>

namespace adaptive_rrt {

/**
 * @brief 2D位置クラス
 */
struct Position {
    double x, y;
    
    Position() : x(0.0), y(0.0) {}
    Position(double x, double y) : x(x), y(y) {}
    
    double distance_to(const Position& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    Position operator+(const Position& other) const {
        return Position(x + other.x, y + other.y);
    }
    
    Position operator-(const Position& other) const {
        return Position(x - other.x, y - other.y);
    }
    
    Position operator*(double scalar) const {
        return Position(x * scalar, y * scalar);
    }
    
    bool operator==(const Position& other) const {
        return std::abs(x - other.x) < 1e-6 && std::abs(y - other.y) < 1e-6;
    }
};

/**
 * @brief RRTノードクラス
 */
struct RRTNode {
    Position position;
    std::shared_ptr<RRTNode> parent;
    std::vector<std::shared_ptr<RRTNode>> children;
    double cost;
    int id;
    bool is_valid;
    
    RRTNode() : cost(0.0), id(0), is_valid(true) {}
    
    RRTNode(const Position& pos, std::shared_ptr<RRTNode> p = nullptr, 
            double c = 0.0, int node_id = 0)
        : position(pos), parent(p), cost(c), id(node_id), is_valid(true) {}
        
    void add_child(std::shared_ptr<RRTNode> child) {
        children.push_back(child);
    }
    
    void remove_child(std::shared_ptr<RRTNode> child) {
        auto it = std::find(children.begin(), children.end(), child);
        if (it != children.end()) {
            children.erase(it);
        }
    }
};

/**
 * @brief 経路上の点クラス
 */
struct PathPoint {
    Position position;
    double cost;
    
    PathPoint() : cost(0.0) {}
    PathPoint(const Position& pos, double c = 0.0) : position(pos), cost(c) {}
};

/**
 * @brief 経路クラス
 */
class Path {
public:
    std::vector<PathPoint> points;
    double total_cost;
    
    Path() : total_cost(0.0) {}
    
    void add_point(const Position& position, double cost = 0.0) {
        points.emplace_back(position, cost);
        total_cost += cost;
    }
    
    void clear() {
        points.clear();
        total_cost = 0.0;
    }
    
    bool empty() const {
        return points.empty();
    }
    
    size_t size() const {
        return points.size();
    }
    
    const PathPoint& operator[](size_t index) const {
        return points[index];
    }
    
    PathPoint& operator[](size_t index) {
        return points[index];
    }
};

/**
 * @brief RRT設定クラス
 */
struct RRTConfig {
    double step_size;               // RRT探索時のステップサイズ [m]
    double goal_tolerance;          // ゴール到達判定の許容距離 [m]
    double goal_bias;               // ゴール方向への探索バイアス [0-1]
    double rewire_radius;           // RRT*での再配線半径 [m]
    int max_iterations;             // 最大反復回数
    double max_distance_to_obstacle; // 障害物からの最小距離 [m]
    bool enable_rrt_star;           // RRT*を有効にするか
    
    // 制御点・経路出力関連
    double path_point_min_distance; // 制御点の最小間隔 [m]
    bool enable_path_smoothing;     // 経路平滑化を有効にするか
    int smoothing_iterations;       // 平滑化反復回数
    double smoothing_weight;        // 平滑化の重み [0-1]
    double curvature_threshold;     // 曲率による平滑化閾値
    
    RRTConfig()
        : step_size(0.5)
        , goal_tolerance(0.2)
        , goal_bias(0.1)
        , rewire_radius(1.0)
        , max_iterations(1000)
        , max_distance_to_obstacle(0.3)
        , enable_rrt_star(true)
        , path_point_min_distance(0.1)
        , enable_path_smoothing(true)
        , smoothing_iterations(5)
        , smoothing_weight(0.3)
        , curvature_threshold(0.5) {}
};

/**
 * @brief 障害物クラス（円形）
 */
struct CircularObstacle {
    Position center;
    double radius;
    
    CircularObstacle() : radius(0.0) {}
    CircularObstacle(const Position& pos, double r) : center(pos), radius(r) {}
    
    bool is_point_inside(const Position& point) const {
        return center.distance_to(point) <= radius;
    }
    
    bool intersects_line(const Position& start, const Position& end) const {
        // 線分と円の交差判定
        Position line_vec = end - start;
        Position start_to_center = center - start;
        
        double line_length_sq = line_vec.x * line_vec.x + line_vec.y * line_vec.y;
        if (line_length_sq < 1e-6) {
            return is_point_inside(start);
        }
        
        double t = std::max(0.0, std::min(1.0, 
            (start_to_center.x * line_vec.x + start_to_center.y * line_vec.y) / line_length_sq));
        
        Position closest = start + line_vec * t;
        return center.distance_to(closest) <= radius;
    }
};

/**
 * @brief RRTツリー統計
 */
struct RRTStatistics {
    int nodes_generated;
    int goal_found_iteration;
    double path_length;
    double computation_time_ms;
    
    RRTStatistics() 
        : nodes_generated(0)
        , goal_found_iteration(-1)
        , path_length(0.0)
        , computation_time_ms(0.0) {}
};

/**
 * @brief 衝突エッジ情報
 */
struct CollisionEdge {
    std::shared_ptr<RRTNode> parent_node;
    std::shared_ptr<RRTNode> child_node;
    Position collision_point;
    double collision_distance;
    
    CollisionEdge(std::shared_ptr<RRTNode> parent, std::shared_ptr<RRTNode> child,
                  const Position& point, double distance)
        : parent_node(parent), child_node(child), collision_point(point), collision_distance(distance) {}
};

/**
 * @brief 動的障害物対応結果
 */
struct DynamicObstacleHandlingResult {
    bool success;
    int collision_edges_identified;
    int nodes_invalidated;
    int subtrees_reconnected;
    int new_nodes_sampled;
    std::optional<Path> final_path;
    
    DynamicObstacleHandlingResult()
        : success(false)
        , collision_edges_identified(0)
        , nodes_invalidated(0)
        , subtrees_reconnected(0)
        , new_nodes_sampled(0) {}
};

/**
 * @brief サンプリング補完結果
 */
struct SamplingComplementResult {
    int new_nodes_count;
    double sampling_efficiency;
    bool path_to_goal_found;
    std::vector<std::shared_ptr<RRTNode>> new_nodes;
    std::optional<Path> final_path;
    
    SamplingComplementResult()
        : new_nodes_count(0)
        , sampling_efficiency(0.0)
        , path_to_goal_found(false) {}
};

} // namespace adaptive_rrt

#endif // ADAPTIVE_RRT_TYPES_HPP_