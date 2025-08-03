#include "adaptive_rrt/rrt_planner.hpp"
#include <cmath>
#include <chrono>
#include <algorithm>
#include <queue>

namespace adaptive_rrt {

RRTPathPlanner::RRTPathPlanner(const nav_msgs::msg::OccupancyGrid& grid,
                               double resolution, const Position& origin,
                               double wall_clearance_distance,
                               const RRTConfig& config)
    : width_(grid.info.width)
    , height_(grid.info.height)
    , resolution_(resolution)
    , origin_(origin)
    , wall_clearance_distance_(wall_clearance_distance)
    , config_(config)
    , random_generator_(std::random_device{}())
    , uniform_dist_(0.0, 1.0) {
    
    // 占有格子をコピー
    grid_.resize(height_, std::vector<int>(width_));
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int index = y * width_ + x;
            grid_[y][x] = grid.data[index];
        }
    }
    
    // 膨張マップを作成
    create_inflated_grid();
}

std::optional<Path> RRTPathPlanner::plan_path(const Position& start_pos, 
                                              const Position& goal_pos) {
    return run_rrt(start_pos, goal_pos, inflated_grid_);
}

std::optional<Path> RRTPathPlanner::plan_path_with_dynamic_obstacles(
    const Position& start_pos, 
    const Position& goal_pos,
    const std::vector<std::pair<Position, double>>& dynamic_obstacles,
    const std::vector<std::pair<Position, double>>& static_obstacles) {
    
    // 一時的な障害物情報を含む膨張マップを作成
    auto temp_grid = create_temporary_grid_with_obstacles(dynamic_obstacles, static_obstacles);
    
    return run_rrt(start_pos, goal_pos, temp_grid);
}

std::pair<int, int> RRTPathPlanner::world_to_grid(const Position& world_pos) const {
    int grid_x = static_cast<int>((world_pos.x - origin_.x) / resolution_);
    int grid_y = static_cast<int>((world_pos.y - origin_.y) / resolution_);
    return {grid_x, grid_y};
}

Position RRTPathPlanner::grid_to_world(const std::pair<int, int>& grid_pos) const {
    double world_x = origin_.x + grid_pos.first * resolution_;
    double world_y = origin_.y + grid_pos.second * resolution_;
    return Position(world_x, world_y);
}

bool RRTPathPlanner::is_valid_position(const Position& pos) const {
    return is_valid_position(pos, inflated_grid_);
}

bool RRTPathPlanner::is_valid_position(const Position& pos, 
                                       const std::vector<std::vector<int>>& inflated_grid) const {
    auto grid_pos = world_to_grid(pos);
    int x = grid_pos.first;
    int y = grid_pos.second;
    
    // 境界チェック
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
        return false;
    }
    
    // 占有チェック
    return inflated_grid[y][x] < OCCUPIED_THRESHOLD;
}

bool RRTPathPlanner::is_valid_line(const Position& start, const Position& end) const {
    return is_valid_line(start, end, inflated_grid_);
}

bool RRTPathPlanner::is_valid_line(const Position& start, const Position& end,
                                   const std::vector<std::vector<int>>& inflated_grid) const {
    // Bresenhamアルゴリズムの変形で線分を検証
    double distance = start.distance_to(end);
    int num_checks = static_cast<int>(distance / (resolution_ * 0.5)) + 1;
    
    for (int i = 0; i <= num_checks; ++i) {
        double t = static_cast<double>(i) / num_checks;
        Position check_pos = start + (end - start) * t;
        
        if (!is_valid_position(check_pos, inflated_grid)) {
            return false;
        }
    }
    
    return true;
}

const std::vector<std::vector<int>>& RRTPathPlanner::get_inflated_grid() const {
    return inflated_grid_;
}

const RRTStatistics& RRTPathPlanner::get_statistics() const {
    return statistics_;
}

void RRTPathPlanner::update_config(const RRTConfig& new_config) {
    config_ = new_config;
}

void RRTPathPlanner::create_inflated_grid() {
    inflated_grid_ = grid_;
    
    // 膨張半径をピクセル単位に変換
    int inflation_radius = static_cast<int>(wall_clearance_distance_ / resolution_) + 1;
    
    // 元の占有セルを記録
    std::vector<std::pair<int, int>> occupied_cells;
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (grid_[y][x] >= OCCUPIED_THRESHOLD) {
                occupied_cells.emplace_back(x, y);
            }
        }
    }
    
    // 各占有セルの周囲を膨張
    for (const auto& cell : occupied_cells) {
        int center_x = cell.first;
        int center_y = cell.second;
        
        for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
            for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
                int x = center_x + dx;
                int y = center_y + dy;
                
                // 境界チェック
                if (x >= 0 && x < width_ && y >= 0 && y < height_) {
                    // ユークリッド距離で膨張範囲を決定
                    double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                    if (distance <= wall_clearance_distance_) {
                        inflated_grid_[y][x] = 100;  // 占有マーク
                    }
                }
            }
        }
    }
}

std::vector<std::vector<int>> RRTPathPlanner::create_temporary_grid_with_obstacles(
    const std::vector<std::pair<Position, double>>& dynamic_obstacles,
    const std::vector<std::pair<Position, double>>& static_obstacles) {
    
    // ベースとなる膨張マップをコピー
    auto temp_grid = inflated_grid_;
    
    // 全ての障害物を統合して処理
    std::vector<std::pair<Position, double>> all_obstacles;
    all_obstacles.insert(all_obstacles.end(), dynamic_obstacles.begin(), dynamic_obstacles.end());
    all_obstacles.insert(all_obstacles.end(), static_obstacles.begin(), static_obstacles.end());
    
    // 各障害物に対して膨張処理
    for (const auto& obstacle : all_obstacles) {
        const Position& obs_pos = obstacle.first;
        double obs_radius = obstacle.second;
        
        // 障害物の位置をグリッド座標に変換
        auto grid_pos = world_to_grid(obs_pos);
        int center_x = grid_pos.first;
        int center_y = grid_pos.second;
        
        // 膨張半径を計算
        double total_radius = obs_radius + wall_clearance_distance_;
        int inflation_cells = static_cast<int>(total_radius / resolution_) + 1;
        
        // 障害物周囲を膨張
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
            for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                int x = center_x + dx;
                int y = center_y + dy;
                
                // 境界チェック
                if (x >= 0 && x < width_ && y >= 0 && y < height_) {
                    // ユークリッド距離で膨張範囲を決定
                    double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                    if (distance <= total_radius) {
                        temp_grid[y][x] = 100;  // 占有マーク
                    }
                }
            }
        }
    }
    
    return temp_grid;
}

std::optional<Path> RRTPathPlanner::run_rrt(const Position& start_pos,
                                            const Position& goal_pos,
                                            const std::vector<std::vector<int>>& inflated_grid) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 統計情報を初期化
    statistics_ = RRTStatistics();
    
    // 開始位置とゴール位置の妥当性確認
    if (!is_valid_position(start_pos, inflated_grid)) {
        return std::nullopt;
    }
    
    if (!is_valid_position(goal_pos, inflated_grid)) {
        return std::nullopt;
    }
    
    // ツリーを初期化
    tree_.clear();
    auto start_node = std::make_shared<RRTNode>(start_pos, nullptr, 0.0, 0);
    tree_.push_back(start_node);
    
    std::shared_ptr<RRTNode> goal_node = nullptr;
    
    // RRTメインループ
    for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
        Position rand_pos;
        
        // ゴールバイアス
        if (uniform_dist_(random_generator_) < config_.goal_bias) {
            rand_pos = goal_pos;
        } else {
            rand_pos = generate_random_position();
        }
        
        // 最も近いノードを見つける
        auto nearest_node = find_nearest_node(rand_pos);
        if (!nearest_node) {
            continue;
        }
        
        // ステップサイズ分移動
        Position new_pos = steer(nearest_node->position, rand_pos);
        
        // 新しい位置が有効かチェック
        if (!is_valid_position(new_pos, inflated_grid) || 
            !is_valid_line(nearest_node->position, new_pos, inflated_grid)) {
            continue;
        }
        
        // 新しいノードを作成
        double new_cost = nearest_node->cost + nearest_node->position.distance_to(new_pos);
        auto new_node = std::make_shared<RRTNode>(new_pos, nearest_node, new_cost, tree_.size());
        
        // RRT*の場合、近傍ノードで再配線
        if (config_.enable_rrt_star) {
            auto near_nodes = find_near_nodes(new_pos, config_.rewire_radius);
            
            // 最適な親ノードを探す
            for (const auto& near_node : near_nodes) {
                double potential_cost = near_node->cost + near_node->position.distance_to(new_pos);
                if (potential_cost < new_cost && 
                    is_valid_line(near_node->position, new_pos, inflated_grid)) {
                    new_node->parent = near_node;
                    new_cost = potential_cost;
                    new_node->cost = new_cost;
                }
            }
            
            tree_.push_back(new_node);
            
            // 近傍ノードを再配線
            rewire_tree(new_node, near_nodes, inflated_grid);
        } else {
            tree_.push_back(new_node);
        }
        
        // ゴール到達チェック
        if (new_pos.distance_to(goal_pos) <= config_.goal_tolerance) {
            goal_node = new_node;
            statistics_.goal_found_iteration = iteration;
            break;
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    statistics_.computation_time_ms = 
        std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    statistics_.nodes_generated = tree_.size();
    
    if (!goal_node) {
        return std::nullopt;
    }
    
    // 経路を再構築
    Path path = reconstruct_path(goal_node);
    statistics_.path_length = path.total_cost;
    
    // パスをスムージング
    if (config_.enable_path_smoothing) {
        path = iterative_smooth_path(path, inflated_grid);
        path = curvature_based_smoothing(path, inflated_grid);
    }
    
    // 制御点間隔を調整
    path = adjust_path_spacing(path);
    
    return path;
}

Position RRTPathPlanner::generate_random_position() {
    double x = origin_.x + uniform_dist_(random_generator_) * width_ * resolution_;
    double y = origin_.y + uniform_dist_(random_generator_) * height_ * resolution_;
    return Position(x, y);
}

std::shared_ptr<RRTNode> RRTPathPlanner::find_nearest_node(const Position& target_pos) {
    if (tree_.empty()) {
        return nullptr;
    }
    
    std::shared_ptr<RRTNode> nearest = tree_[0];
    double min_distance = nearest->position.distance_to(target_pos);
    
    for (const auto& node : tree_) {
        double distance = node->position.distance_to(target_pos);
        if (distance < min_distance) {
            min_distance = distance;
            nearest = node;
        }
    }
    
    return nearest;
}

Position RRTPathPlanner::steer(const Position& from_pos, const Position& to_pos) {
    double distance = from_pos.distance_to(to_pos);
    
    if (distance <= config_.step_size) {
        return to_pos;
    }
    
    // 方向ベクトルを正規化してステップサイズをかける
    Position direction = to_pos - from_pos;
    double scale = config_.step_size / distance;
    
    return from_pos + direction * scale;
}

std::vector<std::shared_ptr<RRTNode>> RRTPathPlanner::find_near_nodes(
    const Position& center_pos, double radius) {
    
    std::vector<std::shared_ptr<RRTNode>> near_nodes;
    
    for (const auto& node : tree_) {
        if (node->position.distance_to(center_pos) <= radius) {
            near_nodes.push_back(node);
        }
    }
    
    return near_nodes;
}

void RRTPathPlanner::rewire_tree(std::shared_ptr<RRTNode> new_node,
                                 const std::vector<std::shared_ptr<RRTNode>>& near_nodes,
                                 const std::vector<std::vector<int>>& inflated_grid) {
    
    for (auto& near_node : near_nodes) {
        // 新しいノード経由の方がコストが低いかチェック
        double new_cost = new_node->cost + new_node->position.distance_to(near_node->position);
        
        if (new_cost < near_node->cost && 
            is_valid_line(new_node->position, near_node->position, inflated_grid)) {
            
            // 親ノードを変更
            near_node->parent = new_node;
            near_node->cost = new_cost;
            
            // 子ノードのコストも再計算（再帰的に）
            // 簡略化のため、この実装では深い再帰は避ける
        }
    }
}

Path RRTPathPlanner::reconstruct_path(std::shared_ptr<RRTNode> goal_node) {
    Path path;
    
    std::vector<std::shared_ptr<RRTNode>> path_nodes;
    std::shared_ptr<RRTNode> current = goal_node;
    
    // ゴールからスタートまで逆順にたどる
    while (current) {
        path_nodes.push_back(current);
        current = current->parent;
    }
    
    // 逆順にして正しい順序にする
    std::reverse(path_nodes.begin(), path_nodes.end());
    
    // パスポイントを追加
    double total_cost = 0.0;
    for (size_t i = 0; i < path_nodes.size(); ++i) {
        double segment_cost = 0.0;
        if (i > 0) {
            segment_cost = path_nodes[i-1]->position.distance_to(path_nodes[i]->position);
            total_cost += segment_cost;
        }
        path.add_point(path_nodes[i]->position, segment_cost);
    }
    
    path.total_cost = total_cost;
    return path;
}

Path RRTPathPlanner::smooth_path(const Path& path, 
                                 const std::vector<std::vector<int>>& inflated_grid) {
    if (path.points.size() <= 2) {
        return path;
    }
    
    Path smoothed_path;
    smoothed_path.add_point(path.points[0].position, 0.0);
    
    size_t current_idx = 0;
    while (current_idx < path.points.size() - 1) {
        size_t furthest_idx = current_idx + 1;
        
        // 現在の点からできるだけ遠い点まで直線でつなげるかチェック
        for (size_t i = current_idx + 2; i < path.points.size(); ++i) {
            if (is_valid_line(path.points[current_idx].position, 
                              path.points[i].position, inflated_grid)) {
                furthest_idx = i;
            } else {
                break;
            }
        }
        
        // 次のポイントを追加
        double segment_cost = path.points[current_idx].position.distance_to(
            path.points[furthest_idx].position);
        smoothed_path.add_point(path.points[furthest_idx].position, segment_cost);
        
        current_idx = furthest_idx;
    }
    
    return smoothed_path;
}

Path RRTPathPlanner::adjust_path_spacing(const Path& path) {
    if (path.points.size() <= 2) {
        return path;
    }
    
    Path adjusted_path;
    adjusted_path.add_point(path.points[0].position, 0.0);
    
    double accumulated_distance = 0.0;
    size_t last_added_idx = 0;
    
    for (size_t i = 1; i < path.points.size(); ++i) {
        double segment_distance = path.points[i-1].position.distance_to(path.points[i].position);
        accumulated_distance += segment_distance;
        
        // 最小間隔に達したらポイントを追加
        if (accumulated_distance >= config_.path_point_min_distance || i == path.points.size() - 1) {
            adjusted_path.add_point(path.points[i].position, accumulated_distance);
            accumulated_distance = 0.0;
            last_added_idx = i;
        }
    }
    
    // 最後の点が追加されていない場合は追加
    if (last_added_idx != path.points.size() - 1) {
        double final_distance = path.points[last_added_idx].position.distance_to(
            path.points.back().position);
        adjusted_path.add_point(path.points.back().position, final_distance);
    }
    
    return adjusted_path;
}

Path RRTPathPlanner::iterative_smooth_path(const Path& path,
                                           const std::vector<std::vector<int>>& inflated_grid) {
    if (path.points.size() <= 2) {
        return path;
    }
    
    Path smoothed_path = path;
    
    for (int iteration = 0; iteration < config_.smoothing_iterations; ++iteration) {
        Path temp_path;
        temp_path.add_point(smoothed_path.points[0].position, 0.0);
        
        // 中間点を平滑化
        for (size_t i = 1; i < smoothed_path.points.size() - 1; ++i) {
            const Position& prev = smoothed_path.points[i-1].position;
            const Position& curr = smoothed_path.points[i].position;
            const Position& next = smoothed_path.points[i+1].position;
            
            // 平滑化された位置を計算
            Position smoothed_pos(
                curr.x + config_.smoothing_weight * ((prev.x + next.x) / 2.0 - curr.x),
                curr.y + config_.smoothing_weight * ((prev.y + next.y) / 2.0 - curr.y)
            );
            
            // 平滑化後の位置が有効かチェック
            if (is_valid_position(smoothed_pos, inflated_grid) &&
                is_valid_line(prev, smoothed_pos, inflated_grid) &&
                is_valid_line(smoothed_pos, next, inflated_grid)) {
                double segment_cost = prev.distance_to(smoothed_pos);
                temp_path.add_point(smoothed_pos, segment_cost);
            } else {
                // 平滑化が失敗した場合は元の点を使用
                double segment_cost = prev.distance_to(curr);
                temp_path.add_point(curr, segment_cost);
            }
        }
        
        // 最後の点を追加
        double final_cost = temp_path.points.back().position.distance_to(
            smoothed_path.points.back().position);
        temp_path.add_point(smoothed_path.points.back().position, final_cost);
        
        smoothed_path = temp_path;
    }
    
    return smoothed_path;
}

Path RRTPathPlanner::curvature_based_smoothing(const Path& path,
                                               const std::vector<std::vector<int>>& inflated_grid) {
    if (path.points.size() <= 3) {
        return path;
    }
    
    Path smoothed_path;
    smoothed_path.add_point(path.points[0].position, 0.0);
    
    for (size_t i = 1; i < path.points.size() - 1; ++i) {
        double curvature = calculate_curvature(path, i);
        
        // 曲率が閾値以上の場合は平滑化を適用
        if (curvature > config_.curvature_threshold) {
            const Position& prev = path.points[i-1].position;
            const Position& curr = path.points[i].position;
            const Position& next = path.points[i+1].position;
            
            // 曲率に応じた平滑化強度を計算
            double smoothing_factor = std::min(1.0, curvature / config_.curvature_threshold * config_.smoothing_weight);
            
            Position smoothed_pos(
                curr.x + smoothing_factor * ((prev.x + next.x) / 2.0 - curr.x),
                curr.y + smoothing_factor * ((prev.y + next.y) / 2.0 - curr.y)
            );
            
            // 平滑化後の位置が有効かチェック
            if (is_valid_position(smoothed_pos, inflated_grid) &&
                is_valid_line(smoothed_path.points.back().position, smoothed_pos, inflated_grid)) {
                double segment_cost = smoothed_path.points.back().position.distance_to(smoothed_pos);
                smoothed_path.add_point(smoothed_pos, segment_cost);
            } else {
                double segment_cost = smoothed_path.points.back().position.distance_to(curr);
                smoothed_path.add_point(curr, segment_cost);
            }
        } else {
            // 曲率が小さい場合はそのまま追加
            double segment_cost = smoothed_path.points.back().position.distance_to(path.points[i].position);
            smoothed_path.add_point(path.points[i].position, segment_cost);
        }
    }
    
    // 最後の点を追加
    double final_cost = smoothed_path.points.back().position.distance_to(
        path.points.back().position);
    smoothed_path.add_point(path.points.back().position, final_cost);
    
    return smoothed_path;
}

double RRTPathPlanner::calculate_curvature(const Path& path, size_t index) {
    if (index == 0 || index >= path.points.size() - 1 || path.points.size() < 3) {
        return 0.0;
    }
    
    const Position& p1 = path.points[index-1].position;
    const Position& p2 = path.points[index].position;
    const Position& p3 = path.points[index+1].position;
    
    // ベクトルを計算
    double v1x = p2.x - p1.x;
    double v1y = p2.y - p1.y;
    double v2x = p3.x - p2.x;
    double v2y = p3.y - p2.y;
    
    // ベクトルの長さ
    double len1 = std::sqrt(v1x * v1x + v1y * v1y);
    double len2 = std::sqrt(v2x * v2x + v2y * v2y);
    
    if (len1 < 1e-6 || len2 < 1e-6) {
        return 0.0;
    }
    
    // 正規化
    v1x /= len1;
    v1y /= len1;
    v2x /= len2;
    v2y /= len2;
    
    // 角度変化を計算（外積の大きさ）
    double cross_product = std::abs(v1x * v2y - v1y * v2x);
    
    // 曲率を計算（角度変化 / 平均セグメント長）
    double avg_length = (len1 + len2) / 2.0;
    return cross_product / avg_length;
}

} // namespace adaptive_rrt