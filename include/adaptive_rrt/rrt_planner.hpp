#ifndef ADAPTIVE_RRT_RRT_PLANNER_HPP_
#define ADAPTIVE_RRT_RRT_PLANNER_HPP_

#include "adaptive_rrt/types.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <optional>
#include <random>
#include <memory>

namespace adaptive_rrt {

/**
 * @brief RRTアルゴリズムによる経路計画クラス
 */
class RRTPathPlanner {
public:
    /**
     * @brief コンストラクタ
     * @param grid 占有格子地図
     * @param resolution グリッドの解像度 [m/cell]
     * @param origin グリッドの原点座標
     * @param wall_clearance_distance 壁からの最小距離 [m]
     * @param config RRT設定
     */
    RRTPathPlanner(const nav_msgs::msg::OccupancyGrid& grid,
                   double resolution, const Position& origin,
                   double wall_clearance_distance,
                   const RRTConfig& config = RRTConfig());

    /**
     * @brief RRTアルゴリズムによる経路計画
     * @param start_pos 開始位置（世界座標）
     * @param goal_pos 目標位置（世界座標）
     * @return 計画された経路（失敗時はstd::nullopt）
     */
    std::optional<Path> plan_path(const Position& start_pos, const Position& goal_pos);

    /**
     * @brief 動的障害物を考慮したRRT経路計画
     * @param start_pos 開始位置（世界座標）
     * @param goal_pos 目標位置（世界座標）
     * @param dynamic_obstacles 動的障害物情報
     * @param static_obstacles 静的障害物情報
     * @return 計画された経路（失敗時はstd::nullopt）
     */
    std::optional<Path> plan_path_with_dynamic_obstacles(
        const Position& start_pos, 
        const Position& goal_pos,
        const std::vector<std::pair<Position, double>>& dynamic_obstacles,
        const std::vector<std::pair<Position, double>>& static_obstacles = {});

    /**
     * @brief 世界座標をグリッド座標に変換
     * @param world_pos 世界座標
     * @return グリッド座標
     */
    std::pair<int, int> world_to_grid(const Position& world_pos) const;

    /**
     * @brief グリッド座標を世界座標に変換
     * @param grid_pos グリッド座標
     * @return 世界座標
     */
    Position grid_to_world(const std::pair<int, int>& grid_pos) const;

    /**
     * @brief 位置が有効かチェック（膨張マップ使用）
     * @param pos 世界座標
     * @return 有効性
     */
    bool is_valid_position(const Position& pos) const;

    /**
     * @brief 位置が有効かチェック（指定膨張マップ使用）
     * @param pos 世界座標
     * @param inflated_grid 使用する膨張マップ
     * @return 有効性
     */
    bool is_valid_position(const Position& pos, 
                          const std::vector<std::vector<int>>& inflated_grid) const;

    /**
     * @brief 線分が有効かチェック（障害物との衝突なし）
     * @param start 開始位置
     * @param end 終了位置
     * @return 有効性
     */
    bool is_valid_line(const Position& start, const Position& end) const;

    /**
     * @brief 線分が有効かチェック（指定膨張マップ使用）
     * @param start 開始位置
     * @param end 終了位置
     * @param inflated_grid 使用する膨張マップ
     * @return 有効性
     */
    bool is_valid_line(const Position& start, const Position& end,
                       const std::vector<std::vector<int>>& inflated_grid) const;

    /**
     * @brief 膨張マップを取得
     * @return 膨張マップの参照
     */
    const std::vector<std::vector<int>>& get_inflated_grid() const;

    /**
     * @brief 最新の統計情報を取得
     * @return RRT統計情報
     */
    const RRTStatistics& get_statistics() const;

    /**
     * @brief 設定を更新
     * @param new_config 新しい設定
     */
    void update_config(const RRTConfig& new_config);

    // 動的障害物対応機能
    
    /**
     * @brief 障害物と衝突するエッジを特定
     * @param dynamic_obstacles 動的障害物リスト
     * @return 衝突エッジのリスト
     */
    std::vector<CollisionEdge> identify_collision_edges(
        const std::vector<std::pair<Position, double>>& dynamic_obstacles);

    /**
     * @brief 衝突エッジとその子ノードを無効化
     * @param dynamic_obstacles 動的障害物リスト
     * @return 無効化されたノードのリスト
     */
    std::vector<std::shared_ptr<RRTNode>> invalidate_collision_edges_and_children(
        const std::vector<std::pair<Position, double>>& dynamic_obstacles);

    /**
     * @brief 孤立したサブツリーを根ノードに再接続
     * @return 再接続されたサブツリーの数
     */
    int reconnect_isolated_subtrees();

    /**
     * @brief 新しいサンプリングによる補完
     * @return サンプリング補完結果
     */
    SamplingComplementResult complement_with_new_sampling();

    /**
     * @brief 動的障害物対応の統合処理
     * @param dynamic_obstacles 動的障害物リスト
     * @return 処理結果
     */
    DynamicObstacleHandlingResult handle_dynamic_obstacles(
        const std::vector<std::pair<Position, double>>& dynamic_obstacles);

    /**
     * @brief ツリーのスナップショットを取得（テスト用）
     * @return 現在のツリー状態
     */
    std::vector<std::shared_ptr<RRTNode>> get_tree_snapshot() const;

    /**
     * @brief ツリーの整合性を検証（テスト用）
     * @return 整合性が保たれているか
     */
    bool verify_tree_consistency() const;

    /**
     * @brief 経路が障害物と衝突していないか検証（テスト用）
     * @param path 検証する経路
     * @param obstacles 障害物リスト
     * @return 衝突がないか
     */
    bool verify_path_collision_free(const Path& path,
        const std::vector<std::pair<Position, double>>& obstacles) const;

    /**
     * @brief 経路の滑らかさを検証（テスト用）
     * @param path 検証する経路
     * @return 滑らかさが許容範囲内か
     */
    bool verify_path_smoothness(const Path& path) const;

private:
    // グリッドデータ
    int width_;
    int height_;
    double resolution_;
    Position origin_;
    double wall_clearance_distance_;
    RRTConfig config_;
    
    // 占有格子データ
    std::vector<std::vector<int>> grid_;
    std::vector<std::vector<int>> inflated_grid_;
    
    // 占有閾値
    static constexpr int OCCUPIED_THRESHOLD = 65;
    static constexpr int FREE_THRESHOLD = 25;
    
    // RRT関連
    std::vector<std::shared_ptr<RRTNode>> tree_;
    std::mt19937 random_generator_;
    std::uniform_real_distribution<double> uniform_dist_;
    RRTStatistics statistics_;

    /**
     * @brief 壁からの距離を考慮した膨張マップを作成
     */
    void create_inflated_grid();

    /**
     * @brief 動的障害物を考慮した一時的な占有グリッドを作成
     * @param dynamic_obstacles 動的障害物情報（位置と半径のペア）
     * @param static_obstacles 静的障害物情報（位置と半径のペア）
     * @return 動的障害物を反映した占有グリッド
     */
    std::vector<std::vector<int>> create_temporary_grid_with_obstacles(
        const std::vector<std::pair<Position, double>>& dynamic_obstacles,
        const std::vector<std::pair<Position, double>>& static_obstacles = {});

    /**
     * @brief RRTアルゴリズムの実装
     * @param start_pos 開始位置
     * @param goal_pos 目標位置
     * @param inflated_grid 使用する膨張マップ
     * @return 計画された経路（失敗時はstd::nullopt）
     */
    std::optional<Path> run_rrt(const Position& start_pos,
                               const Position& goal_pos,
                               const std::vector<std::vector<int>>& inflated_grid);

    /**
     * @brief ランダムな位置を生成
     * @return ランダム位置
     */
    Position generate_random_position();

    /**
     * @brief ツリー内で最も近いノードを探す
     * @param target_pos 目標位置
     * @return 最も近いノード
     */
    std::shared_ptr<RRTNode> find_nearest_node(const Position& target_pos);

    /**
     * @brief 指定方向にステップサイズ分移動した新しい位置を計算
     * @param from_pos 移動元位置
     * @param to_pos 移動先方向
     * @return 新しい位置
     */
    Position steer(const Position& from_pos, const Position& to_pos);

    /**
     * @brief RRT*での近傍ノードを探す
     * @param center_pos 中心位置
     * @param radius 探索半径
     * @return 近傍ノードのリスト
     */
    std::vector<std::shared_ptr<RRTNode>> find_near_nodes(
        const Position& center_pos, double radius);

    /**
     * @brief ノードの再配線（RRT*）
     * @param new_node 新しいノード
     * @param near_nodes 近傍ノード
     * @param inflated_grid 使用する膨張マップ
     */
    void rewire_tree(std::shared_ptr<RRTNode> new_node,
                     const std::vector<std::shared_ptr<RRTNode>>& near_nodes,
                     const std::vector<std::vector<int>>& inflated_grid);

    /**
     * @brief ゴールまでの経路を再構築
     * @param goal_node ゴールノード
     * @return 再構築された経路
     */
    Path reconstruct_path(std::shared_ptr<RRTNode> goal_node);

    /**
     * @brief 経路をスムージング
     * @param path 元の経路
     * @param inflated_grid 使用する膨張マップ
     * @return スムージングされた経路
     */
    Path smooth_path(const Path& path, 
                     const std::vector<std::vector<int>>& inflated_grid);

    /**
     * @brief 制御点の間隔を調整
     * @param path 元の経路
     * @return 間隔調整された経路
     */
    Path adjust_path_spacing(const Path& path);

    /**
     * @brief 反復平滑化による経路最適化
     * @param path 元の経路
     * @param inflated_grid 使用する膨張マップ
     * @return 平滑化された経路
     */
    Path iterative_smooth_path(const Path& path,
                               const std::vector<std::vector<int>>& inflated_grid);

    /**
     * @brief 曲率に基づく平滑化
     * @param path 元の経路
     * @param inflated_grid 使用する膨張マップ
     * @return 平滑化された経路
     */
    Path curvature_based_smoothing(const Path& path,
                                   const std::vector<std::vector<int>>& inflated_grid);

    /**
     * @brief 経路点の曲率を計算
     * @param path 経路
     * @param index 対象点のインデックス
     * @return 曲率
     */
    double calculate_curvature(const Path& path, size_t index) const;

    // 動的障害物対応のプライベート関数
    
    /**
     * @brief エッジが障害物と衝突するかチェック
     * @param parent 親ノード
     * @param child 子ノード  
     * @param obstacle 障害物（位置と半径）
     * @return 衝突情報（衝突する場合）
     */
    std::optional<CollisionEdge> check_edge_collision(
        std::shared_ptr<RRTNode> parent,
        std::shared_ptr<RRTNode> child,
        const std::pair<Position, double>& obstacle);

    /**
     * @brief ノードとその子ノードを再帰的に無効化
     * @param node 無効化するノード
     * @param invalidated_nodes 無効化されたノードのリスト
     */
    void invalidate_node_recursively(std::shared_ptr<RRTNode> node,
                                     std::vector<std::shared_ptr<RRTNode>>& invalidated_nodes);

    /**
     * @brief 孤立したサブツリーを特定
     * @return 孤立サブツリーのルートノードリスト
     */
    std::vector<std::shared_ptr<RRTNode>> find_isolated_subtrees();

    /**
     * @brief サブツリーの再接続を試行
     * @param subtree_root サブツリーのルートノード
     * @return 再接続に成功したか
     */
    bool attempt_subtree_reconnection(std::shared_ptr<RRTNode> subtree_root);

    /**
     * @brief ツリー構造を更新（親子関係の維持）
     */
    void update_tree_structure();
};

} // namespace adaptive_rrt

#endif // ADAPTIVE_RRT_RRT_PLANNER_HPP_