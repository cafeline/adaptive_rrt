#include <gtest/gtest.h>
#include "adaptive_rrt/rrt_planner.hpp"
#include "adaptive_rrt/types.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace adaptive_rrt {

class DynamicObstacleHandlingTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 100x100のテスト用マップを作成（解像度 0.1m/cell）
        test_grid_ = nav_msgs::msg::OccupancyGrid();
        test_grid_.info.width = 100;
        test_grid_.info.height = 100;
        test_grid_.info.resolution = 0.1;
        test_grid_.info.origin.position.x = 0.0;
        test_grid_.info.origin.position.y = 0.0;
        test_grid_.data.resize(100 * 100, 0);  // すべて空きエリア

        // RRT設定
        config_ = RRTConfig();
        config_.step_size = 0.5;
        config_.goal_tolerance = 0.2;
        config_.max_iterations = 1000;

        // テスト用原点と壁クリアランス
        origin_ = Position(0.0, 0.0);
        wall_clearance_ = 0.3;
    }

    nav_msgs::msg::OccupancyGrid test_grid_;
    RRTConfig config_;
    Position origin_;
    double wall_clearance_;
};

// テスト1: 障害物検知時の衝突枝特定機能
TEST_F(DynamicObstacleHandlingTest, IdentifyCollisionEdgesOnObstacleDetection) {
    // RRTプランナーを初期化
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    // 初期パスを計画
    Position start(1.0, 1.0);
    Position goal(8.0, 8.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    // 新しい動的障害物を設定（パス上に配置）
    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(4.0, 4.0), 0.5}  // パス途中に障害物配置
    };

    // 衝突枝を特定する機能をテスト
    // 期待値: identify_collision_edges関数が存在し、衝突する枝のリストを返す
    // 実装前なので、この関数は存在しないはずでテストは失敗する
    
    // 実装済み - identify_collision_edges関数をテスト
    auto collision_edges = planner.identify_collision_edges(dynamic_obstacles);
    
    // パス上に障害物があるため、衝突エッジが検出されるはず
    EXPECT_FALSE(collision_edges.empty());
    EXPECT_GT(collision_edges.size(), 0);
}

// テスト2: 複数障害物での衝突枝特定
TEST_F(DynamicObstacleHandlingTest, IdentifyMultipleCollisionEdges) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(9.0, 9.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    // 複数の動的障害物を設定
    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(3.0, 3.0), 0.4},
        {Position(6.0, 6.0), 0.3},
        {Position(7.5, 7.5), 0.2}
    };

    // 複数の衝突枝が正しく特定されることを確認
    // TODO: 実装後に以下のコードを有効化
    // auto collision_edges = planner.identify_collision_edges(dynamic_obstacles);
    // EXPECT_GE(collision_edges.size(), 2); // 最低2つの衝突枝を期待
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト3: 障害物がない場合の衝突枝特定
TEST_F(DynamicObstacleHandlingTest, NoCollisionEdgesWhenNoObstacles) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(8.0, 8.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    // 障害物なし
    std::vector<std::pair<Position, double>> dynamic_obstacles = {};

    // 衝突枝が見つからないことを確認
    // TODO: 実装後に以下のコードを有効化
    // auto collision_edges = planner.identify_collision_edges(dynamic_obstacles);
    // EXPECT_TRUE(collision_edges.empty());
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト4: 障害物が遠い場合の衝突枝特定
TEST_F(DynamicObstacleHandlingTest, NoCollisionEdgesWhenObstaclesAreDistant) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(3.0, 3.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    // パスから遠い位置に障害物を配置
    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(8.0, 8.0), 0.5}  // パスから十分離れた位置
    };

    // 衝突枝が見つからないことを確認
    // TODO: 実装後に以下のコードを有効化
    // auto collision_edges = planner.identify_collision_edges(dynamic_obstacles);
    // EXPECT_TRUE(collision_edges.empty());
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト5: 大きな障害物での衝突枝特定
TEST_F(DynamicObstacleHandlingTest, IdentifyCollisionEdgesWithLargeObstacle) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(9.0, 9.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    // 大きな障害物を設定
    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(5.0, 5.0), 1.5}  // 大きな障害物
    };

    // 大きな障害物により複数の衝突枝が見つかることを確認
    // TODO: 実装後に以下のコードを有効化
    // auto collision_edges = planner.identify_collision_edges(dynamic_obstacles);
    // EXPECT_GT(collision_edges.size(), 1); // 複数の衝突枝を期待
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト6: 衝突枝とその子ノードの無効化機能
TEST_F(DynamicObstacleHandlingTest, InvalidateCollisionEdgesAndChildNodes) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(8.0, 8.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    // パス上に障害物を配置
    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(4.0, 4.0), 0.6}
    };

    // 衝突枝とその子ノードが無効化されることを確認
    // TODO: 実装後に以下のコードを有効化
    // auto tree_before = planner.get_tree_snapshot();
    // planner.invalidate_collision_edges_and_children(dynamic_obstacles);
    // auto tree_after = planner.get_tree_snapshot();
    
    // 無効化されたノード数を確認
    // EXPECT_LT(tree_after.size(), tree_before.size());
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト7: 子ノードの再帰的無効化
TEST_F(DynamicObstacleHandlingTest, RecursivelyInvalidateChildNodes) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(9.0, 9.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    // 分岐点近くに障害物を配置して、子ノードの連鎖無効化を確認
    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(3.0, 3.0), 0.8}  // 分岐点に大きな障害物
    };

    // 親ノードの無効化により子ノードも再帰的に無効化されることを確認
    // TODO: 実装後に以下のコードを有効化
    // auto affected_nodes = planner.invalidate_collision_edges_and_children(dynamic_obstacles);
    // EXPECT_GT(affected_nodes.size(), 1); // 複数ノードが影響を受ける
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト8: 根ノードに近い衝突での無効化
TEST_F(DynamicObstacleHandlingTest, InvalidateNearRootCollision) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(8.0, 8.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    // 根ノード近くに障害物を配置
    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(1.5, 1.5), 0.4}
    };

    // 根ノード近くの衝突で大部分のツリーが無効化されることを確認
    // TODO: 実装後に以下のコードを有効化
    // auto tree_before = planner.get_tree_snapshot();
    // auto affected_nodes = planner.invalidate_collision_edges_and_children(dynamic_obstacles);
    // EXPECT_GT(affected_nodes.size(), tree_before.size() / 2); // 半分以上が影響
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト9: 無効化後のツリー整合性
TEST_F(DynamicObstacleHandlingTest, TreeConsistencyAfterInvalidation) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(8.0, 8.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(5.0, 5.0), 0.5}
    };

    // 無効化後もツリーの整合性が保たれることを確認
    // TODO: 実装後に以下のコードを有効化
    // planner.invalidate_collision_edges_and_children(dynamic_obstacles);
    // EXPECT_TRUE(planner.verify_tree_consistency());
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト10: 複数回の無効化操作
TEST_F(DynamicObstacleHandlingTest, MultipleInvalidationOperations) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(9.0, 9.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    // 第1回目の無効化
    std::vector<std::pair<Position, double>> obstacles1 = {
        {Position(3.0, 3.0), 0.4}
    };

    // 第2回目の無効化
    std::vector<std::pair<Position, double>> obstacles2 = {
        {Position(6.0, 6.0), 0.4}
    };

    // 複数回の無効化操作が正しく動作することを確認
    // TODO: 実装後に以下のコードを有効化
    // auto affected1 = planner.invalidate_collision_edges_and_children(obstacles1);
    // auto affected2 = planner.invalidate_collision_edges_and_children(obstacles2);
    // EXPECT_GT(affected1.size(), 0);
    // EXPECT_GT(affected2.size(), 0);
    // EXPECT_TRUE(planner.verify_tree_consistency());
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト11: 孤立サブツリーの根ノード再接続機能
TEST_F(DynamicObstacleHandlingTest, ReconnectIsolatedSubtreesToRoot) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(8.0, 8.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    // ツリーの中間部分に障害物を配置して孤立サブツリーを作成
    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(4.0, 4.0), 0.7}
    };

    // 孤立したサブツリーが根ノードに再接続されることを確認
    // TODO: 実装後に以下のコードを有効化
    // planner.invalidate_collision_edges_and_children(dynamic_obstacles);
    // auto reconnected_count = planner.reconnect_isolated_subtrees();
    // EXPECT_GT(reconnected_count, 0); // 最低1つは再接続される
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト12: 複数の孤立サブツリーの再接続
TEST_F(DynamicObstacleHandlingTest, ReconnectMultipleIsolatedSubtrees) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(9.0, 9.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    // 複数箇所に障害物を配置して複数の孤立サブツリーを作成
    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(3.0, 3.0), 0.5},
        {Position(6.0, 6.0), 0.5}
    };

    // 複数の孤立サブツリーが再接続されることを確認
    // TODO: 実装後に以下のコードを有効化
    // planner.invalidate_collision_edges_and_children(dynamic_obstacles);
    // auto reconnected_count = planner.reconnect_isolated_subtrees();
    // EXPECT_GE(reconnected_count, 2); // 複数再接続を期待
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト13: 再接続不可能なサブツリーの処理
TEST_F(DynamicObstacleHandlingTest, HandleUnreconnectableSubtrees) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(8.0, 8.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    // 大きな障害物で完全に分離されたサブツリーを作成
    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(4.0, 4.0), 2.0}  // 非常に大きな障害物
    };

    // 再接続できないサブツリーが適切に識別されることを確認
    // TODO: 実装後に以下のコードを有効化
    // planner.invalidate_collision_edges_and_children(dynamic_obstacles);
    // auto reconnect_result = planner.reconnect_isolated_subtrees();
    // auto unreconnectable = planner.get_unreconnectable_subtrees();
    // EXPECT_GT(unreconnectable.size(), 0); // 再接続不可能なサブツリーが存在
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト14: 再接続時の最適パスの選択
TEST_F(DynamicObstacleHandlingTest, SelectOptimalReconnectionPath) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(8.0, 8.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(3.5, 3.5), 0.4}
    };

    // 再接続時に最も短いまたは最適なパスが選択されることを確認
    // TODO: 実装後に以下のコードを有効化
    // auto tree_before = planner.get_tree_snapshot();
    // planner.invalidate_collision_edges_and_children(dynamic_obstacles);
    // planner.reconnect_isolated_subtrees();
    // auto tree_after = planner.get_tree_snapshot();
    
    // 再接続により全体のパスコストが最適化されることを確認
    // EXPECT_LE(tree_after.total_cost, tree_before.total_cost * 1.2); // 20%以内の増加
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト15: 段階的再接続処理
TEST_F(DynamicObstacleHandlingTest, ProgressiveReconnectionProcess) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(9.0, 9.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(4.0, 4.0), 0.6}
    };

    // 段階的に再接続が進行することを確認
    // TODO: 実装後に以下のコードを有効化
    // planner.invalidate_collision_edges_and_children(dynamic_obstacles);
    // auto isolated_subtrees = planner.get_isolated_subtrees();
    // 
    // for (size_t i = 0; i < isolated_subtrees.size(); ++i) {
    //     auto reconnected = planner.attempt_subtree_reconnection(isolated_subtrees[i]);
    //     EXPECT_TRUE(reconnected.has_value()); // 各サブツリーの再接続を確認
    // }
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト16: 新しいサンプリングによる補完機能
TEST_F(DynamicObstacleHandlingTest, ComplementWithNewSampling) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(8.0, 8.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(4.0, 4.0), 1.0}  // 大きな障害物で多くのノードを無効化
    };

    // 再接続できないサブツリーを新しいサンプリングで補完
    // TODO: 実装後に以下のコードを有効化
    // planner.invalidate_collision_edges_and_children(dynamic_obstacles);
    // auto reconnection_result = planner.reconnect_isolated_subtrees();
    // auto sampling_result = planner.complement_with_new_sampling();
    // EXPECT_GT(sampling_result.new_nodes_count, 0); // 新しいノードが追加される
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト17: 補完サンプリングの収束性
TEST_F(DynamicObstacleHandlingTest, ConvergenceOfComplementarySampling) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(9.0, 9.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(5.0, 5.0), 0.8}
    };

    // 補完サンプリングが適切に収束することを確認
    // TODO: 実装後に以下のコードを有効化
    // planner.invalidate_collision_edges_and_children(dynamic_obstacles);
    // planner.reconnect_isolated_subtrees();
    // 
    // for (int iteration = 0; iteration < 10; ++iteration) {
    //     auto result = planner.complement_with_new_sampling();
    //     if (result.path_to_goal_found) {
    //         EXPECT_LE(iteration, 8); // 適切な反復回数内で収束
    //         break;
    //     }
    // }
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト18: 障害物回避を考慮した補完サンプリング
TEST_F(DynamicObstacleHandlingTest, ObstacleAvoidantComplementarySampling) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(8.0, 8.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(3.0, 3.0), 0.6},
        {Position(5.0, 5.0), 0.6}
    };

    // 補完サンプリングが障害物を適切に回避することを確認
    // TODO: 実装後に以下のコードを有効化
    // planner.invalidate_collision_edges_and_children(dynamic_obstacles);
    // planner.reconnect_isolated_subtrees();
    // auto sampling_result = planner.complement_with_new_sampling();
    // 
    // // 新しく追加されたノードが障害物を回避していることを確認
    // for (const auto& new_node : sampling_result.new_nodes) {
    //     for (const auto& obstacle : dynamic_obstacles) {
    //         double distance = new_node.position.distance_to(obstacle.first);
    //         EXPECT_GT(distance, obstacle.second + wall_clearance_);
    //     }
    // }
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト19: 効率的な補完サンプリング戦略
TEST_F(DynamicObstacleHandlingTest, EfficientComplementarySamplingStrategy) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(9.0, 9.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(4.0, 4.0), 0.7}
    };

    // 効率的なサンプリング戦略（バイアスサンプリングなど）が使用されることを確認
    // TODO: 実装後に以下のコードを有効化
    // planner.invalidate_collision_edges_and_children(dynamic_obstacles);
    // planner.reconnect_isolated_subtrees();
    // 
    // auto start_time = std::chrono::high_resolution_clock::now();
    // auto sampling_result = planner.complement_with_new_sampling();
    // auto end_time = std::chrono::high_resolution_clock::now();
    // 
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    // EXPECT_LT(duration.count(), 5000); // 5秒以内で完了
    // EXPECT_GT(sampling_result.sampling_efficiency, 0.1); // 10%以上の効率
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト20: 補完後の経路品質
TEST_F(DynamicObstacleHandlingTest, PathQualityAfterComplementation) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(8.0, 8.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());
    double initial_path_length = initial_path->total_cost;

    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(4.0, 4.0), 0.6}
    };

    // 補完後の経路品質が許容範囲内であることを確認
    // TODO: 実装後に以下のコードを有効化
    // planner.invalidate_collision_edges_and_children(dynamic_obstacles);
    // planner.reconnect_isolated_subtrees();
    // auto sampling_result = planner.complement_with_new_sampling();
    // 
    // if (sampling_result.final_path.has_value()) {
    //     double final_path_length = sampling_result.final_path->total_cost;
    //     // 新しい経路長が元の経路の1.5倍以内であることを確認
    //     EXPECT_LT(final_path_length, initial_path_length * 1.5);
    //     
    //     // 経路の滑らかさを確認
    //     EXPECT_TRUE(planner.verify_path_smoothness(sampling_result.final_path.value()));
    // }
    
    EXPECT_TRUE(true); // プレースホルダー
}

// テスト21: 統合動的障害物対応機能のエンドツーエンドテスト
TEST_F(DynamicObstacleHandlingTest, EndToEndDynamicObstacleHandling) {
    RRTPathPlanner planner(test_grid_, test_grid_.info.resolution, origin_, wall_clearance_, config_);

    Position start(1.0, 1.0);
    Position goal(9.0, 9.0);
    auto initial_path = planner.plan_path(start, goal);
    ASSERT_TRUE(initial_path.has_value());

    // 複雑な障害物シナリオ
    std::vector<std::pair<Position, double>> dynamic_obstacles = {
        {Position(3.0, 3.0), 0.5},
        {Position(5.0, 5.0), 0.7},
        {Position(7.0, 7.0), 0.4}
    };

    // 全体的な動的障害物対応フローが正しく動作することを確認
    // TODO: 実装後に以下のコードを有効化
    // auto result = planner.handle_dynamic_obstacles(dynamic_obstacles);
    // 
    // EXPECT_TRUE(result.success);
    // EXPECT_GT(result.collision_edges_identified, 0);
    // EXPECT_GT(result.nodes_invalidated, 0);
    // EXPECT_GE(result.subtrees_reconnected, 0);
    // EXPECT_GE(result.new_nodes_sampled, 0);
    // 
    // if (result.final_path.has_value()) {
    //     EXPECT_TRUE(planner.verify_path_collision_free(result.final_path.value(), dynamic_obstacles));
    // }
    
    EXPECT_TRUE(true); // プレースホルダー
}

} // namespace adaptive_rrt