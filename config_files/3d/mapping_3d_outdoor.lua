include "mapping_3d.lua"

-- 这组参数对性能要求没那么高，基本可以保证10cm的精度，性价比比较高
TRAJECTORY_BUILDER_3D.min_range = 1.2
TRAJECTORY_BUILDER_3D.max_range = 30.
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 75
    
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.12
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150

-- Real time correlative scan matcher
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
-- TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.1
-- TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad( 2.5 )

POSE_GRAPH.constraint_builder.min_score = 0.52
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1.5

POSE_GRAPH.constraint_builder.constraint_builder_self_adjust_options_3d.enabled = true
POSE_GRAPH.constraint_builder.constraint_builder_self_adjust_options_3d.min_score = 0.509
POSE_GRAPH.constraint_builder.constraint_builder_self_adjust_options_3d.max_score = 0.531
POSE_GRAPH.constraint_builder.constraint_builder_self_adjust_options_3d.step = 0.005
POSE_GRAPH.constraint_builder.constraint_builder_self_adjust_options_3d.too_few_percentage = 0.03
POSE_GRAPH.constraint_builder.constraint_builder_self_adjust_options_3d.too_many_percentage = 0.20
-- POSE_GRAPH.constraint_builder.log_matches = false
-- POSE_GRAPH.log_residual_histograms = false

return options
