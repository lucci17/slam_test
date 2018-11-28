-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
}

-- Trajectory Builder 3D
MAP_BUILDER.use_trajectory_builder_3d = true
TRAJECTORY_BUILDER_3D.min_range = 1.
TRAJECTORY_BUILDER_3D.max_range = 30.
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 50
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = TRAJECTORY_BUILDER_3D.max_range
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true

-- Optimization
POSE_GRAPH.optimize_every_n_nodes = TRAJECTORY_BUILDER_3D.submaps.num_range_data
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.constraint_builder.min_score = 0.53
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth = 12
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.rotation_weight = 10.
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 20
POSE_GRAPH.constraint_builder.max_constraint_distance = TRAJECTORY_BUILDER_3D.max_range * 0.75
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 
	POSE_GRAPH.constraint_builder.max_constraint_distance
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1.5
POSE_GRAPH.log_residual_histograms = false

-- 暂时影响不大的参数，可以保留，对性能可能有极小的提升
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 9e3
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 3e5
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 2
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 25
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 5e2


return options