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

include "mapping_3d.lua"

TRAJECTORY_BUILDER_3D.min_range = 1.0
TRAJECTORY_BUILDER_3D.max_range = 10.
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05
-- TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 10.
-- TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.15

-- 调整数据区
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 2
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 12
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4e2
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 2.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window  = 1.0
POSE_GRAPH.constraint_builder.max_constraint_distance = 3.
POSE_GRAPH.constraint_builder.min_score = 0.56

-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth = 6

POSE_GRAPH.constraint_builder.constraint_builder_self_adjust_options_3d.enabled = false
-- TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 1
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 2
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
		


return options