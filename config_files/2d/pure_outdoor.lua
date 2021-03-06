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

-- Set it 2d not 3d
-- this param was initialized to false
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- Global Parameters for Trajectory builder 2d
TRAJECTORY_BUILDER_2D.min_range = 1.
TRAJECTORY_BUILDER_2D.max_range = 20.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.18
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 2.

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres Scan Matcher
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight 	= 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight 	= 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight 		= 100.

-- Submaps
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80
POSE_GRAPH.optimize_every_n_nodes = 80
TRAJECTORY_BUILDER_2D.submaps.resolution = 0.05
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.miss_probability = 0.49

-- --Optimization
POSE_GRAPH.optimization_problem.huber_scale = 1e3
POSE_GRAPH.global_sampling_ratio = 0.001
POSE_GRAPH.global_constraint_search_after_n_seconds = 20.
POSE_GRAPH.constraint_builder.log_matches = false
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 4.;
-- POSE_GRAPH.optimization_problem.rotation_weight = 1e3
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.9

TRAJECTORY_BUILDER.pure_localization = true
POSE_GRAPH.optimize_every_n_nodes = 10

return options
