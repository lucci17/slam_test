include "mapping_3d_outdoor.lua"
include "pure_localization_3d.lua"

POSE_GRAPH.constraint_builder.log_matches = false
POSE_GRAPH.log_residual_histograms = false

POSE_GRAPH.constraint_builder.min_score = 0.55
-- TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false

return options