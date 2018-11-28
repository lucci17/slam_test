XY_TRANSFORM =  {
  translation = { 0., 0., 0. },
  rotation = { 0., -math.pi / 2., 0., },
}

VOXEL_SIZE = 0.1

options = {
  tracking_frame = "base_link",
  pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 1.,
      max_range = 60.,
    },

    -- Gray X-Rays. These only use geometry to color pixels. 
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_xy_all",
      transform = XY_TRANSFORM,
      draw_trajectories = false,
    },
  }
}

return options
