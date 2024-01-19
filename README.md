# ros2_navigation_utils

A collection of node useful for mobile robot's navigation

## Follow path action server

An action server to actuate a path, like a carrot-planner. Very simple. Ignore possible collisions.

It has few parameters:

- `tau`: the time in second to converge to the path [seconds]
- `horizon`: how much to look ahead of the current position [meters]
- `frame_id`: in which frame to perform the control

The action goal has the following fields:

- `path`: the path to follow. Should be in a TF-frame connected to `frame_id`
- `speed`: the desired linear speed at which to follow the path
- `angular_speed`: the desired angular_speed speed at which to follow the path. If set to zero, the controller ignores angular motions. 
- `spatial_goal_tolerance` and `angular_goal_tolerance`: how near to the goal to arrive
- `turn_ahead`: Whether to align with the path curve orientation instead of the orientation of the path poses.
- `orientation_interpolation`: which interpolation to apply to the path orientation, see `kind` in the [scipy.interpolate.interp1d](https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.interp1d.html) docs.

