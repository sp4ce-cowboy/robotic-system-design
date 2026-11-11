# Robotic System Design: architecture

> [!NOTE]
> This architecture write-up was added by
> [Claude Opus 5.5](https://docs.anthropic.com/en/docs/claude-code) in Nov 2026 using my
> [agentic-repo-analysis](https://github.com/sp4ce-cowboy/agentic-workflows) skill, from the
> project's code and the team's report.

Back to the [README](README.md).

## 1. Packages

| Package | Contents |
|---|---|
| `ee3305_bringup` | Launch files (project, SLAM, localisation), parameters (`project.yaml`, `nav2.yaml`), the saved map, an RViz config |
| `ee3305_nav` | The nodes: `map_server`, `estimator`, `behavior`, `planner`, `controller` |

## 2. System

The behavior node issues goals; the planner reads the occupancy grid and the pose, and publishes a
`nav_msgs/Path`; the controller follows it and publishes `geometry_msgs/Twist` on `cmd_vel` to the
robot in Gazebo. RViz shows the map, the path and the robot.

## 3. The controller (my part)

[`controller.cpp`](https://github.com/sp4ce-cowboy/robotic-system-design/blob/54daac50e1fe1d8af0c7f0010ea4dc71b6893996/src/ee3305_nav/src/controller.cpp):
1. **Validate** the path and pose first; skip the cycle if either is missing.
2. **Closest point, then lookahead point:** walk the path from the closest point until the distance
   reaches the lookahead distance; fall back to the last point.
3. **Into the robot's frame:** rotate the lookahead point by the robot's heading.
4. **Curvature** `2y / (x² + y²)`, and angular velocity = curvature × linear velocity.
5. **Limits:** clamp the change in linear and angular velocity by the maximum accelerations over the
   time step, then clamp to the maximum velocities.
6. **Stop** when within `stop_thres` of the goal, publishing zero velocity.

The lookahead distance trades smoothness (longer) against tight tracking (shorter), and was tuned
in simulation.

## 4. The planner

[`run`](https://github.com/sp4ce-cowboy/robotic-system-design/blob/54daac50e1fe1d8af0c7f0010ea4dc71b6893996/src/ee3305_nav/src/planner.cpp#L175-L260): nodes with g, h, f and a parent pointer; an open list ordered by f; octile
distance as h for 8-connected moves; a visited grid; neighbour checks against the grid bounds and
obstacles; early exit with "No path found"; the path rebuilt from the goal through parent pointers;
and the remaining nodes freed.

## 5. Limits and next steps

- The pure-pursuit lookahead is fixed; scaling it with speed or curvature would track tight turns
  better.
- A* gives jagged grid paths; smoothing them would suit the controller.
