# MRS Octomap Planner

## General description
This package comprises the algorithms for planning of paths in 3D environments represented as an [octomap](https://octomap.github.io/). The path planning is followed by generation of a trajectory along the planned path, which is then sent as a request for trajectory tracking to the [MRS trajectory tracker](https://github.com/ctu-mrs/mrs_uav_trackers). The node considers the current state of the UAV during the trajectory generation process and correctly appends the trajectory following the new path to the current prediction horizon without violating dynamic constraints. The **Octomap Planner** implements also periodic replanning of the path, checks that the current trajectory is not in collision with the most recent map of the environment and prevents potential collisions by cutting off the coliding segment of the trajectory.

After the planning request to **user-specified goal** is obtained, the **Octomap Planner** proceeds as follows: 

* gets an **initial condition** for planning from the current MPC prediction (x seconds in the future),
* starts the planning algorithm to get a path from **initial condition** to the **user-specified goal**,
* cuts the generated path to the required length to prevent generating unnecesarry long trajectory,
* sends the trajectory generation request to [MRS UAV trajectory generator](https://github.com/ctu-mrs/mrs_uav_trajectory_generation),
* publishes the obtained trajectory reference to [MRS trajectory tracker](https://github.com/ctu-mrs/mrs_uav_trackers), 
* repeats the above described steps until the **user-specified goal** is reached while preventing potential collisions with the environment.

## Configuration and use



### ROS interface

Input: The **user-specified goal** can be passed to **Octomap Planner** as a service of type [mrs_msgs/Vec4](https://ctu-mrs.github.io/mrs_msgs/srv/Vec4.html) or [mrs_msgs/ReferenceStampedSrv](https://ctu-mrs.github.io/mrs_msgs/srv/ReferenceStampedSrv.html) on topics:
```
/uav*/octomap_planner/goto
/uav*/octomap_planner/reference
```

The planning can be stopped by calling service 
```
/uav*/octomap_planner/stop
```
which stops the planning process and sets the UAV to hovering state. 

Output: after each planning phase the node calls [/uav*/control_manager/trajectory_reference](https://ctu-mrs.github.io/mrs_msgs/srv/TrajectoryReferenceSrv.html) service provided by the [ControlManager](https://github.com/ctu-mrs/mrs_uav_managers#controlmanager). 

### Planner types
The **Octomap Planner** enables to switch between **MRS A* planner** implementation and **MRS SubT planner** using the service of type _std/String_.
```
/uav*/octomap_planner/set_planner subt/mrs
```
Both implementations in default setting provide the optimal path given the current map represented by an [octomap](https://octomap.github.io/). The [**MRS SubT Planner**](https://github.com/ctu-mrs/mrs_subt_planning_lib) provides functionality to push the path further from the obstacles above the specified safety distance and lowers the computational demands in the majority of path planning instances. Further, it can be parametrized to allow generation of paths up to x-times longer than optimal path, which significantly speeds up the planning process. The paths produced by **MRS A* Planner** are generally more straight, which makes the planner favoured for fast flight in open environments with low obstacle density.

#### Minimal Planner
TBW

### Planning for sensors with limited horizontal field of view

The **Octomap planner** allows to sample the heading reference in a direction of flight and also with a constant _heading_offset_ from a direction of flight. The use of this feature is recommended for UAVs that build occupancy maps using sensors with a limited horizontal field of view (e.g., depth cameras) and either fly in potentially dynamic environments or apply localization methods that experience drift.

```yaml
trajectory_generator:
  turn_in_flight_direction: true
  heading_offset: 0.0 # [rad]
```

## Dependencies

* [mrs_lib](https://github.com/ctu-mrs/mrs_lib)
* [mrs_msgs](https://github.com/ctu-mrs/mrs_msgs)
* [mrs_modules_msgs](https://github.com/ctu-mrs/mrs_modules_msgs)
* [mrs_subt_planning_lib](https://github.com/ctu-mrs/mrs_subt_planning_lib)
* [octomap_ros](https://github.com/OctoMap/octomap_ros)
