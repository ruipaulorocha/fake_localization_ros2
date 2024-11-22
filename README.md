# `fake_localization_ros2`
A ROS node that simply forwards odometry information.

This version was migrated by [Rui P. Rocha](mailto:rprocha@isr.uc.pt) to ROS2 Humble distro from the original `fake_localization` package developed by Ioan A. Sucan which is described in [ROS Wiki](http://wiki.ros.org/fake_localization) and whose source code is
available in this [GitHub repository](https://github.com/ros-planning/navigation/tree/noetic-devel/fake_localization) and in ROS Noetic distro (ROS1).

#### Description
The `fake_localization_ros2` package provides a single node, `fake_localization`, which substitutes for a localization system, providing a subset of the ROS API used by `amcl`.
This node is most frequently used during simulation as a method to provide localization based on perfect odometry in a computationally inexpensive manner.
Specifically, `fake_localization` converts odometry data into pose, particle cloud, and transform data of the form published by `amcl`.

The node `fake_localization` is also available as a composable node, `fake_localization_ros2::FakeOdomNode`, which can be launched dynamically inside a ROS container and use intra-process communication.


#### Nodes

`fake_localization` substitutes for a localization system, providing a subset of the ROS API used by `amcl`.

##### Subscribed Topics
- `odom` (`nav_msgs::msg::Odometry` message type)
    - The position of the robot as published by a simulator.
- `initialpose` (`geometry_msgs::msg::PoseWithCovarianceStamped` message type)
    - Allows for setting the pose of `fake_localization` using tools like `rviz2` to give a custom offset from the ground truth source being published.

##### Published Topics
- `amcl_pose` (geometry_msgs::msg::PoseWithCovarianceStamped` message type)
    - Just passes through the pose reported by a simulator.
- `particlecloud` (`geometry_msgs::msg::PoseArray` message type)
    - A particle cloud used to visualize the robot's pose in `rviz2`.


#### Parameters
- `odom_topic` (string, default: `"odom"`)
    - The name of the topic of the robot's odometry to be subscribed.
- `odom_frame_id` (string, default: `"odom"`)
    - The name of the odometric frame of the robot.
- `global_frame_id` (string, default: `map`)
    - The frame in which to publish the `global_frame_id→odom_frame_id` transform over `tf`. New in 1.1.3
- `base_frame_id` (string, default: `base_link`)
    - The base frame of the robot. New in 1.1.3
- `delta_x` (double, default: 0.0)
    - The x offset between the origin of the simulator coordinate frame and the map coordinate frame published by `fake_localization`.
- `delta_y` (double, default: 0.0)
    - The y offset between the origin of the simulator coordinate frame and the map coordinate frame published by `fake_localization`.
- `delta_yaw` (double, default: 0.0)
    - The yaw offset between the origin of the simulator coordinate frame and the map coordinate frame published by `fake_localization`.
- `transform_tolerance` (double, default: 0.1)
    - The default `tf` lag in seconds.


#### Provided tf Transforms
`<value of global_frame_id parameter> → <value of odom_frame_id parameter>` passed on from the simulator over `tf`.
