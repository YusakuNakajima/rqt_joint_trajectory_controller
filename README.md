# rqt_joint_trajectory_controller (OnoLab fork)

Standalone copy of `rqt_joint_trajectory_controller` from
[ros-controls/ros2_controllers](https://github.com/ros-controls/ros2_controllers)
(`humble` branch, 2.54.1), extracted with its history by `git subtree split`.
Only this package is included, so building it never shadows the released
`joint_trajectory_controller` or other controllers.

- `upstream-humble`: unmodified upstream history
- `humble`: this fork's changes (use this branch)

## Changes from upstream

Upstream republishes a position-only, single-point trajectory every 100 ms.
joint_trajectory_controller interpolates position-only points linearly from its
current state, so the velocity jumps on every command and real robots
(e.g. UR with `scaled_joint_trajectory_controller`) move jerkily.

1. **Send only when the command changes.** Enabling control mode sends nothing
   until a joint value is edited, and an unchanged command is not re-sent.
2. **Zero target velocities in the command point.** The controller then uses a
   cubic spline from its current position *and velocity* to the target at rest,
   so a new command blends with the motion in progress with continuous velocity.
3. **Duration accounts for the cubic peak velocity** (1.5 x average), so that,
   starting from rest, no joint exceeds speed scaling x URDF velocity limit.
4. **Built with `ament_cmake`** instead of `ament_python` (`setup.py` removed).

The UI and every other behavior are unchanged. Disabling control mode still
lets the last command finish (it does not stop the robot).
