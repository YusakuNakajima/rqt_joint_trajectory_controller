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

1. **Send only when the command or the speed scaling changes.** Enabling
   control mode sends nothing until a joint value is edited, and an unchanged
   command is not re-sent. Changing the speed scaling re-sends the current
   command (only if one was already sent in this control session), so the
   motion in progress adopts the new speed, as with upstream.
2. **Zero target velocities in the command point.** The controller then uses a
   cubic spline from its current position *and velocity* to the target at rest,
   so a new command blends with the motion in progress with continuous velocity.
3. **Velocity-limited duration that accounts for the motion in progress.** The
   cubic's peak velocity is computed from each joint's starting velocity (both
   the measured and the reference velocity in `controller_state`), and the
   duration is lengthened until no joint exceeds speed scaling x URDF velocity
   limit (or the velocity it already has, which cannot change instantly).
4. **Fail closed.** Enabling control mode only records the current state as the
   baseline (untouched joints are frozen there), even if the controller state
   arrives after enabling. Commands are held back while the controller state
   is older than 0.5 s and sent once a fresh state arrives. A joint without a finite, positive URDF velocity
   limit makes the command not to be sent.
5. **Built with `ament_cmake`** instead of `ament_python` (`setup.py` removed).

Not limited: joint acceleration and jerk (no validated limits are available).
A cubic command keeps them finite, whereas upstream's position-only command
changes the velocity instantaneously, but a short duration can still mean a
large acceleration. Keep the speed scaling low on real robots.

The UI and every other behavior are unchanged. Disabling control mode still
lets the last command finish (it does not stop the robot).
