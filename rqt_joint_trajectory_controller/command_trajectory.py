# Copyright 2026 Apache License, Version 2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Build the command trajectory sent by the plugin (ROS/Qt-free so it can be unit tested).

The command is a single point with explicit zero velocities. With positions only,
joint_trajectory_controller interpolates linearly from its current state, so the
velocity jumps every time a new command arrives. With velocities set, it uses a
cubic spline from the current position *and velocity* to the target at rest, so
consecutive commands join with continuous velocity.
"""

import math

# Peak velocity of a rest-to-rest cubic segment is 1.5 * distance / duration.
CUBIC_PEAK_VELOCITY_FACTOR = 1.5
# Duration growth per step while searching for a duration that respects the limits.
_DURATION_GROWTH = 1.05
_MAX_DURATION_STEPS = 400


def cubic_peak_velocity_rad_s(distance_rad, start_velocity_rad_s, duration_s):
    """
    Return the largest |velocity| [rad/s] of the cubic used by joint_trajectory_controller.

    The cubic starts with start_velocity_rad_s and ends at rest, distance_rad away,
    after duration_s.
    """
    t = duration_s
    a = (start_velocity_rad_s * t - 2.0 * distance_rad) / t ** 3
    b = (3.0 * distance_rad - 2.0 * start_velocity_rad_s * t) / t ** 2
    peak = abs(start_velocity_rad_s)  # the velocity at t = duration_s is zero
    if a != 0.0:
        t_vertex = -b / (3.0 * a)
        if 0.0 < t_vertex < t:
            peak = max(peak, abs(3.0 * a * t_vertex ** 2 + 2.0 * b * t_vertex
                                 + start_velocity_rad_s))
    return peak


def command_duration_s(commands_rad, positions_rad, start_velocities_rad_s,
                       max_velocities_rad_s, min_duration_s, speed_scale):
    """
    Return the duration [s] of the command segment.

    Every joint's velocity stays within speed_scale x its maximum velocity, or within
    the velocity it already has if that is larger (it cannot change instantly). The
    duration is never shorter than min_duration_s / speed_scale.
    start_velocities_rad_s holds a list of candidates per joint (e.g. measured and
    reference velocity); all of them are respected.
    Acceleration is not limited (no validated limits are available), but unlike a
    position-only command it stays finite.
    Raises ValueError (send nothing) for a non-positive speed scale or a joint
    velocity limit that is not finite and positive.
    Units are rad and rad/s for revolute joints; for prismatic joints the same
    arguments are in m and m/s (the URDF limits of the joint itself).
    """
    if not speed_scale > 0.0:
        raise ValueError("speed_scale must be positive")
    joints = []
    dur_s = min_duration_s / speed_scale
    for cmd_rad, pos_rad, v0s_rad_s, max_vel_rad_s in zip(
        commands_rad, positions_rad, start_velocities_rad_s, max_velocities_rad_s
    ):
        if not (math.isfinite(max_vel_rad_s) and max_vel_rad_s > 0.0):
            raise ValueError(f"invalid joint velocity limit: {max_vel_rad_s}")
        distance_rad = cmd_rad - pos_rad
        limit_rad_s = max_vel_rad_s * speed_scale
        dur_s = max(dur_s, CUBIC_PEAK_VELOCITY_FACTOR * abs(distance_rad) / limit_rad_s)
        joints.append((distance_rad, v0s_rad_s, limit_rad_s))

    for _ in range(_MAX_DURATION_STEPS):
        if all(
            cubic_peak_velocity_rad_s(distance_rad, v0_rad_s, dur_s)
            <= max(limit_rad_s, abs(v0_rad_s)) * (1.0 + 1e-9)
            for distance_rad, v0s_rad_s, limit_rad_s in joints
            for v0_rad_s in v0s_rad_s
        ):
            return dur_s
        dur_s *= _DURATION_GROWTH
    raise ValueError("no duration keeps the joints within their velocity limits")


def command_has_changed(commands, last_sent_commands):
    """Return True when the commands differ from the last ones that were sent."""
    return last_sent_commands is None or list(commands) != list(last_sent_commands)
