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

# Peak velocity of a rest-to-rest cubic segment is 1.5 * distance / duration.
CUBIC_PEAK_VELOCITY_FACTOR = 1.5
# Duration growth per step while searching for a duration that respects the limits.
_DURATION_GROWTH = 1.05
_MAX_DURATION_STEPS = 400


def cubic_peak_velocity(distance, start_velocity, duration_s):
    """
    Return the largest |velocity| of the cubic used by joint_trajectory_controller.

    The cubic starts with start_velocity and ends at rest, distance away, after duration_s.
    """
    t = duration_s
    a = (start_velocity * t - 2.0 * distance) / t ** 3
    b = (3.0 * distance - 2.0 * start_velocity * t) / t ** 2
    peak = abs(start_velocity)  # the velocity at t = duration_s is zero
    if a != 0.0:
        t_vertex = -b / (3.0 * a)
        if 0.0 < t_vertex < t:
            peak = max(peak, abs(3.0 * a * t_vertex ** 2 + 2.0 * b * t_vertex + start_velocity))
    return peak


def command_duration_s(commands, positions, start_velocities, max_velocities, min_duration_s,
                       speed_scale):
    """
    Return the duration of the command segment in seconds.

    Every joint's velocity stays within speed_scale x its maximum velocity, or within
    the velocity it already has if that is larger (it cannot change instantly). The
    duration is never shorter than min_duration_s / speed_scale. start_velocities may
    contain several candidates per joint (e.g. measured and reference velocity); all
    of them are respected.
    """
    if speed_scale <= 0.0:
        raise ValueError("speed_scale must be positive")
    joints = []
    dur_s = min_duration_s / speed_scale
    for cmd, pos, v0s, max_vel in zip(commands, positions, start_velocities, max_velocities):
        distance = cmd - pos
        limit = max_vel * speed_scale
        dur_s = max(dur_s, CUBIC_PEAK_VELOCITY_FACTOR * abs(distance) / limit)
        joints.append((distance, v0s, limit))

    for _ in range(_MAX_DURATION_STEPS):
        if all(
            cubic_peak_velocity(distance, v0, dur_s) <= max(limit, abs(v0)) * (1.0 + 1e-9)
            for distance, v0s, limit in joints
            for v0 in v0s
        ):
            return dur_s
        dur_s *= _DURATION_GROWTH
    raise ValueError("no duration keeps the joints within their velocity limits")


def command_has_changed(commands, last_sent_commands):
    """Return True when the commands differ from the last ones that were sent."""
    return last_sent_commands is None or list(commands) != list(last_sent_commands)
