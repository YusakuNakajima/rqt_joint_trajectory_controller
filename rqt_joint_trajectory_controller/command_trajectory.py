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


def command_duration_s(commands_rad, positions_rad, max_velocities_rad_s, min_duration_s,
                       speed_scale):
    """
    Return the duration of the command segment in seconds.

    Chosen so that, starting from rest, no joint exceeds speed_scale times its
    maximum velocity, and never shorter than min_duration_s / speed_scale.
    """
    if speed_scale <= 0.0:
        raise ValueError("speed_scale must be positive")
    dur_s = [min_duration_s]
    for cmd, pos, max_vel in zip(commands_rad, positions_rad, max_velocities_rad_s):
        dur_s.append(CUBIC_PEAK_VELOCITY_FACTOR * abs(cmd - pos) / max_vel)
    return max(dur_s) / speed_scale


def command_has_changed(commands, last_sent_commands):
    """Return True when the commands differ from the last ones that were sent."""
    return last_sent_commands is None or list(commands) != list(last_sent_commands)
