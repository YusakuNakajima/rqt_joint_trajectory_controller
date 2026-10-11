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

"""Unit tests for command_trajectory.py (ROS/Qt-free)."""

import pytest

from rqt_joint_trajectory_controller.command_trajectory import (
    CUBIC_PEAK_VELOCITY_FACTOR,
    command_duration_s,
    command_has_changed,
)


def test_small_motion_uses_min_duration_scaled():
    assert command_duration_s([0.01], [0.0], [1.0], 0.5, 0.5) == pytest.approx(1.0)


def test_large_motion_keeps_cubic_peak_below_scaled_max_velocity():
    cmd, pos, max_vel, scale = [2.0, 0.0], [0.0, 0.0], [1.0, 1.0], 0.5
    dur_s = command_duration_s(cmd, pos, max_vel, 0.5, scale)
    peak_rad_s = CUBIC_PEAK_VELOCITY_FACTOR * 2.0 / dur_s
    assert peak_rad_s == pytest.approx(max_vel[0] * scale)


def test_slowest_joint_sets_the_duration():
    dur_s = command_duration_s([1.0, 1.0], [0.0, 0.0], [2.0, 0.5], 0.0, 1.0)
    assert dur_s == pytest.approx(CUBIC_PEAK_VELOCITY_FACTOR * 1.0 / 0.5)


def test_rejects_non_positive_speed_scale():
    with pytest.raises(ValueError):
        command_duration_s([1.0], [0.0], [1.0], 0.5, 0.0)


def test_command_has_changed():
    assert command_has_changed([0.0, 1.0], None)
    assert not command_has_changed([0.0, 1.0], [0.0, 1.0])
    assert command_has_changed([0.0, 1.1], [0.0, 1.0])
