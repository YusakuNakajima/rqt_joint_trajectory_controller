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
    cubic_peak_velocity_rad_s,
)

AT_REST = [0.0]


def sampled_peak_velocity(distance, start_velocity, duration_s, samples=20000):
    """Brute-force max |v| of the controller's cubic (independent of the closed form)."""
    t = duration_s
    a = (start_velocity * t - 2.0 * distance) / t ** 3
    b = (3.0 * distance - 2.0 * start_velocity * t) / t ** 2
    return max(
        abs(3.0 * a * (t * k / samples) ** 2 + 2.0 * b * (t * k / samples) + start_velocity)
        for k in range(samples + 1)
    )


def test_small_motion_uses_min_duration_scaled():
    assert command_duration_s([0.01], [0.0], [AT_REST], [1.0], 0.5, 0.5) == pytest.approx(1.0)


def test_large_motion_from_rest_keeps_cubic_peak_at_scaled_max_velocity():
    dur_s = command_duration_s([2.0, 0.0], [0.0, 0.0], [AT_REST, AT_REST], [1.0, 1.0], 0.5, 0.5)
    assert CUBIC_PEAK_VELOCITY_FACTOR * 2.0 / dur_s == pytest.approx(0.5)


def test_slowest_joint_sets_the_duration():
    dur_s = command_duration_s([1.0, 1.0], [0.0, 0.0], [AT_REST, AT_REST], [2.0, 0.5], 0.0, 1.0)
    assert dur_s == pytest.approx(CUBIC_PEAK_VELOCITY_FACTOR * 1.0 / 0.5)


def test_rejects_non_positive_speed_scale():
    with pytest.raises(ValueError):
        command_duration_s([1.0], [0.0], [AT_REST], [1.0], 0.5, 0.0)


@pytest.mark.parametrize('distance, v0, duration_s', [
    (1.0, 0.0, 2.0), (-2.31, 0.55, 3.0), (2.31, -0.55, 3.465), (0.1, 0.8, 0.5), (0.0, 0.3, 1.0),
])
def test_cubic_peak_velocity_matches_sampling(distance, v0, duration_s):
    assert cubic_peak_velocity_rad_s(distance, v0, duration_s) == pytest.approx(
        sampled_peak_velocity(distance, v0, duration_s), rel=1e-4)


def test_reversing_joint_in_flight_stays_within_scaled_limit():
    # Review case: moving at 0.55 rad/s, new target 2.31 rad away in the opposite direction.
    max_vel, scale = 1.0, 1.0
    dur_s = command_duration_s([-2.31], [0.0], [[0.55]], [max_vel], 0.5, scale)
    assert sampled_peak_velocity(-2.31, 0.55, dur_s) <= max_vel * scale + 1e-6


def test_every_candidate_start_velocity_is_respected():
    # measured and reference velocities may differ; both must stay within the limit
    dur_s = command_duration_s([-1.0], [0.0], [[0.0, 0.6]], [1.0], 0.1, 1.0)
    assert sampled_peak_velocity(-1.0, 0.0, dur_s) <= 1.0 + 1e-6
    assert sampled_peak_velocity(-1.0, 0.6, dur_s) <= 1.0 + 1e-6


def test_joint_already_faster_than_scaled_limit_is_not_accelerated_further():
    # e.g. speed scaling was just lowered; the joint cannot slow down instantly
    dur_s = command_duration_s([1.0], [0.0], [[0.9]], [1.0], 0.1, 0.5)
    assert sampled_peak_velocity(1.0, 0.9, dur_s) <= 0.9 + 1e-6


@pytest.mark.parametrize('max_vel_rad_s', [0.0, -1.0, float('nan'), float('inf')])
def test_invalid_velocity_limit_sends_nothing(max_vel_rad_s):
    with pytest.raises(ValueError):
        command_duration_s([1.0], [0.0], [AT_REST], [max_vel_rad_s], 0.5, 1.0)


def test_command_has_changed():
    assert command_has_changed([0.0, 1.0], None)
    assert not command_has_changed([0.0, 1.0], [0.0, 1.0])
    assert command_has_changed([0.0, 1.1], [0.0, 1.0])
