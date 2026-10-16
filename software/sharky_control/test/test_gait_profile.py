import copy

import pytest

from sharky_control.gait_profile import (
    DEFAULT_SPEED_SCALE,
    DEFAULT_PROFILE,
    MAX_SPEED_SCALE,
    MIN_SPEED_SCALE,
    clamp_speed_scale,
    front_support_target,
    tilt_targets,
    turn_targets,
    validate_profile,
)


def test_left_and_right_turns_are_mirrors():
    left = turn_targets(DEFAULT_PROFILE, 'left')
    right = turn_targets(DEFAULT_PROFILE, 'right')

    assert left['FL'] == right['FR']
    assert left['FR'] == right['FL']
    assert left['RL'] == right['RR']
    assert left['RR'] == right['RL']
    assert left != right


def test_invalid_turn_direction_is_rejected():
    with pytest.raises(ValueError):
        turn_targets(DEFAULT_PROFILE, 'sideways')


def test_profile_cannot_exceed_hardware_envelope():
    profile = copy.deepcopy(DEFAULT_PROFILE)
    profile['angles']['foot_up'] = 70.0

    with pytest.raises(ValueError, match='immutable'):
        validate_profile(profile)


def test_speed_scale_defaults_and_clamps():
    assert DEFAULT_SPEED_SCALE == 3.0
    assert MAX_SPEED_SCALE == 5.0
    assert clamp_speed_scale(0.1) == MIN_SPEED_SCALE
    assert clamp_speed_scale(DEFAULT_SPEED_SCALE) == DEFAULT_SPEED_SCALE
    assert clamp_speed_scale(99.0) == MAX_SPEED_SCALE


def test_front_support_preload_advances_twenty_degrees_without_overshoot():
    assert front_support_target(DEFAULT_PROFILE) == 90.0

    short_stride = copy.deepcopy(DEFAULT_PROFILE)
    short_stride['angles']['rot_forward_front'] = 85.0
    assert front_support_target(short_stride, 40.0) == 85.0


def test_tilt_poses_keep_one_end_standing_and_retract_the_other():
    up = tilt_targets(DEFAULT_PROFILE, 'up')
    down = tilt_targets(DEFAULT_PROFILE, 'down')

    assert up['FL'] == up['FR'] == DEFAULT_PROFILE['angles']['foot_down_front']
    assert down['RL'] == down['RR'] == DEFAULT_PROFILE['angles']['foot_down_rear']
    assert DEFAULT_PROFILE['angles']['foot_neutral'] < up['RL'] < up['FL']
    assert DEFAULT_PROFILE['angles']['foot_neutral'] < down['FL'] < down['RL']
