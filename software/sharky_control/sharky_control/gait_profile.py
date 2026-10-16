#!/usr/bin/env python3

"""Load and validate SHARKY gait tuning without weakening hardware limits."""

import copy
import json
from pathlib import Path


ACTIVE_PROFILE_PATH = Path.home() / '.config' / 'sharky' / 'gaits' / 'active.json'

MIN_SPEED_SCALE = 0.5
DEFAULT_SPEED_SCALE = 3.0
MAX_SPEED_SCALE = 5.0
FRONT_SUPPORT_ADVANCE_DEG = 20.0

HARDWARE_LIMITS = {
    0: (15.0, 110.0),
    1: (75.0, 178.0),
    2: (70.0, 165.0),
    3: (2.0, 105.0),
    4: (30.0, 110.0),
    5: (75.0, 175.0),
    6: (70.0, 150.0),
    7: (5.0, 105.0),
}

DEFAULT_PROFILE = {
    'schema_version': 1,
    'name': 'smooth_crawl_v1',
    'angles': {
        'rot_forward_front': 15.0,
        'rot_forward_rear': 30.0,
        'rot_back': 110.0,
        'rot_neutral': 90.0,
        'foot_up': 75.0,
        'foot_down_front': 178.0,
        'foot_down_rear': 175.0,
        'foot_neutral': 90.0,
    },
    'timings': {
        'lift': 0.30,
        'transfer': 0.45,
        'lower': 0.30,
        'pose_rotation': 0.55,
        'pose_feet': 0.65,
    },
}


def clamp_speed_scale(value):
    """Clamp a nominal gait-rate multiplier to the supported range."""
    return max(MIN_SPEED_SCALE, min(float(value), MAX_SPEED_SCALE))


def front_support_target(profile, advance_degrees=FRONT_SUPPORT_ADVANCE_DEG):
    """Brace the planted front leg ahead of its normal rearward support pose.

    Logical shoulder angles decrease toward the front of the robot.  Clamp the
    preload to the commissioned forward target so profile changes can never
    drive this helper beyond the validated gait envelope.
    """
    angle = profile['angles']
    advance = max(0.0, float(advance_degrees))
    return max(
        float(angle['rot_forward_front']),
        float(angle['rot_back']) - advance,
    )


def _hardware_targets(profile):
    """Expand logical angles into every hardware-facing channel target."""
    angle = profile['angles']
    return {
        0: [angle['rot_forward_front'], angle['rot_back'], angle['rot_neutral']],
        1: [angle['foot_up'], angle['foot_down_front'], angle['foot_neutral']],
        2: [180.0 - angle['rot_forward_front'], 180.0 - angle['rot_back'],
            180.0 - angle['rot_neutral']],
        3: [180.0 - angle['foot_up'], 180.0 - angle['foot_down_front'],
            180.0 - angle['foot_neutral']],
        4: [angle['rot_forward_rear'], angle['rot_back'], angle['rot_neutral']],
        5: [angle['foot_up'], angle['foot_down_rear'], angle['foot_neutral']],
        6: [180.0 - angle['rot_forward_rear'], 180.0 - angle['rot_back'],
            180.0 - angle['rot_neutral']],
        7: [180.0 - angle['foot_up'], 180.0 - angle['foot_down_rear'],
            180.0 - angle['foot_neutral']],
    }


def turn_targets(profile, direction):
    """Return logical rotation targets for a left or right pivot.

    Keeping this mapping independent of ROS makes the left/right mirror
    relationship straightforward to regression-test.
    """
    angle = profile['angles']
    forward = {
        'FL': angle['rot_forward_front'],
        'FR': angle['rot_forward_front'],
        'RL': angle['rot_forward_rear'],
        'RR': angle['rot_forward_rear'],
    }
    backward = {leg: angle['rot_back'] for leg in forward}

    if direction == 'left':
        return {
            'FL': backward['FL'],
            'FR': forward['FR'],
            'RL': backward['RL'],
            'RR': forward['RR'],
        }
    if direction == 'right':
        return {
            'FL': forward['FL'],
            'FR': backward['FR'],
            'RL': forward['RL'],
            'RR': backward['RR'],
        }
    raise ValueError("turn direction must be 'left' or 'right'")


def tilt_targets(profile, direction, retraction_fraction=0.45):
    """Return a stable four-foot body-tilt pose in logical servo degrees.

    One end stays at the defined standing extension while the opposite end is
    retracted part-way toward neutral.  This keeps every target inside the
    already commissioned standing/neutral envelope.
    """
    if direction not in ('up', 'down'):
        raise ValueError("tilt direction must be 'up' or 'down'")
    if not 0.0 <= float(retraction_fraction) <= 1.0:
        raise ValueError('tilt retraction_fraction must be between 0 and 1')

    angle = profile['angles']
    neutral = float(angle['foot_neutral'])
    front_down = float(angle['foot_down_front'])
    rear_down = float(angle['foot_down_rear'])
    front_retracted = front_down + (neutral - front_down) * retraction_fraction
    rear_retracted = rear_down + (neutral - rear_down) * retraction_fraction

    if direction == 'up':
        return {
            'FL': front_down,
            'FR': front_down,
            'RL': rear_retracted,
            'RR': rear_retracted,
        }
    return {
        'FL': front_retracted,
        'FR': front_retracted,
        'RL': rear_down,
        'RR': rear_down,
    }


def validate_profile(profile):
    if not isinstance(profile, dict):
        raise ValueError('gait profile must be a JSON object')
    if profile.get('schema_version') != 1:
        raise ValueError('unsupported gait profile schema_version')
    if not isinstance(profile.get('name'), str) or not profile['name'].strip():
        raise ValueError('gait profile requires a non-empty name')

    expected_angles = set(DEFAULT_PROFILE['angles'])
    expected_timings = set(DEFAULT_PROFILE['timings'])
    angles = profile.get('angles')
    timings = profile.get('timings')
    if not isinstance(angles, dict) or set(angles) != expected_angles:
        raise ValueError(f'angles must contain exactly: {sorted(expected_angles)}')
    if not isinstance(timings, dict) or set(timings) != expected_timings:
        raise ValueError(f'timings must contain exactly: {sorted(expected_timings)}')

    for name, value in angles.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'angle {name} must be numeric')
    for name, value in timings.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f'timing {name} must be numeric')
        if not 0.05 <= float(value) <= 1.5:
            raise ValueError(f'timing {name} must be between 0.05 and 1.5 seconds')

    for channel, targets in _hardware_targets(profile).items():
        minimum, maximum = HARDWARE_LIMITS[channel]
        for target in targets:
            if not minimum <= float(target) <= maximum:
                raise ValueError(
                    f'profile target {target:.1f} for channel {channel} is outside '
                    f'the immutable {minimum:.1f}-{maximum:.1f} degree envelope'
                )
    return profile


def load_profile(path=ACTIVE_PROFILE_PATH):
    if not Path(path).is_file():
        return copy.deepcopy(DEFAULT_PROFILE), 'built-in default'
    with Path(path).open(encoding='utf-8') as profile_file:
        profile = json.load(profile_file)
    return validate_profile(profile), str(path)


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Validate a SHARKY gait profile')
    parser.add_argument('profile', type=Path)
    args = parser.parse_args()
    profile, source = load_profile(args.profile)
    print(f"valid gait profile: {profile['name']} ({source})")


if __name__ == '__main__':
    main()
