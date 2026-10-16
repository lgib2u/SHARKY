import adsk.core
import adsk.fusion
import json
import math
import os
import traceback


OUTPUT_PATH = os.path.expanduser(os.environ.get(
    'SHARKY_FUSION_OUTPUT',
    '~/Documents/fusion_revolute_setup.json',
))

# Limits are relative to the current CAD pose, which is treated as neutral.
# SHARKY's driver and servos expose at most 180 degrees, so collision-free CAD
# clearance beyond this range is deliberately not enabled.
JOINTS = {
    'Rigid 16': ('rear_right_knuckle_joint', 'LegR2', 'FootR2', 7),
    'Rigid 17': ('rear_right_shoulder_joint', 'base_link', 'LegR2', 6),
    'Rigid 18': ('front_right_knuckle_joint', 'LegR1', 'FootR1', 3),
    'Rigid 19': ('front_right_shoulder_joint', 'base_link', 'LegR1', 2),
    'Rigid 20': ('front_left_shoulder_joint', 'LegL1', 'base_link', 0),
    'Rigid 21': ('front_left_knuckle_joint', 'LegL1', 'FootL1', 1),
    'Rigid 22': ('rear_left_shoulder_joint', 'LegL2', 'base_link', 4),
    'Rigid 23': ('rear_left_knuckle_joint', 'LegL2', 'FootL2', 5),
}


def _occurrence_name(occurrence):
    return occurrence.fullPathName or occurrence.name


def _geometry(joint):
    for attribute in ('geometryOrOriginTwo', 'geometryOrOriginOne'):
        geometry = getattr(joint, attribute, None)
        if geometry is None:
            continue
        nested = getattr(geometry, 'geometry', None)
        return nested if nested is not None else geometry
    return None


def _vector_values(vector):
    return [round(float(value), 9) for value in vector.asArray()]


def _find_joint(root, name):
    for joint in root.allJoints:
        if joint.name == name:
            return joint
    return None


def _find_joint_by_fragments(root, expected_fragments):
    matches = []
    for joint in root.allJoints:
        actual = (
            _occurrence_name(joint.occurrenceOne),
            _occurrence_name(joint.occurrenceTwo),
        )
        if all(any(fragment in name for name in actual)
               for fragment in expected_fragments):
            matches.append(joint)
    if len(matches) != 1:
        raise RuntimeError(
            f'Expected exactly one joint for {expected_fragments}; '
            f'found {[joint.name for joint in matches]}'
        )
    return matches[0]


def _validate_joint(joint, expected_fragments):
    actual = {
        _occurrence_name(joint.occurrenceOne),
        _occurrence_name(joint.occurrenceTwo),
    }
    for fragment in expected_fragments:
        if not any(fragment in name for name in actual):
            raise RuntimeError(
                f'{joint.name}: expected component fragment {fragment!r}; '
                f'found {sorted(actual)}'
            )


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if design is None:
            ui.messageBox('No active Fusion design.', 'SHARKY revolute setup')
            return

        root = design.rootComponent

        # Validate the complete expected assembly before changing the document.
        for old_name, (_, first, second, _) in JOINTS.items():
            joint = _find_joint_by_fragments(root, (first, second))
            if int(joint.jointMotion.jointType) not in (0, 1):
                raise RuntimeError(
                    f'{joint.name} is neither rigid nor revolute '
                    f'(type {joint.jointMotion.jointType})'
                )
            _validate_joint(joint, (first, second))

        results = []
        for old_name, (new_name, first, second, channel) in JOINTS.items():
            design.timeline.moveToEnd()
            joint = _find_joint_by_fragments(root, (first, second))
            geometry = _geometry(joint)
            if geometry is None:
                raise RuntimeError(f'{old_name}: missing joint geometry')

            # The selected cylindrical hinge faces define the intended pivot as
            # the joint coordinate system's primary (Z) axis.
            expected_axis = geometry.primaryAxisVector.copy()
            expected_axis.normalize()

            if int(joint.jointMotion.jointType) == 0:
                joint.timelineObject.rollTo(True)
                if not joint.setAsRevoluteJointMotion(
                        adsk.fusion.JointDirections.ZAxisJointDirection):
                    raise RuntimeError(f'{old_name}: revolute conversion failed')
                design.timeline.moveToEnd()

            joint = _find_joint_by_fragments(root, (first, second))
            joint.name = new_name
            motion = adsk.fusion.RevoluteJointMotion.cast(joint.jointMotion)
            if motion is None:
                raise RuntimeError(f'{new_name}: converted motion is not revolute')

            limits = motion.rotationLimits
            limits.minimumValue = math.radians(-90.0)
            limits.maximumValue = math.radians(90.0)
            limits.isMinimumValueEnabled = True
            limits.isMaximumValueEnabled = True

            axis = motion.rotationAxisVector.copy()
            axis.normalize()
            alignment = abs(axis.dotProduct(expected_axis))
            if alignment < 0.999:
                raise RuntimeError(
                    f'{new_name}: revolute axis does not match the selected hinge '
                    f'(absolute dot product {alignment:.6f})'
                )

            results.append({
                'old_name': old_name,
                'name': new_name,
                'pca9685_channel': channel,
                'occurrence_one': _occurrence_name(joint.occurrenceOne),
                'occurrence_two': _occurrence_name(joint.occurrenceTwo),
                'joint_type': int(motion.jointType),
                'axis': _vector_values(axis),
                'minimum_deg': -90.0,
                'maximum_deg': 90.0,
            })

        design.timeline.moveToEnd()
        report = {
            'document': app.activeDocument.name,
            'reference_pose': 'current CAD pose treated as neutral/zero',
            'limit_basis': (
                '180-degree actuator capability; simplified CAD sweep found no '
                'tighter collision boundary'
            ),
            'joints': results,
        }
        with open(OUTPUT_PATH, 'w', encoding='utf-8') as report_file:
            json.dump(report, report_file, indent=2, sort_keys=True)
            report_file.write('\n')

        ui.messageBox(
            'Converted 8 SHARKY joints to revolute motion.\n\n'
            'Each joint is limited to -90 to +90 degrees around the current pose.\n'
            'The document remains open for inspection before saving.\n\n'
            + OUTPUT_PATH,
            'SHARKY revolute setup',
        )
    except Exception:
        if ui is not None:
            ui.messageBox(
                'SHARKY revolute setup failed:\n\n' + traceback.format_exc(),
                'SHARKY revolute setup',
            )


def stop(context):
    pass
