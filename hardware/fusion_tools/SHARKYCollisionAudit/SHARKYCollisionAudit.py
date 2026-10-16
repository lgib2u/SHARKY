import adsk.core
import adsk.fusion
import json
import math
import os
import traceback


OUTPUT_PATH = os.path.expanduser(os.environ.get(
    'SHARKY_FUSION_OUTPUT',
    '~/Documents/fusion_collision_sweep.json',
))
COARSE_STEP_DEG = 5
INTERSECTION_TOLERANCE_CM3 = 0.002


def _point(point):
    return [round(float(value), 9) for value in point.asArray()]


def _vector(vector):
    return [round(float(value), 9) for value in vector.asArray()]


def _occurrence_name(occurrence):
    return occurrence.fullPathName or occurrence.name


def _joint_origin(joint):
    for attribute in ('geometryOrOriginTwo', 'geometryOrOriginOne'):
        geometry = getattr(joint, attribute, None)
        if geometry is None:
            continue
        if hasattr(geometry, 'origin'):
            return geometry.origin.copy()
        nested = getattr(geometry, 'geometry', None)
        if nested is not None and hasattr(nested, 'origin'):
            return nested.origin.copy()
    transform = getattr(joint, 'transform', None)
    if transform is not None:
        return transform.translation.asPoint()
    return None


def _line_distance(point, origin, axis):
    direction = origin.vectorTo(point)
    cross = direction.crossProduct(axis)
    return cross.length / max(axis.length, 1e-12)


def _infer_axis(joint, origin):
    candidates = []
    for occurrence in (joint.occurrenceOne, joint.occurrenceTwo):
        if occurrence is None:
            continue
        for body in occurrence.bRepBodies:
            for face in body.faces:
                cylinder = adsk.core.Cylinder.cast(face.geometry)
                if cylinder is None:
                    continue
                axis = cylinder.axis.copy()
                axis.normalize()
                distance = _line_distance(origin, cylinder.origin, axis)
                # Servo-pivot faces in this model use a 4 mm radius.  Distance
                # to the joint line remains the primary criterion.
                score = distance + abs(float(cylinder.radius) - 0.4) * 0.05
                candidates.append((score, distance, cylinder.radius, axis))
    if not candidates:
        return None, None
    candidates.sort(key=lambda item: item[0])
    _, distance, radius, axis = candidates[0]
    return axis, {
        'axis_line_error_cm': round(float(distance), 9),
        'matched_cylinder_radius_cm': round(float(radius), 9),
    }


def _bounds_overlap(first, second):
    return not (
        first.maxPoint.x < second.minPoint.x
        or first.minPoint.x > second.maxPoint.x
        or first.maxPoint.y < second.minPoint.y
        or first.minPoint.y > second.maxPoint.y
        or first.maxPoint.z < second.minPoint.z
        or first.minPoint.z > second.maxPoint.z
    )


def _intersection_volume(manager, first, second):
    if not _bounds_overlap(first.boundingBox, second.boundingBox):
        return 0.0
    target = manager.copy(first)
    tool = manager.copy(second)
    try:
        success = manager.booleanOperation(
            target,
            tool,
            adsk.fusion.BooleanTypes.IntersectionBooleanType,
        )
        if not success:
            return 0.0
        return max(0.0, float(target.physicalProperties.volume))
    except RuntimeError:
        return 0.0


def _leg_key(name):
    for key in ('L1', 'R1', 'L2', 'R2'):
        if key in name:
            return key
    return None


def _moving_occurrences(joint, occurrences):
    names = [_occurrence_name(joint.occurrenceOne), _occurrence_name(joint.occurrenceTwo)]
    foot = next((name for name in names if 'Foot' in name), None)
    if foot:
        return [occurrences[foot]], 'knuckle'

    leg = next((name for name in names if 'Leg' in name), None)
    key = _leg_key(leg or '')
    moving = [occurrences[leg]]
    matching_foot = next(
        (occurrence for name, occurrence in occurrences.items()
         if 'Foot' in name and _leg_key(name) == key),
        None,
    )
    if matching_foot is not None:
        moving.append(matching_foot)
    return moving, 'shoulder'


def _body_records(occurrences):
    records = []
    for occurrence in occurrences:
        for body in occurrence.bRepBodies:
            records.append((_occurrence_name(occurrence), body))
    return records


def _sweep_joint(joint, all_occurrences, manager):
    origin = _joint_origin(joint)
    axis, axis_match = _infer_axis(joint, origin)
    if origin is None or axis is None:
        return {'name': joint.name, 'error': 'could not infer joint origin/axis'}

    moving_occurrences, role = _moving_occurrences(joint, all_occurrences)
    moving_names = {_occurrence_name(item) for item in moving_occurrences}
    fixed_occurrences = [
        occurrence for name, occurrence in all_occurrences.items()
        if name not in moving_names
    ]
    moving_source = _body_records(moving_occurrences)
    fixed_source = _body_records(fixed_occurrences)

    baseline = {}
    for moving_name, moving_body in moving_source:
        for fixed_name, fixed_body in fixed_source:
            key = f'{moving_name} <> {fixed_name}'
            baseline[key] = _intersection_volume(manager, moving_body, fixed_body)

    cache = {}

    def sample(angle_deg):
        angle_deg = int(angle_deg)
        if angle_deg in cache:
            return cache[angle_deg]
        rotation = adsk.core.Matrix3D.create()
        rotation.setToRotation(math.radians(angle_deg), axis, origin)
        collisions = []
        for moving_name, moving_body in moving_source:
            rotated = manager.copy(moving_body)
            manager.transform(rotated, rotation)
            for fixed_name, fixed_body in fixed_source:
                key = f'{moving_name} <> {fixed_name}'
                volume = _intersection_volume(manager, rotated, fixed_body)
                excess = volume - baseline[key]
                if excess > INTERSECTION_TOLERANCE_CM3:
                    collisions.append({
                        'pair': key,
                        'intersection_cm3': round(volume, 6),
                        'increase_from_zero_cm3': round(excess, 6),
                    })
        collisions.sort(
            key=lambda item: item['increase_from_zero_cm3'], reverse=True
        )
        cache[angle_deg] = {
            'safe': not collisions,
            'collisions': collisions[:5],
        }
        return cache[angle_deg]

    sample(0)
    negative_safe = 0
    negative_collision = None
    for angle in range(-COARSE_STEP_DEG, -181, -COARSE_STEP_DEG):
        if sample(angle)['safe']:
            negative_safe = angle
        else:
            negative_collision = angle
            break

    positive_safe = 0
    positive_collision = None
    for angle in range(COARSE_STEP_DEG, 181, COARSE_STEP_DEG):
        if sample(angle)['safe']:
            positive_safe = angle
        else:
            positive_collision = angle
            break

    if negative_collision is not None:
        for angle in range(negative_safe - 1, negative_collision - 1, -1):
            if sample(angle)['safe']:
                negative_safe = angle
            else:
                negative_collision = angle
                break
    if positive_collision is not None:
        for angle in range(positive_safe + 1, positive_collision + 1):
            if sample(angle)['safe']:
                positive_safe = angle
            else:
                positive_collision = angle
                break

    return {
        'name': joint.name,
        'fusion_joint_type': int(joint.jointMotion.jointType),
        'role': role,
        'occurrence_one': _occurrence_name(joint.occurrenceOne),
        'occurrence_two': _occurrence_name(joint.occurrenceTwo),
        'moving_occurrences': sorted(moving_names),
        'origin_cm': _point(origin),
        'inferred_axis': _vector(axis),
        'axis_match': axis_match,
        'safe_relative_range_deg': [negative_safe, positive_safe],
        'safe_span_deg': positive_safe - negative_safe,
        'first_negative_collision_deg': negative_collision,
        'first_negative_collision': (
            sample(negative_collision)['collisions']
            if negative_collision is not None else []
        ),
        'first_positive_collision_deg': positive_collision,
        'first_positive_collision': (
            sample(positive_collision)['collisions']
            if positive_collision is not None else []
        ),
        'baseline_intersections_cm3': {
            key: round(value, 6) for key, value in baseline.items() if value > 0.0
        },
    }


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if design is None:
            ui.messageBox('No active Fusion design.', 'SHARKY collision audit')
            return

        root = design.rootComponent
        occurrences = {
            _occurrence_name(occurrence): occurrence
            for occurrence in root.allOccurrences
        }
        manager = adsk.fusion.TemporaryBRepManager.get()
        results = []
        for joint in root.allJoints:
            results.append(_sweep_joint(joint, occurrences, manager))

        report = {
            'document': app.activeDocument.name,
            'method': {
                'description': (
                    'Read-only temporary-BRep rotation sweep relative to the current '
                    'CAD pose. Safe range is the continuous interval containing 0 deg.'
                ),
                'coarse_step_deg': COARSE_STEP_DEG,
                'refinement_step_deg': 1,
                'intersection_increase_tolerance_cm3': (
                    INTERSECTION_TOLERANCE_CM3
                ),
            },
            'joints': results,
        }
        os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
        with open(OUTPUT_PATH, 'w', encoding='utf-8') as output_file:
            json.dump(report, output_file, indent=2, sort_keys=True)
            output_file.write('\n')

        ui.messageBox(
            'Collision sweep complete.\n\n'
            f'Joints analysed: {len(results)}\n\n'
            f'{OUTPUT_PATH}',
            'SHARKY collision audit',
        )
    except Exception:
        message = 'Failed:\n{}'.format(traceback.format_exc())
        if ui:
            ui.messageBox(message, 'SHARKY collision audit')
        else:
            print(message)
