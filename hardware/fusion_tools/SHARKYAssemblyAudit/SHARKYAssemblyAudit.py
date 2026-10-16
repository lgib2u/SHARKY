import adsk.core
import adsk.fusion
import json
import os
import traceback


OUTPUT_PATH = os.path.expanduser(os.environ.get(
    'SHARKY_FUSION_OUTPUT',
    '~/Documents/fusion_assembly_manifest.json',
))


def _point(point):
    return [round(float(value), 9) for value in point.asArray()]


def _matrix(matrix):
    return [round(float(value), 9) for value in matrix.asArray()]


def _occurrence_name(occurrence):
    if occurrence is None:
        return None
    return occurrence.fullPathName or occurrence.name


def _cylindrical_faces(occurrence):
    """Describe cylindrical faces that can identify servo pivot axes."""
    cylinders = []
    for body in occurrence.bRepBodies:
        for face in body.faces:
            cylinder = adsk.core.Cylinder.cast(face.geometry)
            if cylinder is None:
                continue
            cylinders.append({
                'body': body.name,
                'radius_cm': round(float(cylinder.radius), 9),
                'origin_cm': _point(cylinder.origin),
                'axis': _point(cylinder.axis),
                'area_cm2': round(float(face.area), 9),
                'bounding_box_cm': {
                    'minimum': _point(face.boundingBox.minPoint),
                    'maximum': _point(face.boundingBox.maxPoint),
                },
            })
    return cylinders


def _inertia(properties):
    success, xx, yy, zz, xy, yz, xz = properties.getXYZMomentsOfInertia()
    if not success:
        return None
    return {
        'ixx': round(float(xx), 12),
        'iyy': round(float(yy), 12),
        'izz': round(float(zz), 12),
        'ixy': round(float(xy), 12),
        'iyz': round(float(yz), 12),
        'ixz': round(float(xz), 12),
        'unit': 'kg*cm^2',
    }


def _joint_origin(joint):
    for attribute in ('geometryOrOriginTwo', 'geometryOrOriginOne'):
        geometry = getattr(joint, attribute, None)
        if geometry is None:
            continue
        if hasattr(geometry, 'origin'):
            return _point(geometry.origin)
        nested = getattr(geometry, 'geometry', None)
        if nested is not None and hasattr(nested, 'origin'):
            return _point(nested.origin)
    transform = getattr(joint, 'transform', None)
    if transform is not None:
        return _point(transform.translation)
    return None


def _joint_record(joint, source):
    motion = joint.jointMotion
    record = {
        'name': joint.name,
        'source': source,
        'type': int(motion.jointType),
        'occurrence_one': _occurrence_name(joint.occurrenceOne),
        'occurrence_two': _occurrence_name(joint.occurrenceTwo),
        'origin_cm': _joint_origin(joint),
    }
    if hasattr(motion, 'rotationAxisVector'):
        record['rotation_axis'] = _point(motion.rotationAxisVector)
    if hasattr(motion, 'slideDirectionVector'):
        record['slide_axis'] = _point(motion.slideDirectionVector)
    return record


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if design is None:
            ui.messageBox('No active Fusion design.', 'SHARKY assembly audit')
            return

        root = design.rootComponent
        manifest = {
            'document': app.activeDocument.name,
            'root_component': root.name,
            'fusion_internal_length_unit': 'cm',
            'occurrences': [],
            'joints': [],
        }

        for occurrence in root.allOccurrences:
            properties = occurrence.physicalProperties
            bounds = occurrence.boundingBox
            manifest['occurrences'].append({
                'name': occurrence.name,
                'full_path': occurrence.fullPathName,
                'component': occurrence.component.name,
                'transform_cm': _matrix(occurrence.transform2),
                'mass_kg': float(properties.mass),
                'center_of_mass_cm': _point(properties.centerOfMass),
                'inertia_at_com': _inertia(properties),
                'bounding_box_cm': {
                    'minimum': _point(bounds.minPoint),
                    'maximum': _point(bounds.maxPoint),
                },
                'body_count': occurrence.bRepBodies.count,
                'cylindrical_faces': _cylindrical_faces(occurrence),
            })

        for joint in root.allJoints:
            manifest['joints'].append(_joint_record(joint, 'allJoints'))
        for joint in root.allAsBuiltJoints:
            manifest['joints'].append(_joint_record(joint, 'allAsBuiltJoints'))

        os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
        with open(OUTPUT_PATH, 'w', encoding='utf-8') as output_file:
            json.dump(manifest, output_file, indent=2, sort_keys=True)
            output_file.write('\n')

        ui.messageBox(
            'Assembly audit complete.\n\n'
            f"Occurrences: {len(manifest['occurrences'])}\n"
            f"Joints: {len(manifest['joints'])}\n\n"
            f'{OUTPUT_PATH}',
            'SHARKY assembly audit',
        )
    except Exception:
        message = 'Failed:\n{}'.format(traceback.format_exc())
        if ui:
            ui.messageBox(message, 'SHARKY assembly audit')
        else:
            print(message)
