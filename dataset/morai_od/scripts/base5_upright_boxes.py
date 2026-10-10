"""Native MORAI full-RPY boxes -> enclosing upright 7D detector boxes.

This is a derived training-label policy, never a rewrite of sensor annotations.
Keep the geometric center and native yaw; enclose all eight rotated corners.
The resulting box is not a minimum-volume box or a new visibility measurement.
"""
import numpy as np

BOX_GEOMETRY_POLICY = 'native_rpy_fixed_yaw_enclosure_v1'
LEGACY_BOX_GEOMETRY_POLICY = 'native_size_yaw_only_legacy'


def upright_bbox(center, size, rotation_rpy):
    center = np.asarray(center, dtype=np.float64)
    size = np.asarray(size, dtype=np.float64)
    rpy = np.asarray(rotation_rpy, dtype=np.float64)
    if any(value.shape != (3,) for value in (center, size, rpy)):
        raise ValueError('Native box center, size and RPY must each have three values')
    if not np.isfinite(np.r_[center, size, rpy]).all() or (size <= 0).any():
        raise ValueError('Native box must be finite with positive dimensions')
    roll, pitch, yaw = rpy
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    # R_native = Rz(yaw) Ry(pitch) Rx(roll). In the retained yaw frame,
    # the symmetric box extents are abs(Ry Rx) @ native dimensions.
    # Avoid cancelling two numerical Rz matrices: upright boxes stay exact.
    tilt_rotation = np.array([[cp, sp * sr, sp * cr],
                              [0., cr, -sr], [-sp, cp * sr, cp * cr]])
    extent = np.abs(tilt_rotation) @ size
    return np.r_[center, extent, yaw].tolist()


def component_bbox(component, policy=BOX_GEOMETRY_POLICY):
    if policy == BOX_GEOMETRY_POLICY:
        return upright_bbox(component['center_lidar_m'], component['size_m'],
                            component['rotation_rpy'])
    if policy == LEGACY_BOX_GEOMETRY_POLICY:
        # Explicit reproduction only; never silently select it for new Base5.
        upright_bbox(component['center_lidar_m'], component['size_m'], component['rotation_rpy'])
        return list(component['center_lidar_m']) + list(component['size_m']) + [component['rotation_rpy'][2]]
    raise ValueError('Unknown Base5 box geometry policy: ' + str(policy))
