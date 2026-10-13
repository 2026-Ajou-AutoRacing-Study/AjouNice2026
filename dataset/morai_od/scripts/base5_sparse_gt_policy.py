"""Conservative opt-in exception; projection is not an occlusion detector."""
import numpy as np

from audit_base5_camera_support import projected_support


def reviewed_camera_support(sample_id, instance_id, box, cameras, reviewed,
                            maximum_distance_m=10., minimum_extent_px=4.):
    if (sample_id, int(instance_id)) not in reviewed:
        return False, 'not_manually_reviewed'
    if np.linalg.norm(np.asarray(box[:2])) > maximum_distance_m:
        return False, 'outside_nearfield'
    for camera in cameras.values():
        support = projected_support(box, camera['lidar2cam'], camera['cam2img'],
                                    camera['width'], camera['height'])
        extent = support.get('resized_256x704_px', [0., 0.])
        if support['supported'] and min(extent) >= minimum_extent_px:
            return True, 'reviewed_nearfield_camera_support'
    return False, 'outside_model_crop_or_too_small'
