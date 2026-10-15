"""Diagnostic subtype recall after merging, not subtype AP or deploy thresholds."""
from collections import Counter
import numpy as np


def subgroup_recall(rows, thresholds=(.05, .10, .30), radius=1.):
    result = {}
    for threshold in thresholds:
        total, hits = Counter(), Counter()
        for row in rows:
            info, pred = row['info'], row['prediction']
            gt = np.asarray(info['gt_boxes'])
            names = np.asarray(info['gt_names'])
            original = np.asarray(row['original_semantic_class'])
            assets = np.asarray(row['assets'])
            if len(original) != len(gt) or len(assets) != len(gt):
                raise ValueError('Subgroup identity must align with GT boxes')
            eligible = np.flatnonzero(names == 'pedestrian')
            groups = {int(i): ('original_' + original[i], 'asset_' + assets[i]) for i in eligible}
            for values in groups.values():
                total.update(values)
            scores = np.asarray(pred['scores_3d'])
            labels = np.asarray(pred['labels_3d'])
            boxes = np.asarray(pred['boxes_3d'].tensor)
            selected = np.flatnonzero((labels == 1) & (scores >= threshold))
            selected = selected[np.argsort(-scores[selected], kind='stable')]
            unmatched = set(map(int, eligible))
            # Match against ALL human/animal pedestrian GT together. A prediction
            # already assigned to a human cannot also count as an animal hit.
            for index in selected:
                if not unmatched:
                    break
                candidates = sorted(unmatched)
                distances = np.linalg.norm(gt[candidates, :2] - boxes[index, :2], axis=1)
                nearest = int(np.argmin(distances))
                if distances[nearest] <= radius:
                    match = candidates[nearest]
                    unmatched.remove(match)
                    hits.update(groups[match])
        for group, count in total.items():
            prefix = 'subgroup/' + group + '/'
            result[prefix + 'gt_count'] = count
            result[prefix + f'recall_1m_score{round(threshold * 100):03d}'] = hits[group] / count
    return result
