#!/usr/bin/env python3
"""Host ROS-shell tests; no ROS master, GPU, simulator or external writes."""
import unittest

from benchmark_ros_cpu_contention import active_outputs, balanced_subset, raw_topics, stats


class ProbeContracts(unittest.TestCase):
    def source(self):
        return {'samples':[
            {'id':f'{group}_{index}','sample_group':group}
            for group,count in (('representative',200),('stress',100))
            for index in range(count)]}

    def test_balanced_sizes_and_identity(self):
        for count in (24,96,192,300):
            with self.subTest(count=count):
                rows=balanced_subset(self.source(),count)
                self.assertEqual(len(rows),count)
                self.assertEqual(len({r['id'] for r in rows}),count)
                self.assertEqual(sum(r['sample_group']=='stress' for r in rows),count//3)

    def test_deterministic_order(self):
        self.assertEqual(balanced_subset(self.source(),96),balanced_subset(self.source(),96))

    def test_duplicate_identity_rejected(self):
        source=self.source();source['samples'][1]['id']=source['samples'][0]['id']
        with self.assertRaises(ValueError):balanced_subset(source,96)

    def test_raw_inputs_only(self):
        topics=['/sensor/gps','/sensor/imu','/sensor/lidar','/Ego_topic','/Object_topic',
                '/Competition_topic','/tf','/tf_static','/ctrl_cmd','/planning/trajectory',
                '/localization/kinematic_state','/perception/object_recognition/objects']
        self.assertEqual(raw_topics(topics),topics[:6])

    def test_no_dead_stack_pass(self):
        self.assertFalse(active_outputs({}))

    def test_empty_actor_or_path_rejected(self):
        result={key:{'count':100,'nonempty':100} for key in
                ('localization','prediction','trajectory','control')}
        self.assertTrue(active_outputs(result))
        for key in ('prediction','trajectory'):
            broken={k:dict(v) for k,v in result.items()};broken[key]['nonempty']=0
            self.assertFalse(active_outputs(broken))

    def test_traffic_requires_published_messages(self):
        result={key:{'count':100,'nonempty':100} for key in
                ('localization','prediction','trajectory','control')}
        self.assertFalse(active_outputs(result,True))
        result['traffic']={'count':100}
        self.assertTrue(active_outputs(result,True))

    def test_stats(self):
        self.assertEqual(stats([]),{'count':0})
        self.assertEqual(stats([1,2,3])['mean'],2)


if __name__=='__main__':unittest.main()
