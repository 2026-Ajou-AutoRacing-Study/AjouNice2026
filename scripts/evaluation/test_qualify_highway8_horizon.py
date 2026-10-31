import copy
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parent))
from qualify_highway8_horizon import compare_prefix,compare_native,diagnose_route_failure,matched_gt_comparison


class Message(S):
    def serialize(self,stream):
        stream.write(json.dumps(vars(self),sort_keys=True,default=vars).encode())


def message(count=31,confidence=.75):
    return Message(header=S(seq=1,frame='map'),objects=[S(uuid=[1]*16,score=.9,
        kinematics=S(predicted_paths=[S(confidence=confidence,path=list(range(count)))]))])


class PrefixTest(unittest.TestCase):
    def test_unchanged(self):
        self.assertEqual(0,compare_prefix(message(),message())[0])

    def test_eight_second_suffix(self):
        self.assertEqual(1,compare_prefix(message(),message(81))[0])

    def test_six_seconds_rejected(self):
        with self.assertRaisesRegex(ValueError,'Partial'):
            compare_prefix(message(),message(61))

    def test_prefix_change_rejected(self):
        value=message(81);value.objects[0].kinematics.predicted_paths[0].path[3]=999
        with self.assertRaisesRegex(ValueError,'First3'):
            compare_prefix(message(),value)

    def test_confidence_change_rejected(self):
        with self.assertRaisesRegex(ValueError,'First3'):
            compare_prefix(message(),message(81,.7))

    def test_object_change_rejected(self):
        value=message(81);value.objects=[]
        with self.assertRaisesRegex(ValueError,'Object count'):
            compare_prefix(message(),value)

    def test_partial_topk_rejected(self):
        baseline=message();baseline.objects[0].kinematics.predicted_paths.append(copy.deepcopy(baseline.objects[0].kinematics.predicted_paths[0]))
        value=copy.deepcopy(baseline);value.objects[0].kinematics.predicted_paths[0].path=list(range(81))
        with self.assertRaisesRegex(ValueError,'Partial'):
            compare_prefix(baseline,value)

    def test_empty_frame(self):
        a=message();a.objects=[]
        self.assertEqual(0,compare_prefix(a,copy.deepcopy(a))[0])


class RouteFailureTest(unittest.TestCase):
    def network(self,following=(2,),allowed=(1,2)):
        records={1:S(following_ids=following),2:S(following_ids=())}
        def geometry(ids):
            return S(length=40.*len(ids)),S(project=lambda *args:S(arc_length=0.))
        net=S(config=S(maximum_route_depth=10),records=records,_route_geometry=geometry)
        args=(0.,0.,0.,0.,10.,(.1,3.),(.1,3.,8.),(S(lanelet_ids=(1,)),))
        return net,args,dict(allowed_lanelet_ids=allowed)

    def test_ambiguous_branch(self):
        net,args,kw=self.network((2,3))
        self.assertEqual('ambiguous_successor',diagnose_route_failure(net,args,kw)['reason'])

    def test_leaves_highway(self):
        net,args,kw=self.network(allowed=(1,))
        self.assertEqual('successor_leaves_highway',diagnose_route_failure(net,args,kw)['reason'])

    def test_route_ended(self):
        net,args,kw=self.network(())
        self.assertEqual('route_ended',diagnose_route_failure(net,args,kw)['reason'])


def native_message(count=31):
    poses=[S(position=S(x=i*.1,y=0.,z=0.),orientation=S(x=0.,y=0.,z=0.,w=1.)) for i in range(count)]
    path=S(path=poses,time_step=S(sec=0,nanosec=100000000),confidence=.8)
    obj=Message(object_id=S(uuid=[1]*16),score=.9,kinematics=S(predicted_paths=[path]))
    return Message(header=S(seq=1,frame='map'),objects=[obj])


class NativeTest(unittest.TestCase):
    selected={bytes([1]*16)}

    def test_native_full_horizon(self):
        count,*_=compare_native(native_message(),native_message(81),self.selected,self.selected)
        self.assertEqual(1,count)

    def test_native_topk_confidence_and_prefix_changes_allowed(self):
        value=native_message(81)
        value.objects[0].kinematics.predicted_paths[0].confidence=.4
        extra=copy.deepcopy(value.objects[0].kinematics.predicted_paths[0])
        extra.confidence=.6
        for i,p in enumerate(extra.path):p.position.y=.01*i
        value.objects[0].kinematics.predicted_paths.insert(0,extra)
        result=compare_native(native_message(),value,self.selected,self.selected)
        self.assertEqual(1,result[3]['path_count_changed'])
        self.assertEqual(1,result[3]['ranked_confidences_changed'])
        self.assertEqual(1,result[3]['top1_prefix_changed'])

    def test_tangent_suffix_keeps_exact_prefix_and_motion(self):
        result=compare_native(native_message(),native_message(81),self.selected,
                              self.selected,{next(iter(self.selected)):1.})
        self.assertEqual(1,result[3]['tangent_objects_prefix_exact'])
        self.assertEqual(50,result[3]['tangent_tail_intervals_valid'])

    def test_tangent_changed_confidence_rejected(self):
        value=native_message(81)
        value.objects[0].kinematics.predicted_paths[0].confidence=.4
        with self.assertRaisesRegex(ValueError,'Map3 prefix'):
            compare_native(native_message(),value,self.selected,self.selected,
                           {next(iter(self.selected)):1.})

    def test_tangent_first_extension_discontinuity_rejected(self):
        value=native_message(81)
        value.objects[0].kinematics.predicted_paths[0].path[31].position.y=.02
        with self.assertRaisesRegex(ValueError,'discontinuous'):
            compare_native(native_message(),value,self.selected,self.selected,
                           {next(iter(self.selected)):1.})

    def test_tangent_wrong_terminal_yaw_or_z_rejected(self):
        for field in ('z','yaw'):
            value=native_message(81)
            tail=value.objects[0].kinematics.predicted_paths[0].path[-1]
            if field=='z':
                tail.position.z=.1
            else:
                tail.orientation.z=math.sin(.01);tail.orientation.w=math.cos(.01)
            with self.assertRaisesRegex(ValueError,'terminal yaw/z'):
                compare_native(native_message(),value,self.selected,self.selected,
                               {next(iter(self.selected)):1.})

    def test_nonhighway_must_remain_wire_exact(self):
        value=native_message()
        value.objects[0].kinematics.predicted_paths[0].path[4].position.y=1.
        with self.assertRaisesRegex(ValueError,'Non-highway'):
            compare_native(native_message(),value,set())

    def test_current_state_must_remain_unchanged(self):
        value=native_message(81);value.objects[0].score=.5
        with self.assertRaisesRegex(ValueError,'current state'):
            compare_native(native_message(),value,self.selected,self.selected)

    def test_selected_highway_cannot_fall_back_to_three(self):
        with self.assertRaisesRegex(ValueError,'full8s'):
            compare_native(native_message(),native_message(),self.selected,self.selected)

    def test_clamped_final_positions_rejected(self):
        value=native_message(81)
        value.objects[0].kinematics.predicted_paths[0].path[-1]=copy.deepcopy(value.objects[0].kinematics.predicted_paths[0].path[-2])
        with self.assertRaisesRegex(ValueError,'clamped'):
            compare_native(native_message(),value,self.selected,self.selected)

    def test_stationary_eight_second_path_is_not_misclassified_as_padding(self):
        value=native_message(81)
        for pose in value.objects[0].kinematics.predicted_paths[0].path:
            pose.position.x=0.
        result=compare_native(native_message(),value,self.selected,set())
        self.assertEqual(1,result[0])

    def test_nonfinite_pose_rejected(self):
        value=native_message(81);value.objects[0].kinematics.predicted_paths[0].path[-1].position.x=float('nan')
        with self.assertRaisesRegex(ValueError,'Nonfinite'):
            compare_native(native_message(),value,self.selected,self.selected)

    def test_matched_gt_uses_only_shared_origins(self):
        def row(i,error):return dict(case='case',stamp=float(i),id=i,gt_key=['vehicle',i],
            top1_ADE_m=error,top1_FDE_m=error,minK_ADE_m=error,minK_FDE_m=error)
        result=matched_gt_comparison([row(1,3.),row(2,10.)],[row(1,4.),row(3,1.)])
        self.assertEqual(1,result['common_origins'])
        self.assertEqual(1,result['native_only_origins'])
        self.assertEqual(1,result['append_only_origins'])
        self.assertEqual(3.,result['native_common']['top1_FDE_m']['mean'])
        self.assertEqual(4.,result['append_common']['top1_FDE_m']['mean'])


if __name__=='__main__':
    unittest.main()
