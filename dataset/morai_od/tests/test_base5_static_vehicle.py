import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import base5_static_vehicle as state


class StaticVehicleTests(unittest.TestCase):
    def actor(self):
        return dict(UNIQUEID=1,initPosition=dict(gear=4,pos=dict(x=1.,y=2.,z=3.),
            rot=dict(yaw='40.'),vehicleSaveData=dict(inputData=[0]*6+[4]+[0]*7,
                gearbox=dict(gearMode=4,vehicleMoving=False))),desiredVelocity=0.)

    def test_park_changes_control_state_only(self):
        actor=self.actor();before=copy.deepcopy(actor);state.park(actor)
        self.assertEqual(actor['initPosition']['pos'],before['initPosition']['pos'])
        self.assertEqual(actor['initPosition']['rot'],before['initPosition']['rot'])
        self.assertEqual(actor['initPosition']['gear'],1)
        self.assertEqual(actor['initPosition']['vehicleSaveData']['inputData'][6],1)
        self.assertEqual(actor['initPosition']['vehicleSaveData']['gearbox']['gearMode'],1)
        state.validate_serialized(dict(runtime_actors=[dict(unique_id=1,source_list='vehicleList',
            stationary_vehicle_policy=state.POLICY)]),dict(vehicleList=[actor]),state.POLICY)

    def metadata(self,offset=0.):
        obj=dict(unique_id=1,position=dict(x=1.+offset,y=2.,z=3.))
        return dict(runtime_actors=[dict(unique_id=1,name='GV80',position=dict(x=1.,y=2.,z=3.),
            stationary_vehicle_policy=state.POLICY)],capture_runtime_objects=[dict(npc_list=[obj]),dict(npc_list=[obj])])

    def test_runtime_pose_checks_do_not_assume_message_velocity_units(self):
        self.assertFalse(state.runtime_review(self.metadata(.005))[0])
        self.assertIn('stationary_vehicle_moved_from_placement',state.runtime_review(self.metadata(5.3))[0])
        meta=self.metadata();meta['capture_runtime_objects']=[]
        self.assertIn('stationary_vehicle_runtime_pose_unavailable',state.runtime_review(meta)[0])
        self.assertEqual(state.runtime_review(dict(runtime_actors=[])),([],None))

    def test_nonfinite_pose_and_inconsistent_saved_gear_rejected(self):
        self.assertIn('stationary_vehicle_moved_from_placement',state.runtime_review(self.metadata(float('nan')))[0])
        actor=self.actor();state.park(actor);actor['initPosition']['vehicleSaveData']['gearbox']['gearMode']=4
        with self.assertRaisesRegex(ValueError,'gear selectors'):
            state.validate_serialized(dict(runtime_actors=[dict(unique_id=1,source_list='vehicleList',
                stationary_vehicle_policy=state.POLICY)]),dict(vehicleList=[actor]),state.POLICY)
