"""No master/GPU: actual bridge class, fake publishers/transport, fixed clocks."""
import collections
from concurrent.futures import ThreadPoolExecutor
import copy
import importlib.util
import io
import json
from pathlib import Path
import threading
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField, CompressedImage
from bevfusion_ros.frame_timing import FrameTimingLedger

PACKAGE=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('actual_frame_timing_bridge',PACKAGE/'scripts/bevfusion_ros_bridge.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Publisher:
    def __init__(self,fail=False):self.messages=[];self.fail=fail
    def publish(self,message):
        if self.fail:raise RuntimeError('diagnostic publisher failed')
        self.messages.append(message)


class Transport:
    def __init__(self,fail=False):self.requests=[];self.fail=fail
    def request(self,metadata,blobs):
        self.requests.append((copy.deepcopy(metadata),dict(blobs)))
        if self.fail:raise RuntimeError('transport failed')
        return {'operation':'inference','frame_id':'lidar1','detections':[
            dict(class_name='vehicle',score=.9,center=[5.,1.,1.],size=[4.6,1.9,1.5],yaw=.2)]},{}


def wire(message):
    out=io.BytesIO();message.serialize(out);return out.getvalue()


def bridge(enabled):
    obj=module.BevFusionRosBridge.__new__(module.BevFusionRosBridge)
    obj.frame_timing=FrameTimingLedger(clock=lambda:10.) if enabled else None
    obj.frame_timing_pub=Publisher()
    obj.lidar_deskew_mode='off';obj.sync_tolerance=.05
    obj.camera_order=list(module.DEFAULT_CAMERA_ORDER)
    obj.camera_buffers={name:collections.deque(maxlen=30)for name in obj.camera_order}
    obj.buffer_lock=threading.Lock();obj.pipeline_condition=threading.Condition()
    obj.raw_pending=None;obj.prepared_pending=None;obj.sequence=0;obj.shutting_down=False
    obj.dropped_raw=0;obj.dropped_prepared=0;obj.dropped_deskew_timing=0;obj.dropped_deskew_pose=0
    obj.temporal_sweeps=0;obj.temporal_feature_cache=False;obj.query_memory_context=False
    obj.operation='inference';obj.image_encoding='compressed';obj.point_intensity_source='ros255'
    obj.sensor_config_sha256='fixed';obj.intrinsic_source='horizontal_fov'
    obj.image_final_height=512;obj.image_final_width=1056;obj.model_score_threshold=.1;obj.max_detections=100
    obj.score_threshold=.1;obj.class_score_thresholds={'vehicle':.4,'pedestrian':.1,'obstacle':.1}
    obj.rotated_bev_nms_enabled=True;obj.rotated_bev_nms_iou_threshold=.5;obj.nms_suppressed_total=0
    obj.publish_detector_velocity=False;obj.detector_velocity_noise_std_mps=1.;obj.detector_maximum_velocity_mps=60.
    obj.marker_lifetime=.5;obj.last_marker_count=0;obj.query_memory_diagnostics_handle=None
    obj.transport=Transport();obj.objects_pub=Publisher();obj.marker_pub=Publisher();obj.latency_pub=Publisher()
    obj._publish_status=lambda _:None
    return obj


def cloud():
    msg=PointCloud2(height=1,width=2,point_step=16,row_step=32)
    msg.header.stamp=rospy.Time.from_sec(100.);msg.header.frame_id='lidar1'
    msg.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
                for i,name in enumerate(('x','y','z','intensity'))]
    msg.data=np.asarray([[1.,2.,3.,255.],[4.,5.,6.,128.]],dtype='<f4').tobytes()
    return msg


class FrameTimingTest(unittest.TestCase):
    def setUp(self):
        clock_patch=patch.object(module.rospy.Time,'now',return_value=rospy.Time(100,100_000_000))
        clock_patch.start();self.addCleanup(clock_patch.stop)

    def test_actual_bridge_off_on_raw_jpeg_xyzi_metadata_and_output_are_identical(self):
        outcomes=[]
        for enabled in (False,True):
            obj=bridge(enabled);msg=cloud()
            for name in obj.camera_order:
                image=CompressedImage(format='jpeg',data=b'original-jpeg-'+name.encode())
                image.header.stamp=msg.header.stamp
                obj._camera_callback(image,name)
            with patch.object(module.time,'monotonic',return_value=20.), \
                 patch.object(module.rospy.Time,'now',return_value=rospy.Time.from_sec(100.1)):
                obj._lidar_callback(msg)
                self.assertEqual(len(obj.raw_pending),7)
                frame=obj.raw_pending;obj.raw_pending=None
                if enabled:obj.frame_timing.stage(frame[-1],'preprocess_start')
                prepared=obj._prepare_frame(*frame)
                self.assertEqual(len(prepared),4)
                if enabled:
                    obj.frame_timing.stage(frame[-1],'preprocess_end')
                    obj.frame_timing.stage(frame[-1],'prepared_enqueued')
                obj._request_and_publish(prepared)
            outcomes.append((obj.transport.requests,[wire(m)for m in obj.objects_pub.messages],
                             [wire(m)for m in obj.marker_pub.messages],[wire(m)for m in obj.latency_pub.messages]))
            blobs=prepared[1]
            self.assertEqual(blobs['points'],msg.data)
            for name in obj.camera_order:
                self.assertEqual(blobs['image/'+name],b'original-jpeg-'+name.encode())
            if enabled:
                record=json.loads(obj.frame_timing_pub.messages[0].data)
                self.assertEqual(record['input_stamp_ns'],100_000_000_000)
                self.assertEqual(record['output_stamp_ns'],100_000_000_000)
                self.assertAlmostEqual(record['callback_input_header_age_ms'],100.,places=5)
                self.assertEqual(len(obj.frame_timing.frames),0)
                self.assertEqual(len(obj.frame_timing.active),0)
            else:self.assertEqual(obj.frame_timing_pub.messages,[])
        self.assertEqual(outcomes[0],outcomes[1])

    def test_legacy_latency_remains_post_selection_received_to_response_prepublication(self):
        values=[]
        for enabled in (False,True):
            obj=bridge(enabled)
            if enabled:
                ticket=obj.frame_timing.begin(100_000_000_000,'lidar1',100_010_000_000)
                obj.frame_timing.accept(ticket,10.,100.,{})
            prepared=({'sequence':1}, {},rospy.Time.from_sec(100.),10.)
            with patch.object(module.time,'monotonic',side_effect=[20.,22.,23.]), \
                 patch.object(module.rospy.Time,'now',return_value=rospy.Time.from_sec(100.1)):
                obj._request_and_publish(prepared)
            values.append(obj.latency_pub.messages[0].data)
        self.assertEqual(values,[13000.,13000.])

    def test_camera_rejects_count_without_retaining_or_changing_queue(self):
        obj=bridge(True);msg=cloud()
        obj._lidar_callback(msg)
        self.assertEqual(obj.frame_timing.counters['camera_empty_reject'],1)
        self.assertIsNone(obj.raw_pending);self.assertFalse(obj.frame_timing.frames)
        for name in obj.camera_order:
            image=CompressedImage();image.header.stamp=rospy.Time.from_sec(90.)
            obj._camera_callback(image,name)
        with patch.object(module.rospy,'logwarn_throttle'):
            obj._lidar_callback(msg)
        self.assertEqual(obj.frame_timing.counters['camera_sync_reject'],1)
        self.assertIsNone(obj.raw_pending);self.assertFalse(obj.frame_timing.frames)

    def test_transport_error_cleans_sidecar_without_replacing_exception(self):
        obj=bridge(True);obj.transport=Transport(fail=True)
        obj.frame_timing.accept(obj.frame_timing.begin(1,'lidar1',2),10.,100.,{})
        with self.assertRaisesRegex(RuntimeError,'transport failed'):
            obj._request_and_publish(({'sequence':1},{},rospy.Time.from_sec(100.),10.))
        self.assertFalse(obj.frame_timing.frames);self.assertFalse(obj.frame_timing.active)
        self.assertEqual(obj.frame_timing.counters['request_error'],1)

    def test_raw_latest_only_overwrite_releases_only_replaced_ticket(self):
        obj=bridge(True);msg=cloud()
        for name in obj.camera_order:
            image=CompressedImage();image.header.stamp=msg.header.stamp
            obj._camera_callback(image,name)
        with patch.object(module.time,'monotonic',side_effect=[20.,21.]):
            obj._lidar_callback(msg);obj._lidar_callback(msg)
        self.assertEqual(obj.dropped_raw,1)
        self.assertEqual(obj.frame_timing.counters['raw_overwrite'],1)
        self.assertEqual(list(obj.frame_timing.frames),[21.])
        self.assertEqual(obj.raw_pending[-1],21.)

    def test_prepared_overwrite_and_preprocess_error_keep_existing_queue_policy(self):
        for fail in (False,True):
            obj=bridge(True)
            for key in (20.,21.):
                obj.frame_timing.accept(obj.frame_timing.begin(1,'lidar1',2),key,100.,{})
            obj.raw_pending=(None,None,None,None,None,100.,21.)
            old_prepared=({}, {},rospy.Time.from_sec(100.),20.)
            obj.prepared_pending=old_prepared
            def prepare(*frame):
                obj.shutting_down=True
                if fail:raise ValueError('controlled preprocess failure')
                return {'sequence':2}, {},rospy.Time.from_sec(100.),frame[-1]
            obj._prepare_frame=prepare
            with patch.object(module.rospy,'is_shutdown',return_value=False), \
                 patch.object(module.rospy,'logwarn_throttle'):
                obj._preprocess_loop()
            if fail:
                self.assertIs(obj.prepared_pending,old_prepared)
                self.assertEqual(obj.frame_timing.counters['preprocess_error'],1)
                self.assertEqual(list(obj.frame_timing.frames),[20.])
            else:
                self.assertEqual(obj.prepared_pending[-1],21.)
                self.assertEqual(obj.dropped_prepared,1)
                self.assertEqual(obj.frame_timing.counters['prepared_overwrite'],1)
                self.assertEqual(list(obj.frame_timing.frames),[21.])

    def test_diagnostic_error_does_not_reject_successful_detection(self):
        obj=bridge(True);obj.frame_timing_pub=Publisher(fail=True)
        obj.frame_timing.accept(obj.frame_timing.begin(1,'lidar1',2),10.,100.,{})
        with patch.object(module.rospy,'logwarn_throttle'):
            obj._request_and_publish(({'sequence':1},{},rospy.Time.from_sec(100.),10.))
        self.assertEqual(len(obj.objects_pub.messages),1)
        self.assertEqual(obj.frame_timing.counters['diagnostic_error'],1)
        self.assertFalse(obj.frame_timing.frames);self.assertFalse(obj.frame_timing.active)

    def test_ledger_bounded_drop_collision_shutdown_and_thread_bindings(self):
        ledger=FrameTimingLedger(capacity=4,clock=lambda:1.)
        for i in range(100):ledger.accept(ledger.begin(i,'lidar1',i),float(i),float(i),{})
        self.assertEqual(len(ledger.frames),4);self.assertEqual(ledger.counters['overflow'],96)
        ledger.discard(99.,'raw_overwrite');ledger.discard(98.,'prepared_overwrite')
        self.assertEqual(ledger.counters['raw_overwrite'],1);self.assertEqual(ledger.counters['prepared_overwrite'],1)
        self.assertFalse(ledger.accept(ledger.begin(1,'lidar1',1),97.,1.,{}))
        self.assertEqual(ledger.counters['key_collision'],1)
        def task(i):
            key=float(100+i);ledger.accept(ledger.begin(i,'lidar1',i),key,key,{})
            ledger.bind_request(key);ledger.output_published(i);ledger.discard(key,'request_error');ledger.unbind_request()
        with ThreadPoolExecutor(max_workers=4)as pool:list(pool.map(task,range(100)))
        self.assertLessEqual(len(ledger.frames),4);self.assertFalse(ledger.active)
        ledger.clear();self.assertFalse(ledger.frames)

    def test_launch_flag_opt_in_and_trial_namespace_remap(self):
        for name in ('base3_online','base5_production','base5_trial','base5_e24_calibrated_trial','base5_e9_calibrated_trial'):
            root=ET.parse(PACKAGE/'launch'/(name+'.launch')).getroot()
            args={item.attrib['name']:item.attrib.get('default')for item in root.findall('arg')}
            self.assertEqual(args['publish_frame_timing'],'false')
        root=ET.parse(PACKAGE/'launch/base5_trial.launch').getroot()
        remaps={item.attrib['from']:item.attrib['to']for item in root.findall('group/remap')}
        self.assertEqual(remaps['/perception/bevfusion/frame_timing'],'/perception/base5_trial/frame_timing')


if __name__=='__main__':unittest.main()
