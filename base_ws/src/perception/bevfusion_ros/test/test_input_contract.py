"""Host ROS tests; run with the normal Noetic/AW overlay, no ROS master."""
import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField, CompressedImage

path=Path(__file__).resolve().parents[1]/'scripts/bevfusion_ros_bridge.py'
spec=importlib.util.spec_from_file_location('actual_bridge_input_test',path)
bridge_module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge_module)


class InputContractTest(unittest.TestCase):
    def test_organized_padding_and_big_endian_preserve_xyzi(self):
        for endian in ('<','>'):
            cloud=PointCloud2(height=2,width=1,point_step=16,row_step=32,is_bigendian=endian=='>')
            cloud.fields=[PointField(name=name,offset=index*4,datatype=PointField.FLOAT32,count=1)
                          for index,name in enumerate(('x','y','z','intensity'))]
            values=np.asarray([[1,2,3,255],[4,5,6,128]],dtype=endian+'f4')
            cloud.data=values[0].tobytes()+bytes(16)+values[1].tobytes()+bytes(16)
            np.testing.assert_array_equal(bridge_module.pointcloud_to_xyzi(cloud),values)

    def test_configurable_crop_and_legacy_size(self):
        instance=bridge_module.BevFusionRosBridge.__new__(bridge_module.BevFusionRosBridge)
        instance.camera_shapes={'front':(720,1280)}
        instance.image_reference_width=1280.
        image=np.full((720,1280,3),125,dtype=np.uint8)
        ok,data=cv2.imencode('.jpg',image)
        self.assertTrue(ok)
        message=CompressedImage(data=data.tobytes(),format='jpeg')
        for h,w,scale in ((256,704,.55),(512,1056,.825)):
            instance.image_final_height=h;instance.image_final_width=w;instance.image_resize=scale
            frame,result=instance._preprocess_camera(('front',message))
            self.assertEqual(result.shape,(h,w,3));self.assertEqual(frame,'front')
        instance.image_final_height=1024
        with self.assertRaisesRegex(ValueError,'Resize cannot'):instance._preprocess_camera(('front',message))

    def test_production_launch_is_not_replaced(self):
        root=ET.parse(path.parents[1]/'launch/base3_online.launch').getroot()
        args={item.attrib['name']:item.attrib.get('default') for item in root.findall('arg')}
        self.assertEqual(args['vehicle_score_threshold'],'0.40')
        self.assertEqual(args['pedestrian_score_threshold'],'0.15')
        self.assertEqual(args['obstacle_score_threshold'],'0.10')
        self.assertEqual(args['output_objects'],'/perception/object_recognition/detection/objects')
        trial=ET.parse(path.parents[1]/'launch/base5_trial.launch').getroot()
        remaps={r.attrib['from']:r.attrib['to'] for r in trial.findall('group/remap')}
        for name in ('markers','status','latency_ms'):
            self.assertEqual(remaps['/perception/bevfusion/'+name],'/perception/base5_trial/'+name)

    def test_e24_calibrated_trial_is_named_opt_in_and_keeps_baseline(self):
        launches=path.parents[1]/'launch'
        root=ET.parse(launches/'base5_e24_calibrated_trial.launch').getroot()
        args={item.attrib['name']:item.attrib.get('default') for item in root.findall('arg')}
        self.assertEqual([args[n+'_score_threshold'] for n in ('vehicle','pedestrian','obstacle')],
                         ['0.40','0.10','0.10'])
        self.assertEqual(args['output_objects'],'/perception/base5_trial/detection/objects')
        original=ET.parse(launches/'base5_trial.launch').getroot()
        baseline={item.attrib['name']:item.attrib.get('default') for item in original.findall('arg')}
        self.assertEqual([baseline[n+'_score_threshold'] for n in ('vehicle','pedestrian','obstacle')],
                         ['0.40','0.15','0.10'])

    def test_e9_calibrated_trial_is_separate_and_passes_explicit_scores(self):
        root=ET.parse(path.parents[1]/'launch/base5_e9_calibrated_trial.launch').getroot()
        args={item.attrib['name']:item.attrib.get('default') for item in root.findall('arg')}
        self.assertEqual([args[n+'_score_threshold'] for n in ('vehicle','pedestrian','obstacle')],
                         ['0.30','0.20','0.12'])
        self.assertEqual(args['output_objects'],'/perception/base5_trial/detection/objects')
        included={item.attrib['name']:item.attrib['value'] for item in root.findall('include/arg')}
        for name in ('vehicle','pedestrian','obstacle'):
            self.assertEqual(included[name+'_score_threshold'],'$(arg '+name+'_score_threshold)')


if __name__=='__main__':unittest.main()
