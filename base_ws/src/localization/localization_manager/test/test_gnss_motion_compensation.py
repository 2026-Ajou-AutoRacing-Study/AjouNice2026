#!/usr/bin/env python3
"""Pure/mocked tests; no ROS master, UDP or MORAI is used."""
import importlib.util
import io
import math
from pathlib import Path
import unittest
from unittest.mock import Mock,patch

from geometry_msgs.msg import TransformStamped,TwistWithCovarianceStamped
from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import Imu

SCRIPT=Path(__file__).resolve().parents[1]/'scripts/morai_gps_to_local_odometry.py'
SPEC=importlib.util.spec_from_file_location('gps_motion_compensation',str(SCRIPT))
M=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(M)


def serialized(message):
    result=io.BytesIO();message.serialize(result);return result.getvalue()


def auxiliary(stamp=9.99,speed=10.,heading_deg=90.):
    motion=TwistWithCovarianceStamped();orientation=Imu()
    for message in [motion,orientation]:
        message.header.stamp=M.rospy.Time.from_sec(stamp)
        message.header.frame_id='base_link'
    motion.twist.twist.linear.x=speed
    theta=math.radians(heading_deg)
    orientation.orientation.z=math.sin(theta/2)
    orientation.orientation.w=math.cos(theta/2)
    return motion,orientation


class MotionDeltaTest(unittest.TestCase):
    def delta(self,motion=None,orientation=None,sec=.02,gps=10.):
        return M.motion_compensation_delta(sec,gps,motion,orientation,.1,'base_link')

    def test_zero_mode_has_no_auxiliary_requirements(self):
        self.assertEqual(self.delta(sec=0),((0.,0.),'disabled'))

    def test_corrected_map_heading_used_once(self):
        motion,orientation=auxiliary(heading_deg=90.)
        delta,reason=self.delta(motion,orientation)
        self.assertEqual(reason,'applied');self.assertAlmostEqual(delta[0],0.,places=12)
        self.assertAlmostEqual(delta[1],.2,places=12)

    def test_signed_speed_handles_reverse(self):
        m,q=auxiliary(speed=-5.,heading_deg=0.)
        self.assertEqual(self.delta(m,q),((-.1,0.),'applied'))

    def test_stopped_vehicle_not_given_position_offset(self):
        m,q=auxiliary(speed=0.)
        self.assertEqual(self.delta(m,q),((0.,0.),'applied'))

    def test_missing_auxiliary_keeps_original_position(self):
        m,q=auxiliary()
        for a,b in [(None,None),(m,None),(None,q)]:
            self.assertEqual(self.delta(a,b)[0],(0.,0.))

    def test_requires_identical_source_stamp_pair(self):
        m,q=auxiliary();q.header.stamp=M.rospy.Time.from_sec(9.98)
        self.assertEqual(self.delta(m,q)[1],'unpaired_motion_stamps')

    def test_stale_future_zero_stamps_fall_back(self):
        for stamp,reason in [(9.8,'stale_or_future_motion'),(10.1,'stale_or_future_motion'),(0.,'invalid_motion_stamp')]:
            m,q=auxiliary(stamp=stamp)
            self.assertEqual(self.delta(m,q)[1],reason)

    def test_accepts_leading_slash_but_rejects_wrong_frames(self):
        m,q=auxiliary();m.header.frame_id='/base_link';q.header.frame_id='/base_link'
        self.assertEqual(self.delta(m,q)[1],'applied')
        for message in [m,q]:
            original=message.header.frame_id;message.header.frame_id='map'
            self.assertEqual(self.delta(m,q)[1],'wrong_motion_frame');message.header.frame_id=original

    def test_rejects_nonfinite_velocity_or_quaternion(self):
        for value in [float('nan'),float('inf')]:
            m,q=auxiliary();m.twist.twist.linear.x=value
            self.assertEqual(self.delta(m,q)[1],'nonfinite_motion')
            m,q=auxiliary();q.orientation.w=value
            self.assertEqual(self.delta(m,q)[1],'nonfinite_motion')

    def test_rejects_unavailable_or_invalid_norm_orientation(self):
        m,q=auxiliary();q.orientation_covariance[0]=-1.
        self.assertEqual(self.delta(m,q)[1],'unavailable_orientation')
        for scale in [0.,2.]:
            m,q=auxiliary();q.orientation.z*=scale;q.orientation.w*=scale
            self.assertEqual(self.delta(m,q)[1],'unnormalized_orientation')

    def test_normalization_roundoff_and_quaternion_sign_do_not_change_heading(self):
        m,q=auxiliary();expected=self.delta(m,q)[0]
        q.orientation.z*=-1.0000001;q.orientation.w*=-1.0000001
        actual=self.delta(m,q)[0]
        self.assertAlmostEqual(actual[0],expected[0],places=12)
        self.assertAlmostEqual(actual[1],expected[1],places=12)

    def test_helpers_never_mutate_auxiliary_messages(self):
        m,q=auxiliary();before=serialized(m),serialized(q)
        self.delta(m,q)
        self.assertEqual(before,(serialized(m),serialized(q)))


class ConverterCompensationTest(unittest.TestCase):
    def converter(self,parameters=None):
        values={M.DEFAULT_NOISE_NS+'/gnss/horizontal_stddev_m':.1,
                M.DEFAULT_NOISE_NS+'/gnss/vertical_stddev_m':1.}
        values.update(parameters or {})
        patches=[patch.object(M.rospy,'get_param',side_effect=lambda n,d=None:values.get(n,d)),
                 patch.object(M.rospy,'Publisher'),patch.object(M.rospy,'Subscriber'),
                 patch.object(M.rospy,'loginfo'),patch.object(M.rospy,'logwarn'),
                 patch.object(M.rospy,'logwarn_throttle'),patch.object(M.rospy,'logwarn_once'),
                 patch.object(M.tf2_ros,'Buffer'),patch.object(M.tf2_ros,'TransformListener'),
                 patch.object(M.MoraiGpsToLocalOdometry,'read_gps_height',return_value=.3)]
        for p in patches:p.start();self.addCleanup(p.stop)
        converter=M.MoraiGpsToLocalOdometry()
        transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='morai_map'
        transform.transform.rotation.w=1.;transform.transform.translation.x=4.;transform.transform.translation.y=6.
        converter.tf_buffer.lookup_transform.return_value=transform
        return converter,transform

    @staticmethod
    def fix(stamp=10.):
        fix=GPSMessage();fix.header.seq=17;fix.header.stamp=M.rospy.Time.from_sec(stamp);fix.header.frame_id='morai_map'
        fix.latitude=37.24;fix.longitude=127.09;fix.altitude=29.
        fix.eastOffset=302595.;fix.northOffset=4124145.;fix.status=1
        return fix

    def test_default_zero_preserves_raw_input_and_original_output_contract(self):
        converter,_=self.converter();fix=self.fix();before=serialized(fix)
        converter.gps_callback(fix)
        output=converter.publisher.publish.call_args[0][0]
        x,y=M.wgs84_to_utm(fix.latitude,fix.longitude,52,True)
        self.assertEqual(output.pose.pose.position.x,x-fix.eastOffset+4.)
        self.assertEqual(output.pose.pose.position.y,y-fix.northOffset+6.)
        self.assertEqual(output.pose.pose.position.z,28.7)
        self.assertEqual(output.header.stamp,fix.header.stamp);self.assertEqual(output.header.seq,17)
        self.assertEqual(output.header.frame_id,'map');self.assertIsNot(output.header,fix.header)
        self.assertEqual(before,serialized(fix))
        self.assertIsNone(converter.motion_subscriber);self.assertIsNone(converter.orientation_subscriber)

    def test_enabled_changes_xy_only_after_static_map_transform(self):
        off,_=self.converter();on,tr=self.converter({'~motion_compensation_sec':.02})
        # Nonidentity map-from-morai rotation must not rotate already-map yaw twice.
        yaw=.3;tr.transform.rotation.z=math.sin(yaw/2);tr.transform.rotation.w=math.cos(yaw/2)
        off.tf_buffer.lookup_transform.return_value=tr
        m,q=auxiliary();on.motion_callback(m);on.orientation_callback(q)
        fix=self.fix();off.gps_callback(fix);baseline=off.publisher.publish.call_args[0][0]
        on.gps_callback(fix);candidate=on.publisher.publish.call_args[0][0]
        self.assertAlmostEqual(candidate.pose.pose.position.x,baseline.pose.pose.position.x,places=12)
        self.assertAlmostEqual(candidate.pose.pose.position.y,baseline.pose.pose.position.y+.2,places=12)
        self.assertEqual(candidate.pose.pose.position.z,baseline.pose.pose.position.z)
        self.assertEqual(candidate.pose.covariance,baseline.pose.covariance)
        self.assertEqual(candidate.twist,baseline.twist)
        self.assertEqual(candidate.header,baseline.header)

    def test_missing_invalid_stale_auxiliary_still_publishes_original_gps(self):
        for style in ['missing','stale','invalid']:
            with self.subTest(style=style):
                converter,_=self.converter({'~motion_compensation_sec':.02})
                m,q=auxiliary(stamp=9.8 if style=='stale' else 9.99)
                if style=='invalid':q.orientation.w=float('nan')
                if style!='missing':converter.motion_callback(m);converter.orientation_callback(q)
                fix=self.fix();converter.gps_callback(fix)
                output=converter.publisher.publish.call_args[0][0];x,y=M.wgs84_to_utm(fix.latitude,fix.longitude,52,True)
                self.assertEqual(output.pose.pose.position.x,x-fix.eastOffset+4.)
                self.assertEqual(output.pose.pose.position.y,y-fix.northOffset+6.)

    def test_gps_outage_is_still_rejected_before_compensation(self):
        converter,_=self.converter({'~motion_compensation_sec':.02});m,q=auxiliary()
        converter.motion_callback(m);converter.orientation_callback(q)
        fix=self.fix();fix.latitude=0.;fix.longitude=0.
        converter.gps_callback(fix);converter.publisher.publish.assert_not_called()

    def test_duplicate_fix_is_still_rejected(self):
        converter,_=self.converter({'~motion_compensation_sec':.02});m,q=auxiliary()
        converter.motion_callback(m);converter.orientation_callback(q)
        converter.gps_callback(self.fix());converter.gps_callback(self.fix())
        self.assertEqual(converter.publisher.publish.call_count,1)

    def test_cell_transition_mode_is_unchanged(self):
        converter,_=self.converter({'~motion_compensation_sec':.02,'~quantization_input_mode':'cell_transition'})
        m,q=auxiliary();converter.motion_callback(m);converter.orientation_callback(q)
        converter.gps_callback(self.fix(10.));converter.gps_callback(self.fix(10.02))
        self.assertEqual(converter.publisher.publish.call_count,1)

    def test_constructor_rejects_unbounded_compensation_or_age(self):
        for value in [-.01,.10001,float('inf'),float('nan')]:
            with self.assertRaises(M.rospy.ROSInitException):self.converter({'~motion_compensation_sec':value})
        for value in [0.,-.1,float('inf'),float('nan')]:
            with self.assertRaises(M.rospy.ROSInitException):self.converter({'~max_motion_age_sec':value})


if __name__=='__main__':unittest.main()
