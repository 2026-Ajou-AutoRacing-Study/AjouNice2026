import importlib.util
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

SPEC=importlib.util.spec_from_file_location('trial_timing',Path(__file__).with_name('analyze_traffic_light_trial_timing.py'))
m=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(m)
NS='/traffic_light_trial/traffic_light_node'


def packet(stamp=1000000000,group=201290,status='ready',mono=10.,generated=1020000000):
    return {'topic':'/traffic_light_trial/map_rois','receipt_ros_ns':1030000000,'receipt_monotonic':mono,
       'parsed_debug':{'source_stamp_ns':stamp,'generated_stamp_ns':generated,'group_id':group,
                       'status':status,'reason':'projected' if status=='ready' else 'TF_missing',
                       'pose_image_delta_sec':.01,'rois':[{'eligible':True}]}}


def selection(stamp=1000000000,group=201290,status='matched',mono=10.3):
    return {'topic':NS+'/debug_selection','receipt_ros_ns':1300000000,'receipt_monotonic':mono,
       'parsed_debug':{'source_stamp_ns':stamp,'group_id':group,'status':status,
                       'reason':'map_head_association' if status=='matched' else 'map_packet_missing_or_stale',
                       'mode':'map_association'}}


def capture_manifest(path, rows=None):
    rows = rows if rows is not None else [
        {'environment':'Sunny13','group_regulatory_id':201290,'group_id':'C_val_pose',
         'header_stamp':{'secs':1,'nsecs':0},'native_before':'NEVER_READ_PHASE'},
        {'environment':'Sunny13','group_regulatory_id':201290,'group_id':'C_val_pose',
         'header_stamp':{'secs':3,'nsecs':0},'native_after':'NEVER_READ_PHASE'}]
    path.write_text(json.dumps({'records':rows}))
    return path


class TimingTests(unittest.TestCase):
    def test_quantiles_empty_and_linear_interpolation(self):
        self.assertIsNone(m.quantiles([])['p99'])
        self.assertEqual(m.quantiles([0.,1.,2.])['p50'],1.)
        self.assertAlmostEqual(m.quantiles([0.,1.,2.])['p95'],1.9)

    def test_exact_stamp_group_join_never_nearest_receipt(self):
        a=m.TimingAnalysis();a.add(selection());a.add(packet(stamp=1000000001));a.add(packet(group=201351))
        r=a.report();self.assertEqual(r['exact_join_coverage_recorded_selection_keys'],0.)
        a.add(packet());r=a.report();self.assertEqual(r['exact_join_coverage_recorded_selection_keys'],1.)
        self.assertEqual(r['exact_join_counts']['helper_observed_not_after_selection'],1)
        self.assertAlmostEqual(r['latency_sec']['helper_observer_arrival_minus_selection_observer_arrival_signed_monotonic']['p50'],-.3)

    def test_logger_throttle_is_not_consumer_missing_inference(self):
        a=m.TimingAnalysis();a.add(selection(status='missing'))
        r=a.report(logger_summary={'throttled_by_topic':{NS+'/debug_selection':20}})
        self.assertEqual(r['selection_reason_counts_recorded_events']['map_packet_missing_or_stale'],1)
        self.assertTrue(any('NOT proof' in value for value in r['warnings']))
        self.assertTrue(any('throttling occurred' in value for value in r['warnings']))
        self.assertFalse(r['node_reported_callback_wall_timing']['available'])

    def test_conflicting_duplicate_packets_are_explicit_not_hidden(self):
        a=m.TimingAnalysis();a.add(selection());a.add(packet());a.add(packet(status='missing'))
        r=a.report();self.assertEqual(r['duplicate_key_counts']['helper'],1)
        self.assertEqual(r['conflicting_duplicates']['joined_packet_keys_with_conflicting_status_or_reason'],1)
        self.assertEqual(r['exact_join_counts']['has_recorded_ready_packet'],1)

    def test_negative_ros_clock_delays_not_clamped_into_valid_percentiles(self):
        a=m.TimingAnalysis();a.add(packet(generated=900000000))
        stats=a.report()['latency_sec']['helper_source_to_generated_ROS_clock_assumption']
        self.assertEqual(stats['negative_count'],1)
        self.assertEqual(stats['nonnegative_sec']['count'],0)
        self.assertAlmostEqual(stats['negative_sec']['p50'],-.1)

    def test_old_headerless_signal_stamp_and_runtime_timing_no_accuracy_fields(self):
        a=m.TimingAnalysis();a.add(selection())
        a.add({'topic':'/traffic_light_trial/traffic_signals','receipt_ros_ns':1400000000,'receipt_monotonic':10.4,
               'message':{'stamp':{'sec':1,'nanosec':0},'traffic_light_groups':[{'traffic_light_group_id':201290,'elements':'NEVER_SCORE_LAMPS'}]}})
        a.add({'topic':NS+'/debug_runtime','receipt_ros_ns':1400000000,'receipt_monotonic':10.4,
               'parsed_debug':{'source_stamp_ns':1000000000,'frame_decision':{'group_id':201290,'final_tuples':'NEVER_SCORE_LAMPS'},
                               'frame_timing':{'callback_to_output_sec':.083,'selection_including_map_wait_sec':.03}}})
        r=a.report();self.assertEqual(r['exact_join_counts']['exact_signal_output_key'],1)
        self.assertEqual(r['exact_join_counts']['exact_runtime_key'],1)
        self.assertAlmostEqual(r['node_reported_callback_wall_timing']['fields']['selection_including_map_wait_sec']['nonnegative_sec']['p95'],.03)
        self.assertNotIn('NEVER_SCORE_LAMPS',json.dumps(r))

    def test_zero_and_malformed_stamps_are_unkeyed(self):
        a=m.TimingAnalysis();a.add(selection(stamp=0));a.add(packet(stamp=True))
        r=a.report();self.assertEqual(r['selection_unique_source_group_keys'],0)
        self.assertEqual(r['unkeyed_or_wrong_mode'][NS+'/debug_selection'],1)

    def test_streamed_input_hash_invalid_lines_and_clock_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'data.jsonl';path.write_text(json.dumps(packet())+'\nnot_json\n'+json.dumps(selection())+'\n')
            manifest={'timestamp_contracts':{'common_source_and_receipt_ros_clock_independently_verified':False}}
            r=m.analyze(path,metadata=manifest)
            self.assertEqual(r['input']['lines'],3);self.assertEqual(r['input']['invalid_records'],1)
            self.assertEqual(len(r['input']['sha256']),64)
            self.assertFalse(r['clock_manifest']['common_source_and_receipt_ros_clock_independently_verified'])

    def test_capture_window_inclusive_bounds_all_intermediate_events_and_missing(self):
        with tempfile.TemporaryDirectory() as directory:
            path=capture_manifest(Path(directory)/'manifest.json')
            a=m.CaptureWindowAnalysis(path)
            for stamp in (999999999,1000000000,2000000000,3000000000,3000000001):
                a.add(selection(stamp=stamp,status='missing' if stamp==2000000000 else 'matched'))
                a.add(packet(stamp=stamp,status='missing' if stamp==2000000000 else 'ready'))
            r=a.report();window=r['by_environment_and_group'][0]
            self.assertEqual(window['first_source_header_stamp_ns'],1000000000)
            self.assertEqual(window['last_source_header_stamp_ns'],3000000000)
            self.assertEqual(window['capture_record_count'],2)
            self.assertEqual(window['analysis']['selection_unique_source_group_keys'],3)
            self.assertEqual(window['analysis']['selection_status_counts'],{'matched':2,'missing':1})
            self.assertEqual(window['analysis']['helper_status_counts_recorded_events'],{'ready':2,'missing':1})
            self.assertEqual(r['filter_counts_recorded_events_by_topic'][NS+'/debug_selection']['outside_source_windows'],2)
            self.assertFalse(r['native_color_or_model_correctness_used_for_filtering'])
            self.assertFalse(r['capture_protocol']['closed_loop_certification'])
            self.assertNotIn('NEVER_READ_PHASE',json.dumps(r))

    def test_capture_window_same_group_required_not_receipt_or_near_source(self):
        with tempfile.TemporaryDirectory() as directory:
            a=m.CaptureWindowAnalysis(capture_manifest(Path(directory)/'manifest.json'))
            a.add(selection(stamp=2000000000,group=201351))
            a.add(selection(stamp=2000000000,group=0))
            row=selection(stamp=3000000001);row['receipt_ros_ns']=2000000000;a.add(row)
            row=selection(stamp=1000000000);row['receipt_ros_ns']=9999999999;a.add(row)
            a.add(selection(stamp=0))
            r=a.report();self.assertEqual(r['aggregate']['selection_unique_source_group_keys'],1)
            counts=r['filter_counts_recorded_events_by_topic'][NS+'/debug_selection']
            self.assertEqual(counts['source_in_window_but_group_mismatch'],2)
            self.assertEqual(counts['outside_source_windows'],1)
            self.assertEqual(counts['unkeyed_relevant_events'],1)

    def test_capture_window_filters_runtime_cost_and_typed_signal_groups(self):
        with tempfile.TemporaryDirectory() as directory:
            a=m.CaptureWindowAnalysis(capture_manifest(Path(directory)/'manifest.json'))
            for stamp,timing in ((1000000000,.1),(4000000000,99.)):
                a.add({'topic':NS+'/debug_runtime','parsed_debug':{'source_stamp_ns':stamp,
                       'frame_decision':{'group_id':201290},'frame_timing':{'callback_to_output_sec':timing}}})
            a.add(selection())
            a.add({'topic':'/traffic_light_trial/traffic_signals','receipt_ros_ns':1500000000,
                   'message':{'stamp':{'sec':1,'nanosec':0},'traffic_light_groups':[
                       {'traffic_light_group_id':201290,'elements':'NEVER_SCORE'},
                       {'traffic_light_group_id':201351,'elements':'NEVER_SCORE'}]}})
            r=a.report()['aggregate']
            self.assertEqual(r['exact_join_counts']['exact_signal_output_key'],1)
            self.assertEqual(r['node_reported_callback_wall_timing']['fields']['callback_to_output_sec']['nonnegative_sec']['count'],1)
            self.assertAlmostEqual(r['node_reported_callback_wall_timing']['fields']['callback_to_output_sec']['nonnegative_sec']['p99'],.1)
            self.assertEqual(len(a.aggregate.signals),1)
            self.assertNotIn('NEVER_SCORE',json.dumps(r))

    def test_capture_windows_by_environment_and_group_and_overlap_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            rows=[{'environment':env,'group_regulatory_id':group,'header_stamp':{'secs':sec,'nsecs':0}}
                  for env,group,sec in [('Sunny13',201290,1),('Sunny13',201290,3),
                                        ('Foggy15',201290,5),('Foggy15',201290,7),
                                        ('Sunny13',201351,1),('Sunny13',201351,3)]]
            path=capture_manifest(Path(directory)/'manifest.json',rows);a=m.CaptureWindowAnalysis(path)
            a.add(selection(stamp=2000000000));a.add(selection(stamp=6000000000));a.add(selection(stamp=2000000000,group=201351))
            r=a.report();self.assertEqual(len(r['by_environment_and_group']),3)
            self.assertEqual(r['aggregate']['selection_unique_source_group_keys'],3)
            self.assertEqual([w['analysis']['selection_unique_source_group_keys'] for w in r['by_environment_and_group']],[1,1,1])
            rows.append({'environment':'Foggy15','group_regulatory_id':201290,'header_stamp':{'secs':3,'nsecs':0}})
            capture_manifest(path,rows)
            with self.assertRaisesRegex(ValueError,'Overlapping'):m.load_capture_windows(path)

    def test_capture_manifest_invalid_metadata_fail_not_drop_unfavorable_frames(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'manifest.json'
            for row in ({'environment':'Sunny13','group_id':'pose','header_stamp':{'secs':1,'nsecs':0}},
                        {'environment':'Sunny13','group_regulatory_id':True,'header_stamp':{'secs':1,'nsecs':0}},
                        {'environment':'Sunny13','group_regulatory_id':201290,'header_stamp':{'secs':1,'nsecs':1000000000}},
                        {'environment':'Sunny13','group_regulatory_id':201290,'header_stamp':{'secs':0,'nsecs':0}}):
                capture_manifest(path,[row])
                with self.assertRaises(ValueError):m.load_capture_windows(path)
            capture_manifest(path,[])
            with self.assertRaises(ValueError):m.load_capture_windows(path)

    def test_capture_option_keeps_whole_report_and_manifest_pin_without_mutation(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'data.jsonl';manifest=capture_manifest(Path(directory)/'manifest.json')
            raw=manifest.read_bytes()
            rows=[selection(stamp=500000000),selection(stamp=1000000000),selection(stamp=2000000000,status='missing')]
            path.write_text(''.join(json.dumps(row)+'\n' for row in rows))
            whole=m.analyze(path);scoped=m.analyze(path,capture_manifest=manifest)
            self.assertNotIn('capture_window_analysis',whole)
            self.assertEqual(whole,{key:value for key,value in scoped.items() if key!='capture_window_analysis'})
            self.assertEqual(scoped['selection_unique_source_group_keys'],3)
            capture=scoped['capture_window_analysis']
            self.assertEqual(capture['aggregate']['selection_unique_source_group_keys'],2)
            self.assertEqual(capture['manifest']['sha256'],hashlib.sha256(raw).hexdigest())
            self.assertEqual(manifest.read_bytes(),raw)


if __name__=='__main__':unittest.main()
