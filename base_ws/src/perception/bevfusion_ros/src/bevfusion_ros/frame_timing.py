"""Bounded, optional timing sidecar. Never add fields to sensor/model payloads."""
import collections
import threading
import time


class FrameTimingLedger:
    COUNTERS = ('callbacks', 'accepted', 'completed', 'camera_empty_reject',
                'camera_sync_reject', 'deskew_timing_reject', 'raw_overwrite',
                'prepared_overwrite', 'preprocess_error', 'request_error',
                'overflow', 'key_collision', 'shutdown_discard', 'diagnostic_error')

    def __init__(self, capacity=16, clock=None):
        if int(capacity) < 4:
            raise ValueError('Frame timing capacity must be at least 4')
        self.capacity = int(capacity)
        self.clock = clock or time.monotonic
        self.lock = threading.Lock()
        self.frames = collections.OrderedDict()
        self.active = {}
        self.counters = dict.fromkeys(self.COUNTERS, 0)

    def count(self, reason):
        if reason not in self.counters:
            raise ValueError('Unknown diagnostic counter: '+reason)
        with self.lock:
            self.counters[reason] += 1

    def begin(self, stamp_ns, frame_id, ros_ns):
        start = self.clock()
        self.count('callbacks')
        return dict(callback_start=start, input_stamp_ns=stamp_ns,
                    input_frame_id=frame_id, callback_ros_ns=ros_ns)

    def accept(self, ticket, received_at, reference_stamp, sync_differences):
        with self.lock:
            if received_at in self.frames:
                self._remove(received_at)
                self.counters['key_collision'] += 1
                return False  # Do not misattribute either same-key frame.
            if len(self.frames) >= self.capacity:
                old_key = next(iter(self.frames))
                self._remove(old_key)
                self.counters['overflow'] += 1
            ticket = dict(ticket)
            ticket.update(raw_enqueued=received_at,
                          reference_stamp=float(reference_stamp),
                          sync_difference_sec=dict(sync_differences))
            self.frames[received_at] = ticket
            self.counters['accepted'] += 1
            return True

    def _remove(self, key):
        record = self.frames.pop(key, None)
        for thread_id in [tid for tid, value in self.active.items() if value == key]:
            del self.active[thread_id]
        return record

    def discard(self, key, reason):
        if reason not in self.counters:
            raise ValueError('Unknown diagnostic counter: '+reason)
        with self.lock:
            if self._remove(key) is not None:
                self.counters[reason] += 1

    def stage(self, key, name, when=None):
        when = self.clock() if when is None else when
        with self.lock:
            record = self.frames.get(key)
            if record is not None:
                record[name] = when

    def bind_request(self, key):
        with self.lock:
            thread_id = threading.get_ident()
            if key in self.frames:
                if thread_id not in self.active and len(self.active) >= self.capacity:
                    self.active.pop(next(iter(self.active)))
                    self.counters['overflow'] += 1
                self.active[thread_id] = key
            else:
                self.active.pop(thread_id, None)

    def unbind_request(self):
        with self.lock:
            self.active.pop(threading.get_ident(), None)

    def output_published(self, ros_ns):
        when = self.clock()
        with self.lock:
            record = self.frames.get(self.active.get(threading.get_ident()))
            if record is not None:
                record['objects_published'] = when
                record['objects_published_ros_ns'] = ros_ns

    def finish(self, key, sequence, stamp_ns, end_ros_ns, metadata, response, legacy_latency_ms):
        end = self.clock()
        with self.lock:
            record = self._remove(key)
            if record is None:
                return None
            self.counters['completed'] += 1
            counters = dict(self.counters)
        def duration(first, second):
            if first not in record or second not in record:
                return None
            return (record[second]-record[first])*1000.
        record['pipeline_end'] = end
        input_ns = record['input_stamp_ns']
        output_ros_ns = record.get('objects_published_ros_ns')
        return dict(
            schema_version=1, sequence=int(sequence), operation=response.get('operation'),
            input_stamp_ns=input_ns, output_stamp_ns=stamp_ns,
            input_frame_id=record['input_frame_id'],
            callback_ros_ns=record['callback_ros_ns'], end_ros_ns=end_ros_ns,
            callback_input_header_age_ms=(record['callback_ros_ns']-input_ns)/1.e6 if input_ns else None,
            output_header_age_at_objects_publish_ms=(output_ros_ns-stamp_ns)/1.e6 if output_ros_ns is not None and stamp_ns else None,
            end_output_header_age_ms=(end_ros_ns-stamp_ns)/1.e6 if stamp_ns else None,
            camera_selection_and_enqueue_ms=duration('callback_start','raw_enqueued'),
            raw_queue_wait_ms=duration('raw_enqueued','preprocess_start'),
            host_prepare_sidecar_ms=duration('preprocess_start','preprocess_end'),
            prepared_enqueue_delay_ms=duration('preprocess_end','prepared_enqueued'),
            prepared_queue_wait_ms=duration('prepared_enqueued','request_start'),
            request_and_server_ms=duration('request_start','response_returned'),
            postprocess_to_objects_publish_ms=duration('response_returned','objects_published'),
            callback_to_objects_publish_ms=duration('callback_start','objects_published'),
            received_at_to_objects_publish_ms=duration('raw_enqueued','objects_published'),
            response_to_pipeline_end_ms=duration('response_returned','pipeline_end'),
            legacy_latency_ms=float(legacy_latency_ms),
            legacy_latency_scope='post-camera-selection raw enqueue through response return before publication',
            host_preprocess_ms=metadata.get('host_preprocess_ms'),
            server_preprocess_ms=response.get('preprocess_ms'),
            server_inference_ms=response.get('inference_ms'),
            sync_difference_sec=record['sync_difference_sec'], counters=counters,
        )

    def clear(self):
        with self.lock:
            self.counters['shutdown_discard'] += len(self.frames)
            self.frames.clear()
            self.active.clear()
