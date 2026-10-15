#!/usr/bin/env python3
import unittest
import copy

from audit_base5_offline_tcp_framing import MODES, client_protocol, server_protocol, split_sender, statistics


class BufferSocket:
    def __init__(self):
        self.parts=[]

    def sendall(self,value):
        self.parts.append(value)


class FramingCandidateTests(unittest.TestCase):
    def test_every_sender_preserves_exact_wire_bytes(self):
        metadata=dict(sequence=1,point_dim=4,stamp=123.456)
        blobs={'points':b'xyz'*90,'image/camera_front':b'\xff\xd8\x01\xff\xd9'}
        reference=client_protocol.encode_message(client_protocol.TYPE_REQUEST,metadata,blobs)
        for protocol in (client_protocol,server_protocol):
            for mode in MODES:
                sock=BufferSocket();split_sender(protocol,mode)(sock,protocol.TYPE_REQUEST,metadata,blobs)
                self.assertEqual(b''.join(sock.parts),reference)

    def test_current_sender_already_one_fully_coalesced_send(self):
        sock=BufferSocket();split_sender(client_protocol,'current_one_write')(sock,1,{'seq':1},{'p':b'abc'})
        self.assertEqual(len(sock.parts),1)

    def test_prefix_candidate_two_writes_with_large_payload(self):
        sock=BufferSocket();split_sender(client_protocol,'prefix_coalesced_two_writes')(sock,1,{'seq':1},{'p':b'abc'})
        self.assertEqual(len(sock.parts),2)
        self.assertGreater(len(sock.parts[0]),client_protocol.HEADER.size)

    def test_adverse_control_splits_tiny_header(self):
        sock=BufferSocket();split_sender(client_protocol,'split_header_adverse_control')(sock,1,{'seq':1},{'p':b'abc'})
        self.assertEqual(len(sock.parts),3)
        self.assertEqual(len(sock.parts[0]),client_protocol.HEADER.size)

    def test_empty_response_candidate_has_no_empty_send(self):
        sock=BufferSocket();split_sender(server_protocol,'prefix_coalesced_two_writes')(sock,2,{'detections':[]})
        self.assertEqual(len(sock.parts),1)

    def test_unknown_mode_is_not_silently_treated_as_coalesced(self):
        with self.assertRaises(ValueError):
            split_sender(client_protocol,'unknown_mode')

    def test_message_metadata_and_blob_sources_not_modified(self):
        metadata=dict(sequence=1,cameras=['front','back']);blobs={'points':b'xyz'}
        saved=copy.deepcopy((metadata,blobs))
        for mode in MODES:
            split_sender(client_protocol,mode)(BufferSocket(),1,metadata,blobs)
        self.assertEqual((metadata,blobs),saved)

    def test_byte_statistics_do_not_contain_millisecond_threshold(self):
        values=[517893,604719,650181]
        byte_stats=statistics(values,timing=False)
        self.assertNotIn('over30ms',byte_stats)
        self.assertEqual(byte_stats['max'],650181.)
        self.assertEqual(byte_stats['n'],3)
        self.assertEqual(statistics([.2,44.],timing=True)['over30ms'],1)


if __name__=='__main__':
    unittest.main()
