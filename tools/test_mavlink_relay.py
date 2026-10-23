#!/usr/bin/env python3
from pathlib import Path
import sys
import unittest
import errno
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tools.mavlink_relay import Routes,forward,receive


class RelayTests(unittest.TestCase):
    def setUp(self):self.r=Routes('10.41.10.2','192.168.1.123',14550);self.data=b'\xfdexample'
    def test_fc_telemetry_to_configured_ground(self):
        self.assertEqual(self.r.destination(self.data,('10.41.10.2',14550),'10.41.10.100',10),('192.168.1.123',14550))
    def test_other_ground_or_local_old_process_cannot_inject_or_change_route(self):
        for ip in ('192.168.1.124','10.41.10.100','127.0.0.1'):
            self.assertIsNone(self.r.destination(self.data,(ip,15000),None,10))
            self.assertEqual(self.r.gcs,('192.168.1.123',14550))
    def test_configured_ground_return_port_can_be_learned(self):
        self.assertEqual(self.r.destination(self.data,('192.168.1.123',45000),None,10),('10.41.10.2',14550))
        self.assertEqual(self.r.gcs,('192.168.1.123',45000))
    def test_duplicate_broadcast_and_recovery(self):
        self.assertIsNotNone(self.r.destination(self.data,('10.41.10.2',14550),'10.41.10.100',10))
        self.assertIsNone(self.r.destination(self.data,('10.41.10.2',14550),'10.41.10.255',11))
        self.assertIsNotNone(self.r.destination(self.data,('10.41.10.2',14550),'10.41.10.255',14))
    def test_non_mavlink_does_not_change_ground(self):
        for data in (b'',b'noise'):
            self.assertIsNone(self.r.destination(data,('192.168.1.123',45000),None,10))
            self.assertEqual(self.r.gcs,('192.168.1.123',14550))
    def test_offline_ground_or_full_queue_drops_without_retry(self):
        for error in (TimeoutError('offline'),BlockingIOError('queue full'),OSError('network unreachable')):
            stream=Mock();stream.sendto.side_effect=error
            self.assertFalse(forward(stream,self.data,('192.168.1.123',14550)))
            stream.sendto.assert_called_once()
    def test_successful_forward_and_partial_send_result(self):
        stream=Mock();stream.sendto.return_value=len(self.data)
        self.assertTrue(forward(stream,self.data,('10.41.10.2',14550)))
        stream.sendto.return_value=0
        self.assertFalse(forward(stream,self.data,('10.41.10.2',14550)))
    def test_async_network_error_does_not_kill_relay_but_bad_fd_does(self):
        stream=Mock();stream.recvmsg.side_effect=OSError(errno.ECONNREFUSED,'GCS offline')
        self.assertIsNone(receive(stream))
        stream.recvmsg.side_effect=OSError(errno.EBADF,'invalid socket')
        with self.assertRaises(OSError):receive(stream)


if __name__=='__main__':unittest.main()
