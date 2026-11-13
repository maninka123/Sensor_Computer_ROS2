import importlib.util
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest import mock

ROOT=Path(__file__).resolve().parents[1]


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


SAMPLE=module('stream_sample',ROOT/'src/node_pc/scripts/stream_sample.py')
ACCEPTANCE=module('acceptance_stamp',ROOT/'tools/acceptance_stamp.py')


def sample(sec=100,nanosec=1,width=10,order='<'):
    data=(b'\x00\x01\x00\x00' if order=='<' else b'\x00\x00\x00\x00')
    return data+struct.pack(order+'iII',sec,nanosec,4)+b'cam\0'+struct.pack(order+'II',1,width)


class StreamHealthTests(unittest.TestCase):
    def test_geometry_and_endianness(self):
        for endian in ['<','>']:
            self.assertEqual(SAMPLE.sensor_stamp(sample(order=endian),True),(100,1))

    def test_rejects_empty_malformed_and_uninitialized_samples(self):
        for data in [b'',sample(width=0),sample(sec=0,nanosec=0),sample(nanosec=10**9),sample()[:19]]:
            self.assertIsNone(SAMPLE.sensor_stamp(data,True))

    def test_requires_advancing_samples_and_rejects_repeats(self):
        tracker=SAMPLE.FreshnessTracker(clock=lambda:10)
        tracker.observe('/cloud',sample(),True)
        self.assertNotIn('/cloud',tracker.seen)
        tracker.observe('/cloud',sample(),True)
        self.assertNotIn('/cloud',tracker.seen)
        tracker.observe('/cloud',sample(nanosec=2),True)
        self.assertEqual(tracker.seen['/cloud'],10)
        tracker.clock=lambda:20
        tracker.observe('/cloud',sample(nanosec=2),True)
        self.assertEqual(tracker.seen['/cloud'],10)

    def test_clock_reversal_counts_as_new_data(self):
        tracker=SAMPLE.FreshnessTracker(clock=lambda:10)
        tracker.observe('/cloud',sample(sec=100),True)
        tracker.observe('/cloud',sample(sec=1),True)
        self.assertEqual(tracker.seen['/cloud'],10)


class AcceptanceTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.root=Path(self.tmp.name)
        self.file=self.root/'ros2/src/node_pc/source.cpp'
        self.file.parent.mkdir(parents=True)
        self.file.write_text('original')
        self.stamp=self.root/'.local-deploy/software-tests-passed'
        self.stamp.parent.mkdir()
        self.patches=[mock.patch.object(ACCEPTANCE,'ROOT',self.root),
                      mock.patch.object(ACCEPTANCE,'STAMP',self.stamp),
                      mock.patch.object(ACCEPTANCE.subprocess,'check_output',return_value='sha256:test-image')]
        for patch in self.patches:
            patch.start()
            self.addCleanup(patch.stop)
        self.addCleanup(self.tmp.cleanup)

    def invoke(self,*args):
        with mock.patch.object(sys,'argv',['acceptance_stamp.py',*args]):
            return ACCEPTANCE.main()

    def test_passes_current_result_and_rejects_source_changes(self):
        fingerprint,_=ACCEPTANCE.fingerprint()
        self.invoke('write','--expected',fingerprint)
        self.invoke('check')
        self.file.write_text('changed')
        with self.assertRaises(SystemExit): self.invoke('check')

    def test_rejects_missing_legacy_or_stale_image_result(self):
        with self.assertRaises(SystemExit): self.invoke('check')
        self.stamp.write_text('2026-11-13T00:00:00Z')
        with self.assertRaises(SystemExit): self.invoke('check')
        fingerprint,_=ACCEPTANCE.fingerprint()
        self.invoke('write','--expected',fingerprint)
        with mock.patch.object(ACCEPTANCE.subprocess,'check_output',return_value='sha256:different-image'):
            with self.assertRaises(SystemExit): self.invoke('check')

    def test_changed_source_during_verification_cannot_be_recorded(self):
        fingerprint,_=ACCEPTANCE.fingerprint()
        self.file.write_text('changed during tests')
        with self.assertRaises(SystemExit): self.invoke('write','--expected',fingerprint)
        self.assertFalse(self.stamp.exists())


if __name__=='__main__':
    unittest.main()
