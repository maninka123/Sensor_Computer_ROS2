#!/usr/bin/env python3
"""Hardware-free acceptance tests using real Jazzy nodes, DDS and WebSocket IO."""
import collections
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import unittest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField, Imu
from std_msgs.msg import Bool, Float64, Float32, UInt32, UInt64
import websocket


class PipelineAcceptance(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.tmp = tempfile.TemporaryDirectory(prefix='sensor-jazzy-test-')
        cls.temperature = Path(cls.tmp.name)/'temp'
        cls.temperature.write_text('65000')
        cls.log = open('/workspace/.local-deploy/integration-launch.log', 'w')
        cls.process = subprocess.Popen([
            'ros2', 'launch', 'node_pc', 'pipeline.launch.py', 'hardware:=false',
            'bridge_address:=127.0.0.1', 'bridge_port:=19090',
            'temperature_path:='+str(cls.temperature), 'temperature_poll_interval:=1.0',
        ], stdout=cls.log, stderr=subprocess.STDOUT, start_new_session=True)
        cls.node = Node('acceptance_fixture')
        cls.state = {}
        cls.clouds = collections.deque(maxlen=20)
        cls.states_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.subs = [cls.node.create_subscription(PointCloud2, '/merged_colored_cloud',
                                                 lambda msg: cls.clouds.append(msg), qos_profile_sensor_data)]
        for topic, typ in {
            '/image_enhancement/status': Bool, '/enclosure_correction/status': Bool,
            '/voxel_downsampling/status': Bool, '/voxel_leaf_size/status': Float64,
            '/temperature': Float32, '/image_enhancer/processed_frames': UInt64,
            '/merged_colored_cloud/matched_frames': UInt32,
            '/merged_colored_cloud/input_frames': UInt32,
        }.items():
            cls.subs.append(cls.node.create_subscription(typ, topic,
                lambda msg, *, t=topic: cls.state.__setitem__(t, msg.data), cls.states_qos))
        cls.controls = {topic: cls.node.create_publisher(typ, topic, 1) for topic, typ in {
            '/image_enhancement': Bool, '/voxel_downsampling': Bool,
            '/voxel_leaf_size': Float64, '/enclosure_correction': Bool}.items()}
        cls.image_pub = cls.node.create_publisher(Image, '/camera/image_raw', 2)
        cls.lidar_pub = cls.node.create_publisher(PointCloud2, '/livox/lidar', 2)
        cls.imu_pub = cls.node.create_publisher(Imu, '/livox/imu', 2)
        cls.publish_enabled = True
        cls.stamp_offset = 0
        cls.payload = b''.join(struct.pack('<ffff', 2.0+x*.008, y*.008, z*.008, 100.0)
            for x in range(2) for y in range(-5, 5) for z in range(-5, 5))
        cls.timer = cls.node.create_timer(0.1, cls.publish)
        cls.spin_until(lambda: '/image_enhancement/status' in cls.state and len(cls.clouds)>0, 70)

    @classmethod
    def publish(cls):
        if not cls.publish_enabled:
            return
        now = cls.node.get_clock().now().nanoseconds + cls.stamp_offset
        image = Image(height=366, width=484, encoding='bgr8', step=484*3)
        image.header.frame_id = 'camera'
        image.header.stamp.sec, image.header.stamp.nanosec = divmod(now+32430000, 10**9)
        image.data = bytes([32, 64, 128])*(366*484)
        cloud = PointCloud2(height=1, width=len(cls.payload)//16, point_step=16,
                            row_step=len(cls.payload), is_dense=True, data=cls.payload)
        cloud.header.frame_id = 'livox_frame'
        cloud.header.stamp.sec, cloud.header.stamp.nanosec = divmod(now, 10**9)
        cloud.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(['x', 'y', 'z', 'intensity'])]
        imu = Imu()
        imu.header = cloud.header
        imu.linear_acceleration.z = 9.81
        cls.image_pub.publish(image)
        cls.lidar_pub.publish(cloud)
        cls.imu_pub.publish(imu)

    @classmethod
    def spin_until(cls, predicate, timeout=20):
        deadline = time.monotonic()+timeout
        while time.monotonic()<deadline:
            if cls.process.poll() is not None:
                raise AssertionError('Pipeline exited early; inspect .local-deploy/integration-launch.log')
            rclpy.spin_once(cls.node, timeout_sec=.05)
            if predicate():
                return
        raise AssertionError('Timed out waiting for pipeline condition; state='+repr(cls.state))

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls, 'process'):
            cls.process.send_signal(signal.SIGINT) if cls.process.poll() is None else None
            try:
                cls.process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(cls.process.pid, signal.SIGKILL)
                cls.process.wait(timeout=5)
        if hasattr(cls, 'node'):
            cls.node.destroy_node()
        rclpy.shutdown()
        cls.log.close()
        cls.tmp.cleanup()

    def test_01_native_fusion_and_timestamp_provenance(self):
        cloud = self.clouds[-1]
        self.assertGreater(cloud.width, 0)
        offsets = {field.name: field.offset for field in cloud.fields}
        required = ['x', 'y', 'z', 'rgb', 'source_frame_index', 'source_frame_stamp_sec',
                    'source_frame_stamp_nsec', 'image_stamp_sec', 'image_stamp_nsec']
        self.assertTrue(all(field in offsets for field in required))
        indices = set()
        for at in range(0, len(cloud.data), cloud.point_step):
            def u32(field):
                return struct.unpack_from('<I', cloud.data, at+offsets[field])[0]
            indices.add(u32('source_frame_index'))
            source = u32('source_frame_stamp_sec')+u32('source_frame_stamp_nsec')/1e9
            image = u32('image_stamp_sec')+u32('image_stamp_nsec')/1e9
            self.assertLessEqual(abs(source-image), .05)
            self.assertEqual(u32('rgb'), (128<<16)|(64<<8)|32)
        self.assertGreaterEqual(len(indices), 3)
        self.assertLessEqual(len(indices), 10)

    def test_02_controls_and_invalid_leaf_size(self):
        self.controls['/voxel_downsampling'].publish(Bool(data=False))
        self.spin_until(lambda: self.state.get('/voxel_downsampling/status') is False)
        self.controls['/voxel_leaf_size'].publish(Float64(data=.04))
        self.spin_until(lambda: self.state.get('/voxel_leaf_size/status') == .04)
        for invalid in [-1.0, 0.0, float('nan'), float('inf'), 1e-300, 1e300]:
            self.controls['/voxel_leaf_size'].publish(Float64(data=invalid))
            rclpy.spin_once(self.node, timeout_sec=.05)
        start = time.monotonic()
        self.spin_until(lambda: time.monotonic()-start>1)
        self.assertEqual(self.state.get('/voxel_leaf_size/status'), .04)
        self.controls['/voxel_downsampling'].publish(Bool(data=True))
        self.spin_until(lambda: self.state.get('/voxel_downsampling/status') is True)

    def test_03_enhancement_and_thermal_fallback(self):
        self.addCleanup(lambda: self.controls['/image_enhancement'].publish(Bool(data=False)))
        self.addCleanup(lambda: self.temperature.write_text('65000'))
        self.controls['/image_enhancement'].publish(Bool(data=True))
        self.spin_until(lambda: self.state.get('/image_enhancement/status') is True)
        baseline = self.state.get('/image_enhancer/processed_frames', 0)
        self.spin_until(lambda: self.state.get('/image_enhancer/processed_frames', 0)>baseline)
        self.temperature.write_text('90000')
        self.spin_until(lambda: self.state.get('/image_enhancement/status') is False)
        before = self.clouds[-1].header.stamp
        self.spin_until(lambda: self.clouds[-1].header.stamp != before)
        self.temperature.write_text('65000')
        self.spin_until(lambda: self.state.get('/image_enhancement/status') is True)
        self.temperature.unlink()
        self.spin_until(lambda: self.state.get('/image_enhancement/status') is False)
        self.temperature.write_text('65000')
        self.spin_until(lambda: self.state.get('/image_enhancement/status') is True)
        self.controls['/image_enhancement'].publish(Bool(data=False))
        self.spin_until(lambda: self.state.get('/image_enhancement/status') is False)

    def test_04_enclosure_correction(self):
        self.controls['/enclosure_correction'].publish(Bool(data=True))
        self.spin_until(lambda: self.state.get('/enclosure_correction/status') is True)
        before = self.clouds[-1].header.stamp
        self.spin_until(lambda: self.clouds[-1].header.stamp != before)
        self.controls['/enclosure_correction'].publish(Bool(data=False))
        self.spin_until(lambda: self.state.get('/enclosure_correction/status') is False)

    def test_05_websocket_data_and_control(self):
        ws = websocket.create_connection('ws://127.0.0.1:19090', timeout=.2)
        try:
            ws.send(json.dumps({'op':'subscribe', 'id':'cloud', 'topic':'/merged_colored_cloud',
                                'type':'sensor_msgs/msg/PointCloud2', 'throttle_rate':500, 'queue_length':1}))
            received = []
            def receive():
                try:
                    msg=json.loads(ws.recv())
                    if msg.get('op')=='publish':
                        received.append(msg)
                except websocket.WebSocketTimeoutException:
                    pass
                return bool(received)
            self.spin_until(receive)
            self.assertGreater(received[-1]['msg']['width'], 0)
            ws.send(json.dumps({'op':'advertise','topic':'/voxel_leaf_size','type':'std_msgs/msg/Float64'}))
            ws.send(json.dumps({'op':'publish','topic':'/voxel_leaf_size','msg':{'data':.03}}))
            self.spin_until(lambda: self.state.get('/voxel_leaf_size/status') == .03)
        finally:
            ws.close()
        # Exercise the same portable reception/acknowledgement utility provided
        # to the Windows platform, in a separate native DDS client process.
        with open('/workspace/.local-deploy/central-client-local-check.log','w') as log:
            client=subprocess.Popen(['python3','tools/central_transport_check.py',
                '--transport','both','--url','ws://127.0.0.1:19090','--check-controls'],
                stdout=log,stderr=subprocess.STDOUT)
            self.spin_until(lambda: client.poll() is not None,45)
            self.assertEqual(client.returncode,0,'Portable client check failed; inspect central-client-local-check.log')

    def test_05b_fastdds_client_interoperability(self):
        env=dict(os.environ, RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                 FASTRTPS_DEFAULT_PROFILES_FILE='/workspace/ros2/deploy/fastdds-test.xml')
        with open('/workspace/.local-deploy/fastdds-interoperability.log','w') as log:
            client=subprocess.Popen(['python3','tools/central_transport_check.py',
                '--transport','dds','--frames','3','--timeout','20'],
                env=env, stdout=log,stderr=subprocess.STDOUT)
            self.spin_until(lambda: client.poll() is not None,30)
            self.assertEqual(client.returncode,0,'Fast DDS client failed; inspect fastdds-interoperability.log')

    def test_06_healthcheck_and_missing_stream_detection(self):
        check = subprocess.Popen(['ros2','run','node_pc','healthcheck.py','--once',
                                  '--startup-timeout','15','--max-age','3'])
        self.spin_until(lambda: check.poll() is not None, 20)
        self.assertEqual(check.returncode, 0)
        type(self).publish_enabled=False
        try:
            check = subprocess.Popen(['ros2','run','node_pc','healthcheck.py','--once',
                                      '--startup-timeout','3','--max-age','1'])
            self.spin_until(lambda: check.poll() is not None, 10)
            self.assertNotEqual(check.returncode, 0)
        finally:
            type(self).publish_enabled=True

    def test_06b_bridge_watchdog_tolerates_one_failure_but_detects_sustained_failure(self):
        import base64,hashlib,socketserver,threading
        state={'fail_once':False,'fail_always':False,'failures':0}
        class Handler(socketserver.BaseRequestHandler):
            def handle(self):
                self.request.settimeout(2)
                request=b''
                while b'\r\n\r\n' not in request:
                    chunk=self.request.recv(4096)
                    if not chunk:return
                    request+=chunk
                if state['fail_once'] or state['fail_always']:
                    state['fail_once']=False
                    state['failures']+=1
                    self.request.sendall(b'HTTP/1.1 503 Unavailable\r\nContent-Length: 0\r\n\r\n')
                    return
                headers=dict(line.split(b':',1) for line in request.split(b'\r\n')[1:] if b':' in line)
                key=next(v.strip() for k,v in headers.items() if k.lower()==b'sec-websocket-key')
                accept=base64.b64encode(hashlib.sha1(key+b'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').digest())
                self.request.sendall(b'HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: '+accept+b'\r\n\r\n')
        server=socketserver.ThreadingTCPServer(('127.0.0.1',0),Handler)
        server.daemon_threads=True
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        path=Path(self.tmp.name)/'bridge-watchdog.log'
        with path.open('w') as log:
            process=subprocess.Popen(['ros2','run','node_pc','healthcheck.py',
                '--bridge-url','ws://127.0.0.1:'+str(server.server_address[1]),
                '--bridge-check-interval','.3','--bridge-timeout','.5','--bridge-retries','3',
                '--startup-timeout','10','--max-age','5'],stdout=log,stderr=subprocess.STDOUT)
            try:
                self.spin_until(lambda:'PASS: fresh' in path.read_text(),15)
                state['fail_once']=True
                self.spin_until(lambda:state['failures']==1,5)
                start=time.monotonic()
                self.spin_until(lambda:time.monotonic()-start>1,5)
                self.assertIsNone(process.poll(),'A transient handshake failure restarted healthy acquisition')
                state['fail_always']=True
                self.spin_until(lambda:process.poll() is not None,10)
                self.assertNotEqual(process.returncode,0)
                self.assertIn('FAIL: missing or stalled streams: ROSBridge WebSocket',path.read_text())
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    process.wait(timeout=5)
                server.shutdown();server.server_close();thread.join(timeout=2)

    def test_07_timestamp_reversal_recovers(self):
        type(self).stamp_offset=-30_000_000_000
        before = self.clouds[-1].header.stamp
        try:
            self.spin_until(lambda: self.clouds[-1].header.stamp.sec < before.sec-10, 15)
        finally:
            type(self).stamp_offset=0
        self.spin_until(lambda: self.clouds[-1].header.stamp.sec >= before.sec, 15)

    def test_07a_late_image_and_malformed_clouds_do_not_stop_fusion(self):
        import copy
        now=self.node.get_clock().now().nanoseconds
        image=Image(height=366,width=484,encoding='bgr8',step=484*3)
        image.header.stamp.sec,image.header.stamp.nanosec=divmod(now-80000000,10**9)
        image.data=bytes([255,0,0])*(366*484)
        self.image_pub.publish(image)
        valid=PointCloud2(height=1,width=1,point_step=28,row_step=28,is_dense=True,
                         data=struct.pack('<fffIIII',2.,0.,0.,0,now//10**9,0,0))
        valid.header.stamp=self.node.get_clock().now().to_msg()
        names=['x','y','z','source_frame_index','source_frame_stamp_sec',
               'source_frame_stamp_nsec','image_enhancement_mode']
        valid.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32 if i<3 else PointField.UINT32,count=1)
                      for i,n in enumerate(names)]
        malformed=[]
        item=copy.deepcopy(valid);item.data=b'';malformed.append(item)
        item=copy.deepcopy(valid);item.fields[0].offset=28;malformed.append(item)
        item=copy.deepcopy(valid);item.fields[0].datatype=PointField.FLOAT64;malformed.append(item)
        item=copy.deepcopy(valid);item.is_bigendian=True;malformed.append(item)
        item=copy.deepcopy(valid);item.data=struct.pack('<fffIIII',2.,0.,0.,0,now//10**9,10**9,0);malformed.append(item)
        publishers=[self.node.create_publisher(PointCloud2,t,5) for t in ['/livox/lidar_merged','/livox/lidar_filtered']]
        try:
            self.spin_until(lambda: all(p.get_subscription_count()>0 for p in publishers),5)
            for item in malformed:
                for publisher in publishers:publisher.publish(item)
                rclpy.spin_once(self.node,timeout_sec=.05)
            baseline=self.clouds[-1].header.stamp
            self.spin_until(lambda:self.clouds[-1].header.stamp!=baseline,10)
            self.assertIsNone(self.process.poll())
        finally:
            for publisher in publishers:self.node.destroy_publisher(publisher)

    def test_08_essential_node_exit_stops_complete_launch(self):
        import re
        self.log.flush()
        text = Path('/workspace/.local-deploy/integration-launch.log').read_text()
        match = re.search(r'\[pointcloud_merge_node-\d+\]: process started with pid \[(\d+)\]', text)
        self.assertIsNotNone(match)
        os.kill(int(match.group(1)), signal.SIGKILL)
        deadline = time.monotonic()+15
        while self.process.poll() is None and time.monotonic()<deadline:
            rclpy.spin_once(self.node, timeout_sec=.1)
        self.assertIsNotNone(self.process.poll(), 'Essential node exit did not stop the launch')


if __name__ == '__main__':
    unittest.main(verbosity=2)
