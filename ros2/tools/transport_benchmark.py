#!/usr/bin/env python3
"""Local DDS/WebSocket serialization probe; does not replace LAN/Windows testing."""
import argparse
import base64
import json
from pathlib import Path
import statistics
import struct
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
import websocket


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', default='ws://127.0.0.1:19091')
    parser.add_argument('--points', type=int, default=100000)
    parser.add_argument('--frames', type=int, default=10)
    parser.add_argument('--period', type=float, default=.5)
    parser.add_argument('--output', default='/workspace/.local-deploy/transport-benchmark.json')
    args = parser.parse_args()
    rclpy.init()
    node = Node('transport_serialization_probe')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    topic = '/sensor_computer/transport_probe'
    publisher = node.create_publisher(PointCloud2, topic, qos)
    native = []
    bridged = []
    wire_bytes = 0
    expected = args.points*36
    def native_callback(msg):
        assert len(msg.data)==expected and msg.point_step==36 and msg.width==args.points
        sent = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        native.append((time.time()-sent)*1000)
    subscription = node.create_subscription(PointCloud2, topic, native_callback, qos)
    ws = websocket.create_connection(args.url, timeout=.02)
    ws.send(json.dumps({'op':'subscribe','id':'transport_probe','topic':topic,
                       'type':'sensor_msgs/msg/PointCloud2','queue_length':1,
                       'qos':{'depth':1,'history':'keep_last','reliability':'reliable','durability':'volatile'}}))
    fields = ['x','y','z','rgb','source_frame_index','source_frame_stamp_sec',
              'source_frame_stamp_nsec','image_stamp_sec','image_stamp_nsec']
    cloud = PointCloud2(height=1, width=args.points, point_step=36,
                        row_step=expected, is_dense=True)
    cloud.fields = [PointField(name=field, offset=i*4,
        datatype=PointField.FLOAT32 if i<4 else PointField.UINT32, count=1)
        for i,field in enumerate(fields)]
    cloud.header.frame_id='livox_frame'
    cloud.data=struct.pack('<fffIIIIII', 2.0, 0.0, 0.0, 0x804020, 0, 1, 0, 1, 0)*args.points
    sent=0
    started=time.monotonic()
    next_send=started+2
    deadline=started+max(35, args.frames*args.period+20)
    try:
        while time.monotonic()<deadline and (len(native)<args.frames or len(bridged)<args.frames):
            now=time.monotonic()
            if now>=next_send and sent<args.frames:
                cloud.header.stamp.sec,cloud.header.stamp.nanosec=divmod(time.time_ns(),10**9)
                publisher.publish(cloud)
                sent+=1
                next_send=now+args.period
            rclpy.spin_once(node, timeout_sec=.005)
            try:
                packet=ws.recv()
                message=json.loads(packet)
                if message.get('op')!='publish' or message.get('topic')!=topic:
                    continue
                msg=message['msg']
                assert msg['point_step']==36 and msg['width']==args.points
                payload=base64.b64decode(msg['data']) if isinstance(msg['data'],str) else bytes(msg['data'])
                assert len(payload)==expected
                stamp=msg['header']['stamp']
                assert 'sec' in stamp and 'nanosec' in stamp and 'seq' not in msg['header']
                bridged.append((time.time()-(stamp['sec']+stamp['nanosec']/1e9))*1000)
                wire_bytes+=len(packet.encode() if isinstance(packet,str) else packet)
            except websocket.WebSocketTimeoutException:
                pass
        elapsed=time.monotonic()-started
        def stats(values):
            values=sorted(values)
            return {'received':len(values),'median_ms':round(statistics.median(values),3) if values else None,
                    'p95_ms':round(values[min(len(values)-1, int(.95*len(values)))],3) if values else None}
        result={'scope':'same-host Linux loopback, synthetic messages; Windows and physical LAN unverified',
                'points_per_frame':args.points,'payload_bytes_per_frame':expected,'sent':sent,
                'elapsed_s':round(elapsed,3),'native_dds':stats(native),'rosbridge_json':stats(bridged),
                'rosbridge_wire_bytes':wire_bytes,'both_transports_available':True,
                'complete_delivery':len(native)==args.frames and len(bridged)==args.frames}
        Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
        if not result['complete_delivery']:
            raise SystemExit(1)
    finally:
        ws.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
