#!/usr/bin/env python3
"""Run on the Central PC to validate native DDS and/or ROSBridge delivery.

DDS mode needs a sourced Jazzy installation. WebSocket mode needs websocket-client.
This utility reports reception; it never changes server-side transport availability.
"""
import argparse
import base64
import json
import time


REQUIRED_FIELDS = {'x','y','z','rgb','source_frame_index','source_frame_stamp_sec',
                   'source_frame_stamp_nsec','image_stamp_sec','image_stamp_nsec'}


def validate(width, height, step, row_step, data_size, fields, sec, nanosec):
    assert width>0 and height>0, 'Empty cloud'
    assert step>=36 and row_step>=width*step, 'Invalid cloud layout'
    assert data_size==row_step*height, 'Truncated cloud payload'
    assert REQUIRED_FIELDS.issubset(fields), 'Source/image timestamp metadata is missing'
    assert 0<=nanosec<10**9 and sec>0, 'Invalid ROS 2 timestamp'
    return sec*10**9+nanosec


def check_dds(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2, Imu
    from std_msgs.msg import Float32, Bool
    rclpy.init(args=[])
    node=Node('central_dds_acceptance')
    latest=[]
    seen=set()
    qos=QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.VOLATILE)
    state_qos=QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def cloud_callback(msg):
        stamp=validate(msg.width,msg.height,msg.point_step,msg.row_step,len(msg.data),
                       {field.name for field in msg.fields},msg.header.stamp.sec,msg.header.stamp.nanosec)
        if latest:
            assert stamp>latest[-1], 'Cloud timestamps did not advance'
        latest.append(stamp)
    subscriptions=[node.create_subscription(PointCloud2,'/merged_colored_cloud',cloud_callback,qos)]
    for topic,typ,profile in [('/livox/imu',Imu,qos),('/temperature',Float32,state_qos),
                               ('/image_enhancement/status',Bool,state_qos)]:
        subscriptions.append(node.create_subscription(typ,topic,
            lambda msg, *, t=topic: seen.add(t), profile))
    deadline=time.monotonic()+args.timeout
    try:
        while time.monotonic()<deadline and (len(latest)<args.frames or len(seen)<3):
            rclpy.spin_once(node,timeout_sec=.2)
        assert len(latest)>=args.frames and len(seen)==3, 'DDS cloud/IMU/status delivery timed out'
        return {'transport':'native_dds','valid_clouds':len(latest),'topics_received':sorted(seen)}
    finally:
        node.destroy_node()
        rclpy.shutdown()


def check_websocket(args):
    import websocket
    ws=websocket.create_connection(args.url,timeout=1)
    latest=[]
    seen=set()
    leaf=None
    restore=None
    state_qos={'depth':1,'history':'keep_last','reliability':'reliable','durability':'transient_local'}
    def send(message):
        ws.send(json.dumps(message))
    def receive():
        nonlocal leaf
        try:
            packet=json.loads(ws.recv())
        except websocket.WebSocketTimeoutException:
            return
        if packet.get('op')!='publish':
            if packet.get('level')=='error':
                raise AssertionError(packet)
            return
        topic,msg=packet['topic'],packet['msg']
        if topic=='/merged_colored_cloud':
            assert 'seq' not in msg['header'], 'ROS 1 header format was received'
            stamp=msg['header']['stamp']
            payload=base64.b64decode(msg['data'],validate=True) if isinstance(msg['data'],str) else bytes(msg['data'])
            value=validate(msg['width'],msg['height'],msg['point_step'],msg['row_step'],len(payload),
                           {field['name'] for field in msg['fields']},stamp['sec'],stamp['nanosec'])
            if latest:
                assert value>latest[-1], 'Cloud timestamps did not advance'
            latest.append(value)
        elif topic=='/voxel_leaf_size/status':
            leaf=msg['data']
        else:
            seen.add(topic)
    try:
        send({'op':'subscribe','id':'cloud','topic':'/merged_colored_cloud',
              'type':'sensor_msgs/msg/PointCloud2','queue_length':1,'throttle_rate':500,
              'qos':{'depth':1,'history':'keep_last','reliability':'reliable','durability':'volatile'}})
        for topic,typ in [('/livox/imu','sensor_msgs/msg/Imu'),('/temperature','std_msgs/msg/Float32'),
                          ('/image_enhancement/status','std_msgs/msg/Bool'),('/voxel_leaf_size/status','std_msgs/msg/Float64')]:
            message={'op':'subscribe','topic':topic,'type':typ,'throttle_rate':100,'queue_length':1}
            if topic!='/livox/imu':
                message['qos']=state_qos
            send(message)
        deadline=time.monotonic()+args.timeout
        while time.monotonic()<deadline and (len(latest)<args.frames or len(seen)<3 or leaf is None):
            receive()
        assert len(latest)>=args.frames and len(seen)>=3 and leaf is not None, 'WebSocket delivery timed out'
        control='not requested'
        if args.check_controls:
            restore=leaf
            target=.03 if abs(leaf-.03)>.0001 else .02
            send({'op':'advertise','topic':'/voxel_leaf_size','type':'std_msgs/msg/Float64'})
            send({'op':'publish','topic':'/voxel_leaf_size','msg':{'data':target}})
            deadline=time.monotonic()+10
            while time.monotonic()<deadline and leaf!=target:
                receive()
            assert leaf==target, 'Control acknowledgement timed out'
            send({'op':'publish','topic':'/voxel_leaf_size','msg':{'data':restore}})
            deadline=time.monotonic()+10
            while time.monotonic()<deadline and leaf!=restore:
                receive()
            assert leaf==restore, 'Original voxel size was not restored'
            restore=None
            control='passed; original setting restored'
        return {'transport':'rosbridge','valid_clouds':len(latest),'topics_received':sorted(seen),
                'control_roundtrip':control}
    finally:
        if restore is not None:
            send({'op':'publish','topic':'/voxel_leaf_size','msg':{'data':restore}})
        ws.close()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--transport',choices=['dds','websocket','both'],default='both')
    parser.add_argument('--url',default='ws://10.20.0.61:9090')
    parser.add_argument('--frames',type=int,default=3)
    parser.add_argument('--timeout',type=float,default=30)
    parser.add_argument('--check-controls',action='store_true',help='Briefly change voxel size, verify status, then restore it')
    args=parser.parse_args()
    if args.transport in ['dds','both']:
        print(json.dumps(check_dds(args)),flush=True)
    if args.transport in ['websocket','both']:
        print(json.dumps(check_websocket(args)),flush=True)


if __name__=='__main__':
    main()
