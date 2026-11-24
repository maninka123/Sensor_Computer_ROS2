"""Validate ROS 2 CDR sensor messages without copying bulk sensor payloads."""
import collections
import math
import struct
import time


def sensor_stamp(data, require_geometry=False):
    if len(data)<16:
        return None
    order='<' if data[1]&1 else '>'
    at=4
    def align(size):
        nonlocal at
        at=4+((at-4+size-1)//size)*size
    def u32():
        nonlocal at
        align(4)
        result=struct.unpack_from(order+'I',data,at)[0]
        at+=4
        return result
    def string():
        nonlocal at
        size=u32()
        if size<1 or at+size>len(data) or data[at+size-1]!=0:
            raise ValueError('Invalid CDR string')
        result=data[at:at+size-1]
        at+=size
        return result
    try:
        sec=struct.unpack_from(order+'i',data,at)[0];at+=4
        nanosec=u32()
        if sec<0 or nanosec>=10**9 or (sec==0 and nanosec==0):
            return None
        string()
        if require_geometry=='imu':
            align(8)
            if len(data)-at<37*8:
                return None
            if not all(math.isfinite(value) for value in struct.unpack_from(order+'37d',data,at)):
                return None
        elif require_geometry:
            height,width=u32(),u32()
            if width==0 or height==0:
                return None
            if require_geometry=='image':
                string()  # encoding
                at+=1  # is_bigendian
                step=u32()
                size=u32()
                if step==0 or size<height*step or at+size>len(data):
                    return None
            elif require_geometry=='cloud':
                fields=u32()
                if fields==0 or fields>256:
                    return None
                layout=[]
                for _ in range(fields):
                    name=string();offset=u32()
                    datatype=data[at];at+=1
                    count=u32()
                    sizes={1:1,2:1,3:2,4:2,5:4,6:4,7:4,8:8}
                    if datatype not in sizes or count==0:
                        return None
                    layout.append((name,offset,count*sizes[datatype]))
                at+=1  # is_bigendian
                step,row_step,size=u32(),u32(),u32()
                if step==0 or row_step<width*step or size<height*row_step or at+size>=len(data):
                    return None
                if not {b'x',b'y',b'z'}.issubset({item[0] for item in layout}):
                    return None
                if any(offset+size>step for _,offset,size in layout):
                    return None
        return sec,nanosec
    except (struct.error,IndexError,ValueError):
        return None


class FreshnessTracker:
    def __init__(self, clock=time.monotonic):
        self.clock=clock
        self.seen={}
        self.last_stamp={}
        self.recent={}
        self.count={}

    def observe(self, topic, data, require_geometry=False):
        stamp=sensor_stamp(data,require_geometry)
        recent=self.recent.setdefault(topic,collections.deque(maxlen=32))
        if stamp is None or stamp in recent:
            return False
        # Unique samples after a backward clock jump are new; a repeating cycle
        # of cached stamps is not evidence of a working sensor.
        recent.append(stamp)
        self.last_stamp[topic]=stamp
        self.count[topic]=self.count.get(topic,0)+1
        if self.count[topic]>=2:
            self.seen[topic]=self.clock()
        return True
