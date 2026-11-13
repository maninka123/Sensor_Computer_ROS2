"""Validate raw ROS 2 CDR headers without deserializing bulk sensor payloads."""
import struct
import time


def sensor_stamp(data, require_geometry=False):
    if len(data)<16:
        return None
    order='<' if data[1]&1 else '>'
    try:
        sec,nanosec,frame_length=struct.unpack_from(order+'iII',data,4)
        if nanosec>=10**9 or (sec==0 and nanosec==0) or frame_length<1:
            return None
        end=16+frame_length
        if end>len(data) or data[end-1]!=0:
            return None
        if require_geometry:
            offset=(end+3)&~3
            height,width=struct.unpack_from(order+'II',data,offset)
            if width==0 or height==0:
                return None
        return sec,nanosec
    except (struct.error,IndexError):
        return None


class FreshnessTracker:
    def __init__(self, clock=time.monotonic):
        self.clock=clock
        self.seen={}
        self.last_stamp={}
        self.count={}

    def observe(self, topic, data, require_geometry=False):
        stamp=sensor_stamp(data,require_geometry)
        if stamp is None or stamp==self.last_stamp.get(topic):
            return False
        # A changed stamp after a backward clock jump is a new sample too.
        self.last_stamp[topic]=stamp
        self.count[topic]=self.count.get(topic,0)+1
        if self.count[topic]>=2:
            self.seen[topic]=self.clock()
        return True
