"""Bounded compact IMU history, independent from the GUI and image buffers."""
from array import array
import bisect
import csv
import math


class AngleHistory:
    def __init__(self, seconds=600, max_hz=20):
        if seconds<=0 or max_hz<=0 or seconds*max_hz<1:raise ValueError('History capacity must be positive')
        self.seconds=float(seconds);self.max_hz=float(max_hz)
        self.capacity=int(seconds*max_hz)
        self.times=array('d',[0.])*self.capacity
        self.values=[array('f',[math.nan])*self.capacity for _ in range(3)]
        self.head=0;self.count=0;self.origin=None;self.version=0

    @property
    def buffer_bytes(self):
        return len(self.times)*self.times.itemsize+sum(len(v)*v.itemsize for v in self.values)

    def _index(self, logical):return (self.head+logical)%self.capacity
    def time_at(self, logical):return self.times[self._index(logical)]
    @property
    def oldest(self):return self.time_at(0) if self.count else None
    @property
    def newest(self):return self.time_at(self.count-1) if self.count else None

    def append(self,t,roll,pitch,yaw):
        t=float(t)
        if not math.isfinite(t):return False
        if self.count and t-self.newest < (1/self.max_hz)-1e-8:return False
        if self.origin is None:self.origin=t
        while self.count and self.oldest<t-self.seconds:
            self.head=(self.head+1)%self.capacity;self.count-=1
        if self.count==self.capacity:
            self.head=(self.head+1)%self.capacity;self.count-=1
        index=self._index(self.count);self.times[index]=t
        for target,value in zip(self.values,(roll,pitch,yaw)):
            target[index]=float(value) if value is not None and math.isfinite(value) else math.nan
        self.count+=1;self.version+=1
        return True

    def _lower_bound(self,t):
        lo,hi=0,self.count
        while lo<hi:
            mid=(lo+hi)//2
            if self.time_at(mid)<t:lo=mid+1
            else:hi=mid
        return lo

    def rows(self,start=None,end=None):
        lo=0 if start is None else self._lower_bound(start)
        hi=self.count if end is None else self._lower_bound(end+1e-8)
        for n in range(lo,hi):
            i=self._index(n)
            yield (self.times[i],*(None if math.isnan(v[i]) else v[i] for v in self.values))

    def export_csv(self,path):
        count=0
        with open(path,'w',newline='',encoding='utf-8-sig') as stream:
            writer=csv.writer(stream);writer.writerow(['session_time_s','roll_deg','pitch_deg','yaw_deg'])
            for t,*values in self.rows():
                writer.writerow([f'{t-self.origin:.3f}',*('' if v is None else f'{v:.6f}' for v in values)])
                count+=1
        return count
