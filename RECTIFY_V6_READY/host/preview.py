"""Independent compressed preview rows; incomplete frames never block newer ones."""
import struct,time,zlib
from codec import decode_row

HEADER=struct.Struct('<4sIIHHHBBBBHI')
HEADER_V2=struct.Struct('<4sIIHHHBBBBHHHI')

class PreviewAssembler:
    def __init__(self):self.frames={};self.last_frame=None;self.errors=0;self.incomplete=0
    def feed(self,wire,session):
        if len(wire)<33 or len(wire)>1436:raise ValueError('Preview datagram length')
        if wire[:4]==b'PGV2':
            magic,sid,frame,row,w,h,mode,model,codec,version,total,offset,n,raw_crc=HEADER_V2.unpack_from(wire)
            if sid!=session:return None
            if (w,h)!=(1280,720) or version!=2 or row>=h or mode>2 or model>1 or codec>1:raise ValueError('Preview metadata invalid')
            if not 0<total<=w*2 or offset not in (0,1400) or offset>=total or n!=min(1400,total-offset) or len(wire)!=36+n:raise ValueError('Preview fragment bounds')
            if codec==0 and total!=w*2 or codec==1 and total>=w*2:raise ValueError('Preview codec length')
            encoded=wire[32:-4]
        else:
            magic,sid,frame,row,w,h,mode,model,codec,version,reserved,raw_crc=HEADER.unpack_from(wire)
            if magic!=b'PGV1' or sid!=session:return None
            if (w,h)!=(640,360) or row>=h or mode>2 or model>1 or codec>1 or version!=1 or reserved or len(wire)>1312:raise ValueError('Preview metadata invalid')
            encoded=wire[28:-4];total=len(encoded);offset=0;n=total
        if zlib.crc32(wire[:-4])!=struct.unpack_from('<I',wire,len(wire)-4)[0]:raise ValueError('Preview packet CRC')
        if self.last_frame is not None and (frame-self.last_frame)&0xffffffff>=0x80000000:return None
        if frame==self.last_frame:return None
        if frame not in self.frames:
            self.frames[frame]={'rows':{},'partials':{},'width':w,'height':h,'mode':mode,'model':model,'first_received':time.monotonic()}
            if len(self.frames)>2:
                oldest=min(self.frames,key=lambda k:self.frames[k]['first_received'])
                del self.frames[oldest];self.incomplete+=1
        item=self.frames[frame]
        if (item['mode'],item['model'],item['width'],item['height'])!=(mode,model,w,h):raise ValueError('Preview mode/size changed within frame')
        if row in item['rows']:return None
        partial=item['partials'].setdefault(row,{'identity':(codec,total,raw_crc),'pieces':{}})
        if partial['identity']!=(codec,total,raw_crc):raise ValueError('Inconsistent preview row fragments')
        previous=partial['pieces'].get(offset)
        if previous is not None and previous!=encoded:raise ValueError('Conflicting duplicate fragment')
        partial['pieces'][offset]=encoded
        if sum(map(len,partial['pieces'].values()))!=total:return None
        data=b''.join(partial['pieces'][x] for x in sorted(partial['pieces']))
        item['rows'][row]=(codec,data,raw_crc);del item['partials'][row]
        if len(item['rows'])!=h:return None
        self.last_frame=frame;self.frames.pop(frame)
        item.pop('partials');item.update(width=w,height=h,frame=frame,sequence=session,source='board_ov5640',preview=True)
        return item

def decode_frame(item):
    from codec import decode_checked_frame
    start=time.perf_counter();w=item['width'];h=item['height']
    raw=decode_checked_frame(item['rows'],w,h)
    wire_bytes=sum(len(data) for codec,data,expected in item['rows'].values())
    compressed_rows=sum(codec==1 for codec,data,expected in item['rows'].values())
    meta={k:v for k,v in item.items() if k!='rows'}
    meta.update(decode_seconds=time.perf_counter()-start,compression_ratio=len(raw)/max(1,wire_bytes),compressed_rows=compressed_rows,CRC_verified=True,received_at=time.monotonic())
    return (w,h,bytes(raw),item['frame']),meta
