"""Binary protocols shared with FPGA_IMU_HOST. No demonstration data path."""
from dataclasses import dataclass
import struct
import time
import zlib

def command(kind, payload=b''):
    data=bytes((0xA5,0x5A,kind,len(payload)))+payload
    return data+bytes((sum(data)&255,))

class TelemetryDecoder:
    def __init__(self):
        self.buffer=bytearray();self.errors=0
    def feed(self,data):
        self.buffer.extend(data);result=[]
        while True:
            start=self.buffer.find(b'\xaa\x55')
            if start<0:
                self.buffer[:]=self.buffer[-1:];break
            if start:del self.buffer[:start]
            if len(self.buffer)<4:break
            if self.buffer[2] not in (0x81,0x82,0x83,0x84,0x85) or self.buffer[3]!=68:
                del self.buffer[0];self.errors+=1;continue
            if len(self.buffer)<73:break
            frame=bytes(self.buffer[:73])
            if sum(frame[:-1])&255!=frame[-1]:
                del self.buffer[0];self.errors+=1;continue
            del self.buffer[:73]
            try:result.append(decode_payload(frame[2],frame[4:72]))
            except (ValueError,struct.error):self.errors+=1
        return result

def decode_payload(kind,p):
    if len(p)!=68:raise ValueError('Telemetry payload must have 68 bytes')
    u16=lambda o:struct.unpack_from('<H',p,o)[0]
    i16=lambda o:struct.unpack_from('<h',p,o)[0]
    u32=lambda o:struct.unpack_from('<I',p,o)[0]
    i32=lambda o:struct.unpack_from('<i',p,o)[0]
    out={'kind':kind,'received':time.monotonic(),'bytes':p.hex()}
    if kind==0x81:
        flags=p[0];cfg=p[1]
        out.update(flags=flags,config_flags=cfg,calibration_status=p[2],gyro_code=p[3],accel_code=p[4],rate_code=p[5],sequence=u16(6),
                   imu_frames=u32(52),imu_errors=u32(56),commands=u32(60),command_errors=u32(64))
        raw={'roll':i16(8),'pitch':i16(10),'yaw':i16(12),
             **{f'q{i}':i16(14+2*i) for i in range(4)},
             **{f'a{c}':i16(22+2*i) for i,c in enumerate('xyz')},
             **{f'g{c}':i16(28+2*i) for i,c in enumerate('xyz')},
             **{f'm{c}':i16(34+2*i) for i,c in enumerate('xyz')},
             'mag_temp':i16(40),'pressure':i32(42),'altitude':i32(46),'baro_temp':i16(50)}
        values={}
        for name in ('roll','pitch','yaw'):values[name]=raw[name]*180/32768 if flags&1 else None
        for i in range(4):values[f'q{i}']=raw[f'q{i}']/32768 if flags&2 else None
        accel=(2<<p[4]) if cfg&4 and p[4]<4 else None
        gyro=(250<<p[3]) if cfg&2 and p[3]<4 else None
        for c in 'xyz':
            values[f'a{c}']=raw[f'a{c}']*accel/32768 if flags&4 and accel is not None else None
            values[f'g{c}']=raw[f'g{c}']*gyro/32768 if flags&4 and gyro is not None else None
            values[f'm{c}']=raw[f'm{c}'] if flags&8 else None
        values['mag_temp']=raw['mag_temp']/100 if flags&8 else None
        values['pressure']=raw['pressure'] if flags&16 else None
        values['altitude']=raw['altitude']/100 if flags&16 else None
        values['baro_temp']=raw['baro_temp']/100 if flags&16 else None
        out.update(raw=raw,values=values,accel_range=accel,gyro_range=gyro)
    elif kind==0x82:
        if p[0] not in (1,2,3,4,5,6,7,8,9,10):raise ValueError('Unsupported video telemetry version')
        if p[0]>=3 and p[53]!=0x52:raise ValueError('Missing RETURN_PC_1 capability signature')
        if p[2]>2 or p[3]>1 or p[6]>1 or p[46]>100 or p[51]>1:raise ValueError('Invalid video status fields')
        flags=p[1]
        # Current FPGA packs number/evaluation bits in the opposite order to
        # host commands. Reconstruct command-order flags from explicit status
        # fields so readback confirms actual state on the existing bitstream.
        roi_enabled=bool(p[52]) if p[0]>=2 else (True if p[4]!=0 else None)
        control=(p[2]&3)|((p[3]&1)<<2)|(int(bool(p[51]))<<3)|(int(bool(roi_enabled))<<4)
        if p[0]>=2 and (p[52]>1 or p[7]!=control):raise ValueError('Inconsistent control readback')
        current_protocol=p[0]>=3 and p[53]==0x52
        out.update(return_pc=current_protocol,network_ready=bool(p[54]&2) if current_protocol else None,underflow=bool(p[55]&1) if current_protocol else None,protocol_version=p[0],roi_enabled=roi_enabled,control_reliable=p[0]>=2,
            flags=flags,mode=p[2],model=p[3],evaluation=p[4],gamma=p[5],page=p[6],control=control,wire_control=p[7],uptime_ms=u32(8),
            latency_us=u32(12) if flags&64 else None,latency_min=u32(16) if flags&64 else None,latency_max=u32(20) if flags&64 else None,
            input_fps=u16(24),output_fps=u16(26),repeats=u32(28),skips=u32(32),
            rms=u16(36)/256 if flags&32 else None,maximum=u16(38)/256 if flags&32 else None,
            mtf50=u16(40)/65536 if flags&128 else None,sfr_quarter=u16(42)/4096 if flags&128 else None,
            rows=u16(44),coverage=p[46],position=p[47],frozen=bool(p[50]),numbers=bool(p[51]))
        out.update(lens_capable=p[0]>=4 and bool(p[54]&4),lens_valid=bool(p[56]&1) if p[0]>=4 else False,
                   lens_enabled=bool(p[56]&2) if p[0]>=4 else False,lens_fault=bool(p[56]&4) if p[0]>=4 else False,
                   lens_id=u32(57) if p[0]>=4 else None,lens_errors=u32(61) if p[0]>=4 else None,lens_busy=bool(p[65]) if p[0]>=4 else None)
        out['photo_window']=4 if p[0]>=10 and p[55]&64 else 1
        out['photo_diagnostics']=None
        out.update(raw_capture_capable=p[0]>=6 and bool(p[54]&16),udp_ack_capable=p[0]>=6 and bool(p[54]&32),clean_camera_images=p[0]>=6)
        out.update(extended_imu=p[0]>=8 and bool(p[55]&8),key1_text_control=p[0]>=8 and bool(p[55]&4),camera_text_visible=bool(p[55]&2) if p[0]>=8 else None)
        if p[0]>=5:
            if not p[54]&8:raise ValueError('Missing V5 photo diagnostic capability')
            if p[66]&15>11 or p[67]>2:raise ValueError('Invalid photo diagnostic state')
            out.update(latency_us=None,latency_min=None,latency_max=None,repeats=None,skips=None)
            out['photo_diagnostics']=dict(commands=u32(12),session=u32(16),tx_packets=u32(20),rows=u16(28),aborts=u16(30),
                state=p[66]&15,pending=bool(p[66]&16),preview=bool(p[66]&32),network_fault=bool(p[66]&64),abort_reason=p[67])
    elif kind==0x85:
        if p[0]!=9 or p[1]&128 or (p[1]>>1)&3>2 or p[2]>1 or p[3]>1:raise ValueError('Invalid geometry status')
        out.update(protocol_version=9,ui_flags=p[1],plane_valid=bool(p[2]),busy=bool(p[3]),errors=u32(5),
                   geometry_version=u32(9),display_flags=u32(13),display_sequence=u32(17),bilinear_preview=bool(p[21]&1),leased_ddr_photo=bool(p[21]&2))
    elif kind==0x84:
        if p[0]!=1 or p[1]>1 or p[2:4]!=b'\0\0':raise ValueError('Unsupported resource report')
        used=struct.unpack_from('<4I',p,12);caps=struct.unpack_from('<4I',p,28)
        if caps!=(66600,133200,240,310) or any(a>b for a,b in zip(used,caps)):raise ValueError('Invalid resource capacities/counts')
        out.update(report_valid=bool(p[1]),build_token=f'{struct.unpack_from("<Q",p,4)[0]:016x}',
                   resource_used=used,resource_capacities=caps,source='board_static_post_route_report')
    elif kind==0x83:
        if p[0] not in (1,2):raise ValueError('Unsupported calibration telemetry version')
        if p[1]>1 or p[2]>1:raise ValueError('Invalid calibration status')
        params=struct.unpack_from('<11i',p,8)
        names=['fx','fy','cx','cy','k1','k2','p1' if p[1] else 'k3','p2' if p[1] else 'k4','output_f','output_cx','output_cy']
        out.update(model=p[1],ready=bool(p[2]),uptime_ms=u32(4),
            parameters={name:params[i]/(2**24 if 4<=i<8 else 65536) for i,name in enumerate(names)} if p[2] else {})
        if p[0]==2:
            out.pop('uptime_ms',None)  # V2 byte 4 is the displayed calibration ID.
            names=['fx','fy','cx','cy','k1','k2','p1','p2','inverse_fx','inverse_fy','inverse_zoom']
            out.update(protocol_version=2,calibration_id=u32(4),enabled=bool(p[3]&1),fault=bool(p[3]&2),resolution=[u16(52),u16(54)],errors=u32(56),
                       parameters={name:params[i]/(2**24 if 4<=i<8 else 2**30 if i in (8,9) else 2**20 if i==10 else 65536) for i,name in enumerate(names)} if p[2] else {})
    return out

def pgpk(kind,sequence,payload):
    header=struct.pack('<4sBBHIHH',b'PGPK',1,kind,0,sequence,len(payload),0)
    return header+struct.pack('<I',zlib.crc32(header))+payload+struct.pack('<I',zlib.crc32(payload))

class ImagePacketDecoder:
    def __init__(self):self.buffer=bytearray();self.errors=0
    def feed(self,data):
        self.buffer.extend(data);out=[]
        while True:
            i=self.buffer.find(b'PGPK')
            if i<0:self.buffer[:]=self.buffer[-3:];break
            if i:del self.buffer[:i]
            if len(self.buffer)<20:break
            h=bytes(self.buffer[:16]);magic,version,kind,flags,seq,length,reserved=struct.unpack('<4sBBHIHH',h)
            if version!=1 or length>8192 or zlib.crc32(h)!=struct.unpack_from('<I',self.buffer,16)[0]:
                self.errors+=1;del self.buffer[0];continue
            if len(self.buffer)<24+length:break
            payload=bytes(self.buffer[20:20+length]);checksum=struct.unpack_from('<I',self.buffer,20+length)[0]
            del self.buffer[:24+length]
            if zlib.crc32(payload)!=checksum:self.errors+=1;continue
            out.append((kind,seq,payload))
        return out

class ImageAssembler:
    def __init__(self):self.active=False;self.data=None;self.rows=set();self.format=1;self.metadata={};self.compressed_rows={};self.wire_bytes=0
    def feed(self,kind,seq,payload):
        if kind==0x81:
            if self.active:raise ValueError('Unexpected image INFO during capture')
            if len(payload) not in (20,24,32):raise ValueError('Incorrect image INFO length')
            camera,fmt,w,h,chunk,total,crc,frame=struct.unpack('<BBHHHIII',payload[:20])
            if camera!=2 or fmt not in (1,2) or w!=1920 or h!=1080 or chunk!=w*2 or total!=w*h*2:
                raise ValueError('Image geometry/format does not match FPGA RGB565 path')
            self.format=fmt;self.metadata={};self.compressed_rows={};self.wire_bytes=0
            if fmt==2:
                if len(payload) not in (24,32) or payload[20]>2 or payload[21]>1 or payload[22:24]!=b'\2\1':raise ValueError('Invalid compressed INFO metadata')
                self.metadata=dict(mode=payload[20],model=payload[21],source='board_ov5640',codec='PGD1',integrity='row_crc32')
            if len(payload)==32:
                if fmt!=2:raise ValueError('Unexpected leased frame metadata')
                flags,control_sequence=struct.unpack_from('<II',payload,24)
                if flags&~0x3fe3:raise ValueError('Invalid leased frame flags')
                self.metadata.update(frame_geometry_flags=flags,frame_control_sequence=control_sequence,frame_geometry_association='leased_ddr_frame')
            self.w=w;self.h=h;self.data=bytearray(total);self.rows=set();self.frame=frame;self.sequence=seq;self.active=True
        elif kind==0x82:
            if not self.active or self.format!=1:raise ValueError('Image CHUNK before matching raw INFO')
            if seq!=self.sequence:raise ValueError('Image sequence mismatch')
            if len(payload)!=8+self.w*2:raise ValueError('Incorrect image CHUNK length')
            row,offset=struct.unpack_from('<II',payload)
            if row>=self.h or offset!=row*self.w*2 or row in self.rows:raise ValueError('Invalid or duplicate image row')
            self.data[offset:offset+self.w*2]=payload[8:];self.rows.add(row)
        elif kind==0x85:
            if not self.active or self.format!=2 or seq!=self.sequence:raise ValueError('Compressed row without matching INFO')
            if len(payload)<8 or len(payload)>8+self.w*2:raise ValueError('Compressed row length invalid')
            row,codec,reserved,checksum=struct.unpack_from('<HBBI',payload)
            if row>=self.h or row in self.rows or codec>1 or reserved:raise ValueError('Invalid compressed row')
            self.compressed_rows[row]=(codec,payload[8:],checksum);self.rows.add(row);self.wire_bytes+=len(payload)-8
        elif kind==0x83:
            if not self.active:raise ValueError('Image DONE before INFO')
            if seq!=self.sequence or len(payload)!=8:raise ValueError('Incorrect image DONE')
            total,checksum=struct.unpack('<II',payload);self.active=False
            if total!=len(self.data) or len(self.rows)!=self.h:raise ValueError('Incomplete image; capture discarded')
            if self.format==2:
                from codec import decode_checked_frame
                # The BKB batch decoder verifies every decoded row CRC before
                # returning. Avoid 1080 separate Python/ctypes round trips.
                try:self.data[:]=decode_checked_frame(self.compressed_rows,self.w,self.h)
                except ValueError as exc:raise ValueError('Decompressed row CRC or codec validation failed') from exc
                digest=0
                for _,_,verified_crc in self.compressed_rows.values():digest^=verified_crc
                actual=digest;self.metadata.update(raw_bytes=total,compressed_bytes=self.wire_bytes,compression_ratio=total/max(1,self.wire_bytes))
            else:actual=zlib.crc32(self.data)
            if actual!=checksum:
                raise ValueError('Incomplete image or image CRC mismatch; capture discarded')
            return self.w,self.h,bytes(self.data),self.frame
        return None

def rgb565_to_image(w,h,data):
    import numpy as np
    from PIL import Image
    pixels=np.frombuffer(data,dtype='<u2').reshape(h,w)
    r=(pixels>>11)&31;g=(pixels>>5)&63;b=pixels&31
    rgb=np.stack(((r<<3)|(r>>2),(g<<2)|(g>>4),(b<<3)|(b>>2)),axis=-1).astype('uint8')
    return Image.fromarray(rgb)


def controls_match(packet, requested):
    return packet.get('roi_enabled') is not None and packet['control']==requested
