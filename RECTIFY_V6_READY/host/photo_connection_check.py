"""Actual UDP photo receiver regression. Only OS-assigned loopback sockets; no COM/NIC changes."""
import json,queue,socket,struct,threading,time,zlib
from pathlib import Path
from transport import EthernetImageClient
from protocol import pgpk


def make_photo(sequence):
    width,height=1920,1080;pixels=[];packets=[];digest=0
    info=struct.pack('<BBHHHIII',2,2,width,height,width*2,width*height*2,0,611)+bytes([1,0,2,1])+struct.pack('<II',0x581,63)
    packets.append(pgpk(0x81,sequence,info))
    for row in range(height):
        value=(row*73)^0x73ad;value&=65535
        raw=struct.pack('<H',value)*width;pixels.append(raw)
        encoded=b'\0'+struct.pack('<H',value)*16+b'\2'*(width//16-1)
        checksum=zlib.crc32(struct.pack('<H',row)+raw);digest^=checksum
        packets.append(pgpk(0x85,sequence,struct.pack('<HBBI',row,1,0,checksum)+encoded))
    packets.append(pgpk(0x83,sequence,struct.pack('<II',width*height*2,digest)))
    return b''.join(packets),b''.join(pixels)


def run_photo_checks(payload_size=1400,drop_and_retry=True,stream=None,expected=None,frame_id=611):
    class Control:
        last_video={'photo_window':4,'udp_ack_capable':False}
        def __init__(self):self.condition=threading.Condition();self.acks=[]
        def send(self,kind,payload):
            if kind==0x31:
                sid,chunk=struct.unpack('<II',payload)
                with self.condition:self.acks.append((sid,chunk));self.condition.notify_all()
    sequence=struct.unpack_from('<I',stream,8)[0] if stream is not None else 816000+payload_size
    control=Control();client=EthernetImageClient(queue.Queue(256),control)
    client.IMAGE_PORT=0;client.connect('127.0.0.1',0);address=client.sock.getsockname()
    sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);retry_groups=0
    started=time.perf_counter()
    try:
        client.capture(sequence)
        if stream is None:stream,expected=make_photo(sequence)
        chunks=[stream[i:i+payload_size] for i in range(0,len(stream),payload_size)]
        def send(index,corrupt=False,oversize=False):
            data=chunks[index]+(b'\0' if oversize else b'')
            app=struct.pack('<4sIIHH',b'PGUD',sequence,index,len(data),1)+data
            app+=struct.pack('<I',zlib.crc32(app))
            if corrupt:app=app[:-1]+bytes([app[-1]^1])
            sender.sendto(app,address)
        # An invalid oversized packet and bad CRC must not retire any data.
        if payload_size==1400:send(0,oversize=True)
        send(0,corrupt=True)
        for first in range(0,len(chunks),4):
            end=min(len(chunks),first+4);target=end-1
            for index in range(first,end):
                if drop_and_retry and first==0 and index==2:continue
                send(index)
            deadline=time.monotonic()+3;retried=False
            with control.condition:
                while not any(s==sequence and c>=target for s,c in control.acks):
                    control.condition.wait(.02)
                    if drop_and_retry and first==0 and not retried:
                        for index in range(first,end):send(index)
                        retry_groups+=1;retried=True
                    if time.monotonic()>deadline:raise AssertionError(f'Photo ACK timeout, payload {payload_size}, chunk {target}; last {control.acks[-4:]}')
        deadline=time.monotonic()+5;result=None
        while time.monotonic()<deadline:
            result=client.take_completed()
            if result is not None:break
            time.sleep(.005)
        assert result is not None,'Complete photo not published'
        (width,height,raw,frame),metadata=result
        assert (width,height,frame)==(1920,1080,frame_id) and raw==expected
        assert metadata['frame_geometry_flags']==0x581 and client.crc_errors>=1
        assert not client.capturing and client.failed_capture is None
        return dict(passed=True,payload_bytes=payload_size,datagrams=len(chunks),exact_pixels=width*height,
                    all_packet_row_frame_crcs=True,frame_gamma_metadata_preserved=True,oversize_rejected=payload_size==1400,
                    corrupted_packet_rejected=True,missing_chunk_retried=retry_groups>0,
                    loopback_seconds=time.perf_counter()-started,scope='Loopback receiver/decoder only; not physical camera capture timing')
    finally:
        sender.close();client.close()
        if client.thread:client.thread.join(3)


def run(destination):
    result=dict(passed=True,hardware_connected=False,tests=[run_photo_checks(1400),run_photo_checks(1024)])
    Path(destination).write_text(json.dumps(result,indent=2),encoding='utf-8')
    return result
