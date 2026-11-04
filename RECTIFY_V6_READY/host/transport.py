"""USB-UART controls/telemetry; recoverable UDP frozen-frame transport."""
import queue
import socket
import struct
import threading
import time
import zlib
from protocol import TelemetryDecoder,ImagePacketDecoder,ImageAssembler,command


class Worker:
    def __init__(self,events):
        self.events=events;self.stop=threading.Event();self.lock=threading.RLock();self.thread=None
        self.dropped_events=0
    def emit(self,*event):
        try:self.events.put_nowait(event)
        except queue.Full:self.dropped_events+=1


class SerialClient(Worker):
    STALE_SECONDS=2.5
    def __init__(self,events):
        super().__init__(events);self.port=None;self.stream_enabled=True;self.period_ms=10
        self.last_valid=0.;self.next_retry=0.;self.retry_delay=self.STALE_SECONDS;self.last_uptime=None;self.last_video=None
    def open(self,name):
        if self.thread and self.thread.is_alive():raise RuntimeError('串口任务尚未结束')
        if not name.strip():raise ValueError('请选择参数 COM')
        self.stream_enabled=True;self.last_uptime=None;self.last_video=None
        self.stop.clear();self.thread=threading.Thread(target=self.run,args=(name,),daemon=True);self.thread.start()
    def _write(self,kind,payload=b''):
        if not self.port or self.stop.is_set():raise ConnectionError('参数串口未连接')
        data=command(kind,payload)
        if self.port.write(data)!=len(data):raise OSError('串口指令未完整发送')
    def send(self,kind,payload=b''):
        with self.lock:
            self._write(kind,payload)
            if kind==0x10 and len(payload)==1:
                self.stream_enabled=bool(payload[0]&1);self.next_retry=time.monotonic()+self.STALE_SECONDS
            elif kind==0x11 and len(payload)==2:self.period_ms=int.from_bytes(payload,'little')
    def upload_lens(self,profile,request_id,enabled=True,zoom=None):
        from lens import upload_commands
        commands=upload_commands(profile,request_id,enabled,zoom)
        with self.lock:
            for kind,payload in commands:self._write(kind,payload)
            self._write(0x12)
    def upload_geometry(self,slot,words):
        from geometry import upload_commands
        with self.lock:
            for kind,payload in upload_commands(slot,words):self._write(kind,payload)
            self._write(0x12)
    def _synchronize(self,now):
        with self.lock:
            self._write(0x21)
            self._write(0x11,self.period_ms.to_bytes(2,'little'))
            self._write(0x10,bytes((int(self.stream_enabled),)))
            self._write(0x12)
            self.next_retry=now+self.retry_delay
    def maintain(self,now):
        with self.lock:
            if self.stream_enabled and now-self.last_valid>=self.STALE_SECONDS and now>=self.next_retry:
                self._synchronize(now);self.retry_delay=min(10.,self.retry_delay*2)
                self.emit('serial_recovery','重新同步遥测')
    def close(self):self.stop.set()
    def run(self,name):
        decoder=TelemetryDecoder()
        try:
            import serial
            port=serial.Serial(name,921600,timeout=.05,write_timeout=2)
            with self.lock:self.port=port
            self.last_valid=time.monotonic();self.retry_delay=self.STALE_SECONDS
            self.emit('serial','connected',name+' · 921600 8N1');self._synchronize(self.last_valid)
            while not self.stop.is_set():
                size=min(4096,max(1,getattr(port,'in_waiting',0)))
                for packet in decoder.feed(port.read(size)):
                    self.last_valid=time.monotonic();self.retry_delay=self.STALE_SECONDS
                    self.next_retry=self.last_valid+self.STALE_SECONDS
                    if packet['kind']==0x82:
                        uptime=packet['uptime_ms']
                        if self.last_uptime is not None and uptime<self.last_uptime and self.last_uptime-uptime<0xf0000000:
                            self.emit('board_reset');self._synchronize(self.last_valid)
                        self.last_uptime=uptime;self.last_video=packet
                    self.emit('telemetry',packet)
                self.maintain(time.monotonic())
        except (ImportError,OSError,ValueError) as exc:
            if not self.stop.is_set():self.emit('error','参数串口',str(exc))
        finally:
            with self.lock:
                if self.port:
                    try:self.port.close()
                    except OSError:pass
                    self.port=None
            self.emit('serial','disconnected',f'坏包 {decoder.errors} · 事件丢弃 {self.dropped_events}')


class EthernetImageClient(Worker):
    IMAGE_PORT=8080
    INACTIVITY_SECONDS=4.
    CAPTURE_SECONDS=120.
    # Engineering grace window; longer than RTL's 20 x 100 ms ACK wait.
    FINAL_ACK_GRACE_SECONDS=3.
    def __init__(self,events,control):
        super().__init__(events);self.control=control;self.sock=None;self.capturing=False
        self.listen_ready=threading.Event();self.listen_error=None
        self.capture_sequence=None;self.last_chunk=None;self.last_capture_seconds=None
        self.started=0.;self.last_data=0.;self.completed_ack=None
        self.preview_session=None;self.preview_next_ping=0.;self.preview_frames=queue.Queue(maxsize=1)
        self.preview_latest=None;self.preview_notice=False;self.preview_decode_thread=None;self.preview_drops=0;self.preview_errors=0
        self.preview_last_received=0.;self.preview_incomplete=0
        self.datagrams=0;self.photo_datagrams=0;self.capture_datagrams=0;self.crc_errors=0;self.last_peer=None;self.receive_buffer_bytes=0
        self.completed_image=None;self.failed_capture=None
        self.ack_fallback_session=None;self.udp_ack_attempts=0;self.uart_ack_count=0;self.uart_safety_count=0
    def publish_completed(self,result,metadata):
        # Preserve one completed photo even if a modal/slow UI filled the
        # bounded event queue. A notification is optional; the image is not.
        with self.lock:self.completed_image=(result,metadata)
        self.emit('image',result,metadata)
    def take_completed(self):
        with self.lock:
            result=self.completed_image;self.completed_image=None;return result
    def acknowledge_completed(self,sequence):
        with self.lock:
            if self.completed_image and self.completed_image[1].get('sequence')==sequence:self.completed_image=None
    def take_failed(self):
        with self.lock:
            result=self.failed_capture;self.failed_capture=None;return result
    def send_image_ack(self,session,chunk,duplicate=False,force=False):
        # A retry is also a request to try the established UART path. This
        # recovers automatically if Windows routes broadcast to another NIC
        # or a firewall blocks UDP 8081. Old FPGA versions use UART throughout.
        video=getattr(self.control,'last_video',None) or {}
        # Protocol 13 advertises a four-block cumulative ACK window. Bound
        # UART safety traffic to that window, even if UDP send succeeds but
        # never reaches the FPGA. First/final/duplicate ACKs never wait for
        # another block. Older single-block senders retain per-block ACKs.
        cumulative=isinstance(video,dict) and video.get('photo_window')==4
        uart_due=not cumulative or chunk==0 or chunk%4==3 or duplicate or force
        fast=isinstance(video,dict) and video.get('udp_ack_capable') is True and self.sock is not None
        current=session==self.capture_sequence
        # One unconfirmed UDP ACK is enough to choose the established UART
        # for this photo. Repeating the 100ms board retry wait for EVERY block
        # can exceed the whole-image deadline. Old final ACKs cannot downgrade
        # an independently queued new photo.
        if duplicate:
            if current and fast:self.ack_fallback_session=session
            fast=False
        if current and self.ack_fallback_session==session:fast=False
        if fast:
            payload=b'PGAK'+struct.pack('<II',session,chunk)
            payload+=struct.pack('<I',zlib.crc32(payload))
            if current:self.udp_ack_attempts+=1
            try:self._send_udp_ack(payload)
            except OSError:
                if current:self.ack_fallback_session=session
                fast=False
        if uart_due and (not fast or cumulative):
            self.control.send(0x31,struct.pack('<II',session,chunk))
            if current:
                self.uart_ack_count+=1
                if fast:self.uart_safety_count+=1
    def _send_udp_ack(self,payload):
        # The fixed board IP has no ARP responder. A directed broadcast uses
        # the board's directly connected /24 route on multi-NIC Windows PCs.
        # Keep the old target for older boards and unknown peers; UART recovery
        # still handles a missing route, firewall, lost packet or lost final ACK.
        video=getattr(self.control,'last_video',None) or {}
        target='192.168.1.255' if video.get('photo_window')==4 and self.last_peer=='192.168.1.11' else '255.255.255.255'
        self.sock.sendto(payload,(target,8081))
    def acknowledge_failed(self,sequence):
        with self.lock:
            if self.failed_capture and self.failed_capture[0]==sequence:self.failed_capture=None
    def start_preview(self,session):
        if not self.sock:raise ConnectionError('请先启动网口接收')
        if self.capturing:raise RuntimeError('请等照片接收完成')
        with self.lock:
            self.control.send(0x32,b'\1'+struct.pack('<I',session))
            self.preview_session=session;self.preview_next_ping=0.;self.preview_latest=None;self.preview_notice=False
            self._discard_queued_preview()
            self.preview_last_received=time.monotonic();self.preview_incomplete=0
    def _discard_queued_preview(self):
        while True:
            try:self.preview_frames.get_nowait()
            except queue.Empty:return
    def stop_preview(self):
        with self.lock:
            old=self.preview_session;self.preview_session=None;self.preview_latest=None;self.preview_notice=False
            self._discard_queued_preview()
            if old is not None:
                try:self.control.send(0x32,b'\0'+struct.pack('<I',old))
                except (OSError,RuntimeError):pass
    def take_preview(self):
        with self.lock:
            result=self.preview_latest;self.preview_latest=None;self.preview_notice=False;return result
    def _decode_preview(self,decoder_stop):
        from preview import decode_frame
        while not self.stop.is_set() and not decoder_stop.is_set():
            try:item=self.preview_frames.get(timeout=.1)
            except queue.Empty:continue
            try:
                # A stopped/replaced session must not consume a full-frame
                # decode. Check again afterwards for a stop during decoding.
                with self.lock:
                    if self.preview_session!=item['sequence']:continue
                result=decode_frame(item)
                from codec import preview_rgb_image
                stamp=time.perf_counter();(width,height,raw,_),metadata=result
                metadata['rgb_image']=preview_rgb_image(width,height,raw)
                metadata['rgb_seconds']=time.perf_counter()-stamp
                with self.lock:
                    if self.preview_session!=item['sequence']:continue
                    if self.preview_latest is not None:self.preview_drops+=1
                    self.preview_latest=result
                    self.preview_last_received=time.monotonic()
                    if not self.preview_notice:
                        try:self.events.put_nowait(('preview_ready',));self.preview_notice=True
                        except queue.Full:pass
            except ValueError:self.preview_errors+=1
    def connect(self,address='0.0.0.0',port=8080):
        if self.thread and self.thread.is_alive():raise RuntimeError('以太网接收任务尚未结束')
        if int(port)!=self.IMAGE_PORT:raise ValueError('图像端口固定为 UDP 8080')
        # Reference project: bind synchronously before permitting any command.
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,16*1024*1024)
            sock.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
            sock.settimeout(.1);sock.bind((address,int(port)))
            self.receive_buffer_bytes=sock.getsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF)
            self.stop.clear();self.listen_ready=threading.Event();self.listen_error=None;self.sock=sock
            self.datagrams=0;self.photo_datagrams=0;self.crc_errors=0;self.last_peer=None
            self.thread=threading.Thread(target=self.run,args=(address,int(port),sock),daemon=True);self.thread.start()
        except Exception:
            sock.close();self.sock=None;raise
        if not self.listen_ready.wait(2) or self.sock is None:
            self.close();raise RuntimeError('UDP 接收工作线程未就绪，请查看连接日志')
        if self.listen_error:raise OSError(self.listen_error)
    def cancel(self,reason=None):
        with self.lock:
            active=self.capturing;self.capturing=False
            if active and reason:
                # A dropped event must not leave a failed photo eligible for
                # retry or for accepting a late image under its old context.
                self.failed_capture=(self.capture_sequence,reason)
                self.emit('capture_failed',self.capture_sequence,reason)
    def close(self):self.stop_preview();self.cancel('图像接收已断开');self.stop.set()
    def capture(self,sequence):
        self.stop_preview()
        with self.lock:
            if not self.sock or self.stop.is_set():raise ConnectionError('图像接收未启动')
            if self.capturing:raise RuntimeError('图像传输中')
            self.capture_sequence=sequence;self.capturing=True;self.started=time.monotonic()
            self.ack_fallback_session=None;self.udp_ack_attempts=0;self.uart_ack_count=0;self.uart_safety_count=0
            self.completed_image=None;self.failed_capture=None
            self.capture_datagrams=0;self.first_data_at=None;self.frame_decode_seconds=0.
            self.last_data=self.started;self.last_chunk=None
            try:self.control.send(0x30,sequence.to_bytes(4,'little'))
            except Exception:self.capturing=False;raise
    def run(self,address,port,sock):
        decoder_stop=threading.Event();decoder_thread=None
        try:
            from preview import PreviewAssembler
            preview=PreviewAssembler();preview_sid=None
            parser=ImagePacketDecoder();assembler=ImageAssembler();observed_sequence=None;last_progress=0.
            self.emit('ethernet','connected',f'{address}:{port}')
            decoder_thread=threading.Thread(target=self._decode_preview,args=(decoder_stop,),daemon=True)
            self.preview_decode_thread=decoder_thread;decoder_thread.start()
            self.listen_ready.set()
            while not self.stop.is_set():
                try:data,peer=sock.recvfrom(2048)
                except socket.timeout:data=b''
                now=time.monotonic()
                with self.lock:
                    if self.preview_session is not None and now>=self.preview_next_ping:
                        try:
                            self.control.send(0x32,b'\1'+struct.pack('<I',self.preview_session));self.preview_next_ping=now+1.
                        except (OSError,RuntimeError):self.stop_preview()
                if self.capturing:
                    if now-self.last_data>self.INACTIVITY_SECONDS:self.cancel('图像接收超时（4s 无有效块）')
                    elif now-self.started>self.CAPTURE_SECONDS:self.cancel('图像接收超时（120s）')
                if data:
                    self.datagrams+=1;self.last_peer=peer[0]
                    if data[:4] in (b'PGV1',b'PGV2'):
                        if self.preview_session is None:continue
                        if preview_sid!=self.preview_session:preview=PreviewAssembler();preview_sid=self.preview_session
                        try:
                            ready=preview.feed(data,self.preview_session)
                            self.preview_incomplete=preview.incomplete
                            if ready is not None:
                                try:self.preview_frames.put_nowait(ready)
                                except queue.Full:
                                    try:self.preview_frames.get_nowait()
                                    except queue.Empty:pass
                                    self.preview_frames.put_nowait(ready);self.preview_drops+=1
                        except ValueError:self.preview_errors+=1
                        continue
                    if len(data)<21 or data[:4]!=b'PGUD':continue
                    _,session,chunk,n,version=struct.unpack_from('<4sIIHH',data)
                    if not 0<n<=1400 or version not in (1,2) or len(data)!=20+n:continue
                    if zlib.crc32(data[:-4])!=struct.unpack_from('<I',data,len(data)-4)[0]:self.crc_errors+=1;continue
                    self.photo_datagrams+=1
                    with self.lock:
                        # A new capture command can be queued before the board
                        # sees the previous final ACK. Keep that single ACK
                        # receipt independent of the new session/parser state.
                        receipt=self.completed_ack
                        if receipt and now<receipt[2] and (session,chunk)==receipt[:2]:
                            if version==1:self.send_image_ack(session,chunk,duplicate=True)
                            continue
                        if session!=self.capture_sequence:continue
                        self.capture_datagrams+=1
                        if self.capturing and self.first_data_at is None:self.first_data_at=time.monotonic()
                        if not self.capturing:continue
                        if observed_sequence!=session:
                            parser=ImagePacketDecoder();assembler=ImageAssembler();observed_sequence=session
                        if self.last_chunk is not None and chunk<=self.last_chunk:
                            if version==1:self.send_image_ack(session,chunk,duplicate=True)
                            continue
                        if self.last_chunk is not None and chunk!=self.last_chunk+1:
                            if version==2:self.cancel('UDP 块缺失或乱序，本帧丢弃');assembler.active=False
                            continue
                        if version==2 and self.last_chunk is None and chunk!=0:self.cancel('首块丢失，本帧丢弃');continue
                        try:
                            before_errors=parser.errors;packets=parser.feed(data[16:-4])
                            if parser.errors!=before_errors:raise ValueError('图像包 CRC 错误')
                            if self.last_chunk is None and (not packets or packets[0][0]!=0x81 or packets[0][1]!=session):
                                raise ValueError('首块缺少图像 INFO')
                            if any(seq!=session for _,seq,_ in packets):raise ValueError('图像会话不一致')
                            # ACK receipt before the final whole-image decode.
                            # All PGUD and PGPK CRCs have already been checked.
                            if version==1:self.send_image_ack(session,chunk,force=any(kind==0x83 for kind,_,_ in packets))
                            result=None;final_decode_start=time.perf_counter() if any(kind==0x83 for kind,_,_ in packets) else None
                            for kind,seq,payload in packets:
                                if seq!=session:raise ValueError('图像会话不一致')
                                candidate=assembler.feed(kind,seq,payload)
                                if candidate is not None:result=candidate
                            self.last_chunk=chunk;self.last_data=now
                            if result is not None:
                                self.capturing=False;self.last_capture_seconds=time.monotonic()-self.started
                                self.frame_decode_seconds=time.perf_counter()-final_decode_start if final_decode_start is not None else 0.
                                self.completed_ack=(session,chunk,time.monotonic()+self.FINAL_ACK_GRACE_SECONDS)
                                self.publish_completed(result,dict(sequence=session,seconds=self.last_capture_seconds,transport_version=version,
                                    first_data_wait_seconds=self.first_data_at-self.started,receive_until_done_seconds=now-self.first_data_at,decode_verify_seconds=self.frame_decode_seconds,
                                    ack_transport='none' if version!=1 else 'uart_fallback' if self.ack_fallback_session==session else 'udp_pgak' if self.udp_ack_attempts else 'uart_legacy',
                                    udp_ack_attempts=self.udp_ack_attempts,uart_ack_count=self.uart_ack_count,uart_safety_count=self.uart_safety_count,**assembler.metadata))
                            elif assembler.active and now-last_progress>.15:
                                self.emit('progress',len(assembler.rows),1080);last_progress=now
                        except (ValueError,OSError) as exc:
                            self.cancel(str(exc));assembler.active=False
        except (ImportError,OSError,ValueError) as exc:
            self.listen_error=str(exc);self.listen_ready.set()
            if not self.stop.is_set():self.emit('error','以太网图像',str(exc))
        finally:
            self.stop_preview()
            decoder_stop.set()
            if decoder_thread:decoder_thread.join(1.)
            if sock:sock.close()
            with self.lock:
                self.cancel('图像接收已断开');self.sock=None
            self.listen_ready.set()
            self.emit('ethernet','disconnected',f'UDP {self.datagrams}包 · 照片 {self.photo_datagrams}包 · CRC 错误 {self.crc_errors} · 事件丢弃 {self.dropped_events}')


class CaptureDevice:
    def __init__(self,events):
        self.events=events;self.stop=threading.Event();self.thread=None;self.dropped_frames=0
        self.lock=threading.Lock();self.latest=None;self.notified=False;self.index=None
    def publish(self,image):
        # Keep at most one full-resolution preview frame, even while Tk is busy.
        with self.lock:
            if self.latest is not None:self.dropped_frames+=1
            self.latest=image
            if not self.notified:
                try:self.events.put_nowait(('capture_ready',));self.notified=True
                except queue.Full:pass
    def take_frame(self):
        with self.lock:
            frame=self.latest;self.latest=None;self.notified=False
            return frame
    def start(self,index):
        if self.thread and self.thread.is_alive():raise RuntimeError('采集设备已打开')
        if index<0:raise ValueError('采集设备编号不能小于 0')
        self.take_frame();self.index=index
        self.stop.clear();self.thread=threading.Thread(target=self.run,args=(index,),daemon=True);self.thread.start()
    def run(self,index):
        try:
            import cv2
            cv2.setNumThreads(1)
            from PIL import Image
            cap=cv2.VideoCapture(index,cv2.CAP_DSHOW)
            try:
                if not cap.isOpened():raise OSError('无法打开视频采集设备')
                while not self.stop.is_set():
                    ok,frame=cap.read()
                    if not ok:raise OSError('采集设备无画面')
                    self.publish(Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)))
                    self.stop.wait(.1)
            finally:cap.release()
        except Exception as exc:
            try:self.events.put_nowait(('error','视频采集',str(exc)))
            except queue.Full:pass
        finally:
            try:self.events.put_nowait(('capture_stopped',))
            except queue.Full:pass
