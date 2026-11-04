"""Local-only connection regression: never opens a physical COM port."""
from pathlib import Path
import json,queue,socket,struct,time,zlib,sys
from unittest.mock import Mock
from transport import EthernetImageClient,SerialClient
from preview import HEADER_V2
from serial_ports import diagnostic
from protocol import command


def run(destination):
    events=queue.Queue(256);control=Mock();client=EthernetImageClient(events,control)
    # Use the OS-assigned loopback test port. Production remains UDP 8080.
    client.IMAGE_PORT=0;client.connect('127.0.0.1',0);address=client.sock.getsockname()
    duplicate=EthernetImageClient(queue.Queue(),control);duplicate.IMAGE_PORT=address[1]
    busy_rejected=False
    try:duplicate.connect(*address)
    except OSError:busy_rejected=True
    assert busy_rejected and duplicate.sock is None,'Occupied port must report failure'
    sequence=194125;client.start_preview(sequence)
    sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    p=0x73ad;raw=struct.pack('<H',p)*1280;data=struct.pack('<BH',0,0) # replaced below
    # A literal first group sets the predictor, remaining groups are repeats.
    encoded=b'\0'+struct.pack('<H',p)*16+b'\2'*79
    for row in range(720):
        crc=zlib.crc32(struct.pack('<H',row)+raw)
        header=HEADER_V2.pack(b'PGV2',sequence,31,row,1280,720,1,0,1,2,len(encoded),0,len(encoded),crc)
        packet=header+encoded;packet+=struct.pack('<I',zlib.crc32(packet))
        sender.sendto(packet,address)
        if row%60==0:time.sleep(.001)
    deadline=time.monotonic()+5;result=None
    while time.monotonic()<deadline:
        result=client.take_preview()
        if result is not None:break
        time.sleep(.01)
    assert result is not None,'Complete loopback preview did not arrive'
    (w,h,pixels,frame),metadata=result
    assert (w,h,frame)==(1280,720,31) and pixels==raw*720
    r,g,b=p>>11,(p>>5)&63,p&31;expected=((r<<3)|(r>>2),(g<<2)|(g>>4),(b<<3)|(b>>2))
    assert metadata['rgb_image'].getpixel((100,100))==expected
    client.stop_preview();client.close();client.thread.join(2);assert not client.thread.is_alive()
    client.IMAGE_PORT=address[1];client.connect(*address);client.close();client.thread.join(2)
    assert not client.thread.is_alive(),'Receiver did not reconnect and close cleanly'
    sender.close()
    serial=SerialClient(queue.Queue());serial.port=Mock();serial.port.write.side_effect=lambda b:len(b)
    serial.send(0x32,b'\1'+struct.pack('<I',sequence));serial.port.write.assert_called_with(command(0x32,b'\1'+struct.pack('<I',sequence)))
    serial.port.write.side_effect=lambda b:len(b)-1
    try:serial.send(0x32,b'\0'+struct.pack('<I',sequence))
    except OSError:pass
    else:raise AssertionError('Partial serial write accepted')
    report=dict(passed=True,frozen=bool(getattr(sys,'frozen',False)),hardware_connected=False,production_udp_port=EthernetImageClient.IMAGE_PORT,
                loopback_preview_pixels=w*h,packet_and_row_crc_verified=metadata['CRC_verified'],native_rgb_verified=True,
                occupied_port_reported=True,receiver_reconnected=True,serial_command_exact=True,partial_uart_write_rejected=True,
                port_diagnostic=diagnostic())
    from photo_connection_check import run_photo_checks
    report['full_photo_receiver']=[run_photo_checks(1400),run_photo_checks(1024)]
    Path(destination).write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return report
