"""Query-only performance capture; closes its own serial port on completion.
Run with the existing host parameter COM disconnected. No image/control writes.
"""
from pathlib import Path
import argparse,json,time,sys
from decode_performance import decode
def packets(buffer):
 while True:
  at=buffer.find(b'\xaa\x55')
  if at<0:
   if len(buffer)>1:del buffer[:-1]
   return
  if at:del buffer[:at]
  if len(buffer)<4:return
  length=buffer[3]
  if length!=68:del buffer[0];continue
  total=5+length
  if len(buffer)<total:return
  p=bytes(buffer[:total]);del buffer[:total]
  if sum(p[:-1])&255==p[-1]:yield p[2],p[4:-1]
def main():
 import serial
 ap=argparse.ArgumentParser();ap.add_argument('--port',required=True);ap.add_argument('--seconds',type=int,default=60);ap.add_argument('--output',type=Path,default=Path('hardware_performance.json'));a=ap.parse_args()
 if a.seconds<5:raise SystemExit('Use at least 5 seconds; 60 seconds recommended')
 results=[];seen=set();buffer=bytearray();query=bytes([0xa5,0x5a,0x60,0,0x5f]);next_query=0
 with serial.Serial(a.port,921600,timeout=.05,write_timeout=1) as link:
  deadline=time.monotonic()+a.seconds
  while time.monotonic()<deadline:
   if time.monotonic()>=next_query:link.write(query);next_query=time.monotonic()+.25
   buffer.extend(link.read(max(1,link.in_waiting)))
   for kind,payload in packets(buffer):
    if kind!=0x86:continue
    if int.from_bytes(payload[64:68],'little')!=1:raise RuntimeError('Unsupported performance payload version')
    try:r=decode(f'{int.from_bytes(payload[:64],"little"):0128x}',hardware_measured=True)
    except ValueError:continue # first second after power/reset has no snapshot yet
    if not r['one_second_window_valid'] or r['window'] in seen:continue
    seen.add(r['window']);results.append(r);print(f'window={r["window"]} input={r["accepted_source_fps"]} fresh={r["unique_valid_display_fps"]} scan={r["hdmi_scan_fps"]} latency_ms={r["source_first_pixel_to_hdmi_first_pixel_ms"]}',flush=True)
 a.output.write_text(json.dumps(dict(port=a.port,seconds=a.seconds,records=results,source='FPGA command60/type86 checked SUM8, source-owned hardware timestamps'),indent=2),encoding='utf-8')
 if not results:raise SystemExit('No performance response. Check correct V3 bitstream, COM port and connection.')
 print('Saved',a.output)
if __name__=='__main__':main()
