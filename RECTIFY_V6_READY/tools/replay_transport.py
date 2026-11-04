"""Source-bound V2 tests. Simulated pixels/files only; does not contact a board."""
from pathlib import Path
import argparse,subprocess,sys,json,struct,zlib,hashlib
from sim_runtime import prepare_iverilog
root=Path(__file__).resolve().parents[1];iv,lib,vvp=prepare_iverilog(root)
sys.path.insert(0,str(root/'host/deps'));sys.path.insert(0,str(root/'host'))
from codec import decode_row
from protocol import ImagePacketDecoder
ap=argparse.ArgumentParser();ap.add_argument('--native',action='store_true');args=ap.parse_args()
out=root/'.simulation';out.mkdir(exist_ok=True);results=[]
reader=['source/rtl/network_pair_reader.v','source/rtl/gamma_rgb_preserve.v','source/rtl/camera_gamma_table.v','source/transport/cdc_mailbox.v']
preview=reader+['source/transport/'+s for s in ['rect_preview_stream.v','preview_line_encoder.v','hd_preview_udp_tx.v']]
photo=reader+['source/transport/'+s for s in ['fast_snapshot_ctrl.v','rgb565_line_encoder.v','preview_line_encoder.v','uart_packet_tx.v']]
def run(name,top,source):
 binary=out/(name+'.vvp');p=subprocess.run([str(iv),'-B'+str(lib),'-g2005','-I.','-s',top,'-o',str(binary),*source],cwd=root,capture_output=True,text=True)
 if p.returncode:raise RuntimeError(p.stderr)
 p=subprocess.run([str(vvp),'-M',str(lib),str(binary)],cwd=root,capture_output=True,text=True,timeout=900)
 (out/(name+'.log')).write_text(p.stdout+p.stderr,encoding='utf-8')
 if p.returncode or 'PASS' not in p.stdout:raise RuntimeError(p.stdout+p.stderr)
 results.append(dict(name=name,passed=True,source_sha256={s:hashlib.sha256((root/s).read_bytes()).hexdigest() for s in source}));print('PASS '+name,flush=True)
def pattern(bank,x,y):
 if bank==1:r,g,b=(x+y)%32,(x+3*y)%64,(3*x+y)%32
 elif bank==2:r,g,b=((x+y)%2)*31,((x//3+y//2)%2)*63,((x+y//3)%2)*31
 elif bank==3:r,g,b=(x*7+y*11+x*y)%32,(x*13+y*7+x*y)%64,(x*3+y*17)%32
 else:r,g,b=7,35,21
 return r,g,b
def pixel(bank,x,y):
 r,g,b=pattern(bank,x,y);return (r<<11)|(g<<5)|b
def gold(bank,x,y):
 sx,sy=x*3//2,y*3//2;fx,fy=(1,3)[x%2],(1,3)[y%2]
 q=[pattern(bank,sx+dx,sy+dy) for dy in range(2) for dx in range(2)]
 c=[(q[0][i]*(4-fx)*(4-fy)+q[1][i]*fx*(4-fy)+q[2][i]*(4-fx)*fy+q[3][i]*fx*fy+8)//16 for i in range(3)]
 return (c[0]<<11)|(c[1]<<5)|c[2]
def check_preview(native):
 ow,oh,frames=(1280,720,1) if native else (64,48,4);rows=set();chunks={};packets=0
 name='v2_preview_native_packets.hex' if native else 'v2_preview_packets.hex'
 for line in (root/'verification'/name).read_text().splitlines():
  if not line:continue
  wire=bytes.fromhex(line);assert wire[:8]==b'\x55'*7+b'\xd5';eth=wire[8:-4];assert zlib.crc32(eth)==int.from_bytes(wire[-4:],'little')
  app=eth[42:];assert app[:4]==b'PGV2' and zlib.crc32(app[:-4])==int.from_bytes(app[-4:],'little')
  sid,frame,row,w,h,mode,model,codec,version,total,offset,n,rcrc=struct.unpack('<IIHHHBBBBHHHI',app[4:32]);assert (sid,w,h,mode,model,version)==(17,ow,oh,1,1,2)
  assert n==len(app)-36 and offset+n<=total
  key=frame,row;parts=chunks.setdefault(key,bytearray());assert len(parts)==offset;parts.extend(app[32:-4]);packets+=1
  if len(parts)!=total:continue
  raw=decode_row(codec,parts,ow);assert zlib.crc32(struct.pack('<H',row)+raw)==rcrc
  got=struct.unpack('<'+str(ow)+'H',raw);assert got==tuple(gold(frame-100,x,row) for x in range(ow)),key
  assert key not in rows;rows.add(key)
 assert len(rows)==frames*oh
 results[-1].update(frames=frames,pixels=frames*ow*oh,packets_crc_checked=packets)
run('preview','tb_v2',['verification/tb_v2_network.v']+preview);check_preview(False)
run('photo','tb_v2_photo',['verification/tb_v2_photo.v']+photo)
wire=bytes.fromhex((root/'verification/v2_photo_packets.hex').read_text());d=ImagePacketDecoder();packets=[]
for i in range(0,len(wire),37):packets+=d.feed(wire[i:i+37])
assert d.errors==0 and len(packets)==74
kind,seq,info=packets[0];assert (kind,seq,len(info))==(0x81,17,32)
assert struct.unpack('<BBHHHIII',info[:20])==(2,2,96,72,192,13824,0,101)
assert info[20:24]==bytes([1,1,2,1]) and struct.unpack('<Q',info[24:])[0]==0x1234567800000081
digest=0
for row,(kind,seq,payload) in enumerate(packets[1:-1]):
 yy,codec,reserved,crc=struct.unpack('<HBBI',payload[:8]);assert (kind,seq,yy,reserved)==(0x85,17,row,0)
 raw=decode_row(codec,payload[8:],96);assert zlib.crc32(struct.pack('<H',row)+raw)==crc;digest^=crc
 assert raw==struct.pack('<96H',*(pixel(1,x,row) for x in range(96)))
assert packets[-1]==(0x83,17,struct.pack('<II',13824,digest))
results[-1].update(pixels=96*72,packet_crcs=True,hdmi_vsync_held_low=True)
for name in ['tb_raw_edges','tb_raw_reset_grant','tb_network_zero_latency']:
 run(name,name,['verification/'+name+'_v2.v','source/rtl/network_pair_reader.v','source/rtl/network_read_arbiter.v','source/transport/cdc_mailbox.v'])
for name in ['tb_photo_progress','tb_photo_abort']:
 run(name,name,['verification/'+name+'_v2.v']+['source/transport/'+s for s in ['fast_snapshot_ctrl.v','rgb565_line_encoder.v','preview_line_encoder.v','uart_packet_tx.v']])
if args.native:
 run('preview_native','tb_v2',['verification/tb_v2_network_native.v']+preview);check_preview(True)
(root/'verification/v2_replay_result.json').write_text(json.dumps(dict(passed=True,hardware_tested=False,includes_native=args.native,tests=results),indent=2),encoding='utf-8')
print('PASS V2 CRC, exact pixels, cached/leased frame isolation, reset/cancellation',flush=True)
