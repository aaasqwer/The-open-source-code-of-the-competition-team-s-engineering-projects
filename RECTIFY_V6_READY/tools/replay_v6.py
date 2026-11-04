"""Replay V6 controls, frame pose, lossless photo/window recovery and SFR hold. No board I/O.
Set IVERILOG_HOME to your own Icarus installation. Results go in .simulation/v6.
"""
from pathlib import Path
import sys,subprocess,json,hashlib,re,struct,zlib
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'tools'))
from sim_runtime import prepare_iverilog
iv,lib,vvp=prepare_iverilog(root);out=root/'.simulation/v6';out.mkdir(parents=True,exist_ok=True);records=[]
def run(name,top,sources,extra=(),args=()):
 binary=out/(name+'.vvp')
 p=subprocess.run([str(iv),'-B'+str(lib),'-g2005' if name.startswith('window') else '-g2005-sv','-I.','-s',top,*extra,'-o',str(binary),*sources],cwd=root,capture_output=True,text=True)
 if p.returncode:raise RuntimeError(p.stderr)
 p=subprocess.run([str(vvp),'-M',str(lib),str(binary),*args],cwd=root,capture_output=True,text=True,timeout=300)
 (out/(name+'.log')).write_text(p.stdout+p.stderr,encoding='utf-8')
 print(p.stdout,flush=True)
 if p.returncode or 'PASS' not in p.stdout or 'ERROR:' in p.stdout:raise RuntimeError(name+': '+p.stderr)
 records.append(dict(name=name,passed=True,inputs={s:hashlib.sha256((root/s).read_bytes()).hexdigest() for s in sources}))
 return p.stdout
run('controls','tb_controls',['verification/tb_controls.v','source/rtl/rect_board_ui.v','source/rtl/rect_geometry_bank.v','source/transport/host_command_parser.v'])
run('pose_timing','tb_capture_time',['verification/v5/tb_pose_timing.v','source/rtl/camera_frame_writer.v'])
run('parameter_overlay','tb_v5_parameter_overlay',['verification/v5/tb_parameter_overlay.v','source/rtl/status_cn_overlay.v'])
run('photo_fence','tb_photo_fence',['verification/v5/tb_photo_fence.v','source/rtl/return_pc_bridge.v','source/transport/cdc_mailbox.v'],['-i'])
for window,loss in [(1,0),(4,0),(4,1)]:
 name=f'window_{window}_{loss}';tb=(root/'verification/v5/tb_pc4_window.v').read_text(encoding='utf-8')
 if window==1:
  tb=re.sub(r'  if\(WINDOW==4&&.*?;\n','',tb)
  tb=re.sub(r' initial begin #10000000;.*?end\n',' initial begin #10000000;$fatal(1,"timeout");end\n',tb)
 source=out/(name+'.v');source.write_text(tb,encoding='utf-8');wire=out/(name+'.hex')
 log=run(name,'tb_pc4_window',[source.relative_to(root).as_posix(),'source/transport/ethernet_image_tx.v','source/transport/ethernet_photo_window.v'],
  [f'-Ptb_pc4_window.WINDOW={window}',f'-Ptb_pc4_window.LOSS={loss}'],['+OUT='+wire.relative_to(root).as_posix()])
 seen={};duplicates=0
 for line in wire.read_text().splitlines():
  frame=bytes.fromhex(line);assert frame[:8]==b'\x55'*7+b'\xd5';mac=frame[8:]
  assert zlib.crc32(mac[:-4])==struct.unpack_from('<I',mac,len(mac)-4)[0]
  assert sum(struct.unpack('>10H',mac[14:34]))%65535==0
  length=struct.unpack_from('>H',mac,38)[0];app=mac[42:34+length]
  magic,sid,seq,n,version=struct.unpack_from('<4sIIHH',app)
  assert (magic,sid,version)==(b'PGUD',0x34567890,1) and n<=1400 and len(app)==n+20
  assert zlib.crc32(app[:-4])==struct.unpack_from('<I',app,len(app)-4)[0]
  if seq in seen:assert seen[seq]==frame;duplicates+=1
  else:seen[seq]=frame
  start=seq*1400;assert app[16:-4]==bytes((k*73+(k>>4))&255 for k in range(start,min(14101,start+1400)))
 assert set(seen)==set(range(11))
 if loss:assert duplicates>=2
 records[-1].update(cycles=int(re.search(r'CYCLES=(\d+)',log)[1]),duplicates=duplicates,exact_mac_crc=True)
run('window_timeout','tb_pc4_window_timeout',['verification/v5/tb_pc4_window_timeout.v','source/transport/ethernet_photo_window.v'])
run('photo_encoder_exact','tb_photo_encoder_exact',['verification/v5/tb_photo_encoder_exact.v','source/transport/rgb565_line_encoder.v','source/transport/preview_line_encoder.v'])
run('sfr_hold','tb_sfr_hold',['verification/v5/tb_sfr_hold.v']+['source/evaluation/'+n for n in ['roi_evaluator.v','roi_edge_core.v','straightness_meter.v','sfr_meter.v','measurement_window.v']])
run('mapping_v6','tb_v6_mapping',['verification/v6/tb_v6_mapping.v','source/rtl/rect_brown_node.v'])
run('hidden_osd','tb_v6_hidden_osd',['verification/v6/tb_hidden_osd.v','source/rtl/status_cn_overlay.v','source/rtl/imu_hud_overlay.v','source/rtl/imu_font_cn.v'])
run('dynamic_parameters','tb_dynamic_parameters',['verification/v6/tb_dynamic_parameters.v','source/rtl/status_cn_overlay.v'])
run('lens_pipeline','tb_lens_pipeline',['verification/v6/tb_lens_pipeline.v','verification/v6/lens_field_before.v','source/rtl/rect_lens_field.v','source/rtl/rect_brown_node.v'])
(out/'result.json').write_text(json.dumps(dict(passed=True,hardware_tested=False,tests=records),indent=2),encoding='utf-8')
print('PASS V6 controls, dynamic frame pose and exact reliable photo window, including dropped packets/ACK timeout',flush=True)
