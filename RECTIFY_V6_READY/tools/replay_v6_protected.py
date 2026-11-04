"""Replay V4-specific pixel and measurement fixtures; no board I/O."""
from pathlib import Path
import subprocess,json,hashlib
from sim_runtime import prepare_iverilog
root=Path(__file__).resolve().parents[1];iv,lib,vvp=prepare_iverilog(root)
out=root/'verification/v6/protected_replay';out.mkdir(parents=True,exist_ok=True)
geometry=['source/rtl/'+n for n in ['rect_frame_engine.v','rect_coordinate_tiles.v','rect_projective_pipe.v','rect_wide_mul_pipe.v','rect_imu_affine_pipe.v','rect_tile_sampler.v','rect_lens_field.v','rect_brown_node.v','bilinear_rgb565_pipe.v','imu_rotation_coeff.v','gamma_rgb_preserve.v']]
cases=[('render_gamma','tb_frame_engine',['verification/v4/tb_render_gamma.v']+geometry),
 ('sfr_math','tb_sfr_math',['verification/v4/tb_sfr_math.v','source/evaluation/sfr_meter.v']),
 ('live_roi','tb_live_roi',['verification/v4/tb_live_roi.v']+['source/evaluation/'+n for n in ['roi_evaluator.v','roi_edge_core.v','straightness_meter.v','sfr_meter.v','measurement_window.v']]),
 ('frame_gamma_reader','tb_network_zero_latency',['verification/v4/tb_frame_gamma_reader.v','source/rtl/network_pair_reader.v','source/rtl/network_read_arbiter.v','source/transport/cdc_mailbox.v']),
 ('gamma_exact','tb_gamma_exact',['verification/v4/tb_gamma_exact.v','verification/v4/gamma_reference.v','source/rtl/gamma_rgb_preserve.v']),
 ('scene_adaptation','tb_scene_adaptation',['verification/v4/tb_scene_adaptation.v','source/rtl/adaptive_preprocess_osd.v','source/rtl/scene_light_reference.v','source/rtl/gamma_rgb_preserve.v']),
 ('boot_controls','tb_boot_controls',['verification/v4/tb_boot_controls.v','source/rtl/return_pc_bridge.v','source/transport/host_command_parser.v','source/transport/cdc_mailbox.v'])]
records=[]
for name,top,sources in cases:
 binary=out/(name+'.vvp')
 extra=['-i'] if name=='boot_controls' else []
 subprocess.run([str(iv),'-B'+str(lib),'-g2005-sv',*extra,'-I.','-s',top,'-o',str(binary),*sources],cwd=root,check=True)
 p=subprocess.run([str(vvp),'-M',str(lib),str(binary)],cwd=root,capture_output=True,text=True,timeout=300)
 (out/(name+'.log')).write_text(p.stdout+p.stderr,encoding='utf-8');print(p.stdout,flush=True)
 if p.returncode or 'PASS' not in p.stdout or 'ERROR:' in p.stdout:raise RuntimeError(name+' failed: '+p.stderr)
 records.append(dict(name=name,passed=True,inputs={n:hashlib.sha256((root/n).read_bytes()).hexdigest() for n in sources}))
(root/'verification/v6/protected_replay_result.json').write_text(json.dumps(dict(passed=True,hardware_tested=False,tests=records),indent=2),encoding='utf-8')
print('PASS V6 protected Gamma, frame-metadata and boot-control regressions',flush=True)
