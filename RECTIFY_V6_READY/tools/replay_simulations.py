"""Replay supplied source-bound fixtures without BKB/LHT source dependencies."""
from pathlib import Path
import argparse,subprocess,json,hashlib
from sim_runtime import prepare_iverilog
root=Path(__file__).resolve().parents[1];iv,lib,vvp=prepare_iverilog(root)
ap=argparse.ArgumentParser();ap.add_argument('--full-scan',action='store_true');ap.add_argument('--timeout',type=int,default=1200,help='Per-case simulator wall time; full 1080P scans can take several minutes.')
args=ap.parse_args()
render=['source/rtl/'+n for n in ['rect_frame_engine.v','rect_coordinate_tiles.v','rect_projective_pipe.v','rect_wide_mul_pipe.v','rect_imu_affine_pipe.v','rect_tile_sampler.v','rect_lens_field.v','rect_brown_node.v','bilinear_rgb565_pipe.v','imu_rotation_coeff.v','gamma_rgb_preserve.v']]
cases=[('controls','tb_controls',['verification/tb_controls.v','source/rtl/rect_board_ui.v','source/rtl/rect_geometry_bank.v','source/transport/host_command_parser.v'],[]),
 ('frame','tb_frame_engine',['verification/tb_frame_engine.v']+render,[]),
 ('partial','tb_frame_engine',['verification/tb_partial_frame.v']+render,[]),
 ('perspective','tb_frame_engine',['verification/tb_nonlinear_frame.v']+render,[]),
 ('scan','tb_freeze_scan',['verification/tb_freeze_scan.v','source/rtl/video_frame_scan.v'],[]),
 ('reciprocal','tb_projective',['verification/tb_reciprocal_stress.v','source/rtl/rect_projective_pipe.v','source/rtl/rect_wide_mul_pipe.v'],[])]
if args.full_scan:cases.append(('scan1080','tb_freeze_scan',['verification/tb_freeze_scan.v','source/rtl/video_frame_scan.v'],['-Ptb_freeze_scan.W=1920','-Ptb_freeze_scan.H=1080','-Ptb_freeze_scan.FIFO_BITS=11','-Ptb_freeze_scan.PREFILL=1536','-Ptb_freeze_scan.PAUSE_CYCLES=1000']))
out=root/'.simulation';out.mkdir(exist_ok=True);results=[]
for name,top,source,params in cases:
    binary=out/(name+'.vvp')
    p=subprocess.run([str(iv),'-B'+str(lib),'-g2005-sv','-I.','-s',top,*params,'-o',str(binary),*source],cwd=root,capture_output=True,text=True)
    if p.returncode:raise RuntimeError(p.stdout+p.stderr)
    p=subprocess.run([str(vvp),'-M',str(lib),str(binary)],cwd=root,capture_output=True,text=True,timeout=args.timeout)
    log=p.stdout+p.stderr;(out/(name+'.log')).write_text(log,encoding='utf-8')
    if p.returncode or 'PASS' not in log:raise RuntimeError(log)
    results.append(dict(name=name,passed=True,source_sha256={n:hashlib.sha256((root/n).read_bytes()).hexdigest() for n in source}));print('PASS '+name,flush=True)
    if name=='reciprocal':
        vectors=(root/'verification/reciprocal_stress_vectors.hex').read_text().splitlines();maximum=0
        for line in (root/'verification/reciprocal_stress_points.txt').read_text().splitlines():
            i,b,got_x,got_y=map(int,line.split());value=int(vectors[i],16);x=value&2047;y=(value>>11)&2047;matrix=value>>22
            words=[(matrix>>(32*j))&0xffffffff for j in range(9)];words=[z-2**32 if z&2**31 else z for z in words]
            nx=words[0]*x+words[1]*y+words[2];ny=words[3]*x+words[4]*y+words[5];d=words[6]*x+words[7]*y+words[8]
            bad=abs(d)<262144 or abs(d)>0xffffffff or any(abs(n)>=2**36 or (abs(n)>>4)>=abs(d) for n in [nx,ny])
            q=[(abs(n)*2**22//abs(d))*(-1 if (n<0)^(d<0) else 1) if d else 0 for n in [nx,ny]]
            bad=bad or any(abs(z)>=2**25 for z in q)
            if bool(b)!=bool(bad):raise RuntimeError('Incorrect reciprocal fault flag')
            if not b:maximum=max(maximum,abs(got_x-q[0]),abs(got_y-q[1]))
        if maximum>4:raise RuntimeError('Reciprocal precision exceeds 4 Q12 units')
        results[-1]['max_error_px']=maximum/4096
(out/'result.json').write_text(json.dumps(dict(passed=True,tests=results),indent=2),encoding='utf-8')
