from pathlib import Path
import sys,argparse,tempfile,shutil,json,hashlib,datetime,subprocess,os,re
ap=argparse.ArgumentParser();ap.add_argument('--compile-only',action='store_true');ap.add_argument('--map-only',action='store_true');args=ap.parse_args()
root=Path(__file__).resolve().parents[1];out=root/'verification';out.mkdir(exist_ok=True)
# All initialized memories are required inputs, including diagnostic SFR ROMs.
for source in (root/'source').rglob('*.v'):
 for relative in re.findall(r'\$readmemh\(\s*"([^"\n]+)"',source.read_text(encoding='utf-8',errors='replace')):
  if not (root/relative).is_file():raise RuntimeError('Missing memory initialization file: '+relative+' referenced by '+str(source))
files=[p for folder in ['source','ipcore','calibration'] for p in (root/folder).rglob('*') if p.is_file()]
files+=[root/'build.tcl',root/'RECTIFY_BOARD.fdc',root/'tools/sources.json']
hashes={p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
stage=Path(tempfile.mkdtemp(prefix='rectify_board_build_'))
for p in files:
 q=stage/p.relative_to(root);q.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,q)
env=os.environ.copy();env['RECTIFY_COMPILE_ONLY']='1' if args.compile_only else '0';env['RECTIFY_MAP_ONLY']='1' if args.map_only else '0'
kind='compile' if args.compile_only else 'map' if args.map_only else 'full'
log=out/f'pds_{kind}_{stage.name}.log';started=datetime.datetime.now(datetime.timezone.utc)
with log.open('w',encoding='utf-8') as f:
 result=subprocess.run([os.environ.get('PDS_SHELL','E:/PDS/PDS_2022.2-SP6.4/bin/pds_shell.exe'),'-file','build.tcl','-work_dir','.', '-project_name','RECTIFY_BOARD'],
  cwd=stage,env=env,input='exit\n',text=True,stdout=f,stderr=subprocess.STDOUT,timeout=7200)
text=log.read_text(encoding='utf-8',errors='replace')
shutil.copy2(log,out/f'pds_{kind}.log')
stages=['compile'] if args.compile_only else ['compile','synthesize','dev_map'] if args.map_only else ['compile','synthesize','dev_map','pnr','report_timing','report_power','gen_netlist','gen_bit_stream']
success=result.returncode==0 and not re.search(r'^E:',text,re.M) and all(re.search(r'Executing : '+s+r'(?: [^\n]*)? successfully\.',text) for s in stages)
fresh=hashes=={p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
summary=dict(passed=bool(success and fresh),returncode=result.returncode,scope=kind,started_utc=started.isoformat(),staging_directory=str(stage),
 source_sha256=hashes,input_unchanged=fresh,hardware_tested=False,completed_stages=stages if success else [])
summary['resources']={}
resource_file=stage/('place_route/hdmi_ddr_ov5640_top.prr' if kind=='full' else 'device_map/hdmi_ddr_ov5640_top.dmr')
if resource_file.exists():
 resource_text=resource_file.read_text(encoding='utf-8',errors='replace')
 row=re.search(r'\|\s*hdmi_ddr_ov5640_top\s*\|\s*([\d.]+)\s*\|\s*([\d.]+)\s*\|\s*([\d.]+)\s*\|\s*([\d.]+)\s*\|\s*([\d.]+)\s*\|',resource_text)
 if row:
  summary['resources']=dict(zip(['LUT','FF','distributed_RAM','APM','DRM'],map(float,row.groups())))
  summary['resource_report']=resource_file.relative_to(stage).as_posix()
  summary['resource_report_sha256']=hashlib.sha256(resource_file.read_bytes()).hexdigest()
for name in ['RECTIFY_BOARD.pds']:
 if (stage/name).exists():shutil.copy2(stage/name,root/name)
reports=out/('reports_'+kind);reports.mkdir(exist_ok=True)
for folder in ['compile','synthesize','device_map','place_route','report_timing','report_power']:
 if (stage/folder).exists():
  for p in (stage/folder).rglob('*'):
   if p.is_file() and p.suffix.lower() in ['.rpt','.rtr','.prr','.prt','.dmr','.log','.txt','.fdc']:
    q=reports/p.relative_to(stage);q.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,q)
if kind=='full' and success and fresh:
 timing=stage/'report_timing/hdmi_ddr_ov5640_top.rtr';timing_text=timing.read_text(encoding='utf-8',errors='replace') if timing.exists() else ''
 clean='Design Summary : All Constraints Met.' in timing_text and '(VIOLATED)' not in timing_text
 summary['declared_constraints_met']=clean
 if clean:
  bit=stage/'generate_bitstream/hdmi_ddr_ov5640_top.sbit';assert bit.exists() and bit.stat().st_mtime>=started.timestamp()
  release=root/'release';release.mkdir(exist_ok=True);shutil.copy2(bit,release/'RECTIFY_BOARD.sbit');summary['bitstream_sha256']=hashlib.sha256(bit.read_bytes()).hexdigest()
 else:summary['passed']=False
(out/f'pds_{kind}_result.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in summary.items() if k!='source_sha256'},indent=2),flush=True)
if not summary['passed']:raise SystemExit('Build check failed; see '+str(log))
