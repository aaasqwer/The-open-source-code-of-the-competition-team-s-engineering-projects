"""Read and verify the shipped own-photo K / Brown coefficients / FPGA words."""
from pathlib import Path
import argparse,json,math,re,struct,zlib,hashlib
def audit(root):
 path=root/'calibration/profile.json';p=json.loads(path.read_text(encoding='utf-8'));k=p['K'];d=p['B_D']
 if len(k)!=3 or any(len(r)!=3 for r in k) or len(d)!=4:raise ValueError('Expected K 3x3 and Brown k1,k2,p1,p2')
 if not all(math.isfinite(v) for row in k for v in row) or not all(math.isfinite(v) for v in d):raise ValueError('Non-finite parameter')
 if p['resolution']!=[1920,1080] or not(0<k[0][0] and 0<k[1][1] and 0<=k[0][2]<1920 and 0<=k[1][2]<1080):raise ValueError('Resolution/focal/principal point invalid')
 if any(abs(k[i][j]-value)>1e-9 for i,j,value in [(0,1,0),(1,0,0),(2,0,0),(2,1,0),(2,2,1)]):raise ValueError('Unsupported skew/noncanonical K')
 values=[round(v*65536) for v in [k[0][0],k[1][1],k[0][2],k[1][2]]]+[round(v*2**24) for v in d]+[round(2**30/k[0][0]),round(2**30/k[1][1]),2**20,1]
 if values!=p['packed_parameters']:raise ValueError('Profile words mismatch K/D')
 text=(root/'calibration/profile_config.vh').read_text(encoding='utf-8');raw=int(re.search(r"384'h([0-9a-fA-F]+)",text)[1],16)
 actual=[(raw>>(32*i))&0xffffffff for i in range(12)]
 if actual!=[v&0xffffffff for v in values]:raise ValueError('Compiled FPGA defaults differ from profile')
 return dict(passed=True,scope='Parse and representation audit; not an optical accuracy certificate',resolution=p['resolution'],K=k,Brown_order='k1,k2,p1,p2',Brown_D=d,
  q_formats=['fx/fy/cx/cy Q16','k1/k2/p1/p2 Q24','reciprocal focal Q30','inverse zoom Q20','enabled'],
  max_intrinsic_quantization_error_px=max(abs(a-b/65536) for a,b in zip([k[0][0],k[1][1],k[0][2],k[1][2]],values[:4])),
  words_crc32=f'{zlib.crc32(struct.pack("<12i",*values)):08x}',profile_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),no_old_camera_parameters=p['no_old_camera_parameters'])
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--project',type=Path,default=Path(__file__).resolve().parents[1]);ap.add_argument('--output',type=Path,default=Path('calibration_audit.json'));a=ap.parse_args();r=audit(a.project)
 a.output.write_text(json.dumps(r,indent=2),encoding='utf-8');print('PASS camera matrix / Brown order / exact compiled parameters / CRC; optical accuracy evaluated separately.')
if __name__=='__main__':main()
