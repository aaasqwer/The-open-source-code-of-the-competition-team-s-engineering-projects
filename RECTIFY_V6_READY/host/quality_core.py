"""Reproducible image-plane straightness and slanted-edge SFR evaluation.

No image enhancement, geometric correction, residual-based point deletion or
polynomial edge straightening is performed by this evaluator. A curved edge
must remain curved in the reported measurements. This is an engineering SFR
implementation, not a claim of ISO 12233 laboratory certification.
"""
from pathlib import Path
import argparse,json,hashlib,math
import numpy as np
from edge_detection import locate_rows
from PIL import Image,ImageDraw

def tls(points):
 p=np.asarray(points,float);c=p.mean(0);_,_,v=np.linalg.svd(p-c,full_matrices=False)
 d=v[0];normal=np.array([-d[1],d[0]]);res=(p-c)@normal;span=float(np.ptp((p-c)@d))
 return dict(samples=len(p),span_px=span,rms_px=float(np.sqrt(np.mean(res**2))),max_abs_px=float(np.max(np.abs(res))),
             normalized_rms_percent=float(100*np.sqrt(np.mean(res**2))/span)),res

def bilinear(a,x,y):
 x=np.asarray(x);y=np.asarray(y);ix=np.floor(x).astype(int);iy=np.floor(y).astype(int)
 if np.any(ix<0) or np.any(iy<0) or np.any(ix+1>=a.shape[1]) or np.any(iy+1>=a.shape[0]):raise ValueError('Edge sampling crosses image border')
 fx=x-ix;fy=y-iy
 return (a[iy,ix]*(1-fx)+a[iy,ix+1]*fx)*(1-fy)+(a[iy+1,ix]*(1-fx)+a[iy+1,ix+1]*fx)*fy

def trace_edges(gray,anchors,half_width=8):
 gray=np.asarray(gray,float);points=[];attempted=0;segment_coverage=[]
 for a,b in zip(np.asarray(anchors,float)[:-1],np.asarray(anchors,float)[1:]):
  d=b-a;length=np.linalg.norm(d)
  if length<20:raise ValueError('Each edge segment needs at least 20 pixels')
  n=np.array([-d[1],d[0]])/length
  t=np.linspace(.05,.95,max(2,int(length*.9)+1));centers=a+t[:,None]*d
  offset=np.arange(-half_width,half_width+.01,.25);q=centers[:,None,:]+offset[None,:,None]*n
  p=bilinear(gray,q[:,:,0],q[:,:,1]);g=np.gradient(p,.25,axis=1);energy=np.abs(g)
  peak=energy.argmax(1);r=np.arange(len(t));strength=energy[r,peak]
  # These checks assess detector ambiguity/contrast, never line residual.
  noise=np.median(np.abs(np.diff(p,axis=1)),axis=1)/.9538725524
  good=(peak>5)&(peak<len(offset)-6)&(strength>np.maximum(.6,noise*2.5))
  other=energy.copy();other[np.abs(np.arange(len(offset))[None,:]-peak[:,None])<=6]=0
  good&=other.max(1)<strength*.85
  floor=np.median(np.c_[energy[:,:4],energy[:,-4:]],axis=1)
  weights=np.maximum(g*np.sign(g[r,peak])[:,None]-floor[:,None],0)
  weights[np.abs(np.arange(len(offset))[None,:]-peak[:,None])>16]=0
  loc=(weights*offset).sum(1)/np.maximum(weights.sum(1),1e-12)
  points.extend((centers[good]+loc[good,None]*n).tolist());attempted+=len(t);segment_coverage.append(float(good.mean()))
 if len(points)<30:raise ValueError(f'独立检测仅得到{len(points)}个有效边缘点，需要至少30个。请把框沿边方向延长到80～256像素；若亮暗差很小，请补光。')
 metrics,res=tls(points);metrics.update(attempted=attempted,coverage=len(points)/attempted,segment_coverage=segment_coverage,
       valid=min(segment_coverage)>=.8,selection='gradient contrast and ambiguity only; no deletion based on straightness')
 return np.asarray(points),metrics,res

class SfrConditionError(ValueError):
 def __init__(self,reason,angle=None):
  super().__init__(reason);self.angle=angle

def sfr(image,gamma='srgb',axis='vertical'):
 a=np.asarray(image,float)/255.
 code_luma=a[...,:3]@np.array([.2126,.7152,.0722]) if a.ndim==3 else a
 clipped_fraction=float(((code_luma<=4/255)|(code_luma>=251/255)).mean())
 if gamma=='srgb':a=np.where(a<=.04045,a/12.92,((a+.055)/1.055)**2.4)
 elif gamma!='linear':a=a**float(gamma)
 if a.ndim==3:a=a[...,:3]@np.array([.2126,.7152,.0722])
 if axis=='horizontal':a=a.T
 h,w=a.shape
 if min(h,w)<32:raise ValueError('SFR ROI must be at least 32x32; preferably 80x160')
 # Estimate the edge from unchanged output-code luminance. ESF samples below
 # retain the requested linearization; localization never modifies the image.
 code=code_luma.T if axis=='horizontal' else code_luma
 detection=locate_rows(code*255.)
 good=detection['good'];pos=detection['positions'];rows=detection['rows']
 if good.sum()<30 or good.mean()<.95:
  raise SfrConditionError(f'单边可信覆盖率{good.mean():.1%}，SFR需要至少95%；请框住完整单边，避免交叉；低信噪比时请补光')
 polarity=detection['polarity']
 slope,intercept=np.polyfit(rows[good],pos[good],1);angle=math.degrees(math.atan(abs(slope)))
 edge_rms=float(np.sqrt(np.mean((pos[good]-slope*rows[good]-intercept)**2)))
 if not 2<=round(angle,2)<=8:raise SfrConditionError(f'边缘偏离水平/竖直 {angle:.2f}°，需要 2°～8°；建议约5°',angle)
 if np.min(pos)<10 or np.max(pos)>w-11:raise SfrConditionError('边缘离框边太近，请把边移到框中央',angle)
 yy,xx=np.mgrid[:h,:w];distance=(xx-slope*yy-intercept)/math.sqrt(1+slope*slope)
 half=min(24.,float(min(pos.min(),w-1-pos.max()))-2);step=.25;n=int(2*half/step)
 edges=np.linspace(-half,half,n+1);step=edges[1]-edges[0];index=np.floor((distance-edges[0])/step).astype(int)
 valid=(index>=0)&(index<n);count=np.bincount(index[valid],minlength=n);total=np.bincount(index[valid],weights=a[valid],minlength=n)
 have=count>0
 if have.mean()<.98:raise SfrConditionError('斜边的亚像素采样不足，请延长测试框或调到约5°',angle)
 esf=np.interp(np.arange(n),np.flatnonzero(have),total[have]/count[have]);esf*=polarity
 contrast=float(np.mean(esf[-8:])-np.mean(esf[:8]))
 if contrast<=0 or not np.isfinite(contrast):raise SfrConditionError('黑白两侧没有可靠亮度差，请补光',angle)
 lsf=np.gradient(esf,step)*np.hanning(n)
 f=np.fft.rfftfreq(n,step);mtf=np.abs(np.fft.rfft(lsf));mtf/=mtf[0]
 z=2*np.pi*f*step;correction=np.ones_like(z);correction[1:]=np.minimum(10,z[1:]/np.sin(z[1:]));mtf*=correction
 use=f<=.5;f=f[use];mtf=mtf[use];cross=np.flatnonzero(mtf<=.5)
 mtf50=None if len(cross)==0 else float(np.interp(.5,mtf[cross[0]-1:cross[0]+1][::-1],f[cross[0]-1:cross[0]+1][::-1]))
 reasons=[]
 if edge_rms>=.2:
  if np.median(detection['noise_sigma'])>1 and np.median(detection['cnr'])<20:reasons.append(f'暗光定位噪声较大（边缘位置RMS {edge_rms:.3f}px），无法可靠区分噪声与弯曲；请延长选框或补光')
  else:reasons.append(f'局部边缘偏离拟合直线 {edge_rms:.3f}px，超过SFR的0.200px门限')
 if clipped_fraction>=.01:reasons.append(f'亮度截断像素 {clipped_fraction:.1%}，需要低于1%；请降低过曝或补足暗部照明')
 return dict(method='4x slanted-edge ESF / central derivative / Hann / derivative correction',linearization=gamma,angle_degrees=angle,
   fitted_edge_rms_px=edge_rms,mtf50_cycles_per_pixel=mtf50,mtf_at_nyquist=float(np.interp(.5,f,mtf)),
   engineering_valid=not reasons,rejection_reasons=reasons,clipped_fraction=clipped_fraction,mtf50_above_nyquist=mtf50 is None,slant_range_degrees=[2,8],slant_gate_resolution_degrees=.01,certified_iso12233=False,frequency_cycles_per_pixel=f.tolist(),sfr=mtf.tolist())

def self_test():
 # Independent analytic Gaussian PSF; retain a known bowed edge in straightness.
 tests=[]
 yy,xx=np.mgrid[:256,:112]
 erf=np.vectorize(math.erf,otypes=[float])
 for sigma in [.8,1.4,2.2]:
  for angle in [3.,5.,8.]:
   d=(xx-56-math.tan(math.radians(angle))*(yy-128))/math.sqrt(1+math.tan(math.radians(angle))**2)
   a=255*(.15+.65*(1+erf(d/(sigma*math.sqrt(2))))/2)
   measured=sfr(a,gamma='linear');expected=math.sqrt(math.log(2))/(math.sqrt(2)*math.pi*sigma)
   assert abs(measured['mtf50_cycles_per_pixel']/expected-1)<.06,(sigma,angle,measured,expected)
   tests.append(dict(sigma=sigma,angle=angle,expected_mtf50=expected,measured_mtf50=measured['mtf50_cycles_per_pixel']))
 for amplitude in [0.,3.]:
  edge=50+.08*yy+amplitude*np.sin(np.pi*yy/255);a=255*(.1+.7*(1+erf((xx-edge)/math.sqrt(2)))/2)
  ys=np.linspace(10,245,9);anchors=np.c_[50+.08*ys+amplitude*np.sin(np.pi*ys/255),ys]
  _,metric,_=trace_edges(a,anchors)
  assert metric['rms_px']<.08 if amplitude==0 else metric['rms_px']>.65,metric
 return dict(passed=True,analytic_gaussian_cases=tests,bowed_edge_not_flattened=True)

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--manifest',type=Path);ap.add_argument('--output',type=Path,default=Path('quality_result.json'));ap.add_argument('--self-test',action='store_true');args=ap.parse_args()
 if args.self_test:result=self_test()
 else:
  spec=json.loads(args.manifest.read_text(encoding='utf-8'));records=[]
  for r in spec['images']:
   path=(args.manifest.parent/r['path']).resolve();im=Image.open(path).convert('RGB');a=np.asarray(im);gray=a@np.array([.2126,.7152,.0722]);overlay=im.copy();draw=ImageDraw.Draw(overlay);lines=[]
   for line in r.get('lines',[]):
    try:
     points,metric,res=trace_edges(gray,line['anchors'],line.get('half_width',8));lines.append(dict(name=line['name'],**metric,points=points.tolist(),signed_residual_px=res.tolist()))
     for x,y in points[::4]:draw.ellipse((x-1,y-1,x+1,y+1),fill='red')
    except (ValueError,ArithmeticError,np.linalg.LinAlgError) as exc:
     lines.append(dict(name=line['name'],valid=False,error=str(exc)))
   mtf=[]
   for roi in r.get('sfr_rois',[]):
    try:mtf.append(dict(name=roi['name'],**sfr(a[roi['y']:roi['y']+roi['h'],roi['x']:roi['x']+roi['w']],roi.get('gamma','srgb'),roi.get('axis','vertical'))))
    except ValueError as e:mtf.append(dict(name=roi['name'],engineering_valid=False,error=str(e)))
   overlay_path=args.output.with_name(args.output.stem+'_'+path.stem+'_edges.png');overlay.save(overlay_path)
   records.append(dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),resolution=im.size,role=r['role'],capture_metadata=im.info.get('RECTIFY_CAPTURE'),lines=lines,sfr_rois=mtf,overlay=str(overlay_path)))
  result=dict(records=records,scope='Metrics in original image pixels; compare only same resolution, pose and scale. No curve fitting/deletion by residual.')
 args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
 if not args.self_test:
  for record in records:
   print(f"{Path(record['path']).name}: {record['resolution'][0]}x{record['resolution'][1]}, role={record['role']}")
   for line in record['lines']:
    if 'error' in line:print(f"  {line['name']}: 无效：{line['error']}")
    else:print(f"  {line['name']}: RMS={line['rms_px']:.6f}px, max={line['max_abs_px']:.6f}px, coverage={line['coverage']:.1%}, valid={line['valid']}")
   for roi in record['sfr_rois']:print(f"  {roi['name']}: MTF50={roi.get('mtf50_cycles_per_pixel')} cycles/pixel, valid={roi['engineering_valid']}")
 print(json.dumps(result if args.self_test else dict(output=str(args.output),images=len(records)),ensure_ascii=True))
if __name__=='__main__':main()
