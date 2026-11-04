"""Latest-frame-only ROI measurements. No geometry correction or residual trimming."""
import queue
import threading
import time
import numpy as np
from quality_core import tls, sfr
from edge_detection import locate_rows


def measure(image, roi, axis='vertical', gamma='linear'):
    start = time.perf_counter()
    x0,y0,x1,y1 = map(int, roi)
    a = np.asarray(image.crop((x0,y0,x1,y1)).convert('RGB'), dtype=float)
    if min(a.shape[:2]) < 32: raise ValueError('选区至少 32×32；建议框住一条 128×256 的斜边')
    gray = a @ np.array([.2126,.7152,.0722])
    if axis == 'horizontal': gray = gray.T
    detection=locate_rows(gray)
    positions=detection['positions'];good=detection['good'];rows=detection['rows']
    coverage=float(good.mean())
    if good.sum()<30 or coverage<.8: raise ValueError(f'单一边缘覆盖 {coverage:.0%}；请让一条边贯穿框，避开交叉边/反光')
    points=np.c_[positions[good],rows[good]]
    metric,residual=tls(points)
    # No sample is removed because its residual is large. Curves remain curves.
    if axis=='horizontal': points=points[:,::-1]
    points+=np.array([x0,y0])
    result=dict(straightness=metric,coverage=coverage,points=points.tolist(),residual_px=residual.tolist(),
                roi=list(roi),resolution=list(image.size),axis=axis,linearization=gamma,
                detector=dict(method=detection['method'],noise_sigma=float(np.median(detection['noise_sigma'])),contrast_to_noise=float(np.median(detection['cnr']))))
    try:
        optical=sfr(a,gamma=gamma,axis=axis)
        reasons=list(optical.get('rejection_reasons',[]))
        if coverage<.95:reasons.append(f'单边覆盖率 {coverage:.1%}，需要至少95%')
        optical['engineering_valid']=not reasons
        result['sfr']=optical
        if reasons:result['sfr_reason']='；'.join(reasons)
    except (ValueError,FloatingPointError) as exc:
        result['sfr_reason']=str(exc)
        if getattr(exc,'angle',None) is not None:result['sfr']=dict(angle_degrees=exc.angle,engineering_valid=False)
    result['compute_ms']=(time.perf_counter()-start)*1000
    return result


class LatestMeasurement:
    def __init__(self):
        self.pending=queue.Queue(1);self.results=queue.Queue(1);self.stop=threading.Event()
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    @staticmethod
    def replace(q,value):
        try:q.put_nowait(value)
        except queue.Full:
            try:q.get_nowait()
            except queue.Empty:pass
            try:q.put_nowait(value)
            except queue.Full:pass
    def submit(self,image,roi,axis,gamma,identity):
        self.replace(self.pending,(image,tuple(roi),axis,gamma,dict(identity)))
    def run(self):
        while not self.stop.is_set():
            try:image,roi,axis,gamma,identity=self.pending.get(timeout=.1)
            except queue.Empty:continue
            try:result=measure(image,roi,axis,gamma)
            except (ValueError,ArithmeticError,np.linalg.LinAlgError) as exc:result=dict(error=str(exc))
            result.update(identity=identity,completed_at=time.monotonic());result['_source_image']=image;self.replace(self.results,result)
