"""Small image checks on actual loaded/captured pixels; no nominal scores."""
def image_metrics(image,roi=None):
    import numpy as np
    region=image.crop(roi) if roi else image
    gray=np.asarray(region.convert('L'),dtype=np.float32)
    if min(gray.shape)<16:raise ValueError('ROI 至少需要 16×16 像素')
    dx=np.diff(gray,axis=1);dy=np.diff(gray,axis=0)
    lap=gray[1:-1,:-2]+gray[1:-1,2:]+gray[:-2,1:-1]+gray[2:,1:-1]-4*gray[1:-1,1:-1]
    result={'亮度均值':float(gray.mean()),'亮度标准差':float(gray.std()),'梯度均方':float((dx*dx).mean()+(dy*dy).mean()),'拉普拉斯方差':float(lap.var())}
    # Preserve the old metric names, using the same corrected detector as the
    # new comparison instead of displaying contradictory validity decisions.
    try:
        profile=edge_straightness(image,roi or (0,0,image.width,image.height),'auto')
        result['ROI 直线度 RMS(px)']=profile['rms_px']
        result['ROI 最大偏差(px)']=profile['max_px']
        result['边缘方向']=profile['geometry'];result['边缘角度(°)']=profile['angle_degrees']
        result['有效边缘行数']=profile['samples']
    except ValueError:result['直线度状态']='未形成可靠单条边缘'
    return result


def edge_straightness(image, roi, orientation='auto'):
    """Measure one physical straight edge; never infer lens calibration from a scene."""
    import numpy as np
    if roi is None:
        raise ValueError('请先框选同一条实际直边；每个 ROI 只保留一条边，避免棋盘交点。')
    x0, y0, x1, y1 = map(int, roi)
    if not (0 <= x0 < x1 <= image.width and 0 <= y0 < y1 <= image.height):
        raise ValueError('ROI 超出图像范围，请重新框选。')
    gray = np.asarray(image.crop((x0,y0,x1,y1)).convert('L'), dtype=np.float32)
    if min(gray.shape) < 16:
        raise ValueError('ROI 至少需要 16×16 像素，并沿直边保留足够长度。')
    if orientation not in ('auto', 'vertical', 'horizontal','oblique'):
        raise ValueError('未知的边缘方向。')

    # Five-tap separable Gaussian suppresses sensor noise. Comparing the second
    # strongest edge to the first avoids rejecting a long noisy wall simply
    # because its many tiny gradients sum to more than one real edge.
    smoothed=gray
    for dim in (0,1):
        padding=[(0,0),(0,0)];padding[dim]=(2,2)
        padded=np.pad(smoothed,padding,mode='edge');filtered=np.zeros_like(smoothed)
        for shift,weight in enumerate((1,4,6,4,1)):
            slices=[slice(None),slice(None)];slices[dim]=slice(shift,shift+smoothed.shape[dim])
            filtered+=padded[tuple(slices)]*(weight/16)
        smoothed=filtered
    diagnostics=[]
    def measure(axis):
        data = smoothed if axis == 'vertical' else smoothed.T
        signed = np.diff(data, axis=1)
        energy = np.abs(signed)
        # Do not use a crop-boundary artefact as a measured physical edge.
        energy[:,:2]=0;energy[:,-2:]=0
        peaks = energy.max(axis=1)
        background=np.median(energy,axis=1)
        threshold=np.maximum(3.,background*5)
        independent, positions, polarities = [], [], []
        for row in np.flatnonzero(peaks > threshold):
            edge = int(energy[row].argmax())
            lo, hi = max(0, edge - 4), min(energy.shape[1], edge + 5)
            other=max(float(energy[row,:lo].max(initial=0)),float(energy[row,hi:].max(initial=0)))
            if other>peaks[row]*.8:continue
            weights=np.maximum(0,energy[row,lo:hi]-background[row]*1.5)
            main=float(weights.sum())
            if main<=0:continue
            independent.append(float(row));polarities.append(float(np.sign(signed[row,edge])))
            positions.append(float((np.arange(lo,hi)*weights).sum()/main+.5))
        diagnostics.append((axis,len(positions),data.shape[0]))
        if len(positions) < max(16, data.shape[0] // 2):
            return None
        t, position = np.asarray(independent), np.asarray(positions)
        # A smoothly curved edge is retained. Isolated jumps to noise/texture
        # are rejected using local continuity, not distance from a straight fit.
        padded=np.pad(position,(4,4),mode='edge')
        local=np.median(np.stack([padded[k:k+len(position)] for k in range(9)]),axis=0)
        keep=np.abs(position-local)<=max(3.,data.shape[1]*.01)
        t,position=t[keep],position[keep]
        polarity=np.asarray(polarities)[keep]
        if len(t)<max(16,data.shape[0]//2) or abs(polarity.mean())<.8:return None
        if np.ptp(t) < data.shape[0] * .5:
            return None
        centered_t=t-t.mean();centered_position=position-position.mean()
        slope=float(np.sum(centered_t*centered_position)/np.sum(centered_t*centered_t))
        intercept=float(position.mean()-slope*t.mean())
        fitted = slope * t + intercept
        if axis == 'vertical':
            points = np.column_stack((position + x0, t + y0))
            fit = np.column_stack((fitted + x0, t + y0))
        else:
            points = np.column_stack((t + x0, position + y0))
            fit = np.column_stack((t + x0, fitted + y0))
        # Orthogonal least squares: scan direction is not physical edge type.
        import math
        center=points.mean(axis=0);d=points-center
        theta=.5*math.atan2(float(2*(d[:,0]*d[:,1]).sum()),float((d[:,0]**2).sum()-(d[:,1]**2).sum()))
        ux,uy=math.cos(theta),math.sin(theta)
        along=d[:,0]*ux+d[:,1]*uy;residual=-d[:,0]*uy+d[:,1]*ux
        span=float(np.ptp(along));angle=math.degrees(theta)
        geometry='横边' if abs(angle)<=1 else '竖边' if 90-abs(angle)<=1 else '斜边'
        fitted_endpoints=[(center+v*np.array((ux,uy))).tolist() for v in (along.min(),along.max())]
        rms = float(np.sqrt((residual ** 2).mean()))
        maximum = float(np.abs(residual).max())
        return dict(orientation=axis,scan_axis=axis,geometry=geometry,angle_degrees=angle,rms_px=rms, max_px=maximum,
                    span_px=span, normalized_rms_percent=100 * rms / span,
                    normalized_max_percent=100 * maximum / span,
                    samples=len(t), coverage_percent=100 * len(t) / data.shape[0],
                    method='Gaussian smoothing + dominant edge + continuity check',
                    roi=(x0, y0, x1, y1), points=points.tolist(),
                    fit_endpoints=fitted_endpoints)

    axes = ('vertical', 'horizontal') if orientation in ('auto','oblique') else (orientation,)
    candidates = [result for axis in axes if (result := measure(axis)) is not None]
    if not candidates:
        detail='；'.join(f"{'竖向扫描' if axis=='vertical' else '横向扫描'}候选 {count}/{total} 行" for axis,count,total in diagnostics)
        raise ValueError('未形成可靠连续边缘（'+detail+'）。请让边缘位于框内中间、两侧均留出区域，避开交点和反光；可尝试指定竖边/横边。')
    return max(candidates, key=lambda result: (result['coverage_percent'],result['span_px']))


def compare_distortion(before, after, before_roi, after_roi, orientation='auto'):
    """Compare straight-line residuals in corresponding, manually selected ROIs."""
    original = edge_straightness(before, before_roi, orientation)
    corrected = edge_straightness(after, after_roi, orientation)
    difference=abs(original['angle_degrees']-corrected['angle_degrees'])%180
    if min(difference,180-difference)>30:
        raise ValueError('两个 ROI 检出的边缘方向不一致，请分别选择同一条边，或指定横边/竖边。')
    baseline = original['normalized_rms_percent']
    improvement = None if original['rms_px'] < .25 else (
        100 * (baseline - corrected['normalized_rms_percent']) / baseline)
    return dict(before=original, after=corrected, improvement_percent=improvement,
                scope='同一条实际直边的局部直线度；不能据此证明全画面或完整相机标定正确。')


def distortion_preview(before, after, result):
    """Overlays are display-only; measurements use each original image's pixels."""
    from PIL import Image, ImageDraw
    marked = []
    for label, image, profile in [('BEFORE', before, result['before']), ('AFTER', after, result['after'])]:
        copy = image.convert('RGB').copy()
        draw = ImageDraw.Draw(copy)
        width = max(2, copy.width // 500)
        draw.rectangle(profile['roi'], outline='#06d9dd', width=width)
        draw.line([tuple(point) for point in profile['points']], fill='#06d9dd', width=width)
        draw.line([tuple(point) for point in profile['fit_endpoints']], fill='#ff9c32', width=width)
        header = Image.new('RGB', (copy.width, copy.height + 30), '#142d3b')
        header.paste(copy, (0, 30))
        ImageDraw.Draw(header).text((10, 8), f"{label} | RMS {profile['rms_px']:.3f}px | MAX {profile['max_px']:.3f}px", fill='white')
        marked.append(header)
    height = min(image.height for image in marked)
    marked = [image.resize((round(image.width * height / image.height), height)) for image in marked]
    joined = Image.new('RGB', (sum(image.width for image in marked), height))
    joined.paste(marked[0], (0, 0)); joined.paste(marked[1], (marked[0].width, 0))
    return joined
