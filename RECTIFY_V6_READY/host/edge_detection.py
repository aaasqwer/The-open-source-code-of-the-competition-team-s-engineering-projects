"""Noise-aware localization on unchanged pixels; no line fitting or residual rejection."""
import numpy as np


def locate_rows(gray, border=8):
    a=np.asarray(gray,dtype=float)
    if a.ndim!=2 or min(a.shape)<20:raise ValueError('选区太小，边缘两侧需要保留足够背景')
    h,width=a.shape;rows=np.arange(h);w=width-1
    q=np.pad(a,((0,0),(2,2)),mode='edge')
    smoothed=(q[:,:-4]+4*q[:,1:-3]+6*q[:,2:-2]+4*q[:,3:-1]+q[:,4:])/16
    grad=np.diff(smoothed,axis=1);energy=np.abs(grad)
    peaks=energy.argmax(1);strength=energy[rows,peaks]
    raw_diff=np.diff(a,axis=1);median=np.median(raw_diff,axis=1)
    noise=np.median(np.abs(raw_diff-median[:,None]),axis=1)/.9538725524
    # Noise variance of the differentiated [1,4,6,4,1]/16 kernel.
    gradient_noise=noise*np.sqrt(14/256)
    xx=np.arange(w)+.5;distance=np.abs(xx[None,:]-(peaks+.5)[:,None])
    left_index=np.clip(peaks[:,None]+np.arange(-19,-7),0,width-1)
    right_index=np.clip(peaks[:,None]+np.arange(8,20),0,width-1)
    left=np.median(a[rows[:,None],left_index],axis=1)
    right=np.median(a[rows[:,None],right_index],axis=1)
    delta=right-left;polarity=np.sign(grad[rows,peaks])
    dominant=1 if np.sum(np.sign(delta))>=0 else -1
    other=np.where(distance>7,energy,0).max(1)
    noise_limit=np.maximum(.45,3.5*gradient_noise)
    contrast=np.abs(delta);cnr=contrast/np.maximum(noise,.5)
    good=(peaks>=border)&(peaks<w-border)&(strength>=noise_limit)&(contrast>=np.maximum(8,5*noise))
    good&=(other<np.maximum(strength*.7,noise_limit))&(polarity==dominant)&(np.sign(delta)==dominant)
    floor=np.nanmedian(np.where(distance>10,energy,np.nan),axis=1)
    weights=np.maximum(grad*dominant-floor[:,None],0);weights[distance>5]=0
    positions=(weights*xx).sum(1)/np.maximum(weights.sum(1),1e-12)
    return dict(positions=positions,good=good,rows=rows,polarity=dominant,
                noise_sigma=noise,contrast=contrast,cnr=cnr,strength=strength,
                method='per-row plateau contrast / MAD sensor noise / binomial derivative / local centroid; no residual rejection')
