"""Independent host geometry for the FPGA inverse mapper. Pixel coordinates."""
from pathlib import Path
import sys
import json, math, struct, zlib
import numpy as np
W,H=1920,1080
C=np.array([959.5,539.5])

def profile():
    root=Path(getattr(sys,'_MEIPASS',Path(__file__).resolve().parent.parent))
    return json.loads((root/'calibration/profile.json').read_text(encoding='utf-8'))

def affine(angle=0,sx=1,sy=1,shear=0,tx=0,ty=0):
    vals=np.array([angle,sx,sy,shear,tx,ty],float)
    if not np.isfinite(vals).all() or min(sx,sy)<=0:raise ValueError('参数必须有限，缩放必须为正数')
    a=math.radians(angle);r=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    m=r@np.array([[sx,shear],[0,sy]])
    if abs(np.linalg.det(m))<1e-5:raise ValueError('变换退化')
    a=np.eye(3);a[:2,:2]=m;a[:2,2]=C+np.array([tx,ty])-m@C
    return a

def plane_forward(raw_corners,aspect,p):
    import cv2
    q=np.array(raw_corners,np.float64)
    if q.shape!=(4,2) or not np.isfinite(q).all() or not 0.1<=aspect<=10:raise ValueError('需要四角和0.1～10的宽高比')
    # Four clicks in perimeter order: top-left, top-right, bottom-right, bottom-left.
    if not cv2.isContourConvex(q.astype(np.float32)) or abs(cv2.contourArea(q.astype(np.float32)))<1000:raise ValueError('四角必须围成凸四边形，按左上、右上、右下、左下点击')
    K=np.array(p['K']);D=np.array(p['enhanced_D'])
    ideal=cv2.undistortPointsIter(q[:,None,:],K,D,None,K,(3,100,1e-12)).reshape(4,2)
    dx,dy=residual_displacement(ideal[:,0],ideal[:,1],p);ideal+=np.c_[dx,dy]
    width=min(W-1,(H-1)*aspect);height=width/aspect;x,y=C-[width/2,height/2]
    target=np.array([[x,y],[x+width,y],[x+width,y+height],[x,y+height]],np.float32)
    m=cv2.getPerspectiveTransform(ideal.astype(np.float32),target)
    if abs(np.linalg.det(m))<1e-8:raise ValueError('该平面过于倾斜，无法稳定恢复')
    return m

def quantize(m):
    m=np.array(m,float)
    if m.shape!=(3,3) or not np.isfinite(m).all():raise ValueError('无效矩阵')
    # Normalize the denominator at output center; homogeneous scaling is free.
    d=m[2]@np.r_[C,1.]
    if abs(d)<1e-9:raise ValueError('图像中心接近投影地平线')
    m/=d
    scale=min(1.,1.9/max(1e-9,np.max(np.abs(m[2]))),1900/max(1e-9,np.max(np.abs(m[:2]))))
    m*=scale
    values=[round(float(m[i,j])*2**(30 if i==2 else 20)) for i in range(3) for j in range(3)]
    if any(not -2**31<=v<2**31 for v in values):raise ValueError('矩阵超出定点范围')
    return values

def camera_tilt(pitch,yaw,p):
    """Host commanded virtual camera tilt. Does not guess IMU extrinsics."""
    a,b=map(math.radians,[pitch,yaw])
    rx=np.array([[1,0,0],[0,math.cos(a),-math.sin(a)],[0,math.sin(a),math.cos(a)]])
    ry=np.array([[math.cos(b),0,math.sin(b)],[0,1,0],[-math.sin(b),0,math.cos(b)]])
    K=np.array(p['K'])
    return K@ry@rx@np.linalg.inv(K)

def decode(words):
    m=np.array(words,float).reshape(3,3)
    m[:2]/=2**20;m[2]/=2**30
    return m

def residual_displacement(x,y,p):
    refinement=p.get('nonradial_refinement',{})
    if not refinement.get('enabled'):return np.zeros_like(x,dtype=float),np.zeros_like(y,dtype=float)
    K=np.array(p['K']);nx=(np.asarray(x)-K[0,2])/K[0,0];ny=(np.asarray(y)-K[1,2])/K[1,1]
    dx=np.zeros(np.broadcast_shapes(nx.shape,ny.shape));dy=np.zeros_like(dx)
    for (i,j),(a,b) in zip(refinement['monomial_powers'],refinement['coefficients_pixels']):
        term=nx**i*ny**j;dx+=a*term;dy+=b*term
    return dx,dy

def map_source(x,y,p,enhanced=True):
    K=np.array(p['K']);fx,fy=K[0,0],K[1,1];cx,cy=K[0,2],K[1,2]
    if enhanced and p.get('nonradial_refinement',{}).get('enabled'):
        base_x=np.asarray(x);base_y=np.asarray(y);x=base_x;y=base_y
        for _ in range(2):
            dx,dy=residual_displacement(x,y,p);x=base_x-dx;y=base_y-dy
    a=(x-cx)/fx;b=(y-cy)/fy;r=a*a+b*b
    if enhanced:
        k1,k2,p1,p2,k3,k4,k5,k6=p['enhanced_D'][:8]
        gain=(1+k1*r+k2*r*r+k3*r**3)/(1+k4*r+k5*r*r+k6*r**3)
    else:
        k1,k2,p1,p2=p['B_D'];gain=1+k1*r+k2*r*r
    return fx*(a*gain+2*p1*a*b+p2*(r+2*a*a))+cx,fy*(b*gain+p1*(r+2*b*b)+2*p2*a*b)+cy

def valid_canvas(m,p,all_pixels=True):
    xs=np.arange(W,dtype=float)[None,:] if all_pixels else np.linspace(0,W-1,81)[None,:]
    yy=np.arange(H,dtype=float) if all_pixels else np.linspace(0,H-1,49)
    for start in range(0,len(yy),64):
        ys=yy[start:start+64,None]
        den=m[2,0]*xs+m[2,1]*ys+m[2,2]
        if np.min(den)<=1e-5:return False
        x=(m[0,0]*xs+m[0,1]*ys+m[0,2])/den;y=(m[1,0]*xs+m[1,1]*ys+m[1,2])/den
        if np.min(x)<0 or np.max(x)>W-1 or np.min(y)<0 or np.max(y)>H-1:return False
        for enhanced in [False,True]:
            u,v=map_source(x,y,p,enhanced)
            if not(np.isfinite(u).all() and np.isfinite(v).all()) or np.min(u)<1 or np.max(u)>W-2 or np.min(v)<1 or np.max(v)>H-2:return False
    return True

def inverse_words(forward,p):
    inv=np.linalg.inv(forward)
    def attempt(z):
        t=np.eye(3);t[0,0]=t[1,1]=1/z;t[:2,2]=C*(1-1/z)
        words=quantize(inv@t)
        return words,decode(words)
    # Include every output pixel in final validity, not only four corners.
    z=1.
    while z<=16:
        words,m=attempt(z)
        if valid_canvas(m,p,False):break
        z*=1.15
    else:raise ValueError('16倍裁剪仍有越界；请重新框选平面或减小平移／倾斜')
    lo,hi=1.,z
    for _ in range(20):
        mid=(lo+hi)/2;words,m=attempt(mid)
        if valid_canvas(m,p,False):hi=mid
        else:lo=mid
    z=max(1.,hi*1.002);words,m=attempt(z)
    if not valid_canvas(m,p,True):raise ValueError('完整画布检查未通过，未下发矩阵')
    return words,z

def upload_commands(slot,words):
    if slot not in (0,1) or len(words)!=9:raise ValueError('无效矩阵事务')
    raw=struct.pack('<9i',*words)
    return [(0x50,struct.pack('<BBi',slot,i,v)) for i,v in enumerate(words)]+[(0x51,struct.pack('<BI',slot,zlib.crc32(raw)))]

def chessboard_corners(image):
    import cv2
    raw=np.array(image.convert('RGB'));gray=cv2.cvtColor(raw,cv2.COLOR_RGB2GRAY)
    ok,corners=cv2.findChessboardCornersSB(gray,(9,7),flags=cv2.CALIB_CB_EXHAUSTIVE|cv2.CALIB_CB_ACCURACY)
    if not ok:raise ValueError('未识别完整9×7内角点，请改用四角手选')
    p=corners.reshape(7,9,2)
    return [p[0,0].tolist(),p[0,-1].tolist(),p[-1,-1].tolist(),p[-1,0].tolist()]
