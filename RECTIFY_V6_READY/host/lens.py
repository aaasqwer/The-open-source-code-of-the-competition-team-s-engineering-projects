"""Validated Brown calibration import and atomic FPGA parameter commands.

No default camera calibration: reset/bypass remains the V28 image path.
"""
import json, math, struct, zlib
from pathlib import Path
from calibration_policy import SCHEMA,MODEL,number,digest,mapping_certificate,quality_issues

def load_profile(path):
    p=json.loads(Path(path).read_text(encoding='utf-8-sig'))
    validate_profile(p)
    return p

def validate_profile(p):
    if not isinstance(p,dict):raise ValueError('标定JSON根节点必须是对象')
    if p.get('schema')!=SCHEMA or p.get('distortion_model')!=MODEL:raise ValueError('需要RETURN_PC_1 Brown V2标定JSON及完整质量证据；旧文件请使用标定工具重新生成')
    if p.get('camera')!='CMOS2' or p.get('resolution')!=[1920,1080]:raise ValueError('标定必须对应当前 CMOS2 的 1920×1080 原始输入')
    if p.get('geometry_verified') is not True:raise ValueError('尚未确认同一镜头、分辨率、未回正/裁切的原始图像；此候选标定禁止下发')
    try:words=parameter_words(p,False)
    except (KeyError,TypeError,IndexError) as exc:raise ValueError('标定JSON结构不完整或字段类型错误') from exc
    if p.get('integrity_sha256')!=digest(p):raise ValueError('标定参数／误差证据不一致或文件已被修改，请重新生成完整标定')
    quality=p.get('quality')
    issues=quality_issues(quality)
    if not isinstance(quality,dict) or quality.get('accepted') is not True or issues:
        raise ValueError('标定质量检查未通过：'+('；'.join(issues) or '报告未批准'))
    if not mapping_certificate(words)['certified_non_folding']:raise ValueError('量化后的镜头模型未通过保守无折叠校验，不能启用')

def parameter_words(profile,enabled=True,zoom=None):
    k=profile['camera_matrix'];d=profile['distortion_coefficients']
    if not isinstance(k,(list,tuple)) or len(k)!=3 or any(not isinstance(row,(list,tuple)) or len(row)!=3 for row in k) or not isinstance(d,(list,tuple)) or len(d)!=4:raise ValueError('需要3×3相机矩阵及k1,k2,p1,p2四个系数')
    matrix=[[number(v,'相机矩阵') for v in row] for row in k]
    fx,fy,cx,cy=(matrix[0][0],matrix[1][1],matrix[0][2],matrix[1][2]);k1,k2,p1,p2=[number(v,'畸变系数') for v in d]
    z=number(profile.get('zoom',1.) if zoom is None else zoom,'视场缩放')
    if type(enabled) is not bool:raise ValueError('enable必须是布尔值')
    if any(abs(matrix[i][j]-v)>1e-9 for i,j,v in [(0,1,0),(1,0,0),(2,0,0),(2,1,0),(2,2,1)]):raise ValueError('FPGA不支持非零skew或非标准相机矩阵')
    if not(480<=fx<=8192 and 270<=fy<=8192 and 0<=cx<=1919 and 0<=cy<=1079 and abs(k1)<=1 and abs(k2)<=.5 and abs(p1)<=.125 and abs(p2)<=.125 and 1<=z<=4):
        raise ValueError('参数超出硬件定点范围；不得截断或强行缩放畸变系数')
    values=[round(v*65536) for v in (fx,fy,cx,cy)]+[round(v*2**24) for v in (k1,k2,p1,p2)]
    values += [round(2**30/fx),round(2**30/fy),round(2**20/z),int(enabled)]
    return values

def upload_commands(profile,request_id,enabled=True,zoom=None):
    validate_profile(profile)
    if type(request_id) is not int or not 0<request_id<2**32:raise ValueError('参数事务ID必须为非零32位整数')
    values=parameter_words(profile,enabled,zoom);raw=struct.pack('<12i',*values)
    return [(0x40,struct.pack('<Bi',i,v)) for i,v in enumerate(values)]+[(0x41,struct.pack('<II',request_id,zlib.crc32(raw)))]

def map_point(x,y,profile,zoom=None):
    """Independent floating-point reference, destination ideal -> source distorted."""
    k=profile['camera_matrix'];fx,fy,cx,cy=k[0][0],k[1][1],k[0][2],k[1][2]
    z=profile.get('zoom',1.) if zoom is None else zoom
    x=(959.5+(x-959.5)/z-cx)/fx;y=(539.5+(y-539.5)/z-cy)/fy
    k1,k2,p1,p2=profile['distortion_coefficients'];r=x*x+y*y;rad=1+k1*r+k2*r*r
    return fx*(x*rad+2*p1*x*y+p2*(r+2*x*x))+cx,fy*(y*rad+p1*(r+2*y*y)+2*p2*x*y)+cy
