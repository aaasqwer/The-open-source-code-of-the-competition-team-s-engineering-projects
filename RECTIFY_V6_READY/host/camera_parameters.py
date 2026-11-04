"""Read-only parameters from this project's new photo calibration."""
from geometry import profile
def candidate_profile():return profile()

def parameters_match(packet):
    if not packet or not packet.get('ready'):return False
    p=profile();K=p['K'];target=dict(fx=K[0][0],fy=K[1][1],cx=K[0][2],cy=K[1][2],**dict(zip(['k1','k2','p1','p2'],p['B_D'])))
    values=packet.get('parameters',{})
    return all(k in values and abs(values[k]-v)<(0.0001 if k in ['fx','fy','cx','cy'] else 0.0000003) for k,v in target.items())

def parameter_text(packet=None,local=None,stale=False):
    if local is not None:
        if 'K' not in local or 'B_D' not in local:raise ValueError('请选择本工程calibration/profile.json；旧工程相机参数不能套用')
        K=local['K'];D=local['B_D'];state='本工程新照片标定 · 二阶基础模型 + 径向残差增强'
        values=dict(fx=K[0][0],fy=K[1][1],cx=K[0][2],cy=K[1][2],**dict(zip(['k1','k2','p1','p2'],D)))
    elif packet and packet.get('ready'):
        values=packet.get('parameters',{});state='板端当前完整显示帧参数回读 · ID '+str(packet.get('calibration_id'))
        state+=' · '+('映射故障' if packet.get('fault') else '镜头校正启用' if packet.get('enabled') else '原图旁路')
    else:return '尚未收到有效相机参数；本地参数不代表当前板端状态'
    if stale:state+=' · 历史回读'
    n=lambda k:f'{values.get(k,0):.7f}'
    report=profile()
    quality=f"\n本地配套联合标定重投影RMS {report['enhanced_training_reprojection_rms_px']:.3f} px，不能代替直线度。"
    if packet and not parameters_match(packet):quality='\n板端K/D与本机配套参数不一致，请检查所下载位流。'
    return state+'\n1920×1080 CMOS2\nK = ['+n('fx')+'  0  '+n('cx')+']\n    [0  '+n('fy')+'  '+n('cy')+']\n    [0  0  1]\n'+'\n'.join(k+' = '+n(k) for k in ['k1','k2','p1','p2'])+'\n完整增强系数和径向ROM见calibration/profile.json。'+quality+'\n尚待独立实板验收。'
