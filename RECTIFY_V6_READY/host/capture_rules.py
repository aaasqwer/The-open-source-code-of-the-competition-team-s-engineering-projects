"""Capture readiness and provenance checks, separate from UI presentation."""
def capture_context(video,pending=False):
    if not video:raise RuntimeError('视频遥测未有效回读')
    if video.get('return_pc') and not video.get('network_ready'):raise RuntimeError('板端网口时钟尚未就绪：检查千兆有线链路')
    if pending:raise RuntimeError('控制尚未确认')
    if video.get('frozen'):raise RuntimeError('板端仍处于冻结状态')
    if video.get('page') and not video.get('clean_camera_images'):raise RuntimeError('当前为KEY4数据页，请松开KEY4并返回相机画面后再拍照')
    if video['flags']&3!=3:raise RuntimeError('相机或 DDR 未就绪')
    if video.get('input_fps')==0:raise RuntimeError('OV5640 尚无有效输入帧')
    if video['mode']!=0 and not video['flags']&16:raise RuntimeError('矫正模型未就绪')
    return dict(requested_mode=video['mode'],requested_model=1 if video.get('clean_camera_images') and video['mode']==0 else video['model'],frozen_state=None,invalid=False)


def merge_frozen_state(context,observation):
    """Combine wire image metadata with UART lens state without losing provenance."""
    old=context.get('frozen_state') or {}
    if any(old[key]!=observation[key] for key in old.keys()&observation.keys()):
        raise ValueError('冻结期间模式或镜头标定状态变化，照片已丢弃')
    context['frozen_state']={**old,**observation}


def matching_frozen_video(context,video):
    """Only attach stable readback belonging to this requested photo.

    V5 has a session fence. A pending request still exposes the old controller
    state; state 1 has not yet completed the three-frozen-frame settling wait.
    Older supported firmware has no session field, so only freshness/freeze
    can be checked there; never invent a V5 association for it.
    """
    if not video.get('frozen') or video.get('received',0)<context.get('requested_at',0):return False
    if video.get('protocol_version',0)>=5:
        d=video.get('photo_diagnostics')
        return bool(d and not d['pending'] and d['session']==context['sequence'] and d['state'] in (2,3,4,5,6,7,8,9,11))
    return True


def frozen_video_state(video):
    observed=dict(mode=video['mode'],model=video['model'])
    if video.get('return_pc'):
        observed.update(gamma_step=video['gamma'],hud_enabled=video['numbers'])
        if video.get('protocol_version',0)>=5:observed['data_page']=bool(video['page'])
    if video.get('lens_capable'):
        observed.update(lens_id=video['lens_id'],lens_enabled=video['lens_enabled'],lens_fault=video['lens_fault'])
    if video.get('clean_camera_images'):
        observed.update(hud_enabled=False,data_page=False,source_stage='sensor_rgb565' if video['mode']==0 else 'processed_before_osd')
        if video['mode']==0:observed.update(model=1,gamma_step=0,lens_id=0,lens_enabled=False,lens_fault=False)
    return observed


def validate_reference(label,metadata):
    if not metadata:return
    observed=metadata.get('frozen_state')
    if observed and observed.get('data_page'):raise ValueError('KEY4数据页截图不能作为相机校正前／后参考图')
    if observed and 'lens_enabled' in observed:
        if observed.get('lens_fault'):raise ValueError('照片对应的镜头映射故障，不能作为有效矫正参考')
        if observed['lens_enabled']!=(label!='原始'):raise ValueError('参考图与镜头校正前／后的实际状态不匹配')
        return
    if metadata.get('source')=='board_ov5640':raise ValueError('缺少该帧的冻结镜头状态，不能认定校正前／后；请用V4/V5完整拍照并等待冻结状态回读，720P预览不能代替')
    mode=observed['mode'] if observed else metadata.get('requested_mode')
    expected=0 if label=='原始' else 1
    if mode is not None and mode!=expected:raise ValueError(f'{label}参考图的板端模式不匹配')


def validate_pair(a,b):
    if not a or not b:return
    x=a.get('frozen_state') or dict(model=a.get('requested_model'))
    y=b.get('frozen_state') or dict(model=b.get('requested_model'))
    if x.get('model')!=y.get('model'):raise ValueError('两张参考图采用了不同处理设置')
    if x.get('model')==0 and all(p.get('gamma_step') is not None for p in [x,y]) and x['gamma_step']!=y['gamma_step']:
        raise ValueError('两张参考图的自适应Gamma档位不同，不能将差异归因于镜头校正；请两次均旁路Gamma后重拍')
    if a.get('com') and b.get('com') and a['com']!=b['com']:raise ValueError('两张参考图来自不同参数串口')


def comparison_limits(a,b):
    notes=[]
    states=[(p or {}).get('frozen_state') or {} for p in [a,b]]
    if any('gamma_step' not in p for p in states):notes.append('Gamma档位来源不完整，未验证同一预处理条件')
    if all('hud_enabled' in p for p in states) and states[0]['hud_enabled']!=states[1]['hud_enabled']:notes.append('字幕开关不同，ROI必须避开字幕区域')
    notes.append('曝光、焦点、物理姿态及ROI对应关系未由板端逐帧数据证明；结果为所选图像的描述性对比，不是镜头校正的因果或实板评分证明')
    return notes
