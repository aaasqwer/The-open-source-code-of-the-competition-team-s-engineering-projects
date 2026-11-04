"""Explain validity reported by the current FPGA; never substitute measurements."""
ROI_STATUS={0:'未测量',1:'等待完整 ROI 或状态切换后重新采样',
            2:'边缘稳定，直线度测量有效',3:'ROI 对比度不足',
            4:'有效边缘行不足或边缘强度变化过大',5:'检测到多条边或混合极性',
            6:'等待边缘位置和亮度稳定',7:'左右对比模式暂停 ROI 测量',
            8:'过暗或过曝像素过多',9:'边缘连续性不足'}


def invalid_metric_reasons(video):
    reasons=[]
    split=video.get('mode')==2
    if video.get('latency_us') is None:
        reasons.append('延迟 --：左右对比模式不统计单路帧延迟。' if split else
                       '延迟 --：板端未给出有效结果；冻结抓帧、切换模式或帧缓冲异常时会暂时失效。')
    if video.get('rms') is None:
        reasons.append('直线度 --：'+ROI_STATUS.get(video.get('evaluation'),'ROI 尚无有效测量')+'。')
    if video.get('mtf50') is None:
        reasons.append('MTF50 --：左右对比模式暂停斜边测量。' if split else
                       'MTF50 --：尚无有效斜边 SFR 结果；需单条约 2–8° 斜边、稳定且曝光合适，直线度有效也不保证 MTF50 有效。')
    return reasons
