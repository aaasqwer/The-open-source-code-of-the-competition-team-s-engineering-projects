"""Describe observed stages; do not infer physical delivery from an FPGA TX count."""
PHASES={0:'空闲',1:'等待完整冻结帧',2:'准备图像包',3:'发送图像包／等待ACK',4:'请求下一行',5:'等待像素行',6:'读取行号',7:'启动编码',8:'编码像素行',9:'调度像素行',10:'等待解冻',11:'发送最后一行'}
REASONS={0:'无',1:'源帧下溢或冻结所有权丢失',2:'连续2秒无进展'}

def describe_link(video,receiver):
    listening=receiver.sock is not None and not receiver.stop.is_set()
    text=('UDP已监听' if listening else 'UDP未监听')+f' · 收到 {receiver.datagrams} 包／照片 {receiver.photo_datagrams} 包／CRC错误 {receiver.crc_errors}'
    if not video:return text+'\n视频遥测未就绪：检查CP2102参数串口和板端版本'
    text+='\n'+('板端网络时钟已就绪' if video.get('network_ready') is True else '板端网络时钟未就绪' if video.get('network_ready') is False else '参考位流未回读网络时钟状态')
    d=video.get('photo_diagnostics')
    if not d:return text+'\n当前位流无V5阶段回读，不能据此判断板端是否收到拍照命令'
    phase='等待图像通道空闲' if d['pending'] else PHASES[d['state']]
    text+=f"\n板端：{phase} · 命令 {d['commands']} · 会话 {d['session']} · TX帧 {d['tx_packets']} · 已编码行 {d['rows']}/1080"
    if d['network_fault']:text+=' · 网口传输故障／ACK超时'
    if d['abort_reason']:text+='\n最近中止：'+REASONS[d['abort_reason']]
    if receiver.capturing and d['session']!=receiver.capture_sequence:
        return text+f'\n等待板端确认本次会话 {receiver.capture_sequence}；上述计数来自上次会话'
    if receiver.capturing and not d['pending'] and d['tx_packets'] and receiver.capture_datagrams==0:
        text+='\n板端已启动发包，电脑未收到当前会话的有效照片块：需检查有线链路、接收实例、网卡及防火墙许可；TX计数不证明物理送达'
    return text

def capture_failure(video,sequence):
    d=video.get('photo_diagnostics')
    # A newly queued session can briefly carry the previous abort reason/TX
    # count while preview drains. Never cancel that new request on old state.
    if not d or d['pending'] or d['session']!=sequence:return None
    if d['network_fault']:return '板端网口重传超过上限／ACK失败，请检查USB命令与有线图像链路'
    if d['abort_reason']:return '板端拍照中止：'+REASONS[d['abort_reason']]
    return None
