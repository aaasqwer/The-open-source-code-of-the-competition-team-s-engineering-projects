"""Read the matched local full build. Never claim identification of the live FPGA."""
from pathlib import Path
import json,sys,hashlib

def project_root():
    return Path(sys.executable).resolve().parents[2] if getattr(sys,'frozen',False) else Path(__file__).resolve().parent.parent

def local_report():
    root=project_root()
    try:
        report=json.loads((root/'verification/pds_full_result.json').read_text(encoding='utf-8'))
        if not report.get('passed') or not report.get('declared_constraints_met'):raise ValueError('本地完整构建尚未通过')
        bit=root/'release/RECTIFY_BOARD.sbit'
        if hashlib.sha256(bit.read_bytes()).hexdigest()!=report['bitstream_sha256']:raise ValueError('配套位流校验不匹配')
        return report
    except (OSError,ValueError,KeyError) as exc:return {'report_error':str(exc)}

def describe(packet=None,local=None,stale=False):
    if not local or 'report_error' in local:return '配套构建报告尚不可用：'+str((local or {}).get('report_error','未找到完整工程')),[]
    resources=local.get('resources',{});rows=[]
    for name,capacity in [('LUT',66600),('FF',133200),('APM',240),('DRM',155)]:
        used=resources.get(name)
        if used is not None:rows.append((name,f'{used:g}',str(capacity),f'{used/capacity*100:.2f}%'))
    return '本地配套位流的完整布局布线报告\n位流SHA256 '+local['bitstream_sha256'][:20]+'…\n未识别当前板上位流；不是实时负载。DRM按36K等效。',rows
