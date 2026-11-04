"""Read-only serial discovery: never open a port to look for it."""
from dataclasses import dataclass
import os
import re

@dataclass(frozen=True)
class Port:
    device: str
    description: str = ''
    hwid: str = ''
    preferred: bool = False
    bluetooth: bool = False

    @property
    def label(self):
        kind='CP210x 参数口' if self.preferred else '蓝牙串口' if self.bluetooth else '串口'
        return f'{self.device} · {kind} · {self.description}' if self.description else f'{self.device} · {kind}'

@dataclass(frozen=True)
class Scan:
    ports: tuple
    errors: tuple = ()

def device_name(text):
    value=text.strip()
    if value.startswith('\\\\.\\'):value=value[4:]
    m=re.match(r'COM([1-9][0-9]*)(?=$|[\s·:()（）-])',value,re.I)
    if not m:raise ValueError('请选择串口，或手动输入设备管理器中的 COM 号，例如 COM6')
    return 'COM'+m[1]

def _port(device,description='',hwid='',vid=None):
    device=device_name(device)
    description=(description or '').strip()
    if description.lower() in ('n/a','none'):description=''
    hwid=hwid or '';text=(description+' '+hwid).lower()
    bluetooth=any(s in text for s in ['bthenum','bthmodem','bluetooth','蓝牙'])
    preferred=not bluetooth and (vid==0x10c4 or any(s in text for s in ['cp210','silabser','vid_10c4','vid:pid=10c4:']))
    return Port(device,description,hwid,preferred,bluetooth)

def _pyserial_ports():
    from serial.tools import list_ports
    return [_port(p.device,getattr(p,'description',''),getattr(p,'hwid',''),getattr(p,'vid',None)) for p in list_ports.comports()]

def _registry_ports():
    import winreg
    result=[]
    try:key=winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,r'HARDWARE\DEVICEMAP\SERIALCOMM',0,winreg.KEY_READ)
    except FileNotFoundError:return result
    with key:
        for i in range(winreg.QueryInfoKey(key)[1]):
            target,value,_=winreg.EnumValue(key,i)
            try:result.append(_port(value,'Windows 系统串口',target))
            except (ValueError,AttributeError):continue
    return result

def _dos_ports():
    import ctypes
    from ctypes import wintypes
    query=ctypes.WinDLL('kernel32',use_last_error=True).QueryDosDeviceW
    query.argtypes=(wintypes.LPCWSTR,wintypes.LPWSTR,wintypes.DWORD);query.restype=wintypes.DWORD
    size=32768
    while size<=1048576:
        names=ctypes.create_unicode_buffer(size);count=query(None,names,size)
        if count:break
        error=ctypes.get_last_error()
        if error!=122:raise ctypes.WinError(error)
        size*=2
    else:raise OSError('Windows 串口名称列表超出缓冲区')
    result=[]
    for name in names[:count].split('\0'):
        if not re.fullmatch(r'COM[1-9][0-9]*',name,re.I):continue
        target=ctypes.create_unicode_buffer(4096)
        description=query(name,target,len(target))
        result.append(_port(name,'Windows 系统串口',target.value if description else ''))
    return result

def discover_ports(primary=None,fallbacks=None):
    """Merge OS-reported devices, preserving full USB/BT identity when available."""
    if primary is None:primary=_pyserial_ports
    if fallbacks is None:fallbacks=(_registry_ports,_dos_ports) if os.name=='nt' else ()
    found={};errors=[]
    for source in (primary,*fallbacks):
        try:
            for p in source():
                old=found.get(p.device)
                if old is None:found[p.device]=p
                else:
                    found[p.device]=Port(old.device,old.description or p.description,old.hwid or p.hwid,old.preferred or p.preferred,old.bluetooth or p.bluetooth)
        except (ImportError,OSError,ValueError,AttributeError) as exc:
            errors.append(f'{source.__name__}: {type(exc).__name__}: {exc}')
    ports=tuple(sorted(found.values(),key=lambda p:(not p.preferred,p.bluetooth,int(p.device[3:]))))
    return Scan(ports,tuple(errors))

def choose_device(ports,current=''):
    """Keep an explicit/manual selection. Auto-select only a unique CP210x."""
    if current:
        try:return device_name(current)
        except ValueError:pass
    preferred=[p for p in ports if p.preferred]
    return preferred[0].device if len(preferred)==1 else ''

def scan_hint(scan,selected=''):
    selected_port=next((p for p in scan.ports if p.device==selected),None)
    if selected_port:
        hint=f'选中 {selected_port.device}：{selected_port.description or "系统串口"}'
        if selected_port.bluetooth:hint+='；这是蓝牙串口，请核对板卡参数 USB 对应的 CP210x 端口'
        return hint
    if selected:return f'{selected} 当前未枚举到；可核对设备管理器后手动尝试打开'
    if scan.ports and all(p.bluetooth for p in scan.ports):return '当前仅发现蓝牙串口；请连接板卡参数 USB，确认设备管理器中出现 CP210x 后刷新'
    if not scan.ports:return '未发现已分配的 COM 口；请连接板卡参数 USB 并检查 CP210x 驱动，也可手动输入已知 COM 号'
    return '请按设备名称选择板卡参数口；多个 CP210x 时需要确认对应设备，可手动输入 COM 号'

def diagnostic():
    from dataclasses import asdict
    import serial
    scan=discover_ports()
    return dict(pyserial_version=serial.__version__,ports=[asdict(p) for p in scan.ports],errors=list(scan.errors),suggested_device=choose_device(scan.ports),opens_ports=False)
