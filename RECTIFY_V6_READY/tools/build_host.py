"""Rebuild the Windows host from the delivered source and calibration data.
Install host/requirements.txt and PyInstaller first. No old FPGA project needed.
"""
from pathlib import Path
import PyInstaller.__main__
root=Path(__file__).resolve().parents[1]
PyInstaller.__main__.run([str(root/'run_host.py'),'--name','RECTIFY_HOST','--onedir','--windowed','--noconfirm','--clean',
 '--distpath',str(root/'host_app'),'--workpath',str(root/'.host_build'),'--specpath',str(root/'.host_build'),
 '--paths',str(root/'host'),'--add-data',str(root/'calibration')+':calibration',
 '--add-binary',str(root/'host/pgd1_native.dll')+':.',
 '--hidden-import','serial.tools.list_ports','--hidden-import','serial.tools.list_ports_windows','--hidden-import','serial.tools.list_ports_common','--hidden-import','serial.serialwin32','--hidden-import','winreg','--hidden-import','PIL.ImageTk','--hidden-import','cv2','--collect-all','numpy'])
