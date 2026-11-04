"""Rebuild only the independent evaluation app. Install evaluation/requirements_build.txt first."""
from pathlib import Path
import sys,subprocess
root=Path(__file__).resolve().parents[1]
subprocess.run([sys.executable,'-m','PyInstaller',str(root/'evaluation/evaluation_app.py'),
 '--name','RECTIFY_EVALUATION','--onedir','--windowed','--noconfirm','--clean',
 '--paths',str(root/'evaluation'),'--distpath',str(root/'evaluation_app'),
 '--workpath',str(root/'.evaluation_build'),'--specpath',str(root/'.evaluation_build'),
 '--collect-all','numpy','--hidden-import','serial.tools.list_ports_windows',
 '--hidden-import','serial.tools.list_ports_common','--hidden-import','serial.serialwin32'],check=True)
