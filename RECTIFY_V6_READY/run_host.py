from pathlib import Path
import sys
root=Path(__file__).resolve().parent
sys.path.insert(0,str(root/'host/deps'));sys.path.insert(0,str(root/'host'))
try:
 import numpy,PIL,serial,cv2
except ImportError as exc:
 raise SystemExit('上位机缺少依赖，请用本Python运行 pip install -r host/requirements.txt\n'+str(exc))
from app import main
if '--selftest-connections' in sys.argv:
 from connection_check import run
 run(sys.argv[sys.argv.index('--connections-result')+1])
elif '--diagnose-ports' in sys.argv:
 import json
 from serial_ports import diagnostic
 output=Path(sys.argv[sys.argv.index('--ports-result')+1])
 output.write_text(json.dumps(diagnostic(),ensure_ascii=False,indent=2),encoding='utf-8')
elif '--benchmark-decode' in sys.argv:
 from decode_benchmark import run
 run(sys.argv[sys.argv.index('--benchmark-image')+1],sys.argv[sys.argv.index('--benchmark-result')+1])
elif '--smoke-test' in sys.argv:
 import tkinter as tk
 import json
 from app import Workbench
 import codec
 assert codec.NATIVE_AVAILABLE and codec.NATIVE_FRAME_AVAILABLE,'BKB native image decoder is missing or failed to load'
 root=tk.Tk();root.withdraw();app=Workbench(root);root.update_idletasks();app.closing=True;root.destroy()
 if '--smoke-result' in sys.argv:
  Path(sys.argv[sys.argv.index('--smoke-result')+1]).write_text(json.dumps(dict(passed=True,frozen=bool(getattr(sys,'frozen',False)),native_decoder=codec.NATIVE_AVAILABLE,native_frame_decoder=codec.NATIVE_FRAME_AVAILABLE)),encoding='utf-8')
else:main()
