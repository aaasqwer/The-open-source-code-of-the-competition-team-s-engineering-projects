"""Locate an explicitly selected Icarus installation; no simulator download."""
from pathlib import Path
import os,shutil,tempfile,hashlib
def prepare_iverilog(root):
    selected=os.environ.get('IVERILOG_HOME')
    if selected:original=Path(selected)
    else:
        exe=shutil.which('iverilog.exe') or shutil.which('iverilog')
        if not exe:raise RuntimeError('Set IVERILOG_HOME to your installed Icarus directory.')
        original=Path(exe).resolve().parents[1]
    iv=original/'bin/iverilog.exe';vvp=original/'bin/vvp.exe';lib=original/'lib/ivl'
    if not iv.is_file() or not vvp.is_file():raise RuntimeError('Expected bin/iverilog.exe and bin/vvp.exe in IVERILOG_HOME.')
    if ' ' in str(lib):
        stage=Path(tempfile.gettempdir())/('rectify_iverilog_'+hashlib.sha256(iv.read_bytes()).hexdigest()[:12])
        if ' ' in str(stage):raise RuntimeError('This Icarus Windows build needs a simulator library path without spaces; extract the project to an English short path.')
        if not (stage/'bin/iverilog.exe').exists():shutil.copytree(original,stage)
        iv=stage/'bin/iverilog.exe';lib=stage/'lib/ivl'
    return iv,lib,vvp
