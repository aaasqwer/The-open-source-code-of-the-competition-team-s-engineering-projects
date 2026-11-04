from pathlib import Path
import shutil,subprocess
root=Path(__file__).resolve().parents[1]
gcc=shutil.which('gcc.exe') or shutil.which('gcc')
if not gcc:raise SystemExit('Install a 64-bit MinGW GCC compiler and add it to PATH.')
subprocess.run([gcc,'-O3','-shared','-std=c99','-static-libgcc','-o',str(root/'host/pgd1_native.dll'),str(root/'host/pgd1_native.c')],check=True)
