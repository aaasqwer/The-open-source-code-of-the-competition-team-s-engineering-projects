"""Check delivered bytes; this does not program or interrogate the FPGA."""
from pathlib import Path
import hashlib,json,sys
root=Path(__file__).resolve().parents[1]
manifest=json.loads((root/'release_manifest.json').read_text(encoding='utf-8'))
errors=[]
for name,expected in manifest['sha256'].items():
    p=root/name
    if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=expected:errors.append(name)
if errors:
    print('Package mismatch:\n'+'\n'.join(errors));sys.exit(1)
print('PASS: all '+str(len(manifest['sha256']))+' delivered files match the release manifest.')
