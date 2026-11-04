"""Deterministic fixed-point SFR coefficients, independent of camera calibration."""
from pathlib import Path
import math
root=Path(__file__).resolve().parents[1]/'source/evaluation'
tables={
 'sfr_normal.hex':(5,[round(65536/math.sqrt(1+(i/512)**2)) for i in range(128)]),
 'sfr_window.hex':(4,[round(32767*.5*(1-math.cos(2*math.pi*i/127))) for i in range(128)]),
 'sfr_cos.hex':(4,[round(16384*math.cos(2*math.pi*i/256))&65535 for i in range(256)]),
 'sfr_correction.hex':(4,[16384]+[round(16384*(2*math.pi*i/256)/math.sin(2*math.pi*i/256)) for i in range(1,33)])}
for name,(digits,values) in tables.items():(root/name).write_text(''.join(f'{v:0{digits}x}\n' for v in values),encoding='ascii')
