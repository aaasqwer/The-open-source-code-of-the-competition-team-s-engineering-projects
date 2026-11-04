"""CPU-only decode timing. Does not measure camera FPS or network latency."""
from pathlib import Path
import json,time,struct,zlib,hashlib
import numpy as np
from PIL import Image
import codec
def run(image_path,result_path,rounds=20):
    source=Image.open(image_path).convert('RGB');results=[]
    for w,h in [(1280,720),(1920,1080)]:
        im=source.resize((w,h),Image.Resampling.BILINEAR);rgb=np.array(im,dtype=np.uint16)
        raw=(((rgb[...,0]>>3)<<11)|((rgb[...,1]>>2)<<5)|(rgb[...,2]>>3)).astype('<u2').tobytes();rows={}
        for y in range(h):
            row=raw[y*w*2:(y+1)*w*2];kind,encoded=codec.encode_row(row);rows[y]=(kind,encoded,zlib.crc32(struct.pack('<H',y)+row))
        all_native=(codec._native_dll,codec._native_decode,codec._native_frame)
        modes=['native','fallback'] if w==1280 else ['native']
        for mode in modes:
            if mode=='fallback':codec._native_dll=codec._native_decode=codec._native_frame=None
            else:codec._native_dll,codec._native_decode,codec._native_frame=all_native
            times=[];totals=[]
            for i in range(rounds+3):
                begin=time.perf_counter();decoded=codec.decode_checked_frame(rows,w,h);middle=time.perf_counter();converted=codec.preview_rgb_image(w,h,decoded);end=time.perf_counter()
                assert decoded==raw
                if i>=3:times.append((middle-begin)*1000);totals.append((end-begin)*1000)
            results.append(dict(width=w,height=h,path=mode,rounds=rounds,decode_median_ms=float(np.median(times)),decode_p95_ms=float(np.percentile(times,95)),decode_and_rgb_median_ms=float(np.median(totals)),exact_rgb565=True))
        codec._native_dll,codec._native_decode,codec._native_frame=all_native
    report=dict(scope='Synthetic PGD1 frames from the named image; CPU decode only, excludes NIC, camera exposure, render and Tk display.',native_available=codec.NATIVE_AVAILABLE,native_frame_available=codec.NATIVE_FRAME_AVAILABLE,input_sha256=hashlib.sha256(Path(image_path).read_bytes()).hexdigest(),results=results)
    Path(result_path).write_text(json.dumps(report,indent=2),encoding='utf-8');return report
