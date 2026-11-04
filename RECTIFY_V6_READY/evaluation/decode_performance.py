"""Decode 512-bit PDS ILA probe (hex string, or CSV column --column)."""
from pathlib import Path
import argparse,csv,json
def decode(value,hardware_measured=False):
 n=int(str(value).strip().replace('_','').replace('0x',''),16);w=[(n>>(32*i))&0xffffffff for i in range(15,-1,-1)]
 if w[0]!=0x45563331:raise ValueError('Wrong probe magic; export performance_probe[511:0] in hexadecimal')
 hz=w[1]
 def ms(t):return None if t==0xffffffff or not w[15]&2 else round(1000*t/hz,6)
 return dict(window=w[2],clock_hz=hz,sensor_vs_per_second=w[3]>>16,accepted_source_fps=w[3]&65535,render_publications_per_second=w[4]>>16,
  unique_valid_display_fps=w[4]&65535,hdmi_scan_fps=w[5]>>16,repeated_valid_display_fps=w[5]&65535,
  source_first_pixel_to_render_ms=round(1000*w[6]/hz,6),source_first_pixel_to_hdmi_first_pixel_ms=ms(w[7]),min_hdmi_latency_ms=ms(w[8]),max_hdmi_latency_ms=ms(w[9]),
  last_render_busy_ms=round(1000*w[10]/hz,6),invalid_display_frames=w[11],capture_overflow_flag_rises=w[12],last_display_source_id=w[13],width=w[14]>>16,height=w[14]&65535,
  one_second_window_valid=bool(w[15]&1),latency_valid=bool(w[15]&2),hardware_measured=hardware_measured,
  latency_scope='First digital camera RGB565 pixel to first HDMI pixel before output register; add 1/74.25MHz (13.468ns) for pin register. Exposure and monitor response excluded. Gray CDC sampling uncertainty must be recorded.')
def main():
 ap=argparse.ArgumentParser();ap.add_argument('input');ap.add_argument('--column',default='performance_probe[511:0]');ap.add_argument('--output',type=Path,default=Path('performance_result.json'));args=ap.parse_args()
 path=Path(args.input)
 if path.is_file():
  with path.open(encoding='utf-8-sig',newline='') as f:values=[r[args.column] for r in csv.DictReader(f)]
 else:values=[args.input]
 results=[];seen=set()
 for v in values:
  if 'x' in v.lower().replace('0x',''):continue
  r=decode(v)
  if r['window'] not in seen:results.append(r);seen.add(r['window'])
 args.output.write_text(json.dumps(dict(records=results),ensure_ascii=False,indent=2),encoding='utf-8');print('Decoded',len(results),'windows to',args.output)
if __name__=='__main__':main()
