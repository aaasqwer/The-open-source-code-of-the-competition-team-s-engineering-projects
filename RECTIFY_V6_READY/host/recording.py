"""Own both recording files together; partial startup never enters active state."""
import csv
import json
from pathlib import Path


class TelemetryRecorder:
    def __init__(self,path,fields):
        path=Path(path)
        if not path.suffix:path=path.with_suffix('.jsonl')
        if path.suffix.lower()!='.jsonl':raise ValueError('记录文件扩展名应为 .jsonl')
        self.paths=(path.resolve(),path.with_suffix('.csv').resolve())
        self.json_file=None;self.csv_file=None;self.closed=False
        try:
            # Open both first, then truncate: a locked second file must not
            # truncate an existing first file before the pair is ready.
            self.json_file=self.paths[0].open('a',encoding='utf-8')
            self.csv_file=self.paths[1].open('a',newline='',encoding='utf-8-sig')
            self.json_file.seek(0);self.json_file.truncate()
            self.csv_file.seek(0);self.csv_file.truncate()
            self.writer=csv.DictWriter(self.csv_file,fieldnames=fields);self.writer.writeheader();self.csv_file.flush()
        except Exception:
            self.close();raise
    def write(self,packet):
        if self.closed:raise OSError('记录已关闭')
        self.json_file.write(json.dumps(packet,ensure_ascii=False)+'\n');self.json_file.flush()
        if packet['kind']==0x81:
            row={'monotonic_s':packet['received'],'sequence':packet['sequence'],'imu_frames':packet['imu_frames'],
                 'imu_errors':packet['imu_errors'],**packet['values']}
            extra={'valid_flags':packet['flags'],'config_flags':packet['config_flags'],
                   **{name+'_raw':packet['raw'][name] if packet['flags']&4 else '' for name in ('ax','ay','az','gx','gy','gz')}}
            row.update({k:v for k,v in extra.items() if k in self.writer.fieldnames})
            self.writer.writerow(row);self.csv_file.flush()
    def close(self):
        self.closed=True
        for stream in (self.json_file,self.csv_file):
            if stream:
                try:stream.close()
                except OSError:pass
