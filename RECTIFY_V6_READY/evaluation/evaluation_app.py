"""Separate evaluation entry point; does not import or modify the host app."""
from pathlib import Path
import sys,json,time,threading,queue,contextlib,traceback
import tkinter as tk
from tkinter import ttk,filedialog,messagebox,scrolledtext
import image_quality,capture_performance,serial_ports

def main():
 if '--port-diagnostic' in sys.argv:
  out=Path(sys.argv[sys.argv.index('--port-diagnostic')+1]);out.write_text(json.dumps(serial_ports.diagnostic(),ensure_ascii=False,indent=2),encoding='utf-8');return
 if '--self-test' in sys.argv:
  out=Path(sys.argv[sys.argv.index('--self-test')+1]);out.write_text(json.dumps(image_quality.self_test(),indent=2),encoding='utf-8');return
 root=tk.Tk();root.title('V6 独立测评工具');root.geometry('850x610');root.minsize(760,500)
 events=queue.Queue();active=[False];widgets=[]
 ttk.Label(root,text='赛题二 · 直线度 / MTF50 / 硬件性能',font=('Microsoft YaHei UI',16,'bold')).pack(anchor='w',padx=18,pady=(15,5))
 ttk.Label(root,text='本工具独立运行，原上位机的画面、拍照和串口功能保持原样。').pack(anchor='w',padx=18)
 hardware=ttk.LabelFrame(root,text='硬件帧率与延迟');hardware.pack(fill='x',padx=18,pady=12)
 port=tk.StringVar();combo=ttk.Combobox(hardware,textvariable=port,width=32);combo.grid(row=0,column=0,padx=8,pady=8);widgets.append(combo)
 seconds=tk.StringVar(value='60');ttk.Label(hardware,text='秒数').grid(row=0,column=2);entry=ttk.Entry(hardware,textvariable=seconds,width=7);entry.grid(row=0,column=3,padx=8);widgets.append(entry)
 def refresh():
  scan=serial_ports.discover_ports();combo['values']=[p.label for p in scan.ports]
  device=serial_ports.choose_device(scan.ports,port.get())
  selected=next((p for p in scan.ports if p.device==device),None)
  port.set(selected.label if selected else device)
  events.put(('log',serial_ports.scan_hint(scan,device)+'\n'))
 def job(fn):
  if active[0]:return
  active[0]=True
  for w in widgets:w.configure(state='disabled')
  def worker():
   try:fn();events.put(('log','完成。\n'))
   except BaseException as e:events.put(('error',str(e)));events.put(('log',traceback.format_exc()+'\n'))
   finally:events.put(('done',''))
  threading.Thread(target=worker,daemon=True).start()
 class Log:
  def write(self,s):
   if s:events.put(('log',s))
   return len(s)
  def flush(self):pass
 def capture():
  try:duration=int(seconds.get());assert duration>=5
  except (ValueError,AssertionError):messagebox.showerror('秒数','请输入至少5秒，建议60秒。');return
  if not port.get().strip():messagebox.showerror('串口','请选择或输入板卡参数串口，例如COM6。');return
  path=filedialog.asksaveasfilename(title='保存硬件性能报告',defaultextension='.json',initialfile='hardware_performance.json')
  if not path:return
  try:selected=serial_ports.device_name(port.get())
  except ValueError as e:messagebox.showerror('串口',str(e));return
  events.put(('log','开始前请确保原上位机已关闭参数串口。只发送性能查询，图像网络不变。\n'))
  def run():
   old=sys.argv[:]
   try:
    sys.argv=['capture_performance','--port',selected,'--seconds',str(duration),'--output',path]
    with contextlib.redirect_stdout(Log()):capture_performance.main()
   finally:sys.argv=old
  job(run)
 refresh_button=ttk.Button(hardware,text='刷新串口',command=refresh);refresh_button.grid(row=0,column=1,padx=8);widgets.append(refresh_button)
 start=ttk.Button(hardware,text='开始硬件测量',command=capture);start.grid(row=0,column=4,padx=8);widgets.append(start)
 ttk.Label(hardware,text='先关闭原上位机的参数串口，再测量。测完在原上位机重新打开。可手动输入COM号。').grid(row=1,column=0,columnspan=5,padx=8,pady=(0,8),sticky='w')
 def show_help():
  window=tk.Toplevel(root);window.title('一步一步使用独立测评工具');window.geometry('850x650')
  text=scrolledtext.ScrolledText(window,wrap='word',font=('Microsoft YaHei UI',11));text.pack(fill='both',expand=True,padx=15,pady=15)
  text.insert('end',"""这个工具是复核员：用保存的照片重新测量，或统计板卡真实帧率/延迟。它不负责把照片矫正。

测照片（不需要接板卡）：
1. 打开主上位机的“图像与效果”，打开一张1080P完整照片，或先按K0进入完整矫正画面再拍照。不要用左右分屏截图精测。
2. 静态图无需开连续采集：框选后自动测，也可到“实时直线度 / SFR”点“测量当前图像”。连续画面才需要勾选实时测量。框内不要有棋盘交叉点。
3. 回实时页看结果。测SFR时把板上这条边调到偏离水平/竖直约5°；纯0°仍能测直线度，但不测斜边SFR。
4. 点“导出独立复核包”，选一个空文件夹。程序会保存照片、测量记录和一个名字以 _manifest.json 结尾的区域清单。
5. 打开本工具，点“选择区域清单并计算”，选刚生成的 _manifest.json；再给报告起名并保存。
6. 下方日志显示每条边的RMS、最大偏差、MTF50和是否有效。报告旁边会生成带红色采样点的照片。
7. RMS越小表示检测到的边越接近一条直线；MTF50单位是cycles/pixel。无效要看拒绝原因，不是画面没有清晰度。

测板卡帧率/延迟（需要接板卡）：
1. 保持板卡运行，在主上位机点击参数串口“关闭”，不要关闭网口或断电。
2. 在本工具点“刷新串口”，选同一个板卡COM；秒数填60。
3. 点“开始硬件测量”，选择报告保存位置，等60秒结束。
4. 本工具只查询，不修改相机/矫正模式。完成后关本工具，在主上位机重新打开COM。
5. 一个COM不能被两个程序同时打开。图像离线复核不使用COM，不需要关闭主机串口。

“运行测评算法自检”只检查程序能否正确测已知样本，不是你的镜头实测结果。
""")
  text.configure(state='disabled')
 ttk.Button(root,text='怎么用？一步一步说明',command=show_help).pack(anchor='w',padx=18,pady=4)
 quality=ttk.LabelFrame(root,text='图像直线度与斜边清晰度');quality.pack(fill='x',padx=18,pady=(0,12))
 def quality_run():
  manifest=filedialog.askopenfilename(title='选择评估区域清单',filetypes=[('JSON 清单','*.json')])
  if not manifest:return
  output=filedialog.asksaveasfilename(title='保存质量报告',defaultextension='.json',initialfile='quality_result.json')
  if not output:return
  def run():
   old=sys.argv[:]
   try:
    sys.argv=['image_quality','--manifest',manifest,'--output',output]
    with contextlib.redirect_stdout(Log()):image_quality.main()
    report=json.loads(Path(output).read_text(encoding='utf-8'))
    for record in report.get('records',[]):
     for measured in record.get('sfr_rois',[]):
      if not measured.get('engineering_valid'):
       reason=measured.get('error') or '；'.join(measured.get('rejection_reasons',[])) or '当前选框不满足有效斜边条件'
       events.put(('log',f"  {measured.get('name','SFR')} 无效原因：{reason}\n"))
   finally:sys.argv=old
  job(run)
 def selftest():
  def run():
   r=image_quality.self_test();events.put(('log','自检通过：9组解析高斯模糊MTF50、已知直边和弯曲边。\n'))
  job(run)
 qbutton=ttk.Button(quality,text='选择区域清单并计算',command=quality_run);qbutton.grid(row=0,column=0,padx=8,pady=9);widgets.append(qbutton)
 test=ttk.Button(quality,text='运行测评算法自检',command=selftest);test.grid(row=0,column=1,padx=8);widgets.append(test)
 ttk.Label(quality,text='先在主上位机框选一条边 → 实时页点“导出独立复核包” → 在这里选择生成的 _manifest.json。图像复核不需要接板卡。').grid(row=1,column=0,columnspan=3,padx=8,pady=(0,8),sticky='w')
 log=scrolledtext.ScrolledText(root,height=16,wrap='word',font=('Microsoft YaHei UI',10));log.pack(fill='both',expand=True,padx=18,pady=(0,15))
 def poll():
  try:
   while True:
    kind,s=events.get_nowait()
    if kind=='log':log.insert('end',s);log.see('end')
    elif kind=='error':messagebox.showerror('测评未完成',s)
    elif kind=='done':
     active[0]=False
     for w in widgets:w.configure(state='normal')
  except queue.Empty:pass
  root.after(100,poll)
 try:refresh()
 except Exception as e:events.put(('log',f'枚举端口失败，可手动输入：{e}\n'))
 poll();root.mainloop()
if __name__=='__main__':main()
