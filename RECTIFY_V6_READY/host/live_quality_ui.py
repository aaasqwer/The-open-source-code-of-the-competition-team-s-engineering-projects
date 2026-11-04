"""BKB workbench extension; analysis stays outside capture/decode and Tk threads."""
import json
import queue
import time
import datetime
import threading
import hashlib
from pathlib import Path
import tkinter as tk
from tkinter import ttk,filedialog
from live_quality import LatestMeasurement


class LiveQualityPanel:
    def __init__(self,app,tabs):
        self.app=app;self.worker=LatestMeasurement();self.last_submit=0;self.result=None;self.last_valid_sfr=None;self.epoch=0;self.active_roi=None
        self.enabled=tk.BooleanVar(value=False);self.axis=tk.StringVar(value='竖边');self.gamma=tk.StringVar(value='输出码值')
        pane=ttk.Frame(tabs,padding=12);tabs.add(pane,text='实时直线度 / SFR');self.pane=pane
        row=ttk.Frame(pane);row.pack(fill='x')
        ttk.Checkbutton(row,text='实时测量（连续画面保持采集）',variable=self.enabled,command=self.remeasure).pack(side='left')
        ttk.Combobox(row,textvariable=self.axis,values=['竖边','横边'],width=7,state='readonly').pack(side='left',padx=8)
        ttk.Combobox(row,textvariable=self.gamma,values=['输出码值','假设 sRGB'],width=12,state='readonly').pack(side='left',padx=8)
        ttk.Button(row,text='测量当前图像',command=lambda:self.offer(self.app.current_image,self.app.current_metadata,force=True) if self.app.current_image else None).pack(side='left')
        ttk.Button(row,text='中央测试框',command=self.center).pack(side='left')
        ttk.Button(row,text='导出当前测量',command=self.export).pack(side='left',padx=8)
        ttk.Button(row,text='导出独立复核包',command=self.export_bundle).pack(side='left',padx=8)
        ttk.Label(pane,text='静态照片：框选后自动计算，或点“测量当前图像”；连续画面：勾选实时测量。\n在“图像与效果”拖出选框，框住一条完整边。\n直线度可测直边或弯边；SFR 请使用偏离水平/竖直约 2°～8° 的单一高对比斜边，避免交叉、反光和过曝。').pack(anchor='w',pady=8)
        self.notice=tk.StringVar();ttk.Label(pane,textvariable=self.notice,wraplength=1250).pack(fill='x')
        self.status=tk.StringVar(value='静态图框选后自动计算；连续画面请勾选实时测量');ttk.Label(pane,textvariable=self.status,wraplength=1250,justify='left').pack(fill='x')
        self.canvas=tk.Canvas(pane,bg='#132630',highlightthickness=0,height=230);self.canvas.pack(fill='both',expand=True,pady=10)
        self.canvas.bind('<Configure>',lambda e:self.draw())
        ttk.Label(pane,text='数值以所测图像的像素为单位。720P 预览、1080P 照片和左右对比分屏不能直接比较；相同姿态请用同分辨率完整照片验收。\n输出码值 SFR 包含相机与 Gamma 的影响；“假设 sRGB”也不是测得的 OECF。无效结果显示原因，不当作有效 MTF50。',wraplength=1250).pack(anchor='w',before=self.canvas)
        for child in row.winfo_children():
            if isinstance(child,ttk.Combobox):child.bind('<<ComboboxSelected>>',lambda event:self.remeasure())
        app.root.after(50,self.poll)
    def remeasure(self):
        self.reset()
        if self.app.current_image is not None:self.offer(self.app.current_image,self.app.current_metadata)
    def reset(self):
        self.epoch+=1;self.result=None;self.last_valid_sfr=None;self.active_roi=None;self.last_submit=0;self.status.set('等待新画面' if self.enabled.get() else '静态照片框选后自动计算，或点击“测量当前图像”');self.notice.set('');self.draw()
    def center(self):
        if self.app.current_image is None:return
        w,h=self.app.current_image.size
        rw,rh=(min(256,w-4),min(128,h-4)) if self.axis.get()=='横边' else (min(128,w-4),min(256,h-4))
        self.app.roi=((w-rw)//2,(h-rh)//2,(w+rw)//2,(h+rh)//2);self.app.render_image();self.reset()
        self.offer(self.app.current_image,self.app.current_metadata)
    def offer(self,image,metadata,force=False):
        if image is None or self.app.drag is not None:return
        is_live=bool(metadata.get('preview') and getattr(self.app,'continuous',False) and not force)
        if is_live and not self.enabled.get():return
        now=time.monotonic()
        if is_live and now-self.last_submit<.12:return
        roi=self.app.roi
        if not roi:
            w,h=image.size
            rw,rh=(min(256,w-4),min(128,h-4)) if self.axis.get()=='横边' else (min(128,w-4),min(256,h-4))
            roi=((w-rw)//2,(h-rh)//2,(w+rw)//2,(h+rh)//2);self.app.roi=roi
        signature=(tuple(roi),image.size,self.axis.get(),self.gamma.get(),None if is_live else id(image))
        if signature!=self.active_roi:self.epoch+=1;self.active_roi=signature;self.result=None
        # PGV2 mode 2 is now stamped from the leased split frame, not a later UI poll.
        split=(metadata.get('display_layout')=='split' or (metadata.get('frame_geometry_flags',0)&3)==3)
        if split:
            self.epoch+=1;self.result=None;self.last_valid_sfr=None
            self.status.set('当前是左右分屏演示：每边已经缩小，停止原生直线度/SFR精测。请按K0进入完整矫正画面，拍1080P后测同一选框。分屏RMS与完整图RMS不是同一像素尺度。')
            self.draw();return
        self.last_submit=now
        identity=dict(epoch=self.epoch,frame=metadata.get('frame_id'),source=metadata.get('source'),preview=is_live,is_live=is_live,original_preview=metadata.get('preview',False),
                      frame_geometry_flags=metadata.get('frame_geometry_flags'),association=metadata.get('frame_geometry_association','preview_channel_only'))
        self.worker.submit(image,roi,'horizontal' if self.axis.get()=='横边' else 'vertical','srgb' if self.gamma.get()=='假设 sRGB' else 'linear',identity)
    def poll(self):
        if self.app.closing:self.worker.stop.set();return
        try:
            result=self.worker.results.get_nowait()
            if (self.enabled.get() or not result['identity'].get('is_live')) and result['identity']['epoch']==self.epoch:
                self.result=result
                if result.get('sfr',{}).get('engineering_valid'):self.last_valid_sfr=result
                if 'error' in result:self.status.set('未通过检测：'+result['error'])
                else:
                    m=result['straightness'];s=result.get('sfr',{});mtf=s.get('mtf50_cycles_per_pixel')
                    label=(f'{mtf:.4f} cycles/pixel' if mtf is not None else '>0.5 cycles/pixel（超过本次测量频带，不是画面没有清晰度）') if s.get('engineering_valid') else '本帧不测：'+result.get('sfr_reason','条件不足')
                    angle=s.get('angle_degrees');angle_label=f'{angle:.2f}°' if angle is not None else '未识别'
                    angle_text=f'边缘实测偏角 {angle_label}（相对水平/竖直，要求2°～8°；不是相机支架角度）'
                    detector=result.get('detector',{})
                    if detector:angle_text+=f" · 亮暗差/噪声 {detector['contrast_to_noise']:.1f}"
                    self.status.set(f"{result['resolution'][0]}×{result['resolution'][1]} · 帧 {result['identity']['frame']} · 直线度 RMS {m['rms_px']:.3f} px · 最大 {m['max_abs_px']:.3f} px · 覆盖 {result['coverage']:.0%}\n{angle_text}\nMTF50 {label}\nROI 计算 {result['compute_ms']:.2f} ms · {self.gamma.get()}（非实验室认证）")
                self.draw();self.app.render_image()
        except queue.Empty:pass
        if self.result:
            live=self.result['identity'].get('is_live',False) and getattr(self.app,'continuous',False)
            if live and time.monotonic()-self.result['completed_at']>1.5:self.notice.set('连续采集暂无新测量；以下保留最近一帧数值，未当作当前新帧。')
            elif not live:self.notice.set('静态图测量：RMS、最大偏差和曲线持续保留；数值属于当前保存的像素。')
            else:self.notice.set('连续测量：显示最近完成帧的实际结果。')
        self.app.root.after(50,self.poll)
    def overlay(self):
        if not self.result or 'error' in self.result:return
        if self.result['identity']['epoch']!=self.epoch:return
        # Trace is explicitly latest measured frame, not asserted pixel-exact
        # registration with a potentially newer displayed preview frame.
        if self.result['identity']['frame']!=self.app.current_metadata.get('frame_id'):return
        c=self.app.canvas;ox,oy=self.app.offset;s=self.app.scale
        pts=[v for x,y in self.result['points'][::2] for v in (ox+x*s,oy+y*s)]
        if len(pts)>=4:c.create_line(pts,fill='#06d9dd',width=1)
    def draw(self):
        c=self.canvas;c.delete('all');w=max(400,c.winfo_width());h=max(250,c.winfo_height())
        c.create_text(18,18,anchor='nw',fill='white',text='直线残差（保留弯曲）                                        SFR / MTF 曲线')
        if not self.result:return
        result=self.result;half=w/2
        for left,right in [(35,half-25),(half+35,w-25)]:c.create_rectangle(left,55,right,h-35,outline='#617b88')
        residual=result.get('residual_px',[]);limit=max([.5]+list(map(abs,residual)));pts=[]
        for i,y in enumerate(residual):pts.extend((35+i*(half-60)/max(1,len(residual)-1),(h+20)/2-y*(h-110)/(2*limit)))
        c.create_line(35,(h+20)/2,half-25,(h+20)/2,fill='#617b88')
        if len(pts)>=4:c.create_line(pts,fill='#06d9dd',width=2)
        c.create_text(38,58,anchor='nw',text=f'±{limit:.3f} px',fill='white')
        sf=result.get('sfr',{});pts=[];historical=False
        if sf.get('engineering_valid') and result['identity'].get('is_live') and getattr(self.app,'continuous',False) and time.monotonic()-result['completed_at']>1.5:
            historical=True;c.create_text(half+40,60,anchor='nw',fill='#aab3b8',text='历史有效曲线；当前没有新测量')
        if not sf.get('engineering_valid'):
            previous=self.last_valid_sfr
            if previous and previous['identity']['epoch']==self.epoch and time.monotonic()-previous['completed_at']<=1.5:
                sf=previous['sfr'];historical=True
                age=time.monotonic()-previous['completed_at']
                c.create_text(half+40,60,anchor='nw',fill='#aab3b8',text=f"上次有效曲线 · 帧{previous['identity']['frame']} · {age:.1f}秒前；当前帧无效")
            else:
                sf={};c.create_text(half+40,75,anchor='nw',fill='#ffc34c',text='本帧未满足斜边条件，未绘制有效SFR曲线')
        for f,y in zip(sf.get('frequency_cycles_per_pixel',[]),sf.get('sfr',[])):pts.extend((half+35+f*2*(half-60),h-35-min(1.25,max(0,y))*(h-90)/1.25))
        if len(pts)>=4:c.create_line(pts,fill='#89959b' if historical else '#71e897',width=2,dash=(4,4) if historical else ())
        c.create_text(w-28,h-20,anchor='e',text='0.5 cycles/pixel',fill='white')
    def export(self):
        if not self.result:return
        path=filedialog.asksaveasfilename(defaultextension='.json',filetypes=[('测量结果 JSON','*.json')])
        if path:Path(path).write_text(json.dumps({k:v for k,v in self.result.items() if not k.startswith('_')},ensure_ascii=False,indent=2),encoding='utf-8')

    def export_bundle(self):
        result=self.result
        if not result or 'error' in result or '_source_image' not in result:
            self.status.set('请先获得一份有效直线度测量，再导出复核包');return
        folder=filedialog.askdirectory(title='选择独立复核包保存目录')
        if not folder:return
        # Pin the image owned by the measurement, never the newer displayed one.
        image=result['_source_image'];public={k:v for k,v in result.items() if not k.startswith('_')}
        stamp='REVIEW_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f');folder=Path(folder)
        def write():
            try:
                import numpy as np
                from PIL.PngImagePlugin import PngInfo
                image_path=folder/(stamp+'.png');result_path=folder/(stamp+'_measurement.json');manifest_path=folder/(stamp+'_manifest.json')
                metadata=PngInfo();metadata.add_text('RECTIFY_MEASUREMENT_FRAME',json.dumps(public['identity'],ensure_ascii=False))
                image.save(image_path,pnginfo=metadata,compress_level=1)
                result_path.write_text(json.dumps(public,ensure_ascii=False,indent=2),encoding='utf-8')
                points=public['points'];span=float(np.linalg.norm(np.array(points[-1])-np.array(points[0])))
                anchors_count=min(9,max(2,int(span/24)+1));indices=np.unique(np.linspace(0,len(points)-1,anchors_count).astype(int))
                anchors=[points[int(i)] for i in indices]
                x0,y0,x1,y1=public['roi']
                manifest=dict(images=[dict(path=image_path.name,role='same_frame_independent_review',width=image.width,height=image.height,
                    source_identity=public['identity'],image_sha256=hashlib.sha256(image_path.read_bytes()).hexdigest(),
                    lines=[dict(name='ROI physical edge',anchors=anchors,half_width=8)],
                    sfr_rois=[dict(name='ROI slanted edge',x=x0,y=y0,w=x1-x0,h=y1-y0,axis=public['axis'],gamma=public['linearization'])])],
                    notes=['Same saved image that produced the live result; no later preview substituted.',
                           'Anchors guide an independent detector; all detected points enter one global line fit; no residual-based point deletion.',
                           'linear means output-code SFR here, not a measured linear-light OECF. sRGB is only an assumption.',
                           'Preview and split-view measurements keep their own pixel units; use full 1080P pictures for native acceptance.'])
                manifest_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
                self.app.events.put(('quality_exported',str(manifest_path)))
            except Exception as exc:self.app.events.put(('quality_export_error',str(exc)))
        threading.Thread(target=write,daemon=True).start();self.status.set('正在导出该测量对应的原像素帧与区域清单…')
