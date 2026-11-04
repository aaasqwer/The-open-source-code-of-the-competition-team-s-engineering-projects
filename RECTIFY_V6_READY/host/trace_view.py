"""Three independent, throttled 60-second plots over a 10-minute history."""
import math
import time
from pathlib import Path
import tkinter as tk
from tkinter import ttk,filedialog,messagebox


class TraceView(ttk.Frame):
    def __init__(self,parent,history):
        super().__init__(parent)
        self.history=history;self.live=True;self.start=None;self._updating=False;self.protected_paths=lambda:()
        self._last_key=None;self._dirty=True;self.draw_count=0
        toolbar=ttk.Frame(self);toolbar.pack(fill='x',pady=(0,5))
        ttk.Label(toolbar,text='姿态历史 · 每屏 60 秒 · 保留 10 分钟').pack(side='left')
        ttk.Button(toolbar,text='回到实时',command=self.go_live).pack(side='right')
        ttk.Button(toolbar,text='导出历史 CSV',command=self.export).pack(side='right',padx=5)
        self.autoscale=tk.BooleanVar(value=True)
        ttk.Checkbutton(toolbar,text='纵轴自动放大',variable=self.autoscale,command=self.invalidate).pack(side='right',padx=5)
        self.canvas=tk.Canvas(self,bg='#f2f7f7',height=430,highlightthickness=0)
        self.canvas.pack(fill='both',expand=True);self.canvas.bind('<Configure>',lambda e:self.invalidate())
        self.status=tk.StringVar(value='历史 0 组')
        ttk.Label(self,textvariable=self.status,wraplength=900).pack(anchor='w',pady=5)
        nav=ttk.Frame(self);nav.pack(fill='x')
        ttk.Label(nav,text='历史起点').pack(side='left')
        self.slider=ttk.Scale(nav,from_=0,to=0,command=self.seek);self.slider.pack(side='left',fill='x',expand=True,padx=8)
        self.tick_id=self.after(250,self.tick)

    def invalidate(self):self._dirty=True
    def go_live(self):self.live=True;self.start=None;self.invalidate()
    def seek(self,value):
        if self._updating or self.history.origin is None:return
        self.live=False;self.start=self.history.origin+float(value);self.invalidate()

    def export(self):
        if not self.history.count:messagebox.showinfo('姿态历史','暂无数据');return
        path=filedialog.asksaveasfilename(defaultextension='.csv',filetypes=[('姿态历史 CSV','*.csv')])
        if path:
            try:
                if Path(path).resolve() in self.protected_paths():raise ValueError('该文件正在记录，无法覆盖')
                count=self.history.export_csv(path);self.status.set(f'CSV 已导出 · {count} 组')
            except (OSError,ValueError) as exc:messagebox.showerror('导出失败',str(exc))

    def tick(self):
        if self.winfo_ismapped():self.draw()
        self.tick_id=self.after(250,self.tick)

    def draw(self,now=None):
        h=self.history;now=time.monotonic() if now is None else now
        if h.origin is None:
            start=now-60
        elif self.live:
            start=max(h.oldest,now-60)
        else:
            start=max(h.oldest,min(self.start if self.start is not None else h.oldest,max(h.oldest,h.newest-60)))
        end=start+60
        # A paused historical view does not redraw when new samples arrive.
        key=(round(start,2),h.version if self.live else 0,self.canvas.winfo_width(),self.canvas.winfo_height(),self.autoscale.get())
        if not self._dirty and key==self._last_key:return
        self._dirty=False;self._last_key=key;self.draw_count+=1
        rows=list(h.rows(start,end));c=self.canvas;c.delete('all')
        w=max(c.winfo_width(),100);height=max(c.winfo_height(),1);left=68;right=w-22
        if height<120:return
        panel=height/3;colors=('#147d86','#c57d21','#715caa');labels=('Roll 横滚','Pitch 俯仰','Yaw 航向')
        for j,(color,label) in enumerate(zip(colors,labels)):
            top=j*panel+min(27,panel*.2);bottom=(j+1)*panel-min(25,panel*.2)
            values=[row[j+1] for row in rows if row[j+1] is not None]
            low,high=-180.,180.
            if self.autoscale.get() and values:
                mid=(min(values)+max(values))/2;span=max(10.,(max(values)-min(values))*1.25)
                low=max(-180.,mid-span/2);high=min(180.,mid+span/2)
                if high-low<10:low=max(-180,high-10);high=min(180,low+10)
            c.create_text(left,j*panel+11,text=label+' / °',anchor='w',fill=color,font=('Microsoft YaHei UI',11,'bold'))
            for n in range(5):
                value=low+(high-low)*n/4;y=bottom-(bottom-top)*n/4
                c.create_line(left,y,right,y,fill='#d3e1e3')
                c.create_text(left-6,y,text=f'{value:.1f}',anchor='e',fill='#5d747a')
            for n in range(7):
                x=left+(right-left)*n/6;c.create_line(x,top,x,bottom,fill='#e2ebec')
                if h.origin is not None:c.create_text(x,bottom+11,text=f'{start-h.origin+n*10:.0f}s',fill='#5d747a')
            points=[];previous=None;last_pixel=None
            def flush():
                if len(points)>=4:c.create_line(points,fill=color,width=2)
            for row in rows:
                t=row[0];value=row[j+1]
                if value is None or previous is not None and (t-previous[0]>.5 or abs(value-previous[1])>180):
                    flush();points=[];previous=None;last_pixel=None
                if value is not None:
                    x=left+(t-start)/60*(right-left);y=bottom-(value-low)/(high-low)*(bottom-top)
                    # Only collapse indistinguishable screen pixels, not extrema.
                    pixel=(round(x),round(y))
                    if pixel!=last_pixel:points.extend((x,y));last_pixel=pixel
                    previous=(t,value)
            flush()
        if h.origin is not None:
            earliest=h.oldest-h.origin;latest=max(earliest,h.newest-h.origin-60)
            self._updating=True
            self.slider.configure(from_=earliest,to=max(earliest+.001,latest));self.slider.set(start-h.origin)
            self._updating=False
            self.status.set(f"{'实时' if self.live else '回看'} · {start-h.origin:.1f}–{end-h.origin:.1f}s · {h.count} 组 / {min(600,h.newest-h.oldest):.1f}s")

    def destroy(self):
        if self.tick_id:self.after_cancel(self.tick_id)
        super().destroy()
