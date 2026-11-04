"""Chinese geometry controls; physical KEY0 long-hold is the only plane-enable input."""
import time
import tkinter as tk
from tkinter import ttk
import geometry as g

class GeometryPanel:
    def __init__(self,app,tabs):
        self.app=app;self.points=[];self.picking=False;self.pending=None;self.p=g.profile()
        pane=ttk.Frame(tabs,padding=16);tabs.add(pane,text='矫正与平面');self.pane=pane
        ttk.Label(pane,text='默认镜头旁路、IMU回正开启；短按KEY2开/关回正。K0短按切换镜头矫正；K1短按对比。K4按住看IMU参数；K0长按启用平面正视，松开恢复。',font=('Microsoft YaHei UI',12)).pack(anchor='w',pady=8)
        ttk.Label(pane,text='这里的“原图”仅旁路镜头几何；IMU和Gamma仍可启用。用于标定的真正原图请点“原图拍照”。').pack(anchor='w')
        controls=ttk.LabelFrame(pane,text='屏幕显示',padding=10);controls.pack(fill='x')
        self.mode=tk.StringVar(value='原图');self.imu=tk.BooleanVar(value=True);self.caption=tk.BooleanVar(value=True);self.numbers=tk.BooleanVar(value=True);self.evaluate=tk.BooleanVar(value=False)
        ttk.Combobox(controls,textvariable=self.mode,values=['原图','增强矫正','题目二阶矫正','左右对比'],state='readonly',width=16).pack(side='left')
        for text,var in [('IMU回正',self.imu),('字幕',self.caption),('参数数字',self.numbers),('局部测量',self.evaluate)]:ttk.Checkbutton(controls,text=text,variable=var).pack(side='left',padx=5)
        ttk.Button(controls,text='应用屏幕设置',command=lambda:app.guarded(self.apply_ui)).pack(side='left',padx=10)
        affine=ttk.LabelFrame(pane,text='动态仿射变换（旋转、双轴缩放、剪切、平移）',padding=10);affine.pack(fill='x',pady=10)
        self.params=[]
        for i,(label,value) in enumerate([('旋转°','0'),('横向缩放','1'),('纵向缩放','1'),('剪切','0'),('横移px','0'),('纵移px','0')]):
            ttk.Label(affine,text=label).grid(row=0,column=i,padx=5);v=tk.StringVar(value=value);self.params.append(v);ttk.Entry(affine,textvariable=v,width=11).grid(row=1,column=i,padx=5)
        ttk.Button(affine,text='发送普通画面变换',command=lambda:app.guarded(lambda:self.send_matrix(0))).grid(row=2,column=0,columnspan=3,pady=8,sticky='w')
        ttk.Button(affine,text='恢复普通画面几何',command=lambda:app.guarded(lambda:self.clear(0))).grid(row=2,column=3,columnspan=3,pady=8,sticky='w')
        self.pitch=tk.StringVar(value='0');self.yaw=tk.StringVar(value='0');self.live=tk.BooleanVar(value=False);self.last_sent=None
        ttk.Label(affine,text='相机俯仰°').grid(row=3,column=0);ttk.Entry(affine,textvariable=self.pitch,width=11).grid(row=3,column=1)
        ttk.Label(affine,text='相机偏航°').grid(row=3,column=2);ttk.Entry(affine,textvariable=self.yaw,width=11).grid(row=3,column=3)
        ttk.Checkbutton(affine,text='参数变化时动态更新',variable=self.live).grid(row=3,column=4,columnspan=2)
        plane=ttk.LabelFrame(pane,text='目标平面正视',padding=10);plane.pack(fill='x')
        ttk.Label(plane,text='先拍摄或打开1920×1080原图，再依次点击目标平面的左上、右上、右下、左下。\n宽高比填写目标平面真实宽÷高；棋盘辅助使用8×6个内角点间隔，自动填4/3。').pack(anchor='w')
        row=ttk.Frame(plane);row.pack(fill='x',pady=8)
        self.aspect=tk.StringVar(value='1.3333333333');ttk.Label(row,text='宽÷高').pack(side='left');ttk.Entry(row,textvariable=self.aspect,width=12).pack(side='left',padx=6)
        ttk.Button(row,text='开始四角框选',command=lambda:app.guarded(self.start)).pack(side='left',padx=5)
        ttk.Button(row,text='棋盘格辅助',command=lambda:app.guarded(self.detect)).pack(side='left',padx=5)
        ttk.Button(row,text='发送平面变换',command=lambda:app.guarded(lambda:self.send_matrix(1))).pack(side='left',padx=5)
        ttk.Button(row,text='清除平面',command=lambda:app.guarded(lambda:self.clear(1))).pack(side='left',padx=5)
        self.status=tk.StringVar(value='等待板端状态');ttk.Label(pane,textvariable=self.status,wraplength=1150).pack(anchor='w',pady=14)
        quality=f"当前标定：21张训练图，联合估计固定板坐标；重投影RMS {self.p['enhanced_training_reprojection_rms_px']:.3f} px。"
        quality+='\n对照图边缘直线度RMS约0.716 px，最大采样偏差约3.94 px。重投影误差不能代替直线度。'
        quality+='\n仍需实板及独立平整标定板/直尺验收。几何参数在完整帧边界生效。'
        ttk.Label(pane,text=quality,wraplength=1150).pack(anchor='w')
        app.root.after(250,self.tick)

    def fresh(self):
        p=self.app.latest.get(0x85)
        if not p or time.monotonic()-p['received']>3:raise RuntimeError('请连接本工程参数串口，并等待有效状态回读')
        if p['busy']:raise RuntimeError('上一矩阵事务正在校验')
        if self.pending:raise RuntimeError('上一矩阵事务尚未确认')
        return p

    def apply_ui(self):
        self.fresh();mode=['原图','增强矫正','题目二阶矫正','左右对比'].index(self.mode.get())
        flags=int(mode>0)|(max(0,mode-1)<<1)|(int(self.imu.get())<<3)|(int(self.evaluate.get())<<4)|(int(self.numbers.get())<<5)|(int(self.caption.get())<<6)
        self.app.serial.send(0x53,bytes([flags]));self.status.set('屏幕设置已发送，等待完整帧显示状态')

    def ensure_raw(self):
        a=self.app
        if a.current_image is None or a.current_image.size!=(1920,1080):raise ValueError('需要1920×1080原始图像，720P预览不能用于平面标定')
        frozen=a.current_metadata.get('frozen_state',{})
        if frozen and frozen.get('source_stage')!='sensor_rgb565':raise ValueError('请先使用“原图拍照”，不能在已经矫正的图像上标定四角')

    def start(self):
        self.ensure_raw();self.app.stop_board_capture();self.points=[];self.picking=True
        self.app.main_tabs.select(self.app.image_tab);self.status.set('依次点击左上、右上、右下、左下');self.app.render_image()

    def pick(self,event):
        if not self.picking:return False
        p=self.app.image_point(event)
        if p is not None:
            self.points.append(p);self.picking=len(self.points)<4;self.app.render_image()
            self.status.set(f'已选择{len(self.points)}/4角。回到“矫正与平面”页发送。')
        return True

    def draw(self):
        c=self.app.canvas;ox,oy=self.app.offset;s=self.app.scale
        for i,(x,y) in enumerate(self.points):
            x,y=ox+x*s,oy+y*s;c.create_oval(x-5,y-5,x+5,y+5,outline='#ffc34c',width=2);c.create_text(x+12,y-10,text=str(i+1),fill='#ffc34c')

    def new_image(self):
        self.points=[];self.picking=False

    def detect(self):
        self.ensure_raw();self.app.stop_board_capture();self.points=g.chessboard_corners(self.app.current_image);self.aspect.set(str(4/3));self.picking=False
        self.app.main_tabs.select(self.app.image_tab);self.app.render_image();self.status.set('已定位棋盘内角点矩形，请确认后发送平面变换')

    def send_matrix(self,slot):
        packet=self.fresh()
        from camera_parameters import parameters_match
        calibration=self.app.fresh_packet(0x83)
        if not parameters_match(calibration):raise RuntimeError('板端K/D与配套标定不匹配；请先下载本工程配套位流并等待参数回读')
        if slot and packet['display_flags']&3!=1:raise RuntimeError('本版平面四角按增强模型建立；请先应用增强矫正，等待完整帧显示后再发送。使用平面正视时保持增强模式。')
        if self.imu.get() or packet['ui_flags']&8:raise ValueError('四角与仿射标定时请先关闭IMU回正；固定矩阵后可单独测试IMU，超出可用视野会拒绝该帧')
        self.status.set('正在检查完整1920×1080画布…');self.app.root.update_idletasks()
        forward=g.affine(*[float(v.get()) for v in self.params])
        forward=forward@g.camera_tilt(float(self.pitch.get()),float(self.yaw.get()),self.p)
        if slot:
            self.ensure_raw();forward=forward@g.plane_forward(self.points,float(self.aspect.get()),self.p)
        words,zoom=g.inverse_words(forward,self.p)
        self.app.serial.upload_geometry(slot,words);self.pending=(packet['geometry_version'],packet['errors'],time.monotonic())
        self.last_sent=tuple(v.get() for v in self.params)+(self.pitch.get(),self.yaw.get())
        self.status.set(f'已发送{"平面" if slot else "普通"}变换；防越界裁剪倍率{zoom:.3f}，等待CRC回读')

    def clear(self,slot):
        packet=self.fresh();self.app.serial.send(0x52,bytes([slot]));self.pending=(packet['geometry_version'],packet['errors'],time.monotonic())

    def observe(self,p):
        flags=p['ui_flags']
        if getattr(self,'last_ui_flags',None)!=flags:
            self.last_ui_flags=flags
            self.mode.set(['原图','增强矫正','题目二阶矫正','左右对比'][(1+((flags>>1)&3)) if flags&1 else 0])
            for var,bit in [(self.imu,3),(self.evaluate,4),(self.numbers,5),(self.caption,6)]:var.set(bool(flags&(1<<bit)))
        f=p['display_flags'];mode=f&3
        text=f'已显示：{["原图","增强矫正","题目二阶矫正","左右对比"][mode]} · IMU {"开" if f&64 else "关"}（KEY2） · 平面 {"启用" if f&32 else "未启用"} · 四角 {"已加载" if p["plane_valid"] else "未加载"} · 几何版本 {p["geometry_version"]} · 错误 {p["errors"]}'
        if self.pending:
            version,errors,_=self.pending
            if p['errors']!=errors:text+=' · 矩阵校验失败，旧矩阵保留';self.pending=None
            elif p['geometry_version']!=version:text+=' · 矩阵CRC已确认，等待完整帧切换';self.pending=None
        text+=(' · 本帧Gamma旁路' if f&(1<<13) else f' · 本帧Gamma档{(f>>8)&31}')
        self.status.set(text)

    def tick(self):
        if self.pending and time.monotonic()-self.pending[2]>5:
            self.pending=None;self.status.set('矩阵事务超时，未确认生效；检查参数串口')
        values=tuple(v.get() for v in self.params)+(self.pitch.get(),self.yaw.get())
        if self.live.get() and self.pending is None and values!=self.last_sent:
            try:self.send_matrix(0)
            except (ValueError,RuntimeError,ConnectionError) as exc:self.live.set(False);self.status.set('动态更新已停止：'+str(exc))
        if not self.app.closing:self.app.root.after(250,self.tick)
