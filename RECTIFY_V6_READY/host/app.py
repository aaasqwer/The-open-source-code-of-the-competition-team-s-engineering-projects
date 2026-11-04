"""IMU / FPGA camera workbench. Launch with python app.py."""
import os
# These small ROI fits and GUI conversions do not benefit from a large BLAS
# worker pool. Limit this process only; do not change Windows environment settings.
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
os.environ['MKL_NUM_THREADS']='1'
import json
from pathlib import Path
import queue
import time
import traceback
import threading
import tkinter as tk
from tkinter import ttk,filedialog,messagebox
from history import AngleHistory
from trace_view import TraceView
from pose_view import PoseView
from protocol import rgb565_to_image,controls_match
from recording import TelemetryRecorder
from capture_rules import capture_context,validate_reference,validate_pair,merge_frozen_state,matching_frozen_video,frozen_video_state,comparison_limits
from transport import EthernetImageClient,SerialClient,CaptureDevice
from analysis import image_metrics,edge_straightness,compare_distortion,distortion_preview
from status_help import ROI_STATUS
from lens import load_profile
from camera_parameters import parameter_text,candidate_profile
from resource_info import local_report,describe as describe_resources
from release_version import HOST_REVISION
from serial_ports import discover_ports,choose_device,device_name,scan_hint
from geometry_ui import GeometryPanel
from live_quality_ui import LiveQualityPanel
from link_diagnostics import describe_link,capture_failure

META=[('roll','横滚 Roll','°'),('pitch','俯仰 Pitch','°'),('yaw','航向 Yaw','°'),
      *[(f'a{c}',f'加速度 {c.upper()}','g') for c in 'xyz'],
      *[(f'g{c}',f'角速度 {c.upper()}','°/s') for c in 'xyz'],
      *[(f'm{c}',f'磁场 {c.upper()}','raw') for c in 'xyz'],
      ('pressure','气压','Pa'),('altitude','气压计海拔','m'),('baro_temp','气压计温度','°C'),('mag_temp','磁力计温度','°C'),
      *[(f'q{i}',f'四元数 q{i}','') for i in range(4)]]
VIEWS=['相机原图（无字幕）','当前板端画面（无字幕）','参考工程左右对比'];MODELS=['自动Gamma','Gamma旁路']
def view_label(mode,clean=False,reference=False):
    if reference:return ['参考工程原图','参考工程矫正','参考工程左右对比'][mode]
    return VIEWS[mode] if clean else ['旧版原图模式','旧版HDMI回正（含显示叠加）','旧版左右对比'][mode]
EDGE_AXES={'自动识别':'auto','斜边':'oblique','竖向扫描':'vertical','横向扫描':'horizontal'}
RUN_DATA=[('imu_state','IMU 遥测',''),('data_age','IMU 数据年龄','s'),('imu_frames','有效上传帧','帧'),
          ('imu_errors','帧/校验错误','次'),('accel_range','加速度量程','g'),('gyro_range','陀螺仪量程','°/s'),
          ('calibration_status','校准状态',''),('valid_groups','有效组掩码',''),('commands','指令计数','次'),
          ('command_errors','指令错误','次'),('video_state','视频遥测',''),('mode','照片来源',''),
          ('uptime','板端运行时间','s'),('input_fps','输入帧率','fps'),('output_fps','HDMI标称帧率','fps'),
          ('latency','处理延迟','ms'),('rms','直线度 RMS','px'),('maximum','最大偏差','px'),
          ('mtf50','MTF50','cycles/pixel'),('roi_status','ROI 状态',''),('rows','有效行数','行'),
          ('coverage','覆盖率','%'),('repeats','重复帧','帧'),('skips','跳帧','帧'),('page','IMU 参数页',''),
          ('roi_enabled','ROI 开关',''),('queue_drops','上位机事件丢弃','次'),('preview_drops','预览替换帧','帧'),('ui_errors','上位机事件错误','次')]

# Image measurements belong to the image tab; monitoring stays focused on IMU/link/video timing.
ROI_KEYS={'rms','maximum','mtf50','roi_status','rows','coverage','roi_enabled'}
MONITOR_DATA=[item for item in RUN_DATA if item[0] not in ROI_KEYS]

class Workbench:
    def __init__(self,root):
        self.root=root;root.title(f'FPGA · IMU 与图像评估工作台 · RECTIFY_BOARD {HOST_REVISION} · 原图 / FPGA矫正 / IMU / Gamma')
        w=min(1450,root.winfo_screenwidth()-70);h=min(940,root.winfo_screenheight()-90)
        root.geometry(f'{w}x{h}');root.minsize(min(1120,w),min(720,h))
        self.events=queue.Queue(maxsize=250);self.serial=SerialClient(self.events);self.ethernet=EthernetImageClient(self.events,self.serial);self.capture=CaptureDevice(self.events)
        self.latest={};self.history=AngleHistory(seconds=600,max_hz=20);self.last_refresh=0;self.connected=False;self.capture_seq=time.time_ns()&0xffffffff;self.pending=None;self.control_initialized=False
        self.images={};self.reference_meta={};self.current_metadata={};self.capture_context=None
        self.lens_profile=None;self.lens_pending=None
        self.current_image=None;self.current_source='';self.photo=None;self.roi=None;self.edge_profile=None;self.pc_roi_summary='';self.drag=None;self.offset=(0,0);self.scale=1.
        self.recorder=None;self.closing=False;self.ui_errors=0;self.preview_paused=False
        self.continuous=False;self.capture_job=None;self.ignore_image_until_request=False;self.next_capture=0.;self.board_frame_times=[]
        self.geometry_panel=None;self.live_quality=None;self.build_ui();self.geometry_panel=GeometryPanel(self,self.main_tabs);self.live_quality=LiveQualityPanel(self,self.main_tabs);self.root.after(60,self.poll);root.protocol('WM_DELETE_WINDOW',self.close)
        self.log('RECTIFY_BOARD：默认镜头旁路且IMU回正开启，KEY0切换镜头矫正，KEY2切换IMU。标定仍需实板直线验收。')

    def build_ui(self):
        style=ttk.Style();style.theme_use('clam')
        style.configure('.',font=('Microsoft YaHei UI',10));style.configure('Treeview',rowheight=26)
        style.configure('Title.TLabel',font=('Microsoft YaHei UI',17,'bold'),foreground='#123747')
        style.configure('Value.TLabel',font=('Consolas',26,'bold'),foreground='#126b73')
        base=ttk.Frame(self.root,padding=16);base.pack(fill='both',expand=True)
        head=ttk.Frame(base);head.pack(fill='x',pady=(0,10))
        ttk.Label(head,text='IMU 与图像评估工作台',style='Title.TLabel').pack(side='left')
        self.state=tk.StringVar(value='未连接 · 等待数据');ttk.Label(head,textvariable=self.state).pack(side='right')
        link=ttk.LabelFrame(base,text='数据连接',padding=10);link.pack(fill='x',pady=(0,10))
        self.ip=tk.StringVar(value='0.0.0.0');self.port=tk.StringVar(value='8080')
        ttk.Label(link,text='图像监听地址').grid(row=0,column=0);ttk.Entry(link,textvariable=self.ip,width=17).grid(row=0,column=1,padx=6)
        ttk.Label(link,text='UDP').grid(row=0,column=2);ttk.Entry(link,textvariable=self.port,width=6).grid(row=0,column=3,padx=6)
        self.receive_btn=ttk.Button(link,text='开始接收',command=self.connect_ethernet);self.receive_btn.grid(row=0,column=4,padx=4)
        ttk.Button(link,text='断开',command=self.disconnect_ethernet).grid(row=0,column=5,padx=4)
        ttk.Label(link,text='参数 COM').grid(row=0,column=6,padx=(20,4));self.com=tk.StringVar();self.port_choice=tk.StringVar();self._port_scan_key=None
        self.com_box=ttk.Combobox(link,textvariable=self.port_choice,width=18,postcommand=self.refresh_ports);self.com_box.grid(row=0,column=7)
        self.com_box.bind('<<ComboboxSelected>>',lambda event:self.select_com_port())
        ttk.Button(link,text='刷新',command=self.refresh_ports).grid(row=0,column=8,padx=4)
        ttk.Button(link,text='打开',command=self.open_serial).grid(row=0,column=9,padx=4)
        ttk.Button(link,text='关闭',command=self.serial.close).grid(row=0,column=10,padx=4)
        self.record_btn=ttk.Button(link,text='开始记录',command=self.toggle_record);self.record_btn.grid(row=0,column=11,padx=(20,0))
        self.connection_hint=tk.StringVar(value='CP2102 · 921600 8N1 | UDP未监听；监听地址通常填0.0.0.0')
        ttk.Label(link,textvariable=self.connection_hint).grid(row=1,column=0,columnspan=12,sticky='w',pady=(8,0))
        self.reference_capture=tk.BooleanVar(value=False)
        self.reference_box=ttk.Checkbutton(link,text='允许参考工程V2图像采集（V2不支持本版Gamma／镜头参数控制）',variable=self.reference_capture)
        self.reference_box.grid(row=2,column=0,columnspan=12,sticky='w')
        self.port_hint=tk.StringVar();ttk.Label(link,textvariable=self.port_hint,wraplength=1200).grid(row=3,column=0,columnspan=12,sticky='w',pady=(4,0))
        tabs=ttk.Notebook(base);tabs.pack(fill='both',expand=True)
        monitor=ttk.Frame(tabs,padding=12);images=ttk.Frame(tabs,padding=12);params=ttk.Frame(tabs,padding=12);logs=ttk.Frame(tabs,padding=12)
        for frame,name in [(monitor,'实时监控'),(images,'图像与效果'),(params,'处理参数与控制'),(logs,'连接与日志')]:tabs.add(frame,text=name)
        self.main_tabs=tabs;self.image_tab=images
        tabs.bind('<<NotebookTabChanged>>',lambda event:self.render_image())
        split=ttk.Panedwindow(monitor,orient='horizontal');split.pack(fill='both',expand=True)
        left=ttk.Frame(split);right=ttk.Frame(split);split.add(left,weight=2);split.add(right,weight=3)
        cards=ttk.Frame(left);cards.pack(fill='x',pady=(0,8))
        self.angle_vars={}
        for i,key in enumerate(('roll','pitch','yaw')):
            card=ttk.LabelFrame(cards,text=dict((x[0],x[1]) for x in META)[key],padding=10);card.grid(row=0,column=i,sticky='ew',padx=(0,12 if i<2 else 0));cards.columnconfigure(i,weight=1)
            var=tk.StringVar(value='--');self.angle_vars[key]=var;ttk.Label(card,textvariable=var,style='Value.TLabel').pack(anchor='w')
            ttk.Label(card,text='传感器输出 · deg').pack(anchor='w')
        video_panel=ttk.LabelFrame(left,text='板端视频',padding=8);video_panel.pack(fill='x',pady=(0,6))
        self.video_summary=tk.StringVar(value='输入 -- / 输出 -- fps · 延迟 -- ms')
        ttk.Label(video_panel,textvariable=self.video_summary,justify='left',wraplength=540).pack(fill='x')
        self.link_summary=tk.StringVar(value='拍照链路：等待连接')
        ttk.Label(video_panel,textvariable=self.link_summary,justify='left',wraplength=540).pack(fill='x',pady=(6,0))
        # Retain measurement state for image analysis/protocol tests, without duplicating it on monitoring.
        self.fpga_roi_state=tk.StringVar(value='--');self.pc_roi_state=tk.StringVar(value='--')
        left_tabs=ttk.Notebook(left);left_tabs.pack(fill='both',expand=True);self.monitor_tabs=left_tabs
        values_pane=ttk.Frame(left_tabs);diagnostics=ttk.Frame(left_tabs,padding=8)
        self.pose_view=PoseView(left_tabs);left_tabs.add(self.pose_view,text='三维姿态');left_tabs.add(values_pane,text='IMU 测量');left_tabs.add(diagnostics,text='运行数据')
        left_tabs.select(self.pose_view)
        self.imu_tree=ttk.Treeview(values_pane,columns=('value','unit','status'),show='tree headings',height=12)
        self.imu_tree.heading('#0',text='测量项');self.imu_tree.column('#0',width=145)
        for c,title,width in [('value','实测值',95),('unit','单位',55),('status','有效性',130)]:self.imu_tree.heading(c,text=title);self.imu_tree.column(c,width=width)
        scrollbar=ttk.Scrollbar(values_pane,orient='vertical',command=self.imu_tree.yview);self.imu_tree.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side='right',fill='y');self.imu_tree.pack(fill='both',expand=True)
        for key,label,unit in META:self.imu_tree.insert('','end',iid=key,text=label,values=('--',unit,'等待数据'))
        self.imu_tree.yview_moveto(0)
        self.diag=tk.StringVar(value='--')
        self.trace_view=TraceView(right,self.history);self.trace_view.pack(fill='both',expand=True);self.plot=self.trace_view.canvas
        self.trace_view.protected_paths=lambda:self.recorder.paths if self.recorder else ()
        self.video_status=tk.StringVar(value='--')
        camera_pane=ttk.Frame(left_tabs,padding=6);left_tabs.add(camera_pane,text='相机参数')
        camera_box=ttk.LabelFrame(camera_pane,text='相机内参矩阵及畸变系数',padding=6);camera_box.pack(fill='both',expand=True)
        resource_pane=ttk.Frame(left_tabs,padding=8);left_tabs.add(resource_pane,text='板端资源')
        self.resource_local=local_report();self.resource_state=tk.StringVar(value='等待资源回读')
        ttk.Label(resource_pane,textvariable=self.resource_state,justify='left',wraplength=540).pack(fill='x',pady=(0,10))
        self.resource_tree=ttk.Treeview(resource_pane,columns=('used','capacity','percent'),show='tree headings',height=4)
        self.resource_tree.heading('#0',text='资源');self.resource_tree.column('#0',width=70)
        for key,label in [('used','使用'),('capacity','容量'),('percent','占用')]:
            self.resource_tree.heading(key,text=label);self.resource_tree.column(key,width=110)
        self.resource_tree.pack(fill='x')
        for name in ['LUT','FF','APM','DRM']:self.resource_tree.insert('','end',iid=name,text=name,values=('--','--','--'))
        camera_tools=ttk.Frame(camera_box);camera_tools.pack(fill='x')
        self.camera_local=None;self.camera_candidate=candidate_profile();self.camera_source=tk.StringVar(value='自动（板端优先）')
        camera_selector=ttk.Combobox(camera_tools,textvariable=self.camera_source,values=['自动（板端优先）','板端回读','本地候选（未批准）','本地文件（仅查看）'],state='readonly',width=20)
        camera_selector.pack(side='left');camera_selector.bind('<<ComboboxSelected>>',lambda event:self.refresh_camera_parameters())
        ttk.Button(camera_tools,text='查看标定文件',command=lambda:self.guarded(self.view_camera_file)).pack(side='left',padx=6)
        self.camera_matrix=tk.StringVar(value=parameter_text())
        ttk.Label(camera_box,textvariable=self.camera_matrix,font=('Consolas',9),justify='left').pack(fill='x',pady=(4,0))
        run_pane=ttk.Frame(diagnostics);run_pane.pack(fill='both',expand=True)
        self.run_tree=ttk.Treeview(run_pane,columns=('value','unit'),show='tree headings',height=6)
        self.run_tree.heading('#0',text='运行项');self.run_tree.heading('value',text='实测值');self.run_tree.heading('unit',text='单位')
        self.run_tree.column('#0',width=150);self.run_tree.column('value',width=200);self.run_tree.column('unit',width=85)
        run_scroll=ttk.Scrollbar(run_pane,orient='vertical',command=self.run_tree.yview);self.run_tree.configure(yscrollcommand=run_scroll.set)
        run_scroll.pack(side='right',fill='y');self.run_tree.pack(side='left',fill='both',expand=True)
        for key,label,unit in MONITOR_DATA:self.run_tree.insert('','end',iid=key,text=label,values=('--',unit))
        # The image pane labels live capture and frozen snapshots explicitly.
        toolbar=ttk.Frame(images);toolbar.pack(fill='x')
        ttk.Button(toolbar,text='当前模式拍照',command=self.take_snapshot).pack(side='left')
        self.raw_capture_btn=ttk.Button(toolbar,text='原图拍照（1920×1080）',state='disabled',command=lambda:self.request_capture_mode(0));self.raw_capture_btn.pack(side='left',padx=4)
        ttk.Button(toolbar,text='板端画面拍照',command=lambda:self.request_capture_mode(1)).pack(side='left')
        ttk.Button(toolbar,text='打开图像',command=self.load_image).pack(side='left',padx=6)
        ttk.Button(toolbar,text='保存当前图像',command=self.save_image).pack(side='left')
        self.device=tk.StringVar(value='0')
        ttk.Button(toolbar,text='OV5640 连续采集',command=self.start_capture).pack(side='left',padx=6)
        ttk.Button(toolbar,text='停止连续采集',command=self.stop_board_capture).pack(side='left')
        ttk.Button(toolbar,text='速存预览（720P）',command=self.save_preview_fast).pack(side='left',padx=6)
        self.image_state=tk.StringVar(value='未采集')
        ttk.Label(images,textvariable=self.image_state,wraplength=1250).pack(fill='x',pady=8)
        ipane=ttk.Panedwindow(images,orient='horizontal');ipane.pack(fill='both',expand=True)
        self.canvas=tk.Canvas(ipane,bg='#132630',highlightthickness=0);info=ttk.Frame(ipane,padding=(12,0));ipane.add(self.canvas,weight=4);ipane.add(info,weight=1)
        self.canvas.bind('<Configure>',lambda e:self.render_image());self.canvas.bind('<ButtonPress-1>',self.roi_start);self.canvas.bind('<B1-Motion>',self.roi_move);self.canvas.bind('<ButtonRelease-1>',self.roi_end)
        self.edge_axis=tk.StringVar(value='自动识别')
        ttk.Combobox(info,textvariable=self.edge_axis,values=list(EDGE_AXES),state='readonly',width=15).pack(fill='x',pady=5)
        ttk.Button(info,text='计算 ROI 指标及直线度',command=self.analyze_image).pack(fill='x',pady=5)
        ttk.Button(info,text='记为对比前图（镜头旁路）',command=lambda:self.remember('原始')).pack(fill='x',pady=4)
        ttk.Button(info,text='记为对比后图（镜头校正）',command=lambda:self.remember('矫正')).pack(fill='x',pady=4)
        ttk.Button(info,text='比较两图畸变效果',command=self.compare).pack(fill='x',pady=8)
        self.analysis_status=tk.StringVar(value='未计算')
        ttk.Label(info,textvariable=self.analysis_status,wraplength=420,foreground='#126b73').pack(fill='x',pady=5)
        result_pane=ttk.Frame(info);result_pane.pack(fill='both',expand=True)
        self.analysis_text=tk.Text(result_pane,width=31,height=15,wrap='word',font=('Microsoft YaHei UI',10))
        result_scroll=ttk.Scrollbar(result_pane,orient='vertical',command=self.analysis_text.yview)
        self.analysis_text.configure(yscrollcommand=result_scroll.set);result_scroll.pack(side='right',fill='y');self.analysis_text.pack(side='left',fill='both',expand=True)
        self.analysis_text.insert('end','亮度均值：--\n亮度标准差：--\n梯度均方：--\n拉普拉斯方差：--\n直线度 RMS：-- px\n最大偏差：-- px\n归一化 RMS：--%\n覆盖率：--%')
        control=ttk.LabelFrame(params,text='现有 FPGA 功能控制',padding=12);control.pack(fill='x')
        self.view=tk.StringVar(value=VIEWS[0]);self.model=tk.StringVar(value=MODELS[0]);self.numbers=tk.BooleanVar(value=False);self.eval=tk.BooleanVar(value=False)
        ttk.Label(control,text='照片来源').grid(row=0,column=0);ttk.Combobox(control,textvariable=self.view,values=VIEWS[:2],state='readonly',width=21).grid(row=0,column=1,padx=8)
        ttk.Label(control,text='HDMI/回正预处理').grid(row=0,column=2);ttk.Combobox(control,textvariable=self.model,values=MODELS,state='readonly',width=24).grid(row=0,column=3,padx=8)
        ttk.Checkbutton(control,text='屏幕数字',variable=self.numbers).grid(row=0,column=4,padx=8);ttk.Checkbutton(control,text='照片通道ROI参数',state='disabled',variable=self.eval).grid(row=0,column=5,padx=8)
        ttk.Button(control,text='发送并核对回读',command=self.apply_controls).grid(row=0,column=6,padx=8)
        self.ack=tk.StringVar(value='未发送')
        ttk.Label(control,textvariable=self.ack,wraplength=1200).grid(row=1,column=0,columnspan=7,sticky='w',pady=(10,0))
        self.actual_control_state=tk.StringVar(value='板端状态：--')
        ttk.Label(control,textvariable=self.actual_control_state,wraplength=1200).grid(row=2,column=0,columnspan=7,sticky='w',pady=(6,0))
        telemetry=ttk.Frame(params);telemetry.pack(fill='x',pady=12)
        ttk.Button(telemetry,text='请求一包遥测',command=lambda:self.send(0x12)).pack(side='left')
        ttk.Button(telemetry,text='暂停遥测',command=lambda:self.send(0x10,b'\0')).pack(side='left',padx=6)
        ttk.Button(telemetry,text='恢复遥测',command=lambda:self.send(0x10,b'\1')).pack(side='left')
        ttk.Button(telemetry,text='重新读取 IMU 量程',command=lambda:self.send(0x21)).pack(side='left',padx=8)
        ttk.Label(params,text='镜头参数已按本次照片固化；几何变换、四角框选请使用“矫正与平面”页。').pack(fill='x',pady=6)
        self.cal_state=tk.StringVar(value='标定参数：未标定，镜头旁路')
        ttk.Label(params,textvariable=self.cal_state,wraplength=1250).pack(anchor='w',pady=(0,10))
        self.param_tree=ttk.Treeview(params,columns=('value','meaning'),show='tree headings',height=8);self.param_tree.heading('#0',text='显示帧标定参数');self.param_tree.heading('value',text='FPGA实际回读值');self.param_tree.heading('meaning',text='含义')
        self.param_tree.column('#0',width=140);self.param_tree.column('value',width=220);self.param_tree.column('meaning',width=600);self.param_tree.pack(fill='both',expand=True)
        self.log_text=tk.Text(logs,wrap='word',font=('Consolas',10));self.log_text.pack(fill='both',expand=True)
        self.footer=tk.StringVar(value='')
        ttk.Label(base,textvariable=self.footer).pack(fill='x',pady=(9,0))
        self.refresh_ports()

    def log(self,text):
        self.log_text.insert('end',time.strftime('%H:%M:%S')+'  '+text+'\n')
        if int(self.log_text.index('end-1c').split('.')[0])>2000:self.log_text.delete('1.0','201.0')
        self.log_text.see('end')
    def import_lens(self):
        path=filedialog.askopenfilename(title='选择已确认来源的Brown标定',initialdir=str(Path.cwd()/'calibration'),filetypes=[('Calibration JSON','*.json')])
        if not path:return
        self.lens_profile=load_profile(path);self.lens_zoom.set(str(self.lens_profile.get('zoom',1.)))
        self.ack.set('已导入标定，尚未下发：'+Path(path).name)
    def view_camera_file(self):
        path=filedialog.askopenfilename(title='只读查看相机标定，不下发参数',initialdir=str(Path.cwd()/'calibration/CMOS2_1080P'),filetypes=[('Calibration JSON','*.json')])
        if not path:return
        profile=json.loads(Path(path).read_text(encoding='utf-8-sig'))
        text=parameter_text(local=profile)
        self.camera_local=profile;self.camera_source.set('本地文件（仅查看）');self.camera_matrix.set(text)
    def refresh_camera_parameters(self):
        if not hasattr(self,'camera_matrix'):return
        if self.camera_source.get()=='本地文件（仅查看）':
            text=parameter_text(local=self.camera_local) if self.camera_local else '尚未选择本地标定文件；本区仅供查看，不下发参数'
        elif self.camera_source.get()=='本地候选（未批准）':
            text=parameter_text(local=self.camera_candidate) if self.camera_candidate else '本地候选文件不可用'
        else:
            packet=self.latest.get(0x83);fresh=self.fresh_packet(0x83)
            text=parameter_text(packet,stale=packet is not None and fresh is None)
            if self.camera_source.get()=='自动（板端优先）' and not(packet and packet.get('ready')) and self.camera_candidate:
                board_state='板端未加载有效K/D，镜头校正旁路' if fresh else '板端标定尚无有效回读'
                text=board_state+'\n以下为本地候选，仅查看、未下发、未在板端生效\n'+parameter_text(local=self.camera_candidate)
        if self.camera_matrix.get()!=text:self.camera_matrix.set(text)
    def refresh_resources(self):
        if not hasattr(self,'resource_state'):return
        packet=self.latest.get(0x84);state,rows=describe_resources(packet,self.resource_local,stale=packet is not None and self.fresh_packet(0x84) is None)
        signature=(state,tuple(rows))
        if getattr(self,'_resource_signature',None)==signature:return
        self._resource_signature=signature
        self.resource_state.set(state)
        for name in ['LUT','FF','APM','DRM']:self.resource_tree.item(name,values=('--','--','--'))
        for name,used,capacity,percent in rows:self.resource_tree.item(name,values=(used,capacity,percent))
    def lens_ready(self):
        v=self.fresh_packet(0x82)
        if not v or not v.get('lens_capable'):raise RuntimeError('需要支持镜头参数的V4/V5位流及有效视频遥测')
        if self.ethernet.capturing or v.get('frozen'):raise RuntimeError('拍照冻结期间不能更换标定')
        if self.lens_pending:raise RuntimeError('上一标定事务尚未确认')
        self.capture_seq=((self.capture_seq+1)&0xffffffff) or 1
        return self.capture_seq
    def apply_lens(self):
        if self.lens_profile is None:raise ValueError('请先导入当前镜头的真实标定JSON')
        seq=self.lens_ready();self.serial.upload_lens(self.lens_profile,seq,True,float(self.lens_zoom.get()))
        self.lens_pending=(seq,time.monotonic());self.ack.set('等待完整帧切换并回读标定 ID '+str(seq))
    def clear_lens(self):
        seq=self.lens_ready();self.serial.send(0x42,seq.to_bytes(4,'little'))
        self.lens_pending=(seq,time.monotonic());self.ack.set('等待恢复V28镜头旁路')
    def guarded(self,fn):
        try:return fn()
        except (ValueError,OSError,RuntimeError,ImportError) as exc:messagebox.showerror('操作未完成',str(exc));return None
    def update_link_status(self):
        listening=self.ethernet.sock is not None and not self.ethernet.stop.is_set()
        self.receive_btn.configure(text='接收已启动' if listening else '开始接收',state='disabled' if listening else 'normal')
        self.reference_box.configure(state='disabled' if self.ethernet.capturing else 'normal')
        self.connection_hint.set(('参数串口已连接' if self.connected else '参数串口未连接')+' | '+
          (f'UDP已监听 {self.ip.get()}:{self.port.get()} · 缓冲{self.ethernet.receive_buffer_bytes}字节' if listening else 'UDP未监听 · 监听地址通常填0.0.0.0，不能填板子的IP'))
    def start_image_receiver(self):
        if self.ethernet.sock is None or self.ethernet.stop.is_set():
            try:self.ethernet.connect(self.ip.get().strip(),int(self.port.get()))
            except OSError as exc:raise RuntimeError(f'UDP 8080监听失败：{exc}。检查重复上位机和监听地址，通常填0.0.0.0。') from exc
        self.update_link_status()
    def connect_ethernet(self):self.guarded(self.start_image_receiver)
    def disconnect_ethernet(self):
        self.stop_board_capture();self.ethernet.close();self.image_state.set('图像接收已断开')
    def send(self,kind,payload=b''):self.guarded(lambda:self.serial.send(kind,payload))
    def refresh_ports(self):
        scan=discover_ports();self._port_scan=scan
        current=self.com.get() if self.connected else self.port_choice.get()
        chosen=choose_device(scan.ports,current)
        self.com_box['values']=[p.label for p in scan.ports]
        self.com.set(chosen)
        self.port_choice.set(next((p.label for p in scan.ports if p.device==chosen),chosen))
        self.port_hint.set(scan_hint(scan,chosen))
        key=(scan.ports,scan.errors)
        if key!=self._port_scan_key:
            self._port_scan_key=key
            self.log('串口扫描：'+('；'.join(p.label for p in scan.ports) or '未发现已分配的 COM 口'))
            for error in scan.errors:self.log('串口枚举备用路径：'+error)
    def select_com_port(self):
        try:chosen=device_name(self.port_choice.get())
        except ValueError:chosen=''
        if self.connected:chosen=self.com.get()
        self.com.set(chosen)
        self.port_hint.set(scan_hint(self._port_scan,chosen))
    def open_serial(self):
        def start():
            if self.serial.thread and self.serial.thread.is_alive():raise RuntimeError('请先关闭已打开的串口再切换 COM 号')
            chosen=device_name(self.port_choice.get());self.com.set(chosen)
            self.port_hint.set(scan_hint(self._port_scan,chosen))
            self.serial.open(chosen);self.com_box.configure(state='disabled')
        self.guarded(start)
    def take_snapshot(self):
        video=self.fresh_packet(0x82)
        if not video:self.guarded(self.begin_snapshot);return
        # Wait for the already requested control state instead of presenting
        # a modal error while the board is still confirming a mode change.
        mode=1 if video.get('return_pc') else ((self.pending[0]&3) if self.pending else video['mode'])
        self.request_capture_mode(mode)
    def begin_snapshot(self):
        if self.capture.thread and self.capture.thread.is_alive():messagebox.showinfo('图像来源','请先停止 HDMI 实时采集，再抓取 以太网 图像。');return
        self.capture_seq=(self.capture_seq+1)&0xffffffff
        def start():
            self.ensure_image_receiver()
            video=self.fresh_packet(0x82)
            context=capture_context(video,self.pending is not None or self.lens_pending is not None)
            context['firmware_family']='current' if video.get('return_pc') else 'reference_v2'
            requested_at=time.monotonic()
            context.update(sequence=self.capture_seq,com=self.com.get(),requested_at=requested_at,clean_camera_images=video.get('clean_camera_images',False),video_protocol=video['protocol_version'],
                           operation_requested_at=self.capture_job['requested_at'] if self.capture_job else requested_at)
            self.preview_paused=True;self.capture.take_frame()
            # Consume this request before writing the command. Even a partial
            # UART write has uncertain board effects and must not auto-repeat.
            self.capture_context=context;self.capture_job=None
            try:self.ethernet.capture(self.capture_seq)
            except Exception:
                context['invalid']=True;raise
            self.ignore_image_until_request=False;self.image_state.set('OV5640 图像传输中')
        start()
    def ensure_image_receiver(self):
        if not self.connected:raise RuntimeError('请先打开CP2102参数串口，网口只传图像，拍照命令通过串口发送')
        video=self.fresh_packet(0x82)
        if not video:raise RuntimeError('未收到兼容的视频遥测。原图拍照需要V6及以上位流；连续回正预览需要RETURN_PC_3 V7；参考工程V2需勾选兼容采集')
        if not video.get('return_pc') and not(self.reference_capture.get() and video['protocol_version']==2):raise RuntimeError('板端版本不匹配')
        self.start_image_receiver()
        if not self.ethernet.sock:raise RuntimeError('UDP 接收尚未就绪')

    def start_capture(self):
        def start():
            self.ensure_image_receiver()
            if self.ethernet.capturing:raise RuntimeError('请等照片接收完成')
            if not self.fresh_packet(0x82):raise RuntimeError('请先打开参数串口并等候视频状态')
            video=self.fresh_packet(0x82)
            if video.get('return_pc') and video.get('protocol_version',0)<7:
                raise RuntimeError('连续回正预览需要 RETURN_PC_3 V7 配套位流；当前旧位流的连续预览是原图')
            # V7 preview always reads rendered banks, independently of the
            # raw/processed selection used by the full-resolution photo path.
            preview_video={**video,'mode':1} if video.get('return_pc') else video
            capture_context(preview_video,self.pending is not None or self.lens_pending is not None)
            self.preview_context={key:video.get(key) for key in ('protocol_version','return_pc','clean_camera_images')}
            self.capture_seq=(self.capture_seq+1)&0xffffffff
            self.ethernet.start_preview(self.capture_seq)
            self.continuous=True;self.capture_job=None;self.next_capture=0.;self.board_frame_times=[];self.image_state.set('OV5640 1280×720 板端画面预览（当前按键模式，无字幕）' if self.fresh_packet(0x82).get('protocol_version',0)>=7 else 'OV5640 旧版预览；回正连续采集需要配套V7位流')
        self.guarded(start)
    def stop_board_capture(self):
        self.continuous=False;self.capture_job=None;self.ignore_image_until_request=True;self.ethernet.stop_preview();self.ethernet.cancel()
        self.image_state.set('OV5640 连续采集已停止')
    def request_capture_mode(self,mode):
        if mode==1:
            geometry=self.fresh_packet(0x85)
            if not geometry:
                messagebox.showinfo('等待板端画面','请连接参数串口，等待完整渲染帧状态；镜头旁路模式也支持拍摄回正后的板端画面。');return
        def start():
            video=self.fresh_packet(0x82)
            self.ensure_image_receiver()
            reference=video and not video.get('return_pc') and self.reference_capture.get()
            if mode!=1 and not(reference and mode in (0,1,2)) and not(mode==0 and video.get('raw_capture_capable')):raise RuntimeError('原图拍照需要配套V6位流；当前板端未回读到独立原始帧通道')
            self.ethernet.stop_preview()
            self.view.set(VIEWS[mode])
            now=time.monotonic()
            self.continuous=False;self.board_frame_times=[];self.capture_job=dict(mode=mode,deadline=now+10,requested_at=now)
            self.image_state.set('等待参考工程画面模式 '+str(mode) if reference else '等待板卡 '+VIEWS[mode])
            self.next_capture=0.;self.pump_board_capture(now)
        self.guarded(start)
    def pump_board_capture(self,now):
        if self.continuous or not self.capture_job or self.ethernet.capturing or now<self.next_capture:return
        v=self.fresh_packet(0x82)
        if self.capture_job and now>self.capture_job['deadline']:
            self.capture_job=None;self.image_state.set('模式切换/采集未完成');return
        if not v or not self.ethernet.sock or self.pending or v['frozen']:return
        try:
            if self.capture_job and v['mode']!=self.capture_job['mode']:
                value=(v['control']&~3)|self.capture_job['mode']
                self.serial.send(0x20,bytes((value,)));self.pending=(value,now);self.ack.set('待回读');return
            self.begin_snapshot();self.next_capture=now+.15
        except (OSError,RuntimeError,ValueError) as exc:
            self.next_capture=now+.3;self.image_state.set(str(exc))
    def apply_controls(self):
        if self.ethernet.capturing:messagebox.showinfo('正在抓帧','请等 以太网 图像传输完成后再切换处理参数。');return
        value=VIEWS.index(self.view.get())|(MODELS.index(self.model.get())<<2)|(int(self.numbers.get())<<3)|(int(self.eval.get())<<4)
        def send():
            video=self.fresh_packet(0x82)
            if not video or not video.get('return_pc'):raise RuntimeError('需要RETURN_PC兼容位流V3及以上有效回读；旧板端模型位与Gamma含义不同，禁止混用控制')
            if value&3==0 and not video.get('raw_capture_capable'):raise RuntimeError('原图模式需要配套V6位流')
            self.serial.send(0x20,bytes((value,)));self.pending=(value,time.monotonic());self.ack.set('待回读')
        self.guarded(send)
    def poll(self):
        if self.closing:return
        try:
            for _ in range(150):
                try:event=self.events.get_nowait()
                except queue.Empty:break
                self.handle(event)
            completed=self.ethernet.take_completed()
            if completed is not None:self.handle(('image',completed[0],completed[1]))
            failed=self.ethernet.take_failed()
            if failed is not None:self.handle(('capture_failed',*failed))
            # The latest preview is retained outside the bounded event queue.
            # A lost notice must not strand the last completed frame.
            if self.continuous:self.handle(('preview_ready',))
            # Connection events can be dropped if the bounded queue was full.
            # Reconcile with the actual worker so a closed COM cannot remain
            # labelled connected indefinitely.
            if self.serial.thread is not None:
                actual=self.serial.port is not None and self.serial.thread.is_alive()
                if actual!=self.connected:self.handle(('serial','connected' if actual else 'disconnected','工作线程状态同步'))
            if self.ethernet.thread is not None and not self.ethernet.thread.is_alive() and (self.capture_job or self.continuous):
                self.handle(('ethernet','disconnected','工作线程状态同步'))
            now=time.monotonic()
            self.pump_board_capture(now)
            if getattr(self,'pose_dirty',False) and now-self.last_refresh<.2:
                self.pose_view.update_packet(self.fresh_packet(0x81));self.pose_dirty=False
            if self.continuous and now-self.ethernet.preview_last_received>1:
                self.image_state.set(f'OV5640 等待新帧 · 不完整帧 {self.ethernet.preview_incomplete} · 校验/解码错误 {self.ethernet.preview_errors}')
            if now-self.last_refresh>=.2:self.refresh_values();self.last_refresh=now;self.pose_dirty=False
        except Exception as exc:
            self.ui_errors+=1;self.log(traceback.format_exc());self.footer.set(f'事件处理错误：{type(exc).__name__}')
        finally:
            if not self.closing:self.root.after(5 if self.continuous else 20,self.poll)
    def handle(self,event):
        kind=event[0]
        if kind=='quality_exported':
            self.log('独立复核包已保存：'+event[1])
            self.live_quality.notice.set('复核包已保存。在独立测评工具选择此清单：'+event[1])
        elif kind=='quality_export_error':
            self.log('独立复核包导出失败：'+event[1]);self.live_quality.notice.set('导出失败：'+event[1])
        elif kind=='image_saved':
            self.log(f'原尺寸照片已保存：{event[1]} · PNG写入 {event[2]:.1f} ms（不含采集与传输）')
        elif kind=='fast_saved':
            self.fast_save_busy=False;self.log(f'预览快照已保存：{event[1]} · 写入 {event[2]:.1f} ms（已接收帧，无新曝光）')
        elif kind=='fast_save_error':
            self.fast_save_busy=False;self.log('预览快照保存失败：'+event[1])
        elif kind=='telemetry':
            packet=event[1];self.latest[packet['kind']]=packet
            if packet['kind']==0x85 and self.geometry_panel:self.geometry_panel.observe(packet)
            if packet['kind']==0x81:
                vals=packet['values'];self.history.append(packet['received'],*[vals[x] for x in ('roll','pitch','yaw')])
                self.pose_dirty=True
            if self.recorder:
                try:self.recorder.write(packet)
                except (OSError,ValueError) as exc:
                    self.recorder.close();self.recorder=None;self.record_btn.configure(text='开始记录')
                    self.log('记录失败：'+str(exc));self.footer.set('记录失败：'+str(exc))
            if packet['kind']==0x82:
                if not packet.get('return_pc'):
                    self.actual_control_state.set('参考工程兼容图像采集，旧模型位不解释为Gamma' if self.reference_capture.get() else '板端版本不兼容：旧模型位不解释为Gamma')
                    if self.reference_capture.get() and packet['protocol_version']==2 and self.capture_context and self.ethernet.capturing and matching_frozen_video(self.capture_context,packet):
                        try:merge_frozen_state(self.capture_context,dict(mode=packet['mode'],model=packet['model']))
                        except ValueError as exc:self.capture_context['invalid']=True;self.ethernet.cancel(str(exc))
                    return
                if self.capture_context and self.ethernet.capturing:
                    failure=capture_failure(packet,self.capture_context['sequence'])
                    if failure:
                        self.capture_context['invalid']=True;self.ethernet.cancel(failure)
                        self.image_state.set(failure)
                source_label=view_label(packet['mode'],packet.get('clean_camera_images',False))
                self.actual_control_state.set(f"照片来源：{source_label} · HDMI/回正：{MODELS[packet['model']]} · 屏幕字幕 {'开' if packet['numbers'] else '关'} · 屏幕ROI用KEY3控制")
                if packet.get('lens_capable'):
                    state='故障，已旁路' if packet['lens_fault'] else '开启' if packet['lens_enabled'] else '旁路'
                    self.actual_control_state.set(self.actual_control_state.get()+f" · Brown镜头{state} · 标定ID {packet['lens_id']}")
                    if self.lens_pending and packet['lens_id']==self.lens_pending[0]:
                        self.lens_pending=None;self.ack.set('显示帧已切换：镜头'+state)
                if packet.get('key1_text_control'):
                    self.actual_control_state.set(self.actual_control_state.get()+(' · 相机文字实际显示' if packet['camera_text_visible'] else ' · 相机文字已隐藏（KEY1／KEY4／PC开关）'))
                if not self.control_initialized:
                    self.control_initialized=True;self.view.set(VIEWS[packet['mode']]);self.model.set(MODELS[packet['model']]);self.numbers.set(packet['numbers'])
                    if packet['roi_enabled'] is not None:self.eval.set(packet['roi_enabled'])
                if self.pending and controls_match(packet,self.pending[0]):self.pending=None;self.ack.set('已确认')
                if self.capture_context and self.ethernet.capturing and packet['protocol_version']>=2 and matching_frozen_video(self.capture_context,packet):
                    frozen=frozen_video_state(packet)
                    if frozen.get('data_page'):
                        self.capture_context['invalid']=True;self.ethernet.cancel('拍照冻结到了KEY4数据页，请松开KEY4后重拍');return
                    try:merge_frozen_state(self.capture_context,frozen)
                    except ValueError as exc:self.capture_context['invalid']=True;self.ethernet.cancel(str(exc))
            if packet['kind']==0x83:self.update_parameters(packet)
        elif kind=='ethernet':
            self.log('以太网 图像 '+event[1]+' '+event[2]);self.footer.set('以太网 图像 '+event[1]+' '+event[2])
            if event[1]=='disconnected':
                self.capture_job=None;self.continuous=False;self.image_state.set('OV5640 图像连接已断开')
        elif kind=='serial':
            self.com_box.configure(state='disabled' if event[1]=='connected' else 'normal')
            self.connected=event[1]=='connected';self.state.set('参数串口已连接' if self.connected else '参数串口未连接');self.log('参数串口 '+event[1]+' '+event[2])
            if not self.connected:
                self.lens_pending=None
                self.control_initialized=False;self.pending=None;self.latest.clear();self.continuous=False;self.capture_job=None;self.ethernet.stop_preview()
                if self.ethernet.capturing:self.ethernet.cancel('参数串口断开')
        elif kind=='board_reset':
            self.lens_pending=None
            self.control_initialized=False;self.pending=None;self.latest.clear();self.capture_context=None;self.capture_job=None
            self.continuous=False;self.ethernet.stop_preview()
            if self.ethernet.capturing:self.ethernet.cancel('板卡复位')
            self.log('板卡复位 · 状态重新回读')
        elif kind=='serial_recovery':self.log(event[1]);self.footer.set(event[1])
        elif kind=='capture_failed':
            _,sequence,reason=event;self.ethernet.acknowledge_failed(sequence)
            context=self.capture_context
            if not context or context['sequence']!=sequence or context.get('completed') or context.get('failed'):return
            context.update(invalid=True,failed=True)
            self.log('图像：'+reason);self.footer.set('图像：'+reason)
            if not self.capture_job:self.image_state.set(reason)
        elif kind=='error':
            self.log(event[1]+'：'+event[2]);self.footer.set(event[1]+'：'+event[2])
            if event[1]=='图像':self.image_state.set(event[2])
        elif kind=='progress':
            if self.ethernet.capturing and not self.ignore_image_until_request:self.image_state.set(f'以太网 图像传输 {event[1]}/{event[2]} 行（{event[1]/event[2]:.0%}）')
        elif kind=='preview_ready':
            result=self.ethernet.take_preview()
            if result is None or not self.continuous:return
            (w,h,data,frame),metadata=result
            completed=time.monotonic();self.board_frame_times=(self.board_frame_times+[completed])[-30:]
            rate=(len(self.board_frame_times)-1)/(completed-self.board_frame_times[0]) if len(self.board_frame_times)>1 else 0
            context=dict(source='board_ov5640',preview=True,frozen_state=dict(mode=metadata['mode'],model=metadata['model']),
                         requested_mode=metadata['mode'],requested_model=metadata['model'],com=self.com.get(),sequence=metadata['sequence'],frame_id=frame)
            from codec import preview_rgb_image
            v=getattr(self,'preview_context',None) or self.fresh_packet(0x82) or {};label=view_label(metadata['mode'],v.get('clean_camera_images',False),not v.get('return_pc',True))
            processed=v.get('protocol_version',0)>=7
            if v.get('protocol_version',0)>=10:context['display_layout']='split' if metadata['mode']==2 else 'full'
            if processed:
                context['frozen_state']['source_stage']='rendered_gamma_no_osd'
                if v.get('protocol_version',0)>=10:label=['镜头旁路（IMU/Gamma按设置）','完整矫正（无字幕）','左右对比（缩小演示）'][metadata['mode']]
            prefix='板端画面预览 · 渲染帧' if processed else '板载 OV5640 预览'
            self.set_image(metadata.get('rgb_image') or preview_rgb_image(w,h,data),f"{prefix} #{frame} · {label} · {w}×{h} · {rate:.1f} fps · 解压/CRC通过 {metadata['decode_seconds']*1000:.1f} ms · PGD1 {metadata['compression_ratio']:.2f}× · 压缩行 {metadata['compressed_rows']}/{h}",context,preserve_analysis=True)
        elif kind=='image':
            if len(event)>2:self.ethernet.acknowledge_completed(event[2].get('sequence'))
            if self.ignore_image_until_request:return
            metadata=event[2] if len(event)>2 else {};context=self.capture_context
            if not context or context['invalid'] or metadata.get('sequence')!=context['sequence']:
                self.log('丢弃无匹配上下文的图像');return
            if context.get('completed'):return
            w,h,data,frame=event[1]
            from codec import preview_rgb_image
            converted_at=time.perf_counter();image=preview_rgb_image(w,h,data);rgb_seconds=time.perf_counter()-converted_at
            if self.capture_job and context.get('requested_at',0)<self.capture_job['requested_at']:return
            context.update(source='board_ov5640')
            if metadata.get('source')=='board_ov5640':
                try:merge_frozen_state(context,dict(mode=metadata['mode'],model=metadata['model']))
                except ValueError as exc:self.log(str(exc));return
            frozen=context['frozen_state'];mode=frozen['mode'] if frozen else context['requested_mode']
            if self.capture_job and mode!=self.capture_job['mode']:
                self.image_state.set('实际图像模式尚未匹配，重新采集');return
            mode_label=view_label(mode,context.get('clean_camera_images',False),context.get('firmware_family')=='reference_v2')
            mode_text=('帧内模式：' if metadata.get('source')=='board_ov5640' else '冻结期间回读：' if frozen else '请求时模式：')+mode_label
            self.capture_job=None
            completed=time.monotonic();self.board_frame_times=(self.board_frame_times+[completed])[-10:]
            rate=(len(self.board_frame_times)-1)/(self.board_frame_times[-1]-self.board_frame_times[0]) if len(self.board_frame_times)>1 else 0
            ratio=metadata.get('compression_ratio',1.)
            context.update(compression_ratio=ratio,received_at=completed)
            context['timings_ms']={key:1000*metadata[key] for key in ('first_data_wait_seconds','receive_until_done_seconds','decode_verify_seconds') if key in metadata}
            context['timings_ms']['rgb_conversion_ms']=rgb_seconds*1000
            context['timings_ms']['operation_to_image_ms']=(completed-context.get('operation_requested_at',completed-metadata['seconds']))*1000
            if 'first_data_wait_seconds' in metadata:
                self.log(f"照片耗时分项：首包等待 {metadata['first_data_wait_seconds']*1000:.1f}ms · 接收 {metadata['receive_until_done_seconds']*1000:.1f}ms · 解码校验 {metadata['decode_verify_seconds']*1000:.1f}ms · RGB {rgb_seconds*1000:.1f}ms")
            for key in ('ack_transport','udp_ack_attempts','uart_ack_count'):
                if key in metadata:context[key]=metadata[key]
            if metadata.get('frame_geometry_association')=='leased_ddr_frame':
                context.update(frame_geometry_flags=metadata['frame_geometry_flags'],frame_control_sequence=metadata['frame_control_sequence'],frame_geometry_association='leased_ddr_frame',frame_id=frame)
                if mode!=0:
                    flags=metadata['frame_geometry_flags']
                    actual={0:'镜头旁路',1:'增强矫正',2:'题目二阶矫正',3:'左右对照'}[flags&3]
                    if flags&32:actual+='／平面正视'
                    if flags&64:actual+='／IMU回正'
                    actual+=('／Gamma旁路' if flags&(1<<13) else f'／Gamma档{(flags>>8)&31}')
                    context['frame_gamma_step']=(flags>>8)&31;context['frame_gamma_bypass']=bool(flags&(1<<13))
                    mode_text+=' · 本帧：'+actual
            context['completed']=True
            codec_text=f"PGD1 {ratio:.2f}×" if metadata.get('codec')=='PGD1' else 'RGB565未压缩'
            ack_text={'udp_pgak':'网口确认','uart_fallback':'本次已回退串口确认','uart_legacy':'串口确认','udp_uart_safety':'网口确认＋每窗口串口保障','none':'兼容传输（无确认）'}.get(metadata.get('ack_transport'))
            if ack_text:codec_text+=' · '+ack_text
            total=completed-context.get('operation_requested_at',completed-metadata['seconds'])
            rate_text=f'{rate:.2f} fps' if self.continuous and len(self.board_frame_times)>1 else '统计中' if self.continuous else '单帧'
            self.set_image(image,f"板载 OV5640 #{frame} · {mode_text} · RGB565 校验通过 · 采集解码 {metadata['seconds']*1000:.1f}ms / 总等待 {total*1000:.1f}ms · {codec_text} · {rate_text}",dict(context))
            self.analysis_status.set('已收到'+mode_label+' · 待选 ROI')
        elif kind=='capture_ready':
            image=self.capture.take_frame()
            if image is not None and not self.preview_paused:
                self.set_image(image,f'视频采集设备 #{self.capture.index} · 实时输入')
        elif kind=='capture_stopped':
            if self.current_source.startswith('视频采集设备 #'):
                self.current_source=f'视频采集设备 #{self.capture.index} · 已停止';self.image_state.set(self.current_source)
        elif kind=='frame_stats':
            _,camera,hdmi,cp,hp=event[1];self.footer.set(f'相机 {camera} fps · HDMI {hdmi} fps')

    def fresh_packet(self,kind):
        p=self.latest.get(kind)
        if kind==0x82 and p and not p.get('return_pc') and not(self.reference_capture.get() and p.get('protocol_version')==2):return None
        return p if p and time.monotonic()-p['received']<2 and self.connected else None
    def refresh_values(self):
        p=self.fresh_packet(0x81);v=self.fresh_packet(0x82)
        self.update_link_status();self.link_summary.set(describe_link(v,self.ethernet))
        self.pose_view.update_packet(p)
        for key,var in self.angle_vars.items():
            value=p['values'][key] if p else None;var.set('--' if value is None else f'{value:+.2f}°')
        for key,_,unit in META:
            value=p['values'][key] if p else None
            status='有效' if value is not None else '等待数据 / 已超时'
            if value is None and p and (key.startswith('q') or key in ('mx','my','mz','mag_temp')):
                status='数据组尚未收到／已超时' if v and v.get('extended_imu') else '当前位流未上传该组，需配套V8位流'
            if p and key[:1] in ('a','g') and key not in ('altitude',) and key[1:] in 'xyz' and value is None and p['flags']&4:status='量程未回读 / 已超时'
            if value is None and p and p['flags']&4 and key in ('ax','ay','az','gx','gy','gz'):
                self.imu_tree.item(key,values=(str(p['raw'][key]),'counts','原始值 · 量程待回读'));continue
            self.imu_tree.item(key,values=('--' if value is None else f'{value:.4f}' if key.startswith('q') else f'{value:.3f}',unit,status))
        data={key:'--' for key,_,_ in RUN_DATA}
        self.diag.set('正常' if p else '超时' if self.connected else '未连接')
        data['imu_state']=self.diag.get()
        if p:
            data.update(data_age=f"{time.monotonic()-p['received']:.2f}",imu_frames=p['imu_frames'],imu_errors=p['imu_errors'],
                        accel_range=f"±{p['accel_range']}" if p['accel_range'] else '--',
                        gyro_range=f"±{p['gyro_range']}" if p['gyro_range'] else '--',
                        calibration_status=f"0x{p['calibration_status']:02X}" if p['config_flags']&1 else '--',
                        valid_groups=f"0x{p['flags']&31:02X}",commands=p['commands'],command_errors=p['command_errors'])
        def fmt(x,suffix=''):return '--' if x is None else f'{x:.3f}{suffix}'
        legacy=self.latest.get(0x82)
        incompatible=legacy and not legacy.get('return_pc') and not self.reference_capture.get() and time.monotonic()-legacy['received']<2 and self.connected
        self.video_status.set('版本不兼容' if incompatible else '正常' if v else '超时' if self.connected else '未连接')
        data['video_state']=self.video_status.get()
        if v:
            mode_label=view_label(v['mode'],v.get('clean_camera_images',False),not v.get('return_pc'))
            data.update(mode=mode_label,uptime=f"{v['uptime_ms']/1000:.1f}",input_fps=v['input_fps'],output_fps=v['output_fps'],
                        latency=fmt(None if v['latency_us'] is None else v['latency_us']/1000),rms=fmt(v['rms']),maximum=fmt(v['maximum']),
                        mtf50=fmt(v['mtf50']),roi_status=f"{v['evaluation']} · {ROI_STATUS.get(v['evaluation'],'--')}",
                        rows=v['rows'],coverage=v['coverage'],repeats='--' if v['repeats'] is None else v['repeats'],skips='--' if v['skips'] is None else v['skips'],page='显示中' if v['page'] else '关闭')
            data['roi_enabled']='未知' if v['roi_enabled'] is None else '开启' if v['roi_enabled'] else '关闭'
            self.fpga_roi_state.set(
                f"{VIEWS[min(v['mode'],2)]} · 输入 {v['input_fps']} / 输出 {v['output_fps']} fps · 延迟 {fmt(None if v['latency_us'] is None else v['latency_us']/1000,' ms')}\n"
                f"直线度 RMS {fmt(v['rms'],' px')} / 最大 {fmt(v['maximum'],' px')}\n"
                f"MTF50 {fmt(v['mtf50'],' cycles/pixel')}\n"
                f"ROI {v['evaluation']} · 有效行 {v['rows']} · 覆盖 {v['coverage']}%")
        if not v:self.fpga_roi_state.set(f"视频遥测：{self.video_status.get()}\n直线度 RMS -- px / 最大 -- px\nMTF50 -- cycles/pixel · 有效行 -- · 覆盖 --%")
        if not v:self.actual_control_state.set('板端状态：'+self.video_status.get())
        output_label=(f"HDMI标称 {v['output_fps']} Hz" if v.get('return_pc') else f"参考输出 {v['output_fps']} fps") if v else ''
        self.video_summary.set(f"{mode_label} · 输入 {v['input_fps']} fps / {output_label} · 延迟 {fmt(None if v['latency_us'] is None else v['latency_us']/1000,' ms')}" if v else '输入 -- fps / HDMI标称 -- Hz · 延迟 -- ms')
        data.update(queue_drops=self.serial.dropped_events+self.ethernet.dropped_events,preview_drops=self.capture.dropped_frames+self.ethernet.preview_drops,ui_errors=self.ui_errors)
        for key,_,unit in MONITOR_DATA:self.run_tree.item(key,values=(data[key],unit))
        cal=self.fresh_packet(0x83)
        self.refresh_camera_parameters()
        self.refresh_resources()
        if hasattr(self,'raw_capture_btn'):
            capable=bool(v and v.get('raw_capture_capable'))
            self.raw_capture_btn.configure(state='normal' if capable else 'disabled',text='原图拍照' if capable else '原图拍照（1920×1080）')
        if not cal:
            self.cal_state.set('标定参数：历史值 · 遥测超时' if self.param_tree.get_children() else '标定参数：-- · 只读')
        if self.pending and time.monotonic()-self.pending[1]>5:
            self.pending=None;self.ack.set('回读未确认 · 5s')
        if self.lens_pending and time.monotonic()-self.lens_pending[1]>5:
            self.lens_pending=None;self.ack.set('标定回读未确认；查看参数错误计数和板端状态')

    def update_parameters(self,p):
        self.param_tree.delete(*self.param_tree.get_children())
        self.cal_state.set(('未标定，镜头旁路' if not p['ready'] else '参数CRC／范围已校验 · '+('故障旁路' if p.get('fault') else '已启用' if p.get('enabled') else '旁路'))+f" · 显示帧标定ID {p.get('calibration_id','旧协议')} · V28安装参数保留")
        if p.get('protocol_version')!=2:self.cal_state.set('旧协议只读参数；不代表当前图像链路已标定')
        meaning={'fx':'输入相机 X 焦距 / px','fy':'输入相机 Y 焦距 / px','cx':'输入相机 X 主点 / px','cy':'输入相机 Y 主点 / px',
                 'k1':'径向畸变系数','k2':'径向畸变系数','k3':'鱼眼畸变系数','k4':'鱼眼畸变系数','p1':'切向畸变系数','p2':'切向畸变系数',
                 'output_f':'输出投影焦距 / px','output_cx':'输出 X 主点 / px','output_cy':'输出 Y 主点 / px',
                 'inverse_fx':'1/fx，像素倒数','inverse_fy':'1/fy，像素倒数','inverse_zoom':'1/视场缩放（裁边会损失视场）'}
        for name,value in p['parameters'].items():self.param_tree.insert('','end',text=name,values=(f'{value:.8f}',meaning[name]))
    def draw_traces(self):
        self.trace_view.invalidate();self.trace_view.draw()
    def set_image(self,image,source,metadata=None,preserve_analysis=False):
        same_size=self.current_image is not None and self.current_image.size==image.size
        self.current_metadata=metadata or {}
        self.current_image=image;self.current_source=source;
        if self.geometry_panel:self.geometry_panel.new_image()
        keep_roi=bool(self.live_quality and self.live_quality.enabled.get() and (preserve_analysis or same_size))
        if not keep_roi:self.roi=None;self.drag=None
        self.edge_profile=None;self.image_state.set(source)
        if self.live_quality:self.live_quality.offer(image,self.current_metadata)
        if self.pc_roi_summary:self.pc_roi_state.set('历史计算\n'+self.pc_roi_summary)
        if not preserve_analysis:
            self.analysis_status.set('新图像 · 未计算')
            self.analysis_text.delete('1.0','end');self.analysis_text.insert('end',source+'\n\n亮度均值：--\n亮度标准差：--\n梯度均方：--\n拉普拉斯方差：--\n直线度 RMS：-- px\n最大偏差：-- px\n边缘角度：--°')
        self.render_image()
    def render_image(self):
        # Keep the latest full-resolution image for saving/analysis, but avoid
        # resizing and allocating Tk images while another main tab is selected.
        if self.main_tabs.select()!=str(self.image_tab):return
        c=self.canvas;c.delete('all')
        if self.current_image is None:c.create_text(max(200,c.winfo_width()/2),max(150,c.winfo_height()/2),text='暂无图像',fill='#b7cbd1',font=('Microsoft YaHei UI',18));return
        from PIL import ImageTk,Image
        image=self.current_image;scale=min(max(1,c.winfo_width())/image.width,max(1,c.winfo_height())/image.height)
        size=(max(1,int(image.width*scale)),max(1,int(image.height*scale)))
        self.offset=((c.winfo_width()-size[0])/2,(c.winfo_height()-size[1])/2);self.scale=scale
        self.photo=ImageTk.PhotoImage(image.resize(size,Image.Resampling.BILINEAR));c.create_image(*self.offset,anchor='nw',image=self.photo)
        if self.roi:
            x0,y0,x1,y1=self.roi;ox,oy=self.offset;c.create_rectangle(ox+x0*scale,oy+y0*scale,ox+x1*scale,oy+y1*scale,outline='#20dbc5',width=2)
        if self.geometry_panel:self.geometry_panel.draw()
        if self.live_quality:self.live_quality.overlay()
        if self.edge_profile:
            ox,oy=self.offset
            points=[coordinate for x,y in self.edge_profile['points'] for coordinate in (ox+x*scale,oy+y*scale)]
            fitted=[coordinate for x,y in self.edge_profile['fit_endpoints'] for coordinate in (ox+x*scale,oy+y*scale)]
            if len(points)>=4:c.create_line(points,fill='#06d9dd',width=2)
            c.create_line(fitted,fill='#ff9c32',width=2)
    def image_point(self,event):
        if self.current_image is None:return None
        ox,oy=self.offset;im=self.current_image
        return max(0,min(im.width,int((event.x-ox)/self.scale))),max(0,min(im.height,int((event.y-oy)/self.scale)))
    def roi_start(self,event):
        if self.geometry_panel and self.geometry_panel.pick(event):return
        self.drag=self.image_point(event)
        if self.drag is not None and not (self.live_quality and self.live_quality.enabled.get()) and (self.continuous or self.current_source.startswith('板载 OV5640')):
            self.continuous=False;self.capture_job=None;self.ignore_image_until_request=True;self.ethernet.stop_preview();self.ethernet.cancel()
        if self.drag is not None and self.current_source.startswith('视频采集设备 #'):
            self.preview_paused=True;self.capture.stop.set();self.capture.take_frame()
            self.current_source=f'视频采集设备 #{self.capture.index} · ROI 冻结帧';self.image_state.set(self.current_source)
    def roi_move(self,event):
        p=self.image_point(event)
        if self.drag and p:
            self.roi=(min(self.drag[0],p[0]),min(self.drag[1],p[1]),max(self.drag[0],p[0]),max(self.drag[1],p[1]));self.edge_profile=None;self.analysis_status.set('选区已变更')
            if self.pc_roi_summary:self.pc_roi_state.set('历史计算\n'+self.pc_roi_summary)
            self.render_image()
    def roi_end(self,event):
        self.roi_move(event);self.drag=None
        if self.live_quality:
            self.live_quality.reset()
            if self.current_image is not None:self.live_quality.offer(self.current_image,self.current_metadata)
    def load_image(self):
        name=filedialog.askopenfilename(filetypes=[('图像','*.png *.jpg *.jpeg *.bmp *.tif'),('全部','*.*')])
        if name:
            from PIL import Image
            def load():
                self.stop_board_capture()
                self.preview_paused=True;self.capture.stop.set();self.capture.take_frame()
                with Image.open(name) as image:
                    from image_provenance import restored_layout
                    self.set_image(image.convert('RGB'),f'本地图像 · {Path(name).name}',restored_layout(image))
            self.guarded(load);self.roi=None
    def save_preview_fast(self):
        if self.current_image is None or not self.current_metadata.get('preview'):
            messagebox.showinfo('速存预览','请先启动 OV5640 连续采集；完整1080P照片仍使用原有拍照按钮。');return
        if getattr(self,'fast_save_busy',False):return
        import sys,datetime
        image=self.current_image;metadata=dict(self.current_metadata);source=self.current_source;started=time.perf_counter()
        folder=(Path(sys.executable).resolve().parent if getattr(sys,'frozen',False) else Path(__file__).resolve().parents[1])/'captures'
        name=folder/('PREVIEW_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.png')
        self.fast_save_busy=True;self.log('正在保存已接收的720P预览帧；连续采集保持运行')
        def write():
            from PIL.PngImagePlugin import PngInfo
            try:
                folder.mkdir(exist_ok=True);info=PngInfo()
                info.add_text('RECTIFY_CAPTURE',json.dumps(dict(source=source,resolution=list(image.size),capture=metadata,
                    snapshot_kind='cached_received_preview',fresh_exposure_requested=False),ensure_ascii=False))
                image.save(name,pnginfo=info,compress_level=1)
                self.events.put(('fast_saved',str(name),(time.perf_counter()-started)*1000))
            except Exception as exc:self.events.put(('fast_save_error',str(exc)))
        threading.Thread(target=write,daemon=True).start()

    def save_image(self):
        if self.current_image is None:return
        # Pin the received original pixels before opening the dialog. Neither
        # a new preview nor PNG compression may block the capture worker.
        image=self.current_image;source=self.current_source;capture=dict(self.current_metadata)
        geometry=self.latest.get(0x85)
        name=filedialog.asksaveasfilename(defaultextension='.png',filetypes=[('PNG','*.png')])
        if not name:return
        def write():
            started=time.perf_counter()
            try:
                from PIL.PngImagePlugin import PngInfo
                meta=PngInfo();meta.add_text('RECTIFY_CAPTURE',json.dumps(dict(source=source,resolution=list(image.size),capture=capture,
                    geometry_readback_at_save=geometry,geometry_association='Only capture.frame_geometry with leased_ddr_frame is bound to this frame; no later preview substituted.'),ensure_ascii=False))
                image.save(name,pnginfo=meta,compress_level=1)
                self.events.put(('image_saved',str(name),(time.perf_counter()-started)*1000))
            except Exception as exc:self.events.put(('fast_save_error',str(exc)))
        threading.Thread(target=write,daemon=True).start()
    def analyze_image(self):
        if self.current_image is None:return
        if self.current_metadata.get('display_layout')=='split' or (self.current_metadata.get('frame_geometry_flags',0)&3)==3:
            messagebox.showinfo('请使用完整矫正图','左右分屏已经缩小，不能与完整画面的像素RMS直接比较。请按K0回到完整矫正画面，拍1080P照片，在同一选框位置测量。');return
        def run():
            self.analysis_status.set('正在计算所选 ROI…');self.root.update_idletasks()
            base=image_metrics(self.current_image,self.roi)
            text=self.current_source+'\n\n图像指标\n'+'\n'.join(f'{key}: {value:.5f}' if isinstance(value,float) else f'{key}: {value}' for key,value in base.items())
            try:
                p=edge_straightness(self.current_image,self.roi,EDGE_AXES[self.edge_axis.get()])
                self.edge_profile=p;self.analysis_status.set(f"{p['geometry']} · 相对水平 {p['angle_degrees']:+.2f}° · RMS {p['rms_px']:.3f}px · 覆盖 {p['coverage_percent']:.1f}%")
                self.pc_roi_summary=f"{p['geometry']} · 相对水平 {p['angle_degrees']:+.2f}°\nRMS {p['rms_px']:.3f} px / 最大 {p['max_px']:.3f} px\n归一化 RMS {p['normalized_rms_percent']:.4f}% · 覆盖 {p['coverage_percent']:.1f}%"
                self.pc_roi_state.set('当前图像的手动 ROI 结果\n'+self.pc_roi_summary)
                text+=f"\n\n局部畸变测量\n直线度 RMS：{p['rms_px']:.3f} px\n最大偏差：{p['max_px']:.3f} px\n归一化 RMS：{p['normalized_rms_percent']:.4f}%\n有效采样：{p['samples']}\n覆盖率：{p['coverage_percent']:.1f}%"
            except ValueError as exc:
                self.edge_profile=None;self.analysis_status.set('计算完成 · 边缘无效')
                self.pc_roi_summary='';self.pc_roi_state.set('RMS -- px / 最大 -- px\n边缘无效')
                text+='\n\n局部畸变测量：无效\n'+str(exc)
            self.analysis_text.delete('1.0','end')
            self.analysis_text.insert('end',text)
            self.analysis_text.see('1.0');self.render_image()
        self.guarded(run)
    def remember(self,label):
        if self.current_image is None:return
        def run():
            if self.current_source.startswith(('畸变对比：','参考图对比：')):raise ValueError('请重新抓取或打开独立图像，不能把对比拼图再记为参考图。')
            validate_reference(label,self.current_metadata)
            self.images[label]=(self.current_image.copy(),self.current_source,self.roi)
            self.reference_meta[label]=dict(self.current_metadata)
            self.log(f"已记为{label}参考图：{self.current_source}，ROI={self.roi}")
            self.analysis_status.set(f'{label}参考图已保存')
        self.guarded(run)
    def compare(self):
        if len(self.images)<2:messagebox.showinfo('参考图','请先分别记录原始和矫正参考图。');return
        self.stop_board_capture()
        def run():
            a,asource,aroi=self.images['原始'];b,bsource,broi=self.images['矫正']
            if a.size!=b.size:raise ValueError('两张参考图分辨率不同，请使用同分辨率图像比较')
            validate_pair(self.reference_meta.get('原始'),self.reference_meta.get('矫正'))
            limits='对比边界：\n'+'\n'.join(comparison_limits(self.reference_meta.get('原始'),self.reference_meta.get('矫正')))
            base=[]
            for label,image,source,roi in [('原始',a,asource,aroi),('矫正',b,bsource,broi)]:
                values=image_metrics(image,roi)
                base.append(label+' · '+source+'\n'+'\n'.join(f'{key}: {value:.5f}' if isinstance(value,float) else f'{key}: {value}' for key,value in values.items()))
            base_text='图像指标\n\n'+'\n\n'.join(base)+'\n\n'+limits
            try:result=compare_distortion(a,b,aroi,broi,EDGE_AXES[self.edge_axis.get()])
            except ValueError as exc:
                from PIL import Image
                height=min(a.height,b.height)
                left=a.resize((round(a.width*height/a.height),height));right=b.resize((round(b.width*height/b.height),height))
                joined=Image.new('RGB',(left.width+right.width,height));joined.paste(left,(0,0));joined.paste(right,(left.width,0))
                self.set_image(joined,'参考图对比：左原始 · 右矫正',preserve_analysis=True)
                self.analysis_text.delete('1.0','end');self.analysis_text.insert('end',base_text+'\n\n局部畸变测量：无效\n'+str(exc))
                self.analysis_status.set('基础指标对比完成 · 边缘无效')
                return
            p=result['before'];q=result['after']
            gain=result['improvement_percent']
            verdict='归一化 RMS 改善：--（原图 RMS < 0.25px）' if gain is None else f"归一化 RMS 改善：{gain:+.1f}%\n局部直线度：{'改善' if gain>5 else '降低' if gain < -5 else '接近'}"
            text=f"局部畸变对比\n\n直线度 RMS：\n原图 {p['rms_px']:.3f} px → 矫正 {q['rms_px']:.3f} px\n最大偏差：\n原图 {p['max_px']:.3f} px → 矫正 {q['max_px']:.3f} px\n相对边长 RMS：\n{p['normalized_rms_percent']:.4f}% → {q['normalized_rms_percent']:.4f}%\n覆盖率：{p['coverage_percent']:.1f}% → {q['coverage_percent']:.1f}%\n\n{verdict}\n\n原图来源：{asource}\n矫正来源：{bsource}"
            self.analysis_text.delete('1.0','end');self.analysis_text.insert('end',text+'\n\n'+base_text)
            self.set_image(distortion_preview(a,b,result),'畸变对比：左原始，右矫正；青色检测边缘，橙色拟合直线',preserve_analysis=True)
            self.analysis_status.set('对比完成')
            self.pc_roi_summary=f"原图 RMS {p['rms_px']:.3f} px → 矫正 {q['rms_px']:.3f} px\n最大 {p['max_px']:.3f} px → {q['max_px']:.3f} px"
            self.pc_roi_state.set('最近两图局部畸变比较\n'+self.pc_roi_summary)
        self.guarded(run)
    def toggle_record(self):
        if self.recorder:
            self.recorder.close();self.recorder=None;self.record_btn.configure(text='开始记录');self.log('记录已停止');return
        filename=filedialog.asksaveasfilename(defaultextension='.jsonl',filetypes=[('原始遥测日志','*.jsonl')])
        if not filename:return
        def start():
            fields=['monotonic_s','sequence','imu_frames','imu_errors']+[x[0] for x in META]+['valid_flags','config_flags']+[x+'_raw' for x in ('ax','ay','az','gx','gy','gz')]
            self.recorder=TelemetryRecorder(filename,fields);self.record_btn.configure(text='停止记录');self.log('记录开始：'+filename)
        self.guarded(start)
    def close(self):
        self.closing=True;self.continuous=False;self.capture_job=None
        self.ethernet.close();self.serial.close();self.capture.stop.set()
        for worker in (self.ethernet,self.serial,self.capture):
            thread=worker.thread
            if thread and thread is not threading.current_thread() and thread.is_alive():thread.join(.3)
        self.trace_view.protected_paths=lambda:()
        self.trace_view.destroy()
        if self.recorder:self.recorder.close();self.recorder=None
        self.root.destroy()

def main():
    root=tk.Tk();Workbench(root);root.mainloop()

if __name__=='__main__':main()
