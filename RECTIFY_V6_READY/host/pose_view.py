"""Display-only attitude reference. Sensor readings and CSV remain absolute."""
import math
import tkinter as tk
from tkinter import ttk

def rotation(roll,pitch,yaw):
    r,p,y=map(math.radians,(roll,pitch,yaw));cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);cy,sy=math.cos(y),math.sin(y)
    return ((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),(sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),(-sp,cp*sr,cp*cr))
def relative(reference,current):
    return tuple(tuple(sum(reference[k][i]*current[k][j] for k in range(3)) for j in range(3)) for i in range(3))

class PoseView(ttk.Frame):
    def __init__(self,parent):
        super().__init__(parent);self.reference=None;self.current=None;self.matrix=rotation(0,0,0);self.valid=False
        self.status=tk.StringVar(value='等待有效角度；首次姿态将作为显示零位')
        ttk.Button(self,text='设为平放零位',command=self.zero).pack(anchor='ne')
        self.canvas=tk.Canvas(self,bg='#f2f7f7',height=200,highlightthickness=0);self.canvas.pack(fill='both',expand=True)
        self.canvas.bind('<Configure>',lambda e:self.draw());ttk.Label(self,textvariable=self.status,wraplength=500).pack(anchor='w')
    def zero(self):
        if self.valid and self.current is not None:self.reference=self.current;self.matrix=rotation(0,0,0);self.draw()
    def update_packet(self,packet):
        values=packet.get('values',{}) if packet else {};angles=[values.get(k) for k in ('roll','pitch','yaw')]
        self.valid=all(a is not None and math.isfinite(a) for a in angles)
        if self.valid:
            self.current=rotation(*angles)
            if self.reference is None:self.reference=self.current
            self.matrix=relative(self.reference,self.current);self.status.set('三维姿态有效 · 相对显示零位；原始角度和CSV不变')
        else:self.status.set('姿态已失效 · 保留最后画面')
        if self.winfo_ismapped():self.draw()
    def draw(self):
        c=self.canvas;c.delete('all');w=max(200,c.winfo_width());h=max(150,c.winfo_height());scale=min(w*.27,h*.30)
        def transformed(v):return tuple(sum(self.matrix[i][j]*v[j] for j in range(3)) for i in range(3))
        def project(v):return w/2+scale*(v[0]-.55*v[1]),h*.48+scale*(.30*v[0]+.44*v[1]-v[2])
        vertices=[(-1,-.6,-.1),(1,-.6,-.1),(1,.6,-.1),(-1,.6,-.1),(-1,-.6,.1),(1,-.6,.1),(1,.6,.1),(-1,.6,.1)]
        points=[transformed(v) for v in vertices];faces=[(0,1,2,3),(4,7,6,5),(0,4,5,1),(1,5,6,2),(2,6,7,3),(3,7,4,0)]
        colors=['#355766','#3ca5a7','#67c2ad','#26959b','#377c89','#429bab']
        for index in sorted(range(6),key=lambda k:sum(points[i][1] for i in faces[k]),reverse=True):
            c.create_polygon(*[n for i in faces[index] for n in project(points[i])],fill=colors[index] if self.valid else '#78888c',outline='#163d4c')
        origin=project((0,0,0))
        for v,label,color in [((1.4,0,0),'X','#d55845'),((0,1.2,0),'Y','#25926b'),((0,0,1.2),'Z','#416dc1')]:
            end=project(transformed(v));c.create_line(*origin,*end,fill=color,width=2,arrow='last');c.create_text(end[0]+8,end[1],text=label,fill=color)
