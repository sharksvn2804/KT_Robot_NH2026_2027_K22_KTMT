# -*- coding: utf-8 -*-
"""Pure Pursuit version 2: giao dien hanh lang, Python 3.9+.

Chay: python pure_pursuit_version2.py
Cai thu vien: python -m pip install numpy matplotlib
Kiem tra khong can GUI: python pure_pursuit_version2.py --self-test

Duoc viet lai tu cau truc code nguoi dung cung cap, phat trien tu vi du
Pure Pursuit cua PythonRobotics (Atsushi Sakai, Guillaume Jacquenot).
Nguon: https://github.com/AtsushiSakai/PythonRobotics
Pure Pursuit co ban + quy luat giam toc theo do cong lenh lai, khong phai
ban trien khai day du Regulated Pure Pursuit / Nav2. Chi chay tien.
Pose la tam truc sau. Khoang cach 2 truc banh trong hinh bang WB.
Va cham duoc kiem tra voi hinh chu nhat than xe tai moi buoc dt;
khong phai kiem tra the tich quet lien tuc giua hai buoc.
Dich duoc chap nhan khi khoang cach <= 0.12 m va van toc <= 0.03 m/s.
Cac thong so moi duoc ap dung khi bam 'Chay lai / Ap dung'.
"""
from dataclasses import dataclass, asdict
import argparse
import csv
import math
import numpy as np


def angle_mod(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Parameters:
    k: float = 0.1
    Lfc: float = 2.5
    Kp: float = 1.2
    dt: float = 0.05
    WB: float = 2.9
    max_steer_deg: float = 45.0
    cruise_speed: float = 1.0
    min_speed: float = 0.5
    regulation_radius: float = 8.0
    max_accel: float = 1.0
    max_decel: float = 1.5
    corridor_width: float = 8.0
    vehicle_width: float = 2.0
    leg_length: float = 12.0
    max_time: float = 240.0
    curvature_speed: bool = False
    stop_on_collision: bool = True

    def validate(self):
        for name, value in asdict(self).items():
            if isinstance(value, bool):
                continue
            if not math.isfinite(value) or value < 0 or (value == 0 and name != 'k'):
                raise ValueError(f'{name}: phai la so huu han duong (k cho phep bang 0).')
        if not 0.005 <= self.dt <= 0.1:
            raise ValueError('dt can nam trong [0.005, 0.1] s.')
        if not 1 <= self.max_steer_deg <= 80:
            raise ValueError('Goc lai toi da can nam trong [1, 80] do.')
        if self.min_speed > self.cruise_speed:
            raise ValueError('Toc do cua toi thieu khong duoc lon hon toc do hanh trinh.')
        if self.corridor_width <= self.vehicle_width:
            raise ValueError('Hanh lang can rong hon than xe.')
        if self.leg_length <= self.corridor_width:
            raise ValueError('Chieu dai doan can lon hon be rong hanh lang de thay ro cac khuc cua.')
        if self.leg_length > 100 or self.max_time > 2000:
            raise ValueError('Gioi han giao dien: doan <= 100 m, thoi gian <= 2000 s.')


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    yaw: float = math.pi / 2
    v: float = 0.0

    def update(self, a, delta, p):
        self.x += self.v * math.cos(self.yaw) * p.dt
        self.y += self.v * math.sin(self.yaw) * p.dt
        self.yaw = angle_mod(self.yaw + self.v / p.WB * math.tan(delta) * p.dt)
        self.v = max(0.0, self.v + a * p.dt)


def create_course(leg_length=12.0, step=0.1):
    """Tuyen xung vuong: len, phai, len, trai, len, phai; goc 90 do cung.
    Doi mang vertices nay neu muon doi toan bo hinh dang tuyen.
    """
    d = leg_length
    vertices = np.array([(0, 0), (0, d), (d, d), (d, 2*d),
                         (0, 2*d), (0, 3*d), (d, 3*d)], dtype=float)
    pieces = []
    for j, (a, b) in enumerate(zip(vertices[:-1], vertices[1:])):
        n = max(1, math.ceil(np.linalg.norm(b-a)/step))
        points = np.linspace(a, b, n+1)
        pieces.append(points if j == 0 else points[1:])
    path = np.concatenate(pieces)
    return path[:, 0], path[:, 1], vertices


class TargetCourse:
    def __init__(self, cx, cy):
        self.path = np.column_stack((cx, cy))
        self.nearest = None
        self.target = 0

    def search_target_index(self, state, p):
        xy = np.array([state.x, state.y])
        if self.nearest is None:
            self.nearest = int(np.argmin(np.linalg.norm(self.path-xy, axis=1)))
        else:
            # Guard ind+1, sua loi vuot mang o cuoi tuyen cua code cu.
            while self.nearest + 1 < len(self.path):
                a = np.linalg.norm(self.path[self.nearest]-xy)
                b = np.linalg.norm(self.path[self.nearest+1]-xy)
                if a < b:
                    break
                self.nearest += 1
        lookahead = p.Lfc + p.k * state.v
        ind = self.nearest
        while ind + 1 < len(self.path) and np.linalg.norm(self.path[ind]-xy) < lookahead:
            ind += 1
        self.target = max(self.target, ind)
        return self.target


def pure_pursuit_steer_control(state, course, p):
    ind = course.search_target_index(state, p)
    tx, ty = course.path[ind]
    alpha = math.atan2(ty-state.y, tx-state.x)-state.yaw
    distance = max(math.hypot(tx-state.x, ty-state.y), 1e-6)
    raw = math.atan2(2*p.WB*math.sin(alpha), distance)
    limit = math.radians(p.max_steer_deg)
    return float(np.clip(raw, -limit, limit)), ind


def curvature_speed_reference(delta, p):
    curvature = abs(math.tan(delta)/p.WB)
    scale = min(1.0, 1.0/max(p.regulation_radius*curvature, 1e-6))
    return min(p.cruise_speed, max(p.min_speed, p.cruise_speed*scale))


def body_polygon(state, p):
    # 0.5 m nho ra moi dau; chieu dai than = WB+1.0.
    local = np.array([[-0.5, -p.vehicle_width/2],
                      [p.WB+0.5, -p.vehicle_width/2],
                      [p.WB+0.5, p.vehicle_width/2],
                      [-0.5, p.vehicle_width/2]])
    c, s = math.cos(state.yaw), math.sin(state.yaw)
    return local @ np.array([[c, s], [-s, c]]) + [state.x, state.y]


def clip_to_rectangle(poly, rect):
    """Sutherland-Hodgman: giao polygon loi voi hinh chu nhat."""
    result = [np.asarray(v, float) for v in poly]
    xmin, xmax, ymin, ymax = rect
    for axis, bound, side in [(0, xmin, 1), (0, xmax, -1),
                              (1, ymin, 1), (1, ymax, -1)]:
        source, result = result, []
        if not source:
            break
        previous = source[-1]
        prev_inside = side*(previous[axis]-bound) >= 0
        for current in source:
            inside = side*(current[axis]-bound) >= 0
            if inside != prev_inside:
                t = (bound-previous[axis])/(current[axis]-previous[axis])
                result.append(previous+t*(current-previous))
            if inside:
                result.append(current)
            previous, prev_inside = current, inside
    return np.asarray(result)


def polygon_area(poly):
    if len(poly) < 3:
        return 0.0
    return abs(float(np.dot(poly[:, 0], np.roll(poly[:, 1], 1))
                     - np.dot(poly[:, 1], np.roll(poly[:, 0], 1))))/2


class Corridor:
    """Hop cac hinh chu nhat truc song song, thanh hanh lang goc vuong.
    Hai dau hanh lang duoc noi dai de chua du than xe tai tram.
    Phan hoach thanh o de kiem tra toan bo than xe, khong chi 4 goc.
    """
    def __init__(self, vertices, p):
        half = p.corridor_width/2
        rects = []
        for a, b in zip(vertices[:-1], vertices[1:]):
            rects.append((min(a[0], b[0])-half, max(a[0], b[0])+half,
                          min(a[1], b[1])-half, max(a[1], b[1])+half))
        # Keo dai dau/cuoi theo huong tuyen de than xe khong cham vach tram.
        extension = p.WB+1.0
        for a, b in [(vertices[0], vertices[0]-(vertices[1]-vertices[0])/
                      np.linalg.norm(vertices[1]-vertices[0])*extension),
                     (vertices[-1], vertices[-1]+(vertices[-1]-vertices[-2])/
                      np.linalg.norm(vertices[-1]-vertices[-2])*extension)]:
            rects.append((min(a[0], b[0])-half, max(a[0], b[0])+half,
                          min(a[1], b[1])-half, max(a[1], b[1])+half))
        self.rects = rects
        xs = sorted(set(v for r in rects for v in r[:2]))
        ys = sorted(set(v for r in rects for v in r[2:]))
        self.bounds = (xs[0], xs[-1], ys[0], ys[-1])
        cells = np.zeros((len(xs)-1, len(ys)-1), dtype=bool)
        self.outside = []
        for i in range(len(xs)-1):
            for j in range(len(ys)-1):
                mx, my = (xs[i]+xs[i+1])/2, (ys[j]+ys[j+1])/2
                inside = any(a <= mx <= b and c <= my <= d for a,b,c,d in rects)
                cells[i, j] = inside
                if not inside:
                    self.outside.append((xs[i], xs[i+1], ys[j], ys[j+1]))
        self.walls = []
        for i,j in zip(*np.where(cells)):
            x0,x1,y0,y1 = xs[i],xs[i+1],ys[j],ys[j+1]
            if i == 0 or not cells[i-1,j]: self.walls.append([(x0,y0),(x0,y1)])
            if i == len(xs)-2 or not cells[i+1,j]: self.walls.append([(x1,y0),(x1,y1)])
            if j == 0 or not cells[i,j-1]: self.walls.append([(x0,y0),(x1,y0)])
            if j == len(ys)-2 or not cells[i,j+1]: self.walls.append([(x0,y1),(x1,y1)])

    def collision(self, poly):
        x0,y0 = poly.min(axis=0)
        x1,y1 = poly.max(axis=0)
        a,b,c,d = self.bounds
        if x0 < a-1e-9 or x1 > b+1e-9 or y0 < c-1e-9 or y1 > d+1e-9:
            return True
        for rect in self.outside:
            a,b,c,d = rect
            if x1 <= a or x0 >= b or y1 <= c or y0 >= d:
                continue
            if polygon_area(clip_to_rectangle(poly, rect)) > 1e-9:
                return True
        return False


def tracking_error(x, y, vertices):
    point = np.array([x, y])
    a, u = vertices[:-1], np.diff(vertices, axis=0)
    fraction = np.clip(np.sum((point-a)*u, axis=1)/np.sum(u*u, axis=1), 0, 1)
    return float(np.min(np.linalg.norm(point-(a+fraction[:, None]*u), axis=1)))


class Simulation:
    def __init__(self, p):
        p.validate()
        self.p = p
        cx,cy,self.vertices = create_course(p.leg_length)
        self.course = TargetCourse(cx,cy)
        self.corridor = Corridor(self.vertices,p)
        self.state = State()
        self.t = self.delta = 0.0
        self.ref = p.cruise_speed
        self.reason = ''
        self.collided = False
        self.first_collision = None
        # t,x,y,yaw,v,delta,speed_ref,error,collision,goal_distance
        self.rows = []
        self.course.search_target_index(self.state,p)
        self.record()

    def record(self):
        s = self.state
        hit = self.corridor.collision(body_polygon(s,self.p))
        if hit and self.first_collision is None:
            self.first_collision = self.t
        self.collided = self.collided or hit
        distance = float(np.linalg.norm([s.x-self.vertices[-1,0], s.y-self.vertices[-1,1]]))
        self.rows.append((self.t,s.x,s.y,s.yaw,s.v,self.delta,self.ref,
                          tracking_error(s.x,s.y,self.vertices),int(hit),distance))

    def step(self):
        if self.reason:
            return
        p,s = self.p,self.state
        self.delta, _ = pure_pursuit_steer_control(s,self.course,p)
        self.ref = curvature_speed_reference(self.delta,p) if p.curvature_speed else p.cruise_speed
        distance = float(np.linalg.norm([s.x-self.vertices[-1,0],s.y-self.vertices[-1,1]]))
        # Chi dung o cuoi tuyen khi xe da tien den doan cuoi.
        # Giam toc P theo khoang cach, tach biet voi quy luat giam toc theo cua.
        if self.course.nearest >= len(self.course.path)-int(p.leg_length/0.1)-2:
            self.ref = min(self.ref, 0.25*distance)
            if distance <= 0.10:
                self.ref = 0.0
        a = float(np.clip(p.Kp*(self.ref-s.v), -p.max_decel, p.max_accel))
        s.update(a,self.delta,p)
        self.t += p.dt
        self.record()
        final_distance = self.rows[-1][-1]
        if self.rows[-1][8] and p.stop_on_collision:
            self.reason = 'Dung: than xe vuot bien hanh lang'
        elif final_distance <= 0.12 and s.v <= 0.03:
            self.reason = 'Dat dich: cach <= 0.12 m, v <= 0.03 m/s'
        elif self.t >= p.max_time:
            self.reason = 'Het thoi gian: chua dat dich'

    def metrics(self):
        data = np.asarray(self.rows)
        errors = data[:,7]
        ds = np.hypot(np.diff(data[:,1]),np.diff(data[:,2]))
        # Tich phan hinh thang cua e^2 theo quang duong da di.
        spatial = math.sqrt(float(np.sum((errors[:-1]**2+errors[1:]**2)/2*ds)/ds.sum())) if ds.sum()>0 else 0.0
        return dict(rms=math.sqrt(float(np.mean(errors**2))), rms_distance=spatial,
                    max_error=float(errors.max()), duration=self.t,
                    final_speed=self.state.v, goal_distance=float(data[-1,-1]))


def draw_map(ax, sim):
    from matplotlib.patches import Rectangle, Polygon
    from matplotlib.collections import LineCollection
    for a,b,c,d in sim.corridor.rects:
        ax.add_patch(Rectangle((a,c),b-a,d-c,facecolor='#e9eef3',edgecolor='none'))
    ax.add_collection(LineCollection(sim.corridor.walls,colors='#334155',linewidths=1.8))
    line = sim.course.path
    ax.plot(line[:,0],line[:,1],'--',color='#e34a33',lw=1.4,label='Tuyen tham chieu')
    ax.scatter(*sim.vertices[0],c='#16a34a',s=45,zorder=5)
    ax.scatter(*sim.vertices[-1],c='#e34a33',s=65,marker='s',zorder=5)
    ax.text(*sim.vertices[0], '  A',fontsize=10)
    ax.text(*sim.vertices[-1], '  B',fontsize=10)
    trail, = ax.plot([],[],color='#2563eb',lw=1.8,label='Tam truc sau')
    target, = ax.plot([],[],'x',color='#16a34a',ms=8,label='Diem muc tieu')
    body = Polygon(body_polygon(sim.state,sim.p),closed=True,facecolor='#93c5fd',edgecolor='#1d4ed8',alpha=.8,zorder=6)
    ax.add_patch(body)
    wheels = [ax.plot([],[],color='#172554',lw=3,zorder=7)[0] for _ in range(4)]
    heading, = ax.plot([],[],color='#f97316',lw=2,zorder=8)
    a,b,c,d = sim.corridor.bounds
    ax.set(xlim=(a-2,b+2),ylim=(c-2,d+2),xlabel='x [m]',ylabel='y [m]',title='Hanh lang goc vuong - Pure Pursuit')
    ax.set_aspect('equal',adjustable='box')
    ax.grid(alpha=.2)
    ax.legend(loc='upper left',fontsize=8)
    return trail,target,body,wheels,heading


def update_vehicle(artists,sim):
    trail,target,body,wheels,heading = artists
    data = np.asarray(sim.rows)
    trail.set_data(data[:,1],data[:,2])
    t = sim.course.path[sim.course.target]
    target.set_data([t[0]],[t[1]])
    body.set_xy(body_polygon(sim.state,sim.p))
    body.set_facecolor('#fca5a5' if sim.rows[-1][8] else '#93c5fd')
    s,p = sim.state,sim.p
    c,sn = math.cos(s.yaw),math.sin(s.yaw)
    rotation = np.array([[c,sn],[-sn,c]])
    for artist,(wx,wy,steer) in zip(wheels,[(0,-p.vehicle_width/2,0),(0,p.vehicle_width/2,0),
                                         (p.WB,-p.vehicle_width/2,sim.delta),(p.WB,p.vehicle_width/2,sim.delta)]):
        center = np.array([wx,wy]) @ rotation + [s.x,s.y]
        theta = s.yaw+steer
        dv = .3*np.array([math.cos(theta),math.sin(theta)])
        points = np.array([center-dv,center+dv])
        artist.set_data(points[:,0],points[:,1])
    heading.set_data([s.x,s.x+1.2*c],[s.y,s.y+1.2*sn])


class App:
    def __init__(self, root):
        import tkinter as tk
        from tkinter import ttk
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
        self.tk,self.ttk,self.root = tk,ttk,root
        root.title('Pure Pursuit v2 | Hanh lang va bang tham so')
        root.geometry('1380x880')
        root.minsize(1080,720)
        self.running = False
        self.after_id = None
        self.fields = {}
        self.history = []
        self.figure = Figure(
            figsize=(9, 8),
            dpi=100,
            constrained_layout=True
        )
        left = ttk.Frame(root)
        left.pack(side='left',fill='both',expand=True)
        self.canvas = FigureCanvasTkAgg(self.figure,master=left)
        self.canvas.get_tk_widget().pack(fill='both',expand=True)
        NavigationToolbar2Tk(self.canvas,left).update()
        sidebar = ttk.Frame(root, width=365)
        sidebar.pack(side='right',fill='y')
        scroll_canvas = tk.Canvas(sidebar,width=350,highlightthickness=0)
        scrollbar = ttk.Scrollbar(sidebar,orient='vertical',command=scroll_canvas.yview)
        scrollbar.pack(side='right',fill='y')
        scroll_canvas.pack(side='left',fill='both',expand=True)
        scroll_canvas.configure(yscrollcommand=scrollbar.set)
        panel = ttk.Frame(scroll_canvas,padding=12)
        window_id = scroll_canvas.create_window((0,0),window=panel,anchor='nw')
        panel.bind('<Configure>',lambda event: scroll_canvas.configure(scrollregion=scroll_canvas.bbox('all')))
        scroll_canvas.bind('<Configure>',lambda event: scroll_canvas.itemconfigure(window_id,width=event.width))
        ttk.Label(panel,text='THONG SO MO PHONG',font=('Segoe UI',13,'bold')).pack(anchor='w')
        ttk.Label(panel,text='Sua so, sau do bam Chay lai / Ap dung.',wraplength=325).pack(anchor='w',pady=(2,8))
        grid = ttk.Frame(panel)
        grid.pack(fill='x')
        labels = [('Lfc','Nhin truoc co so [m]'),('k','He so nhin truoc [s]'),
                  ('Kp','Kp toc do [1/s]'),('cruise_speed','Toc do hanh trinh [m/s]'),
                  ('min_speed','Toc do cua toi thieu [m/s]'),('regulation_radius','Ban kinh giam toc [m]'),
                  ('max_accel','Gia toc toi da [m/s2]'),('max_decel','Giam toc toi da [m/s2]'),
                  ('corridor_width','Rong hanh lang [m]'),('leg_length','Dai moi doan [m]'),
                  ('WB','Chieu dai co so [m]'),('vehicle_width','Rong than xe [m]'),
                  ('max_steer_deg','Goc lai toi da [do]'),('dt','Buoc thoi gian [s]'),
                  ('max_time','Thoi gian toi da [s]')]
        defaults = Parameters()
        for i,(key,label) in enumerate(labels):
            ttk.Label(grid,text=label).grid(row=i,column=0,sticky='w',pady=2)
            var = tk.StringVar(value=str(getattr(defaults,key)))
            ttk.Entry(grid,textvariable=var,width=10).grid(row=i,column=1,sticky='e',padx=(8,0))
            self.fields[key] = var
        self.curve = tk.BooleanVar(value=False)
        self.stop_hit = tk.BooleanVar(value=True)
        ttk.Checkbutton(panel,text='Giam toc theo do cong',variable=self.curve).pack(anchor='w',pady=(8,0))
        ttk.Checkbutton(panel,text='Dung khi than xe vuot bien',variable=self.stop_hit).pack(anchor='w')
        controls = ttk.Frame(panel)
        controls.pack(fill='x',pady=8)
        for i,(text,command) in enumerate([('Chay / Tiep tuc',self.start),('Tam dung',self.pause),
                                         ('Chay lai / Ap dung',self.reset),('Mac dinh',self.defaults),
                                         ('Xuat CSV',self.export_csv),('Ket qua cac lan',self.show_history)]):
            ttk.Button(controls,text=text,command=command).grid(row=i//2,column=i%2,sticky='ew',padx=2,pady=3)
        self.status = tk.StringVar()
        ttk.Label(panel,textvariable=self.status,justify='left',wraplength=330,font=('Consolas',10)).pack(anchor='w',pady=4)
        ttk.Label(panel,text='Xe bicycle, chay tien. Goc tuyen 90 do cung.\nGiam toc theo cua khong tu dong tranh tuong.\nMoi nhip hien thi: 5 buoc mo phong.\nDanh gia cham bien tai cac thoi diem dt.',wraplength=330,foreground='#475569').pack(anchor='w',pady=8)
        self.reset()
        root.protocol('WM_DELETE_WINDOW',self.close)

    def read_parameters(self):
        p = Parameters(**{key:float(value.get()) for key,value in self.fields.items()},
                       curvature_speed=self.curve.get(),stop_on_collision=self.stop_hit.get())
        p.validate()
        return p

    def pause(self):
        self.running = False
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
            self.after_id = None

    def reset(self):
        from tkinter import messagebox
        try:
            p = self.read_parameters()
        except ValueError as exc:
            messagebox.showerror('Thong so chua hop le',str(exc))
            return
        self.pause()
        self.sim = Simulation(p)
        self.saved_run = False
        self.figure.clear()
        gs = self.figure.add_gridspec(2,2,width_ratios=[1.25,1])
        self.ax = self.figure.add_subplot(gs[:,0])
        self.av = self.figure.add_subplot(gs[0,1])
        self.ae = self.figure.add_subplot(gs[1,1])
        self.artists = draw_map(self.ax,self.sim)
        self.vline, = self.av.plot([],[],label='Van toc thuc',color='#2563eb')
        self.rline, = self.av.plot([],[],'--',label='Van toc dat',color='#f97316')
        self.eline, = self.ae.plot([],[],color='#9333ea')
        self.av.set(xlabel='t [s]',ylabel='v [m/s]',title='Van toc')
        self.ae.set(xlabel='t [s]',ylabel='e [m]',title='Sai so tam truc sau toi tuyen')
        self.av.legend(fontsize=8)
        self.av.grid(alpha=.25)
        self.ae.grid(alpha=.25)
        self.render()

    def defaults(self):
        p = Parameters()
        for key,var in self.fields.items(): var.set(str(getattr(p,key)))
        self.curve.set(p.curvature_speed)
        self.stop_hit.set(p.stop_on_collision)
        self.reset()

    def start(self):
        if self.running or self.sim.reason: return
        self.running = True
        self.tick()

    def tick(self):
        self.after_id = None
        if not self.running: return
        for _ in range(5):
            self.sim.step()
            if self.sim.reason: break
        self.render()
        if self.sim.reason:
            self.running = False
            if not self.saved_run:
                self.history.append((asdict(self.sim.p),self.sim.metrics(),self.sim.reason,self.sim.collided))
                self.saved_run = True
            print(self.sim.reason,self.sim.metrics())
        else:
            self.after_id = self.root.after(30,self.tick)

    def render(self):
        update_vehicle(self.artists,self.sim)
        data = np.asarray(self.sim.rows)
        for artist,values in [(self.vline,data[:,4]),(self.rline,data[:,6]),(self.eline,data[:,7])]:
            artist.set_data(data[:,0],values)
        for ax in [self.av,self.ae]:
            ax.relim(); ax.autoscale_view()
            ax.set_xlim(0,max(10,self.sim.t+1))
        self.av.set_ylim(0,max(self.sim.p.cruise_speed*1.15,.1))
        self.ae.set_ylim(0,max(.2,float(data[:,7].max())*1.15))
        m = self.sim.metrics()
        hit = 'CO' if self.sim.collided else 'chua'
        status = self.sim.reason or ('Dang chay' if self.running else 'San sang / tam dung')
        self.status.set(f'{status}\nGiam toc cua: {self.sim.p.curvature_speed}\n'
                        f't = {m["duration"]:.2f} s | v = {m["final_speed"]:.3f} m/s\n'
                        f'RMS thoi gian: {m["rms"]:.4f} m\nRMS quang duong: {m["rms_distance"]:.4f} m\n'
                        f'Sai so max: {m["max_error"]:.4f} m\nCach dich: {m["goal_distance"]:.3f} m\n'
                        f'Da vuot bien: {hit}')
        self.canvas.draw_idle()

    def export_csv(self):
        from tkinter import filedialog, messagebox
        filename = filedialog.asksaveasfilename(defaultextension='.csv',initialfile='pure_pursuit_run.csv',filetypes=[('CSV','*.csv')])
        if not filename: return
        try:
            with open(filename,'w',newline='',encoding='utf-8-sig') as handle:
                writer = csv.writer(handle)
                writer.writerow(['# parameters',repr(asdict(self.sim.p))])
                writer.writerow(['t_s','rear_x_m','rear_y_m','yaw_rad','v_m_s','steer_rad',
                                 'v_ref_m_s','tracking_error_m','body_outside','goal_distance_m'])
                writer.writerows(self.sim.rows)
            messagebox.showinfo('Xuat CSV','Da luu du lieu lan chay hien tai.')
        except OSError as exc:
            messagebox.showerror('Khong luu duoc',str(exc))

    def show_history(self):
        top = self.tk.Toplevel(self.root)
        top.title('Cac lan chay da ket thuc - thong so cua tung lan')
        text = self.tk.Text(top,width=105,height=26,wrap='word')
        text.pack(fill='both',expand=True)
        if not self.history: text.insert('end','Chua co lan chay nao ket thuc.\n')
        for i,(p,m,reason,hit) in enumerate(self.history,1):
            text.insert('end',f'Lan {i}: {reason}\nThong so: {p}\nKet qua: {m}\nDa vuot bien: {hit}\n\n')
        text.configure(state='disabled')

    def close(self):
        self.pause()
        self.root.destroy()


def self_test(preview=None):
    # Controller, corridor footprint and end-of-path tests, no GUI needed.
    for regulated in [False,True]:
        p = Parameters(curvature_speed=regulated,stop_on_collision=False)
        sim = Simulation(p)
        assert not sim.rows[0][8], 'Xe khoi tao ngoai hanh lang'
        while not sim.reason: sim.step()
        assert 'Dat dich' in sim.reason, sim.reason
        assert sim.metrics()['goal_distance'] <= .12
        assert sim.state.v <= .03
        assert abs(sim.delta) <= math.radians(p.max_steer_deg)+1e-10
        print('PASS default',regulated,sim.reason,sim.metrics(),'collision=',sim.collided)
    narrow = Simulation(Parameters(corridor_width=2.2))
    while not narrow.reason: narrow.step()
    assert narrow.collided and 'vuot bien' in narrow.reason
    print('PASS narrow corridor collision at',narrow.t)
    assert abs(tracking_error(1,5,np.array([(0,0),(0,10)]))-1)<1e-9
    assert abs(polygon_area(clip_to_rectangle(np.array([(0,0),(2,0),(2,2),(0,2)]),(1,3,1,3)))-1)<1e-9
    print('PASS segment distance and polygon clipping')
    if preview:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,ax = plt.subplots(figsize=(8,10),layout='constrained')
        artists = draw_map(ax,sim)
        update_vehicle(artists,sim)
        fig.savefig(preview,dpi=120)
        print('Preview:',preview)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test',action='store_true')
    parser.add_argument('--preview',help='PNG preview; use with --self-test')
    args = parser.parse_args()
    if args.self_test:
        self_test(args.preview)
    else:
        try:
            import tkinter as tk
        except ImportError:
            raise SystemExit('Thieu Tkinter. Windows: cai Python tu python.org va bat Tcl/Tk. Ubuntu: sudo apt install python3-tk')
        root = tk.Tk()
        App(root)
        root.mainloop()


if __name__ == '__main__':
    main()
