"""Pure Pursuit version 3: hanh lang va so sanh doi chung, Python 3.9+.

Chay: python main.py
Cai thu vien: python -m pip install numpy matplotlib
Kiem tra khong can GUI: python main.py --self-test

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
from dataclasses import asdict, dataclass, replace
import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path

import numpy as np

from parameters import DEFAULT_ROUTE, MODE_LABELS, ROUTES, Parameters
from pid import SpeedPID, curvature_speed_reference
from pure_pursuit import TargetCourse, pure_pursuit_steer_control
from trajectory import (
    Corridor,
    align_on_route,
    body_polygon,
    clip_to_rectangle,
    create_course,
    difference_figure,
    draw_map,
    polygon_area,
    route_projection,
    sample_plot_figure,
    tracking_error,
    update_vehicle,
)


CSV_COLUMNS = ['t_s','rear_x_m','rear_y_m','yaw_rad','v_m_s','steer_rad',
               'v_ref_m_s','tracking_error_m','body_outside','goal_distance_m']


def angle_mod(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


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


class Simulation:
    def __init__(self, p, controller="modified"):
        self.controller = controller
        p.validate()
        self.p = p
        cx,cy,self.vertices = create_course(p.leg_length, route=p.route)
        self.course = TargetCourse(cx,cy)
        self.corridor = Corridor(self.vertices,p)
        self.state = State()
        self.speed_pid = SpeedPID()
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
        self.delta, _ = pure_pursuit_steer_control(s,self.course,p,
                            'lookahead' if self.controller == 'baseline' else 'actual')
        self.ref = curvature_speed_reference(self.delta,p) if p.curvature_speed else p.cruise_speed
        distance = float(np.linalg.norm([s.x-self.vertices[-1,0],s.y-self.vertices[-1,1]]))
        # Chi dung o cuoi tuyen khi xe da tien den doan cuoi.
        # Giam toc P theo khoang cach, tach biet voi quy luat giam toc theo cua.
        if self.course.nearest >= len(self.course.path)-int(p.leg_length/0.1)-2:
            self.ref = min(self.ref, 0.25*distance)
            if distance <= 0.10:
                self.ref = 0.0
        a = self.speed_pid.update(self.ref, s.v, p)
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


def make_pair(p,mode='current'):
    """Khong doi hinh hoc/state/limits/deceleration-to-goal giua hai ban.
    Baseline la quy luat lai Lf cua PythonRobotics da dua vao cung test harness;
    khong tai hien offset tam xe hoac cach dung som cua vi du goc.
    """
    if mode not in MODE_LABELS:
        raise ValueError('Che do so sanh khong hop le')
    baseline = Simulation(replace(p,curvature_speed=False),
                          'modified' if mode == 'speed' else 'baseline')
    new = Simulation(replace(p,curvature_speed=(p.curvature_speed if mode == 'current' else True)),
                     'modified')
    return baseline,new


def comparison_report(baseline,new,mode,aligned):
    m0,m1 = baseline.metrics(),new.metrics()
    labels = [('rms','RMS theo thoi gian [m]'),('rms_distance','RMS theo quang duong xe [m]'),
              ('max_error','Sai so lon nhat [m]'),('duration','Thoi gian dung mo phong [s]'),
              ('goal_distance','Khoang cach dich [m]'),('final_speed','Van toc cuoi [m/s]')]
    lines = [MODE_LABELS[mode], '',
             'QUY UOC DOI CHUNG',
             'Baseline goc: chi dung quy luat lai Lf cua PythonRobotics.',
             'Hai ban dung chung pose tam truc sau, tuyen, thong so vat ly,',
             'gioi han goc lai/gia toc, bo dieu toc PID (cung he so), giam toc den dich va kiem tra vuot bien.',
             'Day khong phai so sanh voi nguyen file demo goc khac dieu kien.',
             'Che do speed: ca hai dung d thuc; chi bat/tat giam toc cua.', '',
             f'Doi chung: controller={baseline.controller}, giam toc={baseline.p.curvature_speed}',
             f'Ban moi: controller={new.controller}, giam toc={new.p.curvature_speed}', '',
             f'{"Chi so":35s} {"Doi chung":>13s} {"Moi":>13s} {"Moi - DC":>13s}']
    for key,label in labels:
        lines.append(f'{label:35s} {m0[key]:13.4f} {m1[key]:13.4f} {m1[key]-m0[key]:13.4f}')
    lines += ['',f'Doi chung: {baseline.reason}; da vuot bien={baseline.collided}; '
              f'lan dau={baseline.first_collision}',
              f'Ban moi: {new.reason}; da vuot bien={new.collided}; lan dau={new.first_collision}',
              '', 'Luu y: RMS tren la tung luot chay; neu mot luot dung som, khong duoc',
              'ket luan tot hon tu RMS toan luot. Doi chieu do thi mien tuyen chung ben duoi.']
    if len(aligned):
        ds = np.diff(aligned[:,0])
        span = aligned[-1,0]-aligned[0,0]
        def common_rms(column):
            e = aligned[:,column]
            return math.sqrt(float(np.sum((e[:-1]**2+e[1:]**2)/2*ds)/span))
        r0,r1 = common_rms(1),common_rms(2)
        lines += [f'Pham vi tien do chung: {aligned[0,0]:.3f} -> {aligned[-1,0]:.3f} m',
                  f'RMS tren cung mien s tham chieu: DC={r0:.5f} m; Moi={r1:.5f} m; '
                  f'Delta={r1-r0:+.5f} m',
                  'RMS mien s khac RMS(ds) o bang: s la tien do tren tuyen, ds la quang duong xe.',
                  'Delta e am: ban moi co sai so nho hon tai cung tien do.',
                  'Khoang cach 2 vi tri khong phai sai so toi tuyen.',
                  'Neu tien do bi lui, phep can chinh chi dung cac moc di qua lan dau.']
    else:
        lines.append('Chua co mien tien do chung du de noi suy.')
    lines += ['', 'THONG SO CHUNG',json.dumps(asdict(new.p),indent=2,ensure_ascii=False)]
    return '\n'.join(lines)


def save_comparison(baseline,new,mode,folder):
    folder = Path(folder)
    folder.mkdir(parents=True,exist_ok=True)
    aligned = align_on_route(baseline,new)
    time_limit = max(baseline.t,new.t,1)
    error_limit = max(.1,max(row[7] for row in baseline.rows+new.rows)*1.15)
    title0 = 'Doi chung: d thuc, toc do co dinh' if mode == 'speed' else 'Pure Pursuit goc: mau so Lf (chuan hoa)'
    title1 = 'Pure Pursuit moi: d thuc'+(' + giam toc cua' if new.p.curvature_speed else ' + toc do co dinh')
    figures = [sample_plot_figure(baseline,title0,time_limit,error_limit,'#2563eb'),
               sample_plot_figure(new,title1,time_limit,error_limit,'#16a34a'),
               difference_figure(aligned,mode)]
    for filename,fig in zip(['01_pp_goc.png','02_pp_moi.png','03_chenh_lech.png'],figures):
        fig.savefig(folder/filename,dpi=150)
    for name,sim in [('pp_goc.csv',baseline),('pp_moi.csv',new)]:
        with (folder/name).open('w',newline='',encoding='utf-8-sig') as handle:
            writer = csv.writer(handle)
            writer.writerow(CSV_COLUMNS)
            writer.writerows(sim.rows)
    with (folder/'chenh_lech_theo_tuyen.csv').open('w',newline='',encoding='utf-8-sig') as handle:
        writer = csv.writer(handle)
        writer.writerow(['s_reference_m','error_baseline_m','error_new_m','delta_error_m',
                         'v_baseline_m_s','v_new_m_s','delta_v_m_s','position_gap_m','delta_arrival_time_s'])
        writer.writerows(aligned)
    report = comparison_report(baseline,new,mode,aligned)
    (folder/'doi_chieu.txt').write_text(report,encoding='utf-8-sig')
    metadata = {'mode':mode,'baseline':asdict(baseline.p),'new':asdict(new.p),
                'baseline_controller':baseline.controller,'new_controller':new.controller,
                'baseline_reason':baseline.reason,'new_reason':new.reason,
                'baseline_metrics':baseline.metrics(),'new_metrics':new.metrics(),
                'shared_harness':True,'baseline_is_verbatim_original_demo':False}
    (folder/'parameters.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
    return figures,report


class App:
    def __init__(self, root):
        import tkinter as tk
        from tkinter import ttk
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
        self.tk,self.ttk,self.root = tk,ttk,root
        root.title('Pure Pursuit v3 | Mo phong va so sanh goc - moi')
        root.geometry('1380x880')
        root.minsize(1080,720)
        self.running = False
        self.after_id = None
        self.fields = {}
        self.history = []
        self.route = DEFAULT_ROUTE
        self.route_label = tk.StringVar()
        self.compare_after = None
        self.comparing = False
        self.result_windows = []
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
        self.route_button = ttk.Menubutton(panel,textvariable=self.route_label)
        route_menu = tk.Menu(self.route_button,tearoff=False)
        for route,(label,_) in ROUTES.items():
            route_menu.add_command(label=label,command=lambda selected=route: self.reset(selected))
        self.route_button.configure(menu=route_menu)
        self.route_button.pack(fill='x',pady=(0,12))
        ttk.Label(panel,text='THONG SO MO PHONG',font=('Segoe UI',13,'bold')).pack(anchor='w')
        ttk.Label(panel,text='Sua so, sau do bam Chay lai / Ap dung.',wraplength=325).pack(anchor='w',pady=(2,8))
        grid = ttk.Frame(panel)
        grid.pack(fill='x')
        labels = [('Lfc','Nhin truoc co so [m]'),('k','He so nhin truoc [s]'),
                  ('Kp','Kp toc do [1/s]'),('Ki','Ki toc do [1/s2]'),
                  ('Kd','Kd toc do'),('cruise_speed','Toc do hanh trinh [m/s]'),
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
        ttk.Label(panel,text='SO SANH GOC - MOI',font=('Segoe UI',10,'bold')).pack(anchor='w',pady=(10,2))
        self.compare_mode = tk.StringVar(value=MODE_LABELS['current'])
        ttk.Combobox(panel,textvariable=self.compare_mode,values=list(MODE_LABELS.values()),
                     state='readonly',width=39).pack(fill='x')
        ttk.Button(panel,text='Chay 2 ban va mo 3 hinh',command=self.compare).pack(fill='x',pady=4)
        ttk.Label(panel,text='Baseline da chuan hoa cung mo hinh, tuyen,\ngioi han va dung dich. Khong phai file goc nguyen ban.',
                  wraplength=325,foreground='#475569').pack(anchor='w')
        controls = ttk.Frame(panel)
        controls.pack(fill='x',pady=8)
        for i,(text,command) in enumerate([('Chay / Tiep tuc',self.start),('Tam dung',self.pause),
                                         ('Chay lai / Ap dung',self.reset),('Mac dinh',self.defaults),
                                         ('Xuat CSV',self.export_csv),('Ket qua cac lan',self.show_history)]):
            ttk.Button(controls,text=text,command=command).grid(row=i//2,column=i%2,sticky='ew',padx=2,pady=3)
        self.status = tk.StringVar()
        ttk.Label(panel,textvariable=self.status,justify='left',wraplength=330,font=('Consolas',10)).pack(anchor='w',pady=4)
        ttk.Label(panel,text='Xe bicycle, chay tien. Cac cua (neu co) la goc 90 do.\nGiam toc theo cua khong tu dong tranh tuong.\nMoi nhip hien thi: 5 buoc mo phong.\nDanh gia cham bien tai cac thoi diem dt.',wraplength=330,foreground='#475569').pack(anchor='w',pady=8)
        self.reset()
        root.protocol('WM_DELETE_WINDOW',self.close)

    def read_parameters(self, route=None):
        p = Parameters(**{key:float(value.get()) for key,value in self.fields.items()},
                       curvature_speed=self.curve.get(),stop_on_collision=self.stop_hit.get(),
                       route=self.route if route is None else route)
        p.validate()
        return p

    def pause(self):
        self.running = False
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
            self.after_id = None

    def reset(self, route=None):
        from tkinter import messagebox
        try:
            p = self.read_parameters(route)
        except ValueError as exc:
            messagebox.showerror('Thong so chua hop le',str(exc))
            return
        self.pause()
        self.sim = Simulation(p)
        self.route = p.route
        self.route_label.set(f'☰  Quỹ đạo · {ROUTES[self.route][0]}')
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
        if self.running or self.sim.reason or self.comparing: return
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

    def compare(self):
        from tkinter import messagebox
        if self.comparing: return
        try:
            p = self.read_parameters()
            mode = next(key for key,value in MODE_LABELS.items() if value == self.compare_mode.get())
            self.pair = make_pair(p,mode)
        except (ValueError,StopIteration) as exc:
            messagebox.showerror('Khong tao duoc so sanh',str(exc)); return
        self.pause()
        self.comparing = True
        self.route_button.state(['disabled'])
        self.comparison_mode = mode
        self.comparison_tick()

    def comparison_tick(self):
        from tkinter import messagebox
        self.compare_after = None
        try:
            for sim in self.pair:
                for _ in range(40):
                    if sim.reason: break
                    sim.step()
            self.status.set('Dang tinh 2 ban...\n'+'\n'.join(
                f'{name}: t={sim.t:.2f} s | {sim.reason or "dang chay"}'
                for name,sim in zip(['Goc','Moi'],self.pair)))
            if not all(sim.reason for sim in self.pair):
                self.compare_after = self.root.after(10,self.comparison_tick)
                return
            folder = Path.cwd()/('pp_comparison_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            figs,report = save_comparison(*self.pair,self.comparison_mode,folder)
            for title,fig in zip(['1 - Pure Pursuit goc (baseline chuan hoa)',
                                  '2 - Pure Pursuit moi', '3 - Do thi chenh lech'],figs):
                self.open_figure(title,fig)
            top = self.tk.Toplevel(self.root)
            top.title('Bang doi chieu - '+str(folder))
            area = self.tk.Text(top,width=105,height=30,wrap='word')
            area.pack(fill='both',expand=True)
            area.insert('end',report+'\n\nDa luu PNG, CSV va thong so tai:\n'+str(folder))
            area.configure(state='disabled')
            self.status.set('Da so sanh xong.\n3 cua so ket qua da mo.\n'+str(folder))
            self.comparing = False
            self.route_button.state(['!disabled'])
        except Exception as exc:
            self.comparing = False
            self.route_button.state(['!disabled'])
            messagebox.showerror('Loi so sanh',str(exc))

    def open_figure(self,title,fig):
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
        window = self.tk.Toplevel(self.root)
        window.title(title)
        window.geometry('1100x780')
        canvas = FigureCanvasTkAgg(fig,master=window)
        canvas.get_tk_widget().pack(fill='both',expand=True)
        NavigationToolbar2Tk(canvas,window).update()
        canvas.draw()
        self.result_windows.append((window,canvas,fig))
        def close_window():
            self.result_windows[:] = [item for item in self.result_windows if item[0] is not window]
            window.destroy()
            fig.clear()
        window.protocol('WM_DELETE_WINDOW',close_window)

    def close(self):
        self.pause()
        if self.compare_after is not None:
            self.root.after_cancel(self.compare_after)
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
        fig,ax = plt.subplots(figsize=(8,10),constrained_layout=True)
        artists = draw_map(ax,sim)
        update_vehicle(artists,sim)
        fig.savefig(preview,dpi=120)
        print('Preview:',preview)


def comparison_self_test():
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    # Khi cung dieu khien, tat giam toc: hai luot phai dong nhat.
    a = Simulation(Parameters())
    b = Simulation(Parameters())
    for _ in range(100): a.step(); b.step()
    assert np.array_equal(a.rows,b.rows)
    aligned = align_on_route(a,b)
    assert np.max(np.abs(aligned[:,3])) == 0
    assert np.max(aligned[:,7]) == 0
    # Kiem tra dung cong thuc baseline va moi tai cung mot trang thai.
    p = Parameters()
    s = State(x=0,y=7.05,yaw=1.3,v=1)
    cx,cy,_ = create_course()
    dc = pure_pursuit_steer_control(s,TargetCourse(cx,cy),p,'lookahead')[0]
    new = pure_pursuit_steer_control(s,TargetCourse(cx,cy),p,'actual')[0]
    assert abs(dc-new)>1e-5
    # That bai: hanh lang hep, khong extrapolate den het tuyen.
    pair = make_pair(replace(p,corridor_width=2.2),'full')
    for sim in pair:
        while not sim.reason: sim.step()
        assert sim.collided
    aligned = align_on_route(*pair)
    assert aligned[-1,0] < 72
    assert aligned[-1,0] <= min(route_projection(sim)[0][-1] for sim in pair)+1e-10
    fig = difference_figure(aligned,'full')
    FigureCanvasAgg(fig).draw()
    print('PASS identity, steering formula, early-collision alignment, difference render')
    for mode in MODE_LABELS:
        pair = make_pair(p,mode)
        for sim in pair:
            while not sim.reason: sim.step()
            assert 'Dat dich' in sim.reason, (mode,sim.reason)
            assert not sim.collided
        print('PASS comparison',mode,[sim.metrics() for sim in pair])


def main():
    parser = argparse.ArgumentParser(description='Pure Pursuit v3: giao dien + 2 hinh rieng + chenh lech')
    parser.add_argument('--self-test',action='store_true')
    parser.add_argument('--preview',help='PNG preview for --self-test')
    parser.add_argument('--compare',action='store_true',help='Tinh va luu 3 hinh, khong can GUI')
    parser.add_argument('--mode',choices=list(MODE_LABELS),default='current')
    parser.add_argument('--curvature-speed',action='store_true',help='Bat giam toc cua cho ban moi trong mode current')
    parser.add_argument('--output',default='pp_comparison_output')
    defaults = Parameters()
    parser.add_argument('--kp', type=float, default=defaults.Kp, help='He so P khi chay --compare')
    parser.add_argument('--ki', type=float, default=defaults.Ki, help='He so I khi chay --compare')
    parser.add_argument('--kd', type=float, default=defaults.Kd, help='He so D khi chay --compare')
    args = parser.parse_args()
    if args.self_test or args.compare:
        import matplotlib
        matplotlib.use('Agg')
    if args.self_test:
        self_test(args.preview)
        comparison_self_test()
    elif args.compare:
        p = Parameters(Kp=args.kp, Ki=args.ki, Kd=args.kd,
                       curvature_speed=args.curvature_speed)
        try:
            pair = make_pair(p, args.mode)
        except ValueError as exc:
            parser.error(str(exc))
        for sim in pair:
            while not sim.reason: sim.step()
        _,report = save_comparison(*pair,args.mode,Path(args.output))
        print(report)
        print('\nDa luu ket qua:',Path(args.output).resolve())
    else:
        try:
            import tkinter as tk
        except ImportError:
            raise SystemExit('Thieu Tkinter. Windows: cai Python co Tcl/Tk; Ubuntu: sudo apt install python3-tk')
        root = tk.Tk()
        App(root)
        root.mainloop()


if __name__ == "__main__":
    main()
