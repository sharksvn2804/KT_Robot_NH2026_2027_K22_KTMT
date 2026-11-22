"""Thí nghiệm 3 bộ điều khiển × 3 quỹ đạo; xuất PNG, GIF, CSV và báo cáo tiếng Việt."""
from dataclasses import asdict, replace
from pathlib import Path
import csv
import json
import math
import numpy as np
from parameters import CONTROLLER_CONFIGS, EXPERIMENT_ROUTES, ROUTES

LABELS = {k: v['label'] for k,v in CONTROLLER_CONFIGS.items()}
COLORS = {'pp_p':'#2563eb', 'pp_pid':'#ea580c','pp_pid_curve':'#16a34a'}
GROUPS = {
    'Hình học xe': ['WB','vehicle_width','max_steer_deg'],
    'Môi trường': ['corridor_width','leg_length','stop_on_collision'],
    'Pure Pursuit': ['Lfc','k'],
    'Điều tốc P/PID': ['Kp','Ki','Kd','max_accel','max_decel'],
    'Giảm tốc cua': ['curvature_speed','regulation_radius','min_speed'],
    'Giảm tốc về đích': ['goal_slowdown_gain'],
    'Thời gian / vận tốc': ['cruise_speed','dt','max_time'],
}
CSV_HEADER = ['t_s','rear_x_m','rear_y_m','yaw_rad','v_m_s','steer_rad',
              'v_ref_m_s','tracking_error_m','body_outside','goal_distance_m']

def simulation_set(base, route):
    from main import Simulation
    result = {}
    for key, config in CONTROLLER_CONFIGS.items():
        options = {k:v for k,v in config.items() if k != 'label'}
        p = replace(base,route=route,**options)
        sim = Simulation(p,controller='modified')  # Cùng công thức lái dùng d thực.
        while not sim.reason:
            sim.step()
        result[key] = sim
    return result

def route_progress(sim):
    """Tiến độ s tăng đơn điệu; cùng phép chiếu được dùng trong main.py."""
    from trajectory import route_projection
    return route_projection(sim)

def common_alignment(sims, step=.05):
    progress = {k: route_progress(sim) for k,sim in sims.items()}
    lo = max(v[0][0] for v in progress.values())
    hi = min(v[0][-1] for v in progress.values())
    if hi-lo <= 1e-8:
        return np.empty((0,1+3*4)), {}
    grid = np.linspace(lo,hi,max(2,int(math.ceil((hi-lo)/step))+1))
    result = [grid]
    for k in LABELS:
        s,data = progress[k]
        result.extend([np.interp(grid,s,data[:,idx]) for idx in (7,4,0,5)])
    return np.column_stack(result), {k: (j*4+1) for j,k in enumerate(LABELS)}

def spatial_rms(s,e):
    if len(s)<2 or s[-1]-s[0]<=0: return float('nan')
    # NumPy >= 2 có trapezoid; NumPy cũ (Anaconda) dùng trapz.
    integrate = np.trapezoid if hasattr(np, 'trapezoid') else np.trapz
    return math.sqrt(float(integrate(e*e,s)/(s[-1]-s[0])))

def plot_route(sims,path,route):
    import matplotlib.pyplot as plt
    from trajectory import draw_map
    fig,ax=plt.subplots(figsize=(9,8),constrained_layout=True)
    first = next(iter(sims.values()))
    draw_map(ax,first)
    # Remove placeholders for vehicle, so overlay focuses only on trajectories.
    for artist in list(ax.lines)[1:]:
        try: artist.remove()
        except ValueError: pass
    for key,sim in sims.items():
        data=np.asarray(sim.rows)
        ax.plot(data[:,1],data[:,2],lw=2.2,color=COLORS[key],label=LABELS[key])
    ax.set_title(f'Quỹ đạo thực tế của 3 đối tượng – {ROUTES[route][0]}')
    ax.legend(loc='best',fontsize=9)
    fig.savefig(path,dpi=165)
    plt.close(fig)

def plot_metrics(sims, aligned, indices, path,route):
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(13,9),constrained_layout=True)
    for key,sim in sims.items():
        dat=np.asarray(sim.rows)
        color=COLORS[key]
        axes[0,0].plot(dat[:,0],dat[:,7],label=LABELS[key],color=color)
        axes[0,1].plot(dat[:,0],dat[:,4],label=LABELS[key],color=color)
        axes[1,0].plot(dat[:,0],dat[:,6],label=LABELS[key],color=color)
        if len(aligned):
            axes[1,1].plot(aligned[:,0],aligned[:,indices[key]],label=LABELS[key],color=color)
    for ax,title,xlabel,ylabel in zip(axes.flat,
            ['Sai số bám đường theo thời gian','Vận tốc thực tế','Vận tốc tham chiếu','Sai số tại cùng tiến độ tuyến s'],
            ['Thời gian (s)']*3+['Tiến độ trên tuyến s (m)'],
            ['Sai số e (m)','Vận tốc (m/s)','Vận tốc tham chiếu (m/s)','Sai số e(s) (m)']):
        ax.set(title=title,xlabel=xlabel,ylabel=ylabel)
        ax.grid(alpha=.25)
        ax.legend(fontsize=7)
    fig.suptitle('Đối chiếu 3 đối tượng – '+ROUTES[route][0])
    fig.savefig(path,dpi=155)
    plt.close(fig)

def plot_score(sims, aligned, indices, path, route):
    import matplotlib.pyplot as plt
    keys=list(LABELS)
    metric_names=['RMS(t) (m)','RMS(ds) (m)','e_max (m)','Thời gian (s)','Cách đích (m)','Vận tốc cuối (m/s)','RMS(s) miền chung (m)']
    vals=[]
    for key in keys:
        m=sims[key].metrics()
        vals.append([m['rms'],m['rms_distance'],m['max_error'],m['duration'],m['goal_distance'],m['final_speed'],
                     spatial_rms(aligned[:,0],aligned[:,indices[key]]) if len(aligned) else np.nan])
    fig,axs=plt.subplots(2,4,figsize=(17,9),constrained_layout=True)
    for i,(ax,name) in enumerate(zip(axs.flat,metric_names)):
        values=[v[i] for v in vals]
        ax.bar(range(3),values,color=[COLORS[k] for k in keys])
        ax.set_xticks(range(3))
        ax.set_xticklabels(
            ['PP + P', 'PP + PID', 'PID + cua'],
            rotation=12
        )
        ax.set_title(name,fontsize=11)
        ymax=max((v for v in values if np.isfinite(v)),default=0.)
        ax.set_ylim(0,max(.001,ymax*1.25))
        for j,value in enumerate(values):
            if np.isfinite(value): ax.text(j,value+max(.001,ymax*.015),f'{value:.3f}',ha='center',fontsize=8)
        ax.grid(axis='y',alpha=.2)
    axs.flat[-1].axis('off')
    fig.suptitle('Chỉ tiêu định lượng – '+ROUTES[route][0])
    fig.savefig(path,dpi=150)
    plt.close(fig)

def create_gif(sim, filepath, caption, color, frames=55):
    """GIF xe chuyển động theo các trạng thái mô phỏng đã lưu; lấy mẫu theo thời gian."""
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.patches import Polygon
    from PIL import Image
    from trajectory import body_polygon
    fig,ax=plt.subplots(figsize=(6.3,5.4),dpi=92)
    ax.set_aspect('equal',adjustable='box')
    for a,b,c,d in sim.corridor.rects:
        from matplotlib.patches import Rectangle
        ax.add_patch(Rectangle((a,c),b-a,d-c,facecolor='#e9eef3',edgecolor='none'))
    from matplotlib.collections import LineCollection
    ax.add_collection(LineCollection(sim.corridor.walls,colors='#64748b',linewidths=1.2))
    vertices=sim.vertices
    ax.plot(vertices[:,0],vertices[:,1],'--',color='#dc2626',lw=1.2,label='Quỹ đạo đặt')
    data=np.asarray(sim.rows)
    trace,=ax.plot([],[],color=color,lw=2,label='Quỹ đạo thực tế')
    vehicle=Polygon(body_polygon(sim.state,sim.p),closed=True,facecolor=color,edgecolor='#111827',alpha=.8)
    ax.add_patch(vehicle)
    a,b,c,d=sim.corridor.bounds
    ax.set_xlim(a-2,b+2);ax.set_ylim(c-2,d+2)
    ax.grid(alpha=.12);ax.legend(fontsize=7,loc='best')
    title=ax.set_title(caption,fontsize=10)
    canvas=FigureCanvasAgg(fig)
    frame_indices=np.unique(np.linspace(0,len(data)-1,min(frames,len(data)),dtype=int))
    frames_out=[]
    from types import SimpleNamespace
    for idx in frame_indices:
        row=data[idx]
        trace.set_data(data[:idx+1,1],data[:idx+1,2])
        vehicle.set_xy(body_polygon(SimpleNamespace(x=row[1],y=row[2],yaw=row[3]),sim.p))
        title.set_text(f'{caption}\nt = {row[0]:.2f} s | v = {row[4]:.2f} m/s | e = {row[7]:.3f} m')
        canvas.draw()
        frames_out.append(Image.frombuffer('RGBA',canvas.get_width_height(),canvas.buffer_rgba(),'raw','RGBA',0,1).convert('RGB'))
    frames_out[0].save(filepath,save_all=True,append_images=frames_out[1:],duration=110,loop=0,optimize=False)
    plt.close(fig)

def write_csv(path,header,rows):
    with open(path,'w',newline='',encoding='utf-8-sig') as f:
        writer=csv.writer(f);writer.writerow(header);writer.writerows(rows)

def run_route(base, route, root, gifs=True):
    folder=Path(root)/route
    folder.mkdir(parents=True,exist_ok=True)
    sims=simulation_set(base,route)
    aligned,idx=common_alignment(sims)
    metrics=[]
    for key,sim in sims.items():
        write_csv(folder/f'{key}.csv',CSV_HEADER,sim.rows)
        m=sim.metrics()
        common_rms=spatial_rms(aligned[:,0],aligned[:,idx[key]]) if len(aligned) else float('nan')
        metrics.append([LABELS[key],m['rms'],m['rms_distance'],m['max_error'],m['duration'],m['goal_distance'],m['final_speed'],common_rms,int(sim.collided),sim.reason])
        if gifs:
            create_gif(sim,folder/f'{key}.gif',LABELS[key],COLORS[key])
    write_csv(folder/'chi_tieu_danh_gia.csv',
        ['Đối tượng','RMS(t) [m]','RMS(ds) [m]','e_max [m]','Thời gian [s]','Khoảng cách đích [m]','Vận tốc cuối [m/s]','RMS(s) miền chung [m]','Vượt biên (0/1)','Trạng thái kết thúc'],metrics)
    if len(aligned):
        write_csv(folder/'doi_chieu_theo_tien_do_s.csv', ['s_reference_m']+[f'{k}_{field}' for k in LABELS for field in ('error_m','speed_m_s','time_s','steer_rad')],aligned)
    plot_route(sims,folder/'01_quy_dao_3_doi_tuong.png',route)
    plot_metrics(sims,aligned,idx,folder/'02_sai_so_va_van_toc.png',route)
    plot_score(sims,aligned,idx,folder/'03_bang_chi_tieu.png',route)
    params={key:{'label':LABELS[key],'groups':{g:{name:getattr(sim.p,name) for name in names} for g,names in GROUPS.items()}} for key,sim in sims.items()}
    (folder/'tham_so.json').write_text(json.dumps({'route':route,'route_label':ROUTES[route][0],'controllers':params},ensure_ascii=False,indent=2),encoding='utf-8')
    lines=[f'BÁO CÁO KỊCH BẢN: {ROUTES[route][0]}','', 'Quy ước: 3 đối tượng có cùng quỹ đạo, pose đầu, hình học, giới hạn và công thức lái dùng khoảng cách thực d.',
           'RMS(t): trung bình mẫu theo thời gian; RMS(ds): RMS tích phân theo quãng đường xe thực đi;',
           'RMS(s): RMS nội suy tại các mốc tiến độ tham chiếu chung, KHÔNG phải RMS(ds).',
           'Nếu xe dừng sớm, phải xem miền s chung trước khi kết luận.','']
    for row in metrics:
        lines += [f'{row[0]}:',f'  RMS(t)={row[1]:.5f} m | RMS(ds)={row[2]:.5f} m | e_max={row[3]:.5f} m',
                  f'  Thời gian={row[4]:.2f} s | Khoảng cách đích={row[5]:.4f} m | v cuối={row[6]:.4f} m/s',
                  f'  RMS(s) trên miền chung={row[7]:.5f} m | Vượt biên={bool(row[8])} | {row[9]}','']
    if len(aligned): lines += [f'Miền s chung: {aligned[0,0]:.3f}–{aligned[-1,0]:.3f} m','']
    lines += ['Các nhóm tham số:']
    for group,names in GROUPS.items():
        lines += [f'  {group}: '+', '.join(f'{key}={getattr(sims["pp_p"].p,key)}' for key in names if key not in ('Kp','Ki','Kd','curvature_speed'))]
    lines += ['', 'Chi tiết cấu hình từng đối tượng xem tham_so.json.']
    (folder/'bao_cao.txt').write_text('\n'.join(lines),encoding='utf-8-sig')
    return metrics

def run_all_experiments(base=None,root='results',gifs=True):
    from parameters import Parameters
    base=base or Parameters()
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    all_rows=[]
    for route in EXPERIMENT_ROUTES:
        print('Đang chạy kịch bản:',ROUTES[route][0],flush=True)
        rows=run_route(base,route,root,gifs=gifs)
        all_rows.extend([[route,*r] for r in rows])
    write_csv(root/'tong_hop_9_luot_chay.csv',
        ['Kịch bản','Đối tượng','RMS(t) [m]','RMS(ds) [m]','e_max [m]','Thời gian [s]',
         'Khoảng cách đích [m]','Vận tốc cuối [m/s]','RMS(s) miền chung [m]','Vượt biên (0/1)','Trạng thái kết thúc'],all_rows)
    (root/'README_KET_QUA.txt').write_text(
        'Mỗi thư mục straight, elbow, zigzag chứa 3 GIF, 3 CSV lượt chạy, CSV chỉ tiêu, CSV đối chiếu theo s, 3 PNG và báo cáo.\n'
        'So sánh công bằng: luôn cùng công thức lái d thực. Bộ PID theo tham số đối tượng, không dùng Ki/Kd nhập từ GUI.\n'
        'Các phương pháp đều dùng cùng bộ điều khiển Pure Pursuit; chỉ thay bộ điều tốc.\n'
        'RMS(s) chỉ được tính trên miền tiến độ s chung của cả 3 đối tượng.\n',encoding='utf-8-sig')
    print('Hoàn tất. Kết quả:',root.resolve(),flush=True)
    return all_rows

if __name__=='__main__':
    import argparse
    from parameters import Parameters
    ap=argparse.ArgumentParser()
    ap.add_argument('--no-gif',action='store_true',help='Bỏ GIF để kiểm thử nhanh')
    ap.add_argument('--output',default='results')
    args=ap.parse_args()
    run_all_experiments(Parameters(),args.output,gifs=not args.no_gif)
