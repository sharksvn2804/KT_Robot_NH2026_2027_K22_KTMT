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
    'Vehicle Geometry': ['WB','vehicle_width','max_steer_deg'],
    'Environment': ['corridor_width','leg_length','stop_on_collision'],
    'Pure Pursuit': ['Lfc','k'],
    'P/PID controller': ['Kp','Ki','Kd','max_accel','max_decel'],
    'Slow down while turing': ['curvature_speed','regulation_radius','min_speed'],
    'Slow down when reaching goal': ['goal_slowdown_gain'],
    'completion time / Velocity': ['cruise_speed','dt','max_time'],
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
    ax.set_title(f'Actual Trajectories – {ROUTES[route][0]}')
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
    [
        'Cross-Track Error vs. Time',
        'Actual Linear Velocity',
        'Reference Linear Velocity',
        'Cross-Track Error vs. Route Progress s'
    ],            
    ['Time (s)']*3+['Route Progress s (m)'],
    ['Tracking Error e (m)', 'Linear Velocity (m/s)', 'Reference Velocity (m/s)', 'Tracking Error e(s) (m)']           
    ):
        ax.set(title=title,xlabel=xlabel,ylabel=ylabel)
        ax.grid(alpha=.25)
        ax.legend(fontsize=7)
    fig.suptitle('Controller Configuration Comparision– '+ROUTES[route][0])
    fig.savefig(path,dpi=155)
    plt.close(fig)

def plot_score(sims, aligned, indices, path, route):
    import matplotlib.pyplot as plt
    keys = list(LABELS)
    metric_names = [
        'Common-Domain RMS(s) (m)', 'RMS(ds) (m)', 'e_max (m)',
        'completion time (s)', 'd_goal (m)', 'v_end (m/s)'
    ]
    vals = []
    for key in keys:
        m = sims[key].metrics()
        common_rms = spatial_rms(aligned[:, 0], aligned[:, indices[key]]) if len(aligned) else np.nan
        vals.append([
            common_rms, m['rms_distance'], m['max_error'],
            m['completion_time'], m['goal_distance'], m['final_speed']
        ])
    fig, axs = plt.subplots(2, 3, figsize=(17, 9), constrained_layout=True)
    for i, (ax, name) in enumerate(zip(axs.flat, metric_names)):
        values = [v[i] for v in vals]
        ax.bar(range(3), values, color=[COLORS[k] for k in keys])
        ax.set_xticks(range(3))
        ax.set_xticklabels(
            ['PP + Kp only', 'PP + PID', 'RPP + PID'],
            rotation=12
        )
        ax.set_title(name, fontsize=11)
        ymax = max((v for v in values if np.isfinite(v)), default=0.)
        ax.set_ylim(0, max(.001, ymax * 1.25))
        for j, value in enumerate(values):
            if np.isfinite(value):
                ax.text(j, value + max(.001, ymax * .015), f'{value:.3f}', ha='center', fontsize=8)
        ax.grid(axis='y', alpha=.2)
    fig.suptitle('Quantitative Performance Metrics – ' + ROUTES[route][0])
    fig.savefig(path, dpi=150)
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
    ax.plot(vertices[:,0],vertices[:,1],'--',color='#dc2626',lw=1.2,label='Reference Trajectory')
    data=np.asarray(sim.rows)
    trace,=ax.plot([],[],color=color,lw=2,label='Actual Trajectory')
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
    frames_out[0].save(filepath,save_all=True,append_images=frames_out[1:],duration =110,loop=0,optimize=False)
    plt.close(fig)

def write_csv(path,header,rows):
    with open(path,'w',newline='',encoding='utf-8-sig') as f:
        writer=csv.writer(f);writer.writerow(header);writer.writerows(rows)

def run_route(base, route, root, gifs=True):
    folder = Path(root) / route
    folder.mkdir(parents=True, exist_ok=True)
    sims = simulation_set(base, route)
    aligned, idx = common_alignment(sims)
    metrics = []
    for key, sim in sims.items():
        write_csv(folder / f'{key}.csv', CSV_HEADER, sim.rows)
        m = sim.metrics()
        common_rms = spatial_rms(aligned[:, 0], aligned[:, idx[key]]) if len(aligned) else float('nan')
        metrics.append([
            LABELS[key], common_rms, m['rms_distance'], m['max_error'],
            m['completion_time'], m['goal_distance'], m['final_speed'],
            int(sim.collided), sim.reason
        ])
        if gifs:
            create_gif(sim, folder / f'{key}.gif', LABELS[key], COLORS[key])

    write_csv(
        folder / 'chi_tieu_danh_gia.csv',
        [
            'Controller', 'Common-Domain Spatial RMS Error(s) [m]', 'RMS(ds) [m]',
            'e_max [m]', 'completion time [s]', 'd_goal [m]', 'v_end [m/s]',
            'Has collision (0/1)', 'Finish State'
        ],
        metrics
    )
    if len(aligned):
        write_csv(
            folder / 'doi_chieu_theo_tien_do_s.csv',
            ['s_reference_m'] + [f'{k}_{field}' for k in LABELS for field in ('error_m', 'speed_m_s', 'time_s', 'steer_rad')],
            aligned
        )
    plot_route(sims, folder / '01_quy_dao_3_doi_tuong.png', route)
    plot_metrics(sims, aligned, idx, folder / '02_sai_so_va_van_toc.png', route)
    plot_score(sims, aligned, idx, folder / '03_bang_chi_tieu.png', route)

    params = {
        key: {
            'label': LABELS[key],
            'groups': {g: {name: getattr(sim.p, name) for name in names} for g, names in GROUPS.items()}
        }
        for key, sim in sims.items()
    }
    (folder / 'tham_so.json').write_text(
        json.dumps({'route': route, 'route_label': ROUTES[route][0], 'controllers': params}, ensure_ascii=False, indent=2),
        encoding='utf-8'
    )

    lines = [
        f'BÁO CÁO KỊCH BẢN: {ROUTES[route][0]}', '',
        'Quy ước: 3 đối tượng có cùng quỹ đạo, pose đầu, hình học, giới hạn và công thức lái dùng khoảng cách thực d.',
        'Common-Domain Spatial RMS Error(s): RMS nội suy tại các mốc tiến độ tham chiếu chung;',
        'RMS(ds): RMS tích phân theo quãng đường thực tế xe lăn bánh.',
        'Nếu xe dừng sớm, phải xem miền s chung trước khi kết luận.', ''
    ]
    for row in metrics:
        lines += [
            f'{row[0]}:',
            f'  Common-Domain RMS(s)={row[1]:.5f} m | RMS(ds)={row[2]:.5f} m | e_max={row[3]:.5f} m',
            f'  completion time={row[4]:.2f} s | d_goal={row[5]:.4f} m | v_end={row[6]:.4f} m/s',
            f'  Has collision={bool(row[7])} | {row[8]}', ''
        ]
    if len(aligned):
        lines += [f'Miền s chung: {aligned[0, 0]:.3f}–{aligned[-1, 0]:.3f} m', '']
    lines += ['Các nhóm tham số:']
    for group, names in GROUPS.items():
        lines += [
            f'  {group}: ' + ', '.join(
                f'{key}={getattr(sims["pp_p"].p, key)}'
                for key in names if key not in ('Kp', 'Ki', 'Kd', 'curvature_speed')
            )
        ]
    lines += ['', 'Chi tiết cấu hình từng đối tượng xem tham_so.json.']
    (folder / 'bao_cao.txt').write_text('\n'.join(lines), encoding='utf-8-sig')
    return metrics

def export_excel_report(all_rows, base, filepath):
    """Xuất tệp Excel gồm 3 sheet: Bảng tổng hợp theo kịch bản, Dữ liệu 9 lượt chạy và Tham số cấu hình."""
    from openpyxl import Workbook
    from openpyxl.styles import Font, PatternFill, Alignment, Border, Side

    wb = Workbook()

    # --- Định nghĩa bộ kiểu dáng chuẩn báo cáo kỹ thuật (Times New Roman) ---
    font_title = Font(name='Times New Roman', size=13, bold=True, color='1F4E78')
    font_header = Font(name='Times New Roman', size=11, bold=True, color='FFFFFF')
    font_scenario = Font(name='Times New Roman', size=11, bold=True, color='111827')
    font_regular = Font(name='Times New Roman', size=11, bold=False, color='111827')
    font_best = Font(name='Times New Roman', size=11, bold=True, color='0F5132')
    font_ok = Font(name='Times New Roman', size=11, bold=True, color='15803D')
    font_fail = Font(name='Times New Roman', size=11, bold=True, color='B91C1C')

    fill_header = PatternFill(start_color='1F4E78', end_color='1F4E78', fill_type='solid')
    fill_alt_1 = PatternFill(start_color='FFFFFF', end_color='FFFFFF', fill_type='solid')
    fill_alt_2 = PatternFill(start_color='F4F7FA', end_color='F4F7FA', fill_type='solid')

    thin_gray = Side(border_style='thin', color='D1D5DB')
    med_navy = Side(border_style='medium', color='1F4E78')

    border_header = Border(top=med_navy, bottom=med_navy, left=thin_gray, right=thin_gray)
    border_cell = Border(top=thin_gray, bottom=thin_gray, left=thin_gray, right=thin_gray)
    border_group_end = Border(top=thin_gray, bottom=med_navy, left=thin_gray, right=thin_gray)

    align_center = Alignment(horizontal='center', vertical='center', wrap_text=True)
    align_left = Alignment(horizontal='left', vertical='center', wrap_text=True)

    # =========================================================================
    # SHEET 1: BẢNG TỔNG HỢP ĐỐI CHIẾU THEO KỊCH BẢN (GIỐNG BẢNG 4.1)
    # =========================================================================
    ws1 = wb.active
    ws1.title = 'Bang_Tong_Hop_KPI'

    ws1.merge_cells('A1:E1')
    ws1['A1'] = 'Quantitative Performance Comparison of the Three Control Configurations Across Path-Tracking Scenarios'
    ws1['A1'].font = font_title
    ws1['A1'].alignment = align_left
    ws1.row_dimensions[1].height = 26

    ctrl_labels = [LABELS[k] for k in LABELS]  # ['PP + Kp only', 'PP + PID', 'RPP + PID']
    headers_s1 = ['Scenarios', 'Performance Metric (KPI)', *ctrl_labels]
    ws1.append(headers_s1)
    ws1.row_dimensions[2].height = 24
    for col_idx in range(1, 6):
        cell = ws1.cell(row=2, column=col_idx)
        cell.font = font_header
        cell.fill = fill_header
        cell.alignment = align_center
        cell.border = border_header

    # Cấu trúc mỗi dòng trong all_rows:
    # [0:route, 1:label, 2:common_rms, 3:rms_ds, 4:e_max, 5:completion time, 6:d_goal, 7:v_end, 8:collided, 9:reason]
    kpi_specs = [
        ('Common-Domain RMS(s) [m]', 2, '0.000', True),
        ('RMS(ds) [m]', 3, '0.000', True),
        ('e_max [m]', 4, '0.000', True),
        ('Completion Time [s]', 5, '0.00', True),
        ('d_goal [m]', 6, '0.000', False),
        ('v_end [m/s]', 7, '0.000', False),
        ('Outcome/ Wall Collision', None, '@', False),
    ]

    current_row = 3
    for s_idx, route in enumerate(EXPERIMENT_ROUTES):
        route_rows = [r for r in all_rows if r[0] == route]
        by_ctrl = {r[1]: r for r in route_rows}
        row_fill = fill_alt_1 if s_idx % 2 == 0 else fill_alt_2
        start_row = current_row

        for k_idx, (kpi_name, data_col, num_fmt, highlight_min) in enumerate(kpi_specs):
            is_last_kpi = (k_idx == len(kpi_specs) - 1)
            row_border = border_group_end if is_last_kpi else border_cell

            # Cột 2: Tên KPI
            c_kpi = ws1.cell(row=current_row, column=2, value=kpi_name)
            c_kpi.font = font_regular
            c_kpi.fill = row_fill
            c_kpi.alignment = align_left
            c_kpi.border = row_border

            # Tìm giá trị nhỏ nhất (tốt nhất) trong các cấu hình không bị Collision (Yes)
            best_val = None
            if highlight_min:
                valid_vals = [
                    by_ctrl[lbl][data_col]
                    for lbl in ctrl_labels
                    if lbl in by_ctrl and by_ctrl[lbl][8] == 0 and math.isfinite(by_ctrl[lbl][data_col])
                ]
                if valid_vals:
                    best_val = min(valid_vals)

            # Cột 3 -> 5: Giá trị của 3 bộ điều khiển
            for c_offset, lbl in enumerate(ctrl_labels):
                cell = ws1.cell(row=current_row, column=3 + c_offset)
                cell.fill = row_fill
                cell.alignment = align_center
                cell.border = row_border

                if lbl not in by_ctrl:
                    cell.value = 'N/A'
                    cell.font = font_regular
                    continue

                rec = by_ctrl[lbl]
                if data_col is not None:
                    val = rec[data_col]
                    if val is None or not math.isfinite(val):
                        cell.value = 'N/A'
                        cell.font = font_regular
                    else:
                        cell.value = round(float(val), 3 if num_fmt == '0.000' else 2)
                        cell.number_format = num_fmt
                        if best_val is not None and abs(val - best_val) <= 1e-9 and rec[8] == 0:
                            cell.font = font_best
                        else:
                            cell.font = font_regular
                else:
                    # Cột trạng thái kết thúc & Collision (Yes)
                    collided = bool(rec[8])
                    reason = str(rec[9])
                    if collided:
                        cell.value = 'Collision (Yes)'
                        cell.font = font_fail
                    elif 'Goal Reached' in reason:
                        cell.value = 'Goal Reached (No)'
                        cell.font = font_ok
                    else:
                        cell.value = f'{reason} (No)'
                        cell.font = font_fail

            ws1.row_dimensions[current_row].height = 20
            current_row += 1

        # Gộp ô cột Kịch bản bên trái
        end_row = current_row - 1
        ws1.merge_cells(start_row=start_row, start_column=1, end_row=end_row, end_column=1)
        c_scen = ws1.cell(row=start_row, column=1, value=f'Scenario {s_idx + 1}:\n{ROUTES[route][0]}')
        c_scen.font = font_scenario
        c_scen.alignment = align_center
        for r_i in range(start_row, end_row + 1):
            ws1.cell(row=r_i, column=1).fill = row_fill
            ws1.cell(row=r_i, column=1).border = border_group_end if r_i == end_row else border_cell

    for col_letter, width in zip(['A', 'B', 'C', 'D', 'E'], [26, 32, 18, 18, 18]):
        ws1.column_dimensions[col_letter].width = width
    ws1.freeze_panes = 'A3'

    # =========================================================================
    # SHEET 2: DỮ LIỆU PHẲNG 9 LƯỢT CHẠY (PHỤC VỤ LỌC & VẼ BIỂU ĐỒ)
    # =========================================================================
    ws2 = wb.create_sheet(title='Du_Lieu_Chi_Tiet')
    headers_s2 = [
        'Scenario', 'Controller Configuration', 'Common-Domain RMS(s) [m]',
        'RMS(ds) [m]', 'e_max [m]', 'completion time [s]', 'd_goal [m]',
        'v_end [m/s]', 'Collision (Yes) (0/1)', 'Trạng thái chi tiết'
    ]
    ws2.append(headers_s2)
    ws2.row_dimensions[1].height = 24
    for col_idx in range(1, len(headers_s2) + 1):
        c = ws2.cell(row=1, column=col_idx)
        c.font = font_header
        c.fill = fill_header
        c.alignment = align_center
        c.border = border_header

    for r_idx, r in enumerate(all_rows, start=2):
        clean_row = [
            ROUTES[r[0]][0], r[1],
            None if not math.isfinite(r[2]) else round(float(r[2]), 5),
            round(float(r[3]), 5), round(float(r[4]), 5),
            round(float(r[5]), 2), round(float(r[6]), 4),
            round(float(r[7]), 4), int(r[8]), r[9]
        ]
        ws2.append(clean_row)
        ws2.row_dimensions[r_idx].height = 20
        for c_idx in range(1, len(headers_s2) + 1):
            c = ws2.cell(row=r_idx, column=c_idx)
            c.font = font_regular
            c.border = border_cell
            c.alignment = align_left if c_idx in (1, 2, 10) else align_center
            if c_idx in (3, 4, 5, 7, 8) and c.value is not None:
                c.number_format = '0.000'
            elif c_idx == 6:
                c.number_format = '0.00'

    for i, w in enumerate([24, 18, 25, 14, 14, 14, 14, 14, 14, 34], start=1):
        ws2.column_dimensions[ws2.cell(row=1, column=i).column_letter].width = w
    ws2.freeze_panes = 'A2'

    # =========================================================================
    # SHEET 3: THAM SỐ CẤU HÌNH THÍ NGHIỆM
    # =========================================================================
    ws3 = wb.create_sheet(title='Tham_So_Cau_Hinh')
    ws3.append(['Nhóm tham số', 'Tên tham số', 'Giá trị chung', 'Ghi chú cấu hình'])
    for col_idx in range(1, 5):
        c = ws3.cell(row=1, column=col_idx)
        c.font = font_header
        c.fill = fill_header
        c.alignment = align_center
        c.border = border_header

    p_row = 2
    for group_name, param_names in GROUPS.items():
        for p_name in param_names:
            val = getattr(base, p_name)
            note = 'Thay đổi theo từng cấu hình (Kp / PID / RPP)' if p_name in ('Kp', 'Ki', 'Kd', 'curvature_speed') else 'Cố định chung cho cả 3 đối tượng'
            ws3.append([group_name, p_name, str(val) if isinstance(val, bool) else val, note])
            for col_idx in range(1, 5):
                c = ws3.cell(row=p_row, column=col_idx)
                c.font = font_regular
                c.border = border_cell
                c.alignment = align_center if col_idx == 3 else align_left
            p_row += 1

    for col_letter, w in zip(['A', 'B', 'C', 'D'], [22, 22, 16, 42]):
        ws3.column_dimensions[col_letter].width = w

    wb.save(filepath)

def run_all_experiments(base=None, root='results', gifs=True):
    from parameters import Parameters
    base = base or Parameters()
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    all_rows = []
    for route in EXPERIMENT_ROUTES:
        print('Đang chạy kịch bản:', ROUTES[route][0], flush=True)
        rows = run_route(base, route, root, gifs=gifs)
        all_rows.extend([[route, *r] for r in rows])
    write_csv(
        root / 'tong_hop_9_luot_chay.csv',
        [
            'Scenario', 'Controller', 'Common-Domain Spatial RMS Error(s) [m]',
            'RMS(ds) [m]', 'e_max [m]', 'completion time [s]', 'd_goal [m]',
            'v_end [m/s]', 'Has collision (0/1)', 'Finish State'
        ],
        all_rows
    )

    # Xuất thêm tệp Excel định dạng chuẩn học thuật
    excel_path = root / 'bao_cao_tong_hop_ket_qua.xlsx'
    export_excel_report(all_rows, base, excel_path)

    print('Hoàn tất. Kết quả Excel:', excel_path.resolve(), flush=True)
    
    (root / 'README_KET_QUA.txt').write_text(
        'Mỗi thư mục straight, elbow, zigzag chứa 3 GIF, 3 CSV lượt chạy, CSV chỉ tiêu, CSV đối chiếu theo s, 3 PNG và báo cáo.\n'
        'So sánh công bằng: luôn cùng công thức lái d thực. Bộ PID theo tham số đối tượng, không dùng Ki/Kd nhập từ GUI.\n'
        'Các phương pháp đều dùng cùng bộ điều khiển Pure Pursuit; chỉ thay bộ điều tốc.\n'
        'Common-Domain Spatial RMS Error(s) chỉ được tính trên miền tiến độ s chung của cả 3 đối tượng.\n',
        encoding='utf-8-sig'
    )
    print('Hoàn tất. Kết quả:', root.resolve(), flush=True)
    return all_rows

if __name__=='__main__':
    import argparse
    from parameters import Parameters
    ap=argparse.ArgumentParser()
    ap.add_argument('--no-gif',action='store_true',help='Bỏ GIF để kiểm thử nhanh')
    ap.add_argument('--output',default='results')
    args=ap.parse_args()
    run_all_experiments(Parameters(),args.output,gifs=not args.no_gif)
