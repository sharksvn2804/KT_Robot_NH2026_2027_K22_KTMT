"""Tạo tuyến, kiểm tra hành lang và vẽ kết quả mô phỏng."""
import math

import numpy as np

from parameters import DEFAULT_ROUTE, MODE_LABELS, ROUTES


def create_course(leg_length=12.0, step=0.1, route=DEFAULT_ROUTE):
    """Scale the selected axis-aligned route and sample it for target search."""
    vertices = np.asarray(ROUTES[route][1], dtype=float) * leg_length
    pieces = []
    for j, (a, b) in enumerate(zip(vertices[:-1], vertices[1:])):
        n = max(1, math.ceil(np.linalg.norm(b-a)/step))
        points = np.linspace(a, b, n+1)
        pieces.append(points if j == 0 else points[1:])
    path = np.concatenate(pieces)
    return path[:, 0], path[:, 1], vertices

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


def route_projection(sim):
    """Chieu len polyline, s la chieu dai TREN TUYEN tham chieu, khong phai
    quang duong xe da di. Lay cac moc tien trien lan dau de noi suy.
    Neu xe lui tien do, chi giu lan vuot moc dau tien; khong noi suy yaw.
    """
    data = np.asarray(sim.rows)
    a,u = sim.vertices[:-1],np.diff(sim.vertices,axis=0)
    lengths = np.linalg.norm(u,axis=1)
    offsets = np.r_[0,np.cumsum(lengths)[:-1]]
    progress = []
    for point in data[:,1:3]:
        fraction = np.clip(np.sum((point-a)*u,axis=1)/(lengths*lengths),0,1)
        projection = a+fraction[:,None]*u
        j = int(np.argmin(np.linalg.norm(point-projection,axis=1)))
        progress.append(offsets[j]+fraction[j]*lengths[j])
    progress = np.maximum.accumulate(progress)
    unique,indices = np.unique(progress,return_index=True)
    return unique,data[indices]


def align_on_route(baseline,new):
    s0,d0 = route_projection(baseline)
    s1,d1 = route_projection(new)
    lo,hi = max(s0[0],s1[0]),min(s0[-1],s1[-1])
    if hi-lo < 1e-6:
        return np.empty((0,9))
    grid = np.linspace(lo,hi,max(2,min(5000,math.ceil((hi-lo)/.05)+1)))
    # Khong extrapolate: chi tinh trong mien ca hai da di qua.
    e0,e1 = np.interp(grid,s0,d0[:,7]),np.interp(grid,s1,d1[:,7])
    v0,v1 = np.interp(grid,s0,d0[:,4]),np.interp(grid,s1,d1[:,4])
    x0,x1 = np.interp(grid,s0,d0[:,1]),np.interp(grid,s1,d1[:,1])
    y0,y1 = np.interp(grid,s0,d0[:,2]),np.interp(grid,s1,d1[:,2])
    gap = np.hypot(x1-x0,y1-y0)
    return np.column_stack((grid,e0,e1,e1-e0,v0,v1,v1-v0,gap,
                            np.interp(grid,s1,d1[:,0])-np.interp(grid,s0,d0[:,0])) )


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
    ax.set(xlim=(a-2,b+2),ylim=(c-2,d+2),xlabel='x [m]',ylabel='y [m]',
           title=f'Hành lang · {ROUTES[sim.p.route][0]}')
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


def sample_plot_figure(sim,title,time_limit,error_limit,color):
    from matplotlib.figure import Figure
    fig = Figure(figsize=(12,8),dpi=120,constrained_layout=True)
    gs = fig.add_gridspec(2,2,width_ratios=[1,1.2])
    ax = fig.add_subplot(gs[:,0])
    artists = draw_map(ax,sim)
    update_vehicle(artists,sim)
    artists[0].set_color(color)
    ax.set_title(f'{title}\n{ROUTES[sim.p.route][0]}',fontsize=11)
    data = np.asarray(sim.rows)
    av,ae = fig.add_subplot(gs[0,1]),fig.add_subplot(gs[1,1])
    av.plot(data[:,0],data[:,4],color=color,label='v thuc')
    av.plot(data[:,0],data[:,6],'--',color='#f97316',label='v dat')
    av.set(xlabel='t [s]',ylabel='v [m/s]',title='Van toc',xlim=(0,time_limit),
           ylim=(0,sim.p.cruise_speed*1.15))
    av.legend()
    ae.plot(data[:,0],data[:,7],color=color)
    ae.set(xlabel='t [s]',ylabel='e [m]',title='Sai so toi tuyen',
           xlim=(0,time_limit),ylim=(0,error_limit))
    for axis in (av,ae): axis.grid(alpha=.25)
    m = sim.metrics()
    note = (f'{sim.reason} | Giam toc cua: {sim.p.curvature_speed} | '
            f'Vuot bien: {sim.collided}\n'
            f'RMS(t)={m["rms"]:.4f} m | RMS(ds)={m["rms_distance"]:.4f} m | '
            f'e_max={m["max_error"]:.4f} m | t={m["duration"]:.2f} s\n'
            f'd_dich={m["goal_distance"]:.3f} m | v_cuoi={m["final_speed"]:.3f} m/s')
    fig.suptitle(note,fontsize=10)
    return fig


def difference_figure(aligned,mode):
    from matplotlib.figure import Figure
    fig = Figure(figsize=(12,8),dpi=120,constrained_layout=True)
    axes = fig.subplots(2,2)
    if len(aligned) == 0:
        for ax in axes.flat:
            ax.text(.5,.5,'Khong du tien do chung de so sanh',ha='center',transform=ax.transAxes)
        return fig
    s = aligned[:,0]
    ax = axes[0,0]
    ax.plot(s,aligned[:,1],color='#2563eb',label='Doi chung')
    ax.plot(s,aligned[:,2],color='#16a34a',label='Ban moi')
    ax.set(title='Sai so tai cung tien do tuyen',ylabel='e [m]')
    ax.legend()
    ax = axes[0,1]
    ax.plot(s,aligned[:,3],color='#9333ea')
    ax.axhline(0,color='black',lw=.7)
    ax.set(title='Delta e = e_moi - e_doi_chung',ylabel='Delta e [m]')
    ax.text(.02,.97,'Am: ban moi bam sat tuyen hon tai moc s nay',
            transform=ax.transAxes,va='top',fontsize=8)
    ax = axes[1,0]
    ax.plot(s,aligned[:,6],color='#ea580c')
    ax.axhline(0,color='black',lw=.7)
    ax.set(title='Delta v = v_moi - v_doi_chung',ylabel='Delta v [m/s]')
    ax = axes[1,1]
    ax.plot(s,aligned[:,7],color='#0891b2')
    ax.set(title='Khoang cach hai vi tri tai cung moc s',ylabel='||p_moi - p_doi_chung|| [m]')
    for ax in axes.flat:
        ax.set_xlabel('Tien do chieu len tuyen s [m]')
        ax.set_xlim(s[0],s[-1])
        ax.grid(alpha=.25)
    fig.suptitle(MODE_LABELS[mode]+f'\nMien chung: {s[0]:.2f} - {s[-1]:.2f} m; '
                 'noi suy lan di qua dau tien, khong so tai cung thoi gian',fontsize=11)
    return fig
