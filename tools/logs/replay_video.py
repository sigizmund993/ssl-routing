# Аргументы: рабочая папка (segments.json и out<N>.txt от replay), путь к mp4.
# Видео 1920×1080, 30 к/с, время лога 1:1: четыре панели — три метода и реальный робот из лога.
# Перед каждым участком 1.5 с стоп-кадр с подписью, после — 2.5 с итогов
import json, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from matplotlib.animation import FFMpegWriter

work, dst = sys.argv[1], sys.argv[2]
segs = json.load(open(f'{work}/segments.json'))
FPS, SIM_HZ, RR, RAD = 30, 60, 90, 180
STEP = SIM_HZ // FPS
# PREVIEW=участок,кадр — вместо видео сохранить png этого кадра на участках до указанного
import os
PREVIEW = tuple(map(int, os.environ['PREVIEW'].split(','))) if os.environ.get('PREVIEW') else None
METHODS = [('SLSQP (минимизатор)', '#ff922b'), ('перебор', '#da77f2'), ('прямая без препятствий', '#e9ecef'),
           ('реальный робот (лог)', '#66d9e8')]
CONTACT = '#ff3b3b'
TEAM = {True: '#fcc419', False: '#4dabf7'}  # жёлтые — номера 0–15
FIELD = '#1f5f35'


def draw_field(ax):
    ax.set_facecolor('#174a29')
    ax.add_patch(Rectangle((-4500, -3000), 9000, 6000, fc=FIELD, ec='white', lw=1.5, alpha=1, zorder=0))
    kw = dict(color='white', lw=1.2, alpha=0.6, zorder=0.5)
    ax.plot([0, 0], [-3000, 3000], **kw)
    ax.add_patch(Circle((0, 0), 500, fill=False, ec='white', lw=1.2, alpha=0.6, zorder=0.5))
    for sx in (-1, 1):
        ax.plot([sx * 4500, sx * 3500, sx * 3500, sx * 4500], [-1000, -1000, 1000, 1000], **kw)
        ax.plot([sx * 4500, sx * 4680, sx * 4680, sx * 4500], [-500, -500, 500, 500], color='white', lw=2, zorder=0.5)


def clearance(pos, real, k):
    others = [q for q in range(32) if q != k]
    d = np.linalg.norm(real[:, others, :2] - pos[:, None, :], axis=2)
    return np.nanmin(d, axis=1) - RAD


fig = plt.figure(figsize=(19.2, 10.8), dpi=100, facecolor='#0b1e12')
writer = FFMpegWriter(fps=FPS, codec='libx264', extra_args=['-pix_fmt', 'yuv420p', '-crf', '20'])
with writer.saving(fig, dst, dpi=100):
    for si, s in enumerate(segs):
        fig.clf()
        k, nf, n = s['k'], s['nf'], s['n']
        real = np.array(s['real'], dtype=float); ball = np.array(s['ball'], dtype=float)
        o = np.loadtxt(f'{work}/out{si}.txt')
        wps = np.array(s['wps'])[:, 1:3]
        # по методам: положения, расчёт, номер точки маршрута, план
        data = []
        for m in range(3):
            r = o[o[:, 1] == m]
            data.append(dict(pos=r[:, 2:4], vel=r[:, 4:6], ms=r[:, 6], cur=r[:, 7].astype(int), plan=r[:, 9:].reshape(-1, 16, 2)))
        data.append(dict(pos=real[:, k, :2], vel=real[:, k, 2:], ms=None, cur=None, plan=None))
        for D in data:
            D['clr'] = clearance(D['pos'], real, k)
            fin = np.where(np.linalg.norm(D['pos'] - wps[-1], axis=1) < 60)[0]
            if D['cur'] is not None:
                fin = fin[D['cur'][fin] == len(wps) - 1]
            D['fin'] = fin[0] if len(fin) else None
        # общий вид: путь реального робота, пути методов, точки маршрута
        pts = np.vstack([D['pos'] for D in data] + [wps])
        x0, y0 = np.nanmin(pts, axis=0) - 900; x1, y1 = np.nanmax(pts, axis=0) + 900
        aspect = 9.3 / 4.75
        cx, cy, w, h = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
        if w / h < aspect: w = h * aspect
        else: h = w / aspect
        team = 'TRAPS' if s['team'] == 'traps' else 'Warthog'
        title = fig.text(0.5, 0.975, '', ha='center', va='top', color='white', fontsize=19, weight='bold')
        fig.text(0.5, 0.006, '★ текущая точка маршрута (точки взяты из пути реального робота), ○ следующие · тонкая линия — план на этом кадре · '
                 'голубой пунктир — реальный робот\nжёлтые — Warthog, синие — TRAPS · '
                 f'красное кольцо — касание (центры ближе {RAD} мм) · остальные роботы проигрываются из лога и нашего робота не видят',
                 ha='center', va='bottom', color='#adb5bd', fontsize=11, linespacing=1.3)
        # подписи — в угол, где меньше всего движения
        fx = (pts[:, 0] - (cx - w / 2)) / w; fy = (pts[:, 1] - (cy - h / 2)) / h
        corners = [(0, 1), (1, 1), (0, 0), (1, 0)]
        busy = [np.sum((np.abs(fx - ux) < 0.42) & (np.abs(fy - uy) < 0.33)) for ux, uy in corners]
        ux, uy = corners[int(np.argmin(busy))]
        ha = 'left' if ux == 0 else 'right'; bx = 0.012 if ux == 0 else 0.988
        axes, art = [], []
        for p in range(4):
            ax = fig.add_axes([0.005 + 0.4975 * (p % 2), 0.052 + 0.459 * (1 - p // 2), 0.4925, 0.452])
            ax.set_xlim(cx - w / 2, cx + w / 2); ax.set_ylim(cy - h / 2, cy + h / 2); ax.set_aspect('equal')
            ax.set_xticks([]); ax.set_yticks([])
            for sp in ax.spines.values(): sp.set_color(METHODS[p][1]); sp.set_linewidth(2.5)
            draw_field(ax)
            name, col = METHODS[p]
            A = {}
            A['others'] = [Circle((0, 0), RR, fc=TEAM[q < 16], ec='#212529', lw=1, zorder=3) for q in range(32)]
            for c in A['others']: ax.add_patch(c)
            A['ball'] = Circle((0, 0), 35, fc='#ff6b00', ec='black', lw=0.8, zorder=4); ax.add_patch(A['ball'])
            A['wps'], = ax.plot(wps[:, 0], wps[:, 1], 'o', mfc='none', mec='white', ms=9, mew=1.5, zorder=2)
            A['wpcur'], = ax.plot([], [], '*', color='gold', mec='black', ms=22, zorder=6)
            A['realtrail'], = ax.plot([], [], '--', color=METHODS[3][1], lw=1.5, alpha=0.8, zorder=2)
            if p < 3:
                A['ghost'] = Circle((0, 0), RR, fill=False, ec=METHODS[3][1], ls='--', lw=1.8, zorder=4); ax.add_patch(A['ghost'])
                A['plan'], = ax.plot([], [], '-', color=col, lw=1.3, alpha=0.9, zorder=5)
            A['trail'], = ax.plot([], [], '-', color=col, lw=2.6, zorder=5)
            A['me'] = Circle((0, 0), RR, fc=col, ec='black', lw=1.5, zorder=7); ax.add_patch(A['me'])
            A['ring'] = Circle((0, 0), RR + 45, fill=False, ec=CONTACT, lw=4, zorder=8, visible=False); ax.add_patch(A['ring'])
            ax.text(bx, 0.975 if uy else 0.27, name, transform=ax.transAxes, va='top' if uy else 'bottom', ha=ha, color=col, fontsize=20, weight='bold', zorder=10,
                    bbox=dict(fc='black', alpha=0.55, ec='none', pad=4))
            A['info'] = ax.text(bx, 0.86 if uy else 0.025, '', transform=ax.transAxes, va='top' if uy else 'bottom', ha=ha, color='white', fontsize=12.5, zorder=10,
                                family='DejaVu Sans Mono', bbox=dict(fc='black', alpha=0.55, ec='none', pad=4))
            A['big'] = ax.text(0.5, 0.5, '', transform=ax.transAxes, ha='center', va='center', color='white', fontsize=24,
                               weight='bold', zorder=11, bbox=dict(fc='black', alpha=0.7, ec='none', pad=10), visible=False)
            axes.append(ax); art.append(A)

        def info(p, f):
            D = data[p]; c = D['clr'][:f + 1]
            fin = D['fin']
            before = c[:fin] if fin is not None and fin <= f else c
            after = c[fin:] if fin is not None and fin <= f else c[:0]
            lines = []
            if D['ms'] is not None:
                ms = D['ms'][:f + 1]
                lines.append(f'расчёт {D["ms"][f]:5.2f} мс  ср {ms.mean():4.2f}  макс {ms.max():5.1f}')
                lines.append(f'точка маршрута {D["cur"][f] + 1}/{len(wps)}')
            sp = np.linalg.norm(D['vel'][f])
            lines.append(f'скорость {sp / 1000:4.2f} м/с')
            lines.append(f'касания в пути  {np.sum(before < 0) / SIM_HZ:4.2f} с' + (f'   мин. зазор {np.nanmin(before):5.0f} мм' if len(before) else ''))
            if fin is not None and fin <= f:
                lines.append(f'доехал за {fin / SIM_HZ:.2f} с' + (f'; стоя: касания {np.sum(after < 0) / SIM_HZ:.2f} с' if np.any(after < 0) else ''))
            return '\n'.join(lines)

        def draw(f, mode):
            t = f / SIM_HZ
            title.set_text(f'{si + 1}/{len(segs)}  {s["cat"]}  ·  {team}, робот {k % 16}  ·  лог: {s["t0"] / 60:.0f} мин {s["t0"] % 60:02.0f} с'
                           f'  ·  t = {t:4.2f} с' + ('   (пауза)' if mode else ''))
            for p, A in enumerate(art):
                D = data[p]
                for q, c in enumerate(A['others']):
                    if q == k or not np.isfinite(real[f, q, 0]): c.set_visible(False)
                    else: c.set_visible(True); c.center = real[f, q, :2]
                if np.isfinite(ball[f, 0]): A['ball'].set_visible(True); A['ball'].center = ball[f]
                else: A['ball'].set_visible(False)
                cur = D['cur'][f] if D['cur'] is not None else int(np.searchsorted(np.array(s['wps'])[:, 0], f))
                cur = min(cur, len(wps) - 1)
                A['wpcur'].set_data([wps[cur, 0]], [wps[cur, 1]])
                A['wps'].set_data(wps[cur + 1:, 0], wps[cur + 1:, 1])
                A['realtrail'].set_data(real[:f + 1, k, 0], real[:f + 1, k, 1])
                if p < 3:
                    A['ghost'].center = real[f, k, :2]
                    A['plan'].set_data(D['plan'][f, :, 0], D['plan'][f, :, 1])
                A['trail'].set_data(D['pos'][:f + 1, 0], D['pos'][:f + 1, 1])
                A['me'].center = D['pos'][f]; A['ring'].center = D['pos'][f]
                hit = D['clr'][f] < 0
                A['ring'].set_visible(bool(hit)); A['me'].set_edgecolor(CONTACT if hit else 'black')
                A['info'].set_text(info(p, f))
                if mode == 'outro':
                    fin = D['fin']
                    c = D['clr'][:fin] if fin is not None else D['clr']
                    A['big'].set_text((f'доехал за {fin / SIM_HZ:.2f} с' if fin is not None else 'не доехал') +
                                      f'\nкасания в пути {np.sum(c < 0) / SIM_HZ:.2f} с')
                    A['big'].set_visible(True)
                else:
                    A['big'].set_visible(False)

        if PREVIEW is not None:
            draw(PREVIEW[1], None); fig.savefig(dst.replace('.mp4', f'_{si}.png'), facecolor=fig.get_facecolor())
            if si == PREVIEW[0]: break
            continue
        draw(0, 'intro')
        cap = fig.text(0.5, 0.53, f'{s["cat"]}\nмаршрут из {len(wps)} точек по пути реального робота', ha='center', va='center',
                       color='white', fontsize=30, weight='bold', zorder=20, bbox=dict(fc='black', alpha=0.75, ec='none', pad=16))
        for _ in range(int(1.5 * FPS)): writer.grab_frame(facecolor=fig.get_facecolor())
        cap.remove()
        for f in range(0, nf, STEP):
            draw(f, None); writer.grab_frame(facecolor=fig.get_facecolor())
        draw(nf - 1, 'outro')
        for _ in range(int(2.5 * FPS)): writer.grab_frame(facecolor=fig.get_facecolor())
        print('участок', si, 'готов', flush=True)
