# Анимация симуляции нескольких своих роботов (multi-app с DUMP=файл).
# Аргументы: файл DUMP, путь к mp4 [номер сцены = 0]. Четыре панели — четыре способа учитывать друг друга.
# SLOW — во сколько раз замедлить (по умолчанию 2), видео обрезается через 0.7 с после того, как все встали.
# Рядом с mp4 сохраняется png последнего кадра (следы целиком)
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.animation import FFMpegWriter

src, dst = sys.argv[1], sys.argv[2]
scene = int(sys.argv[3]) if len(sys.argv) > 3 else 0
SIM_HZ, FPS, RR = 60, 30, 100  # RR = ROBOT_R, контакт — расстояние между центрами меньше 2·RR
slow = float(os.environ.get('SLOW', 2))
MODES = ['не видят друг друга', 'как чужие (постоянная скорость)', 'траектории (приоритет по номеру)', 'строгий приоритет']
COLORS = ['#e8590c', '#1c7ed6', '#2b8a3e', '#ae3ec9', '#f08c00', '#0c8599', '#c2255c', '#5c940d']

rows = [list(map(float, l.split())) for l in open(src)]
data = {}
for r in rows:
    if int(r[0]) == scene:
        data.setdefault(int(r[1]), []).append(r[3:])
pos = {m: np.array(v).reshape(len(v), -1, 2) for m, v in data.items()}
n = next(iter(pos.values())).shape[1]

# конец: последний кадр, где хоть кто-то сдвинулся больше чем на 0.5 мм, плюс 0.7 с
last = 0
for p in pos.values():
    moved = np.where(np.abs(np.diff(p, axis=0)).max(axis=(1, 2)) > 0.5)[0]
    last = max(last, (moved[-1] + 1) if len(moved) else 0)
nFrames = min(len(next(iter(pos.values()))), last + int(0.7 * SIM_HZ))
step = SIM_HZ / (FPS * slow)
frames = [min(nFrames - 1, int(round(i * step))) for i in range(int(nFrames / step) + 1)]

allp = np.concatenate([p[:nFrames].reshape(-1, 2) for p in pos.values()])
lo, hi = allp.min(axis=0) - 350, allp.max(axis=0) + 350
fig, axes = plt.subplots(2, 2, figsize=(12.8, 12.8 * (hi[1] - lo[1]) / (hi[0] - lo[0]) + 0.9), dpi=100)
panels = []
for ax, m in zip(axes.flat, sorted(pos)):
    p = pos[m]
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect('equal')
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.25)
    trails, bodies, labels = [], [], []
    for i in range(n):
        c = COLORS[i % len(COLORS)]
        # старт и цель (цель — где робот встал: в DUMP целей нет)
        ax.add_patch(Circle(p[0, i], RR, fc='none', ec=c, ls=':', lw=1, alpha=0.6))
        ax.add_patch(Circle(p[nFrames - 1, i], RR, fc='none', ec=c, ls='--', lw=1.2))
        trails.append(ax.plot([], [], color=c, lw=1.6)[0])
        b = Circle(p[0, i], RR, fc=c, ec='black', lw=1, alpha=0.9, zorder=3)
        ax.add_patch(b)
        bodies.append(b)
        labels.append(ax.text(*p[0, i], str(i), ha='center', va='center', color='white', fontsize=9, weight='bold', zorder=4))
    # две строки сразу, чтобы tight_layout оставил под заголовок место
    title = ax.set_title('-\n-', fontsize=10, loc='left')
    panels.append((m, ax, trails, bodies, labels, title))


def pairDist(p):
    d = np.linalg.norm(p[:, None, :] - p[None, :, :], axis=2)
    d[np.arange(n), np.arange(n)] = 1e9
    return d


minSoFar = {m: np.minimum.accumulate([pairDist(pos[m][f]).min() for f in range(nFrames)]) for m in pos}
# время приезда каждого робота: последний кадр, где он ещё двигался
arrive = {}
for m, p in pos.items():
    sp = np.linalg.norm(np.diff(p[:nFrames], axis=0), axis=2)
    arrive[m] = [((np.where(sp[:, i] > 0.5)[0][-1] + 2) / SIM_HZ if (sp[:, i] > 0.5).any() else 0) for i in range(n)]
fig.suptitle(f'{n} роботов на круге, каждый едет в противоположную точку (замедлено в {slow:g} раза)\n'
             'пунктир — старт, штрих — цель, красная обводка — контакт (центры ближе 2·ROBOT_R = 200 мм)', fontsize=11)
fig.tight_layout(rect=(0, 0, 1, 0.95))


def draw(f):
    for m, ax, trails, bodies, labels, title in panels:
        p = pos[m]
        d = pairDist(p[f])
        for i in range(n):
            trails[i].set_data(p[:f + 1, i, 0], p[:f + 1, i, 1])
            bodies[i].center = p[f, i]
            hit = d[i].min() < 2 * RR - 1
            bodies[i].set_edgecolor('red' if hit else 'black')
            bodies[i].set_linewidth(3 if hit else 1)
            labels[i].set_position(p[f, i])
        done = f >= nFrames - 1
        gap = minSoFar[m][f] - 2 * RR
        title.set_text(f'{MODES[m]}\nt = {f / SIM_HZ:.2f} с   мин. зазор {gap:.0f} мм' +
                       (f'   приезд ср {np.mean(arrive[m]):.2f} с, последний {max(arrive[m]):.2f} с' if done else ''))
        title.set_color('red' if gap < -1 else 'black')


writer = FFMpegWriter(fps=FPS, bitrate=4000)
with writer.saving(fig, dst, dpi=100):
    for f in frames:
        draw(f)
        writer.grab_frame()
draw(nFrames - 1)
fig.savefig(os.path.splitext(dst)[0] + '.png', dpi=100)
print(f'кадров {len(frames)}, длительность {len(frames) / FPS:.1f} с, сим {nFrames / SIM_HZ:.2f} с')
for m in sorted(pos):
    print(f'{MODES[m]:35s} мин. зазор {minSoFar[m][-1] - 2 * RR:6.0f} мм, приезд ср {np.mean(arrive[m]):.2f} с, max {max(arrive[m]):.2f} с')
