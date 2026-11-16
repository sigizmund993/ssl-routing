# Аргументы: game.npz, рабочая папка. Выбирает в логе участки по 8 с с разными ситуациями и пишет для каждого
# вход replay (seg<N>.txt) и описание (segments.json). Кадры — сетка 60 Гц по ближайшему кадру трекера.
# Маршрут — остановки реального робота (< 0.15 м/с) и точки не реже чем через 2 с, у последней скорость 0.
# Третий аргумент — сколько участков брать в каждой категории (по умолчанию 1, для видео). При нескольких участках
# положения всех роботов в segments.json не пишутся (они нужны только видео), зато для реального робота считается
# та же статистика контактов, что считает replay (real_*), — её читает replay_stats.py
import numpy as np, sys, json
d = np.load(sys.argv[1]); out = sys.argv[2]
COUNT = int(sys.argv[3]) if len(sys.argv) > 3 else 1
t, cmd, R, B = d['t'], d['cmd'], d['robots'], d['ball']
run = np.isin(cmd, [2, 3, 8, 9])
W, DT, RAD = 8.0, 1 / 60, 180.0
TEAMS = {'warthog': (range(0, 16), 1900.0), 'traps': (range(16, 32), 2800.0)}

# сетка 60 Гц на всём логе
grid = np.arange(t[0], t[-1], DT)
idx = np.clip(np.searchsorted(t, grid), 0, len(t) - 1)
ok = run[idx] & (np.abs(t[idx] - grid) < 0.05)
Rg, Bg = R[idx], B[idx]
n = int(W / DT)
cands = []
for team, (ks, vmax) in TEAMS.items():
    for k in ks:
        pos = Rg[:, k, :2]; vel = Rg[:, k, 2:]
        present = np.isfinite(pos[:, 0])
        good = ok & present
        # префиксные суммы, чтобы быстро проверять окно целиком
        cs = np.r_[0, np.cumsum(~good)]
        speed = np.linalg.norm(vel, axis=1)
        others = [o for o in range(32) if o != k]
        dist = np.linalg.norm(Rg[:, others, :2] - pos[:, None, :], axis=2)
        rel = np.linalg.norm(Rg[:, others, 2:] - vel[:, None, :], axis=2)
        dball = np.linalg.norm(Bg[:, :2] - pos, axis=1)
        opp = [o for o in range(32) if (o < 16) != (k < 16)]
        dopp = np.nanmin(np.linalg.norm(Rg[:, opp, :2] - pos[:, None, :], axis=2), axis=1)
        for s in range(0, len(grid) - n, 30):
            if cs[s + n] - cs[s] > 0:
                continue
            sl = slice(s, s + n)
            p = pos[sl]
            travel = np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1))
            if travel < 2500:
                continue
            moving = speed[sl] > 800
            crowd = np.nansum((dist[sl] < 700) & moving[:, None]) * DT
            cross = np.nansum((dist[sl] < 600) & (rel[sl] > 1500)) * DT
            ball = np.mean((dball[sl] < 400) & (dopp[sl] < 500))
            cands.append(dict(team=team, k=k, s=s, travel=travel, crowd=crowd, cross=cross, ball=ball,
                              span=float(np.linalg.norm(p[-1] - p[0]))))
print('кандидатов', len(cands))

cats = [('Длинные перемещения', lambda c: c['travel'] - 3000 * c['crowd']),
        ('Через толпу', lambda c: c['crowd']),
        ('Встречные роботы', lambda c: c['cross']),
        ('Борьба у мяча', lambda c: c['ball'] + c['travel'] / 1e5)]
chosen = []
for name, key in cats:
    got = 0
    for c in sorted(cands, key=key, reverse=True):
        # участки не должны пересекаться по времени у одного робота (для видео — вообще)
        if all(abs(c['s'] - o['s']) > 2 * n or (COUNT > 1 and c['k'] != o['k']) for o in chosen):
            c = dict(c, cat=name); chosen.append(c); got += 1
            if got >= COUNT: break
segs = []
for i, c in enumerate(chosen):
    k, s = c['k'], c['s']
    nf = n + 90  # +1.5 с, чтобы медленный метод успел доехать
    pos = Rg[s:s + nf, k, :2]; vel = Rg[s:s + nf, k, 2:]
    vmax = TEAMS[c['team']][1]
    speed = np.linalg.norm(vel, axis=1)
    # маршрут
    wps, last = [], 0
    for j in range(1, n):
        stop = speed[j] < 150 and j - last >= 30
        if (stop or j - last >= 120) and np.linalg.norm(pos[j] - pos[last]) > 400:
            v = vel[j] if not stop else np.zeros(2)
            sp = np.linalg.norm(v)
            if sp > vmax * 0.95: v = v / sp * vmax * 0.95
            wps.append((j, *pos[j], *v)); last = j
    if np.linalg.norm(pos[n - 1] - pos[last]) > 150 or not wps:
        wps.append((n - 1, *pos[n - 1], 0.0, 0.0))
    else:
        wps[-1] = (*wps[-1][:3], 0.0, 0.0)
    v0 = vel[0]; sp = np.linalg.norm(v0)
    if sp > vmax * 0.99: v0 = v0 / sp * vmax * 0.99
    lines = [f'{nf} {DT:.6f}', ' '.join(f'{x:.1f}' for x in (*pos[0], *v0)), str(len(wps))]
    lines += [' '.join(f'{x:.1f}' for x in w[1:]) for w in wps]
    frames = []
    for j in range(nf):
        obs = [o for o in range(32) if o != k and np.isfinite(Rg[s + j, o, 0])]
        obs.sort(key=lambda o: np.linalg.norm(Rg[s + j, o, :2] - pos[j]))
        obs = obs[:16]
        lines.append(f'{len(obs)} ' + ' '.join(f'{Rg[s+j,o,0]:.1f} {Rg[s+j,o,1]:.1f} {RAD} {Rg[s+j,o,2]:.1f} {Rg[s+j,o,3]:.1f}' for o in obs))
    open(f'{out}/seg{i}.txt', 'w').write('\n'.join(lines) + '\n')
    # реальный робот: зазор до остальных, новые контакты (до этого был свободен), из них на относительной скорости > 1.5 м/с
    oth = [o for o in range(32) if o != k]
    dd = np.linalg.norm(Rg[s:s + nf, oth, :2] - pos[:, None, :], axis=2) - RAD
    dd = np.where(np.isfinite(dd), dd, np.inf)
    near = np.argmin(dd, axis=1); clr = dd[np.arange(nf), near]
    relv = np.linalg.norm(Rg[s:s + nf, oth, 2:][np.arange(nf), near] - vel, axis=1)
    ev = fast = 0; inc = False; clear = False
    for j in range(nf):
        if clr[j] < 0:
            if not inc and clear:
                ev += 1; fast += relv[j] > 1500
            inc = True
        else:
            inc = False; clear = True
    seg = dict(cat=c['cat'], team=c['team'], k=int(k), s=int(s), nf=nf, n=n, t0=float(grid[s] - grid[0]),
               wps=[list(map(float, w)) for w in wps],
               real_contact=int(np.sum(clr < 0)), real_events=int(ev), real_fast=int(fast), real_minclr=float(clr.min()),
               real_arrive=float(wps[-1][0] * DT))
    if COUNT == 1:
        seg.update(real=Rg[s:s + nf, :, :].tolist(), ball=Bg[s:s + nf, :2].tolist())
    segs.append(seg)
    if COUNT == 1: print(f"{i}: {c['cat']}: {c['team']} робот {k % 16}, t={grid[s]-grid[0]:.1f} с, путь {c['travel']/1000:.1f} м, "
          f"толпа {c['crowd']:.1f}, встречные {c['cross']:.1f}, у мяча {c['ball']*100:.0f} %, точек маршрута {len(wps)}")
json.dump(segs, open(f'{out}/segments.json', 'w'))
