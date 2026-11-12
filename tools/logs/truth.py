# Аргументы: рабочая папка (game.npz, meta_*.json, dump_*.jsonl от bench-app), команда (warthog | traps)
# Проверка траекторий по реальному будущему: где остальные роботы были на самом деле
import numpy as np, sys, json
sp, team = sys.argv[1], sys.argv[2]
dumpf = sys.argv[3] if len(sys.argv) > 3 else f'{sp}/dump_{team}.jsonl'  # можно передать другой dump
d = np.load(f'{sp}/game.npz'); t, R = d['t'], d['robots']
meta = json.load(open(f'{sp}/meta_{team}.json'))
dump = {}
for l in open(dumpf):
    r = json.loads(l); dump[(r['scene'], r['method'])] = r
RAD = 180.0
HS = (0.5, 1.0, 2.0, 1e9)
def others_at(i, k, tau):
    """положения остальных роботов в момент t_i + tau (линейная интерполяция), shape (len(tau), 32, 2)"""
    ts = t[i] + tau
    idx = np.clip(np.searchsorted(t, ts), 1, len(t) - 1)
    w = ((ts - t[idx - 1]) / np.maximum(t[idx] - t[idx - 1], 1e-9))[:, None, None]
    P = R[idx - 1, :, :2] * (1 - w) + R[idx, :, :2] * w
    P[:, k] = np.nan
    return P
def real_clear(path, tau, i, k):
    P = others_at(i, k, tau)
    dist = np.linalg.norm(P - path[:, None, :], axis=2) - RAD
    dmin = np.nanmin(np.where(np.isfinite(dist), dist, np.inf), axis=1)
    return [dmin[tau <= H].min() if np.any(tau <= H) else np.inf for H in HS]
names = {0: 'slsqp', 1: 'перебор', 3: 'перебор, кадр 5', 4: 'прямая без препятствий'}
res = {m: [] for m in names}; base = []; Tr = {m: [] for m in names}; pred = {m: [] for m in names}
for s, (i, j, k) in enumerate(meta):
    # реальный робот: его собственный путь
    taus = t[i:j + 1] - t[i]
    base.append(real_clear(R[i:j + 1, k, :2], taus, i, k))
    Treal = t[j] - t[i]
    for m in names:
        r = dump[(s, m)]
        path = np.array(r['path']); T = r['T']
        tau = np.linspace(0, T, len(path))
        if T < 0.5:  # короткая траектория (например, аварийное торможение): дальше робот стоит на месте
            tau = np.r_[tau, 0.5]; path = np.vstack([path, path[-1:]])
        keep = t[i] + tau <= t[-1]
        res[m].append(real_clear(path[keep], tau[keep], i, k))
        ok = r['miss'] <= 50 and r['clearance'] >= 0
        pred[m].append(ok)
        Tr[m].append(T / Treal)
base = np.array(base)
print(f'=== {team}: сцен {len(meta)}')
print(f"{'':26s} {'заход в реальных роботов за 0.5 / 1 / 2 с / весь путь':52s} {'по модели допустим':>18s} {'T / T_real ср, медиана':>24s}")
print(f"{'реальный робот (его путь)':26s} " + ' / '.join(f'{np.mean(base[:, h] < 0) * 100:4.1f}%' for h in range(4)))
for m, n in names.items():
    a = np.array(res[m]); p = np.array(pred[m]); tr = np.array(Tr[m])
    print(f"{n:26s} " + ' / '.join(f'{np.mean(a[:, h] < 0) * 100:4.1f}%' for h in range(4)) +
          f"{'':12s}{p.mean() * 100:5.1f}%      {tr[p].mean():.3f}, {np.median(tr[p]):.3f}")
    # допустим по модели, но заходит в реальных роботов
    if m in (1, 3):
        bad = p & (a[:, 3] < 0)
        print(f"{'':28s}допустим по модели, но реально задевает: {bad.mean() * 100:.1f}% сцен; из них в первые 0.5 с: {np.mean(p & (a[:, 0] < 0)) * 100:.1f}%; "
              f"реальный робот в этих же сценах тоже задевал: {np.mean(base[bad, 3] < 0) * 100:.0f}%")

# 1. только сцены, где в начале робот не вплотную к другим (зазор >= 50 мм в t_i)
startgap = np.array([np.nanmin(np.linalg.norm(R[i, :, :2] - R[i, k, :2], axis=1)[np.arange(32) != k]) - RAD for i, j, k in meta])
free = startgap >= 50
print(f"\nв начале вплотную (зазор < 50 мм): {np.mean(~free) * 100:.0f}% сцен. Остальные {free.sum()} сцен:")
print(f"  {'реальный робот':24s} " + ' / '.join(f'{np.mean(base[free, h] < 0) * 100:4.1f}%' for h in range(4)))
for m, n in names.items():
    a = np.array(res[m])
    print(f"  {n:24s} " + ' / '.join(f'{np.mean(a[free, h] < 0) * 100:4.1f}%' for h in range(4)))
# 2. ошибка прогноза «постоянная скорость» для других роботов
errs = {0.25: [], 0.5: [], 1.0: [], 2.0: []}
for i, j, k in meta[::3]:
    for tau in errs:
        P = others_at(i, k, np.array([tau]))[0]
        pred_ = R[i, :, :2] + R[i, :, 2:4] * tau
        e = np.linalg.norm(P - pred_, axis=1); e = e[np.isfinite(e)]
        errs[tau].extend(e)
print('\nошибка прогноза положения других роботов при постоянной скорости, мм (медиана / p90 / p99):')
for tau, e in errs.items():
    e = np.array(e); print(f'  через {tau} с: {np.median(e):.0f} / {np.percentile(e, 90):.0f} / {np.percentile(e, 99):.0f}')
# 3. T только по сценам, где робот остановился (цель, скорее всего, настоящая)
stopped = np.array([t[j] - t[i] < 2.48 for i, j, k in meta])
print(f'\nсцены, где реальный робот остановился: {stopped.sum()}')
for m in (0, 1, 3, 4):
    p = np.array(pred[m]); tr = np.array(Tr[m]); sel = p & stopped
    print(f'  {names[m]:24s} T / T_real: ср {tr[sel].mean():.3f}, медиана {np.median(tr[sel]):.3f}, p90 {np.percentile(tr[sel], 90):.3f}')

# почему перебор недопустим по своей модели
p = np.array(pred[1]); bad = ~p
near = startgap < 50
print(f"\nперебор недопустим по модели в {bad.sum()} сценах: из них в начале вплотную {np.mean(near[bad]) * 100:.0f}%, "
      f"зазор в начале 50–200 мм {np.mean((startgap[bad] >= 50) & (startgap[bad] < 200)) * 100:.0f}%")
