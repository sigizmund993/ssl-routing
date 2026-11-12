# Скорость и ускорение роботов обеих команд по кадрам, когда игра идёт. Аргумент: game.npz
import numpy as np, sys
d = np.load(sys.argv[1]); t, cmd, R = d['t'], d['cmd'], d['robots']
run = np.isin(cmd, [2, 3, 8, 9])
dt = np.diff(t)
print(f"кадров {len(t)}, игра идёт в {run.mean()*100:.0f} % кадров ({run.sum()*np.median(dt)/60:.1f} мин), шаг кадра медиана {np.median(dt)*1000:.1f} мс")
# производная скорости по окну ±3 кадра (~±42 мс)
W = 3
for team, sl in (('Warthog (жёлтые)', slice(0, 16)), ('TRAPS (синие)', slice(16, 32))):
    v = R[:, sl, 2:4]; speed = np.linalg.norm(v, axis=2)
    a = np.full_like(v, np.nan)
    a[W:-W] = (v[2*W:] - v[:-2*W]) / (t[2*W:] - t[:-2*W])[:, None, None]
    m = run[:, None] & np.isfinite(speed)
    s = speed[m]
    print(f"\n{team}: роботов в кадре {np.isfinite(speed[run]).sum(1).mean():.1f}")
    print(f"  скорость, м/с: медиана {np.median(s)/1000:.2f}, p90 {np.percentile(s,90)/1000:.2f}, p99 {np.percentile(s,99)/1000:.2f}, p99.9 {np.percentile(s,99.9)/1000:.2f}, max {s.max()/1000:.2f}")
    mv = m & (speed > 300)
    vh = v / np.maximum(speed, 1e-9)[..., None]
    along = np.sum(a * vh, axis=2)            # вдоль скорости: + разгон, − торможение
    amag = np.linalg.norm(a, axis=2)
    acc = along[mv & (along > 0)]; dec = -along[mv & (along < 0)]; am = amag[mv]
    am = am[np.isfinite(am)]; acc = acc[np.isfinite(acc)]; dec = dec[np.isfinite(dec)]
    print(f"  |a| при движении > 0.3 м/с, м/с²: медиана {np.median(am)/1000:.2f}, p90 {np.percentile(am,90)/1000:.2f}, p99 {np.percentile(am,99)/1000:.2f}")
    print(f"  разгон вдоль скорости: p90 {np.percentile(acc,90)/1000:.2f}, p99 {np.percentile(acc,99)/1000:.2f};  торможение: p90 {np.percentile(dec,90)/1000:.2f}, p99 {np.percentile(dec,99)/1000:.2f}")
    # доля времени на большой скорости
    print(f"  доля времени быстрее 1 / 2 / 3 м/с: {np.mean(s>1000)*100:.0f} / {np.mean(s>2000)*100:.1f} / {np.mean(s>3000)*100:.2f} %")
