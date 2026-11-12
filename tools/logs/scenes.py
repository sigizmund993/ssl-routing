# Аргументы: game.npz, рабочая папка (туда пишутся scenes_<команда>.txt и meta_<команда>.json)
# Сцены из лога: реальный робот в движении в момент t_i; цель — где он остановился (скорость < 0.15 м/с,
# не раньше чем через 0.5 с) или где оказался через 2.5 с. Препятствия — остальные роботы в момент t_i
# со скоростями, радиус 180 мм (сумма радиусов двух роботов). Скорости обрезаются до vmax команды
import numpy as np, sys, json
d = np.load(sys.argv[1]); out = sys.argv[2]
t, cmd, R = d['t'], d['cmd'], d['robots']
run = np.isin(cmd, [2, 3, 8, 9])
speed = np.linalg.norm(R[:, :, 2:4], axis=2)
teams = {'warthog': (range(0, 16), 1900.0), 'traps': (range(16, 32), 2800.0)}
STRIDE, HMAX, RAD = 36, 2.5, 180.0
gap = np.r_[np.diff(t) > 0.1, True]
for name, (ks, vmax) in teams.items():
    lines, meta = [], []
    for k in ks:
        i = 0
        while i < len(t) - 1:
            if not (run[i] and np.isfinite(speed[i, k]) and speed[i, k] > 300):
                i += 1; continue
            j = i + 1
            while j < len(t) and t[j] - t[i] < HMAX and run[j] and not gap[j - 1] and np.isfinite(speed[j, k]):
                if speed[j, k] < 150 and t[j] - t[i] >= 0.5: break
                j += 1
            ok = j < len(t) and run[min(j, len(t) - 1)] and np.isfinite(speed[min(j, len(t) - 1), k]) and t[j] - t[i] >= 0.5
            if ok:
                s, e = R[i, k], R[j, k]
                if np.hypot(e[0] - s[0], e[1] - s[1]) >= 500:
                    def clamp(vx, vy):
                        n = np.hypot(vx, vy)
                        return (vx, vy) if n <= vmax * 0.999 else (vx / n * vmax * 0.999, vy / n * vmax * 0.999)
                    vs, ve = clamp(s[2], s[3]), clamp(e[2], e[3])
                    obs = [(R[i, o, 0], R[i, o, 1], RAD, R[i, o, 2], R[i, o, 3]) for o in range(32) if o != k and np.isfinite(R[i, o, 0])]
                    lines.append(' '.join(f'{x:.1f}' for x in (s[0], s[1], *vs, e[0], e[1], *ve)) + f' {len(obs)} ' +
                                 ' '.join(' '.join(f'{x:.1f}' for x in ob) for ob in obs))
                    meta.append((int(i), int(j), int(k)))
            i += STRIDE
    open(f'{out}/scenes_{name}.txt', 'w').write('\n'.join(lines) + '\n')
    json.dump(meta, open(f'{out}/meta_{name}.json', 'w'))
    Treal = np.array([t[j] - t[i] for i, j, k in meta])
    print(f'{name}: сцен {len(meta)}, T_real медиана {np.median(Treal):.2f} с, доля «остановился» {np.mean(Treal < HMAX - 0.02)*100:.0f} %, '
          f'препятствий в сцене {np.mean([l.split().__len__() for l in lines]) and np.mean([int(l.split()[8]) for l in lines]):.1f}')
