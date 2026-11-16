# Аргументы: рабочая папка (segments.json от replay_prep.py), затем файлы статистики replay (по одному на прогон).
# Печатает сводку покадровой симуляции по всем участкам: по методам каждого прогона и по реальному роботу
import json, sys, os
import numpy as np
work = sys.argv[1]
segs = json.load(open(f'{work}/segments.json'))
NAMES = ['slsqp', 'перебор', 'прямая без препятствий', 'перебор без новшеств']
print(f'участков {len(segs)}, кадров {sum(s["nf"] for s in segs)}')
hdr = f'{"":34s}{"доехал":>8s}{"приезд ср, с":>14s}{"кадров конт.":>14s}{"новых конт.":>13s}{"быстрых":>9s}{"смен плана":>12s}{"расчёт ср":>11s}{"max, мс":>9s}'
print(hdr)
def row(name, arr, cf, ev, fast, sw='', ms='', mx=''):
    print(f'{name:34s}{arr[0]:>8s}{arr[1]:>14s}{cf:>14}{ev:>13}{fast:>9}{sw:>12}{ms:>11}{mx:>9}')
print('реальный робот' + ' ' * 20 + f'{"":>8s}{np.mean([s["real_arrive"] for s in segs]):>14.3f}'
      f'{sum(s["real_contact"] for s in segs):>14}{sum(s["real_events"] for s in segs):>13}{sum(s["real_fast"] for s in segs):>9}')
for path in sys.argv[2:]:
    rows = [l.split() for l in open(path)]
    label = os.path.basename(path).replace('stats_', '').replace('.txt', '')
    for m, name in enumerate(NAMES):
        r = np.array([[float(x) for x in l[2:]] for l in rows if int(l[1]) == m])
        if len(r) == 0 or (m == 0 and r[:, 7].max() == 0):
            continue
        ok = r[:, 0] >= 0
        # время приезда сравнивается на участках, где доехали все методы этого прогона
        allok = np.all(np.array([[float(l[2]) >= 0 for l in rows if int(l[1]) == mm] for mm in range(len(NAMES))
                                 if not (mm == 0 and r[:, 7].max() == 0 and m != 0) and mm != 0]), axis=0)
        print(f'{label + ": " + name:34s}{int(ok.sum()):>8d}{r[allok, 0].mean():>14.3f}{int(r[:, 1].sum()):>14d}{int(r[:, 2].sum()):>13d}'
              f'{int(r[:, 3].sum()):>9d}{int(r[:, 4].sum()):>12d}{r[:, 5].mean():>11.4f}{r[:, 6].max():>9.3f}')
print('приезд — среднее по участкам, где доехали все методы прогона (кроме slsqp); реальный робот — момент его последней точки маршрута;\n'
      'новый контакт — зазор до другого робота стал < 0 после того, как был >= 0; быстрый — на относительной скорости > 1.5 м/с')
