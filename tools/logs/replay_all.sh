#!/bin/bash
# Покадровая симуляция на всех участках из segments.json и сводка.
# Использование: tools/logs/replay_all.sh <рабочая папка> <метка прогона>
# Перед этим: replay_prep.py <game.npz> <рабочая папка> <участков на категорию>. Параметры перебора и DELAY/COMP/SLSQP
# берутся из окружения (см. replay.cpp), DECEL по умолчанию 6000. Итог — <рабочая папка>/stats_<метка>.txt,
# сводку по нескольким прогонам печатает replay_stats.py
set -e
WORK=$1; LABEL=$2; ROOT=$(cd "$(dirname "$0")/../.." && pwd)
LIBS="-lsfml-graphics -lsfml-window -lsfml-system -lnlopt"
# ограничения команд — как в run.sh (первый лог: Warthog — жёлтые, TRAPS — синие)
# пересборка, если бинарник старше исходников
build() {
    local bin=$1; shift
    if [ ! -x "$bin" ] || [ -n "$(find "$ROOT/include" "$ROOT/bench.cpp" "$ROOT/tools/logs/replay.cpp" -newer "$bin" -name '*.[ch]*' -print -quit)" ]; then
        g++ -Wall -O3 "$@" "$ROOT/tools/logs/replay.cpp" -o "$bin" $LIBS
    fi
}
build "$WORK/replay-warthog" -DMAX_VEL=1900 -DMAX_ACC=3000
build "$WORK/replay-traps" -DMAX_VEL=2800 -DMAX_ACC=4000
rm -f "$WORK/stats_$LABEL.txt"
export DECEL=${DECEL:-6000}
python3 -I -c "
import json, sys
for i, s in enumerate(json.load(open(sys.argv[1] + '/segments.json'))): print(i, s['team'])" "$WORK" |
    xargs -P "$(nproc)" -L 1 sh -c '"$0/replay-$2" "$0/seg$1.txt" "$0/out_$1.tmp" "$0/stats_'"$LABEL"'.txt" && rm -f "$0/out_$1.tmp"' "$WORK"
python3 -I "$ROOT/tools/logs/replay_stats.py" "$WORK" "$WORK/stats_$LABEL.txt"
