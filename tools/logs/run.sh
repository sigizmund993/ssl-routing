#!/bin/bash
# Прогон перебора на сценах из лога реальной игры SSL (.log.gz).
# Использование: tools/logs/run.sh <лог.log.gz> <рабочая папка>
# Нужны protoc и python3 с numpy и protobuf. Скорость и ускорение роботов команд заданы ниже (оценены dyn.py
# на логе Warthog Robotics vs TRAPS, RoboCup 2026); для другого лога сначала посмотри вывод dyn.py.
set -e
LOG=$1; WORK=$2; ROOT=$(cd "$(dirname "$0")/../.." && pwd)
mkdir -p "$WORK/pb"
(cd "$ROOT/../ssl_packet_package/proto/ssl" && protoc -I. --python_out="$WORK/pb" tracker/*.proto vision/*.proto \
    state/ssl_gc_referee_message.proto state/ssl_gc_common.proto state/ssl_gc_game_event.proto geom/ssl_gc_geometry.proto)
python3 -I "$ROOT/tools/logs/extract.py" "$LOG" "$WORK/pb" "$WORK/game.npz"
python3 -I "$ROOT/tools/logs/dyn.py" "$WORK/game.npz"
python3 -I "$ROOT/tools/logs/scenes.py" "$WORK/game.npz" "$WORK"
LIBS="-lsfml-graphics -lsfml-window -lsfml-system -lnlopt"
g++ -Wall -O3 -DMAX_VEL=1900 -DMAX_ACC=3000 "$ROOT/bench.cpp" -o "$WORK/bench-warthog" $LIBS
g++ -Wall -O3 -DMAX_VEL=2800 -DMAX_ACC=4000 "$ROOT/bench.cpp" -o "$WORK/bench-traps" $LIBS
for TEAM in warthog traps; do
    # торможение у обеих команд до ~6 м/с² (p99), разгон — MAX_ACC при сборке
    DECEL=6000 SCENES="$WORK/scenes_$TEAM.txt" DUMP="$WORK/dump_$TEAM.jsonl" "$WORK/bench-$TEAM" 100000 42 file > "$WORK/bench_$TEAM.txt"
    python3 -I "$ROOT/tools/logs/truth.py" "$WORK" "$TEAM"
done
