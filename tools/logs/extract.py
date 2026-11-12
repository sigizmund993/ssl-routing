# Кадры трекера TIGERs из лога SSL (.log.gz) → numpy. Аргументы: лог, папка с python-модулями proto, выходной .npz
# Кадры трекера TIGERs → numpy: время, команда судьи, мяч, роботы (команда, id, x, y, vx, vy) в мм и мм/с
import gzip, struct, sys, numpy as np
sys.path.insert(0, sys.argv[2])
from tracker import ssl_vision_wrapper_tracked_pb2 as T
from state import ssl_gc_referee_message_pb2 as R
f = gzip.open(sys.argv[1], 'rb'); f.read(16)
cmd = -1; stage = -1; names = None
times, cmds, balls, robots = [], [], [], []
while True:
    h = f.read(16)
    if len(h) < 16: break
    ts, typ, size = struct.unpack('>qii', h)
    data = f.read(size)
    if typ == 3:
        m = R.Referee(); m.ParseFromString(data); cmd = m.command; stage = m.stage
        names = (m.yellow.name, m.blue.name)
    elif typ == 5:
        m = T.TrackerWrapperPacket(); m.ParseFromString(data)
        if m.source_name != 'TIGERs': continue
        fr = m.tracked_frame
        times.append(fr.timestamp); cmds.append(cmd)
        b = fr.balls[0] if len(fr.balls) else None
        balls.append([b.pos.x*1000, b.pos.y*1000, b.vel.x*1000, b.vel.y*1000] if b else [np.nan]*4)
        rr = np.full((32, 4), np.nan)  # 0..15 жёлтые, 16..31 синие
        for r in fr.robots:
            k = r.robot_id.id + (16 if r.robot_id.team == 2 else 0)
            if r.robot_id.id < 16:
                rr[k] = [r.pos.x*1000, r.pos.y*1000, r.vel.x*1000, r.vel.y*1000]
        robots.append(rr)
np.savez_compressed(sys.argv[3], t=np.array(times), cmd=np.array(cmds), ball=np.array(balls), robots=np.array(robots))
print('кадров', len(times), 'команды (жёлтые, синие):', names)
