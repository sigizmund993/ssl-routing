/*
Покадровая симуляция по логу реальной игры: наш робот едет по маршруту реального робота (точки маршрута — его
остановки), остальные роботы проигрываются из лога. На каждом кадре метод заново строит траекторию из текущего
состояния к текущей точке маршрута, робот проезжает по ней dt и всё повторяется. Методы независимы, у каждого своё
состояние робота.

Запуск: replay <вход> <выход> [файл статистики]. MAX_VEL и MAX_ACC задаются при сборке (-DMAX_VEL=... -DMAX_ACC=...),
торможение и прочие параметры перебора — переменные окружения, как у bench-app (DECEL, HORIZON, BUDGET, TWO_VIA, ...).
DELAY=<кадров> — задержка между командой и её исполнением: робот исполняет команду скорости, выданную DELAY кадров
назад. COMP=1 — планировать из состояния, в котором робот окажется после уже отправленных команд (compensateDelay).
SLSQP=0 — не считать минимизатор (он самый медленный).

Вход (текст):
  nFrames dt
  x y vx vy                      начальное состояние робота
  nWp, затем nWp строк x y vx vy точки маршрута (у последней скорость 0)
  на каждый кадр: n и n препятствий x y rad vx vy
Выход: строка на (кадр, метод):
  кадр метод x y vx vy мс_расчёта номер_точки T_плана, затем 16 точек плана x y
Статистика (дописывается в файл, строка на метод):
  вход метод время_приезда(-1 — не доехал) кадров_контакта новых_контактов из_них_быстрых(>1.5 м/с) смен_плана
  мс_расчёта_ср мс_расчёта_max мин_зазор кадров
*/
#define main benchMain
#include "../../bench.cpp"
#undef main

// перебор в замкнутом контуре: тёплый старт и генератор живут между кадрами
struct Loop
{
    sampling::State state;
    mt19937 gen{42};
    sampling::Params params;
} loops[2];

MethodOut runLoop(Loop &loop, const Scene &s)
{
    MethodOut out;
    Timer timer;
    sampling::Result res = sampling::plan(s.start, s.end, s.vStart, s.vEnd, s.enemies, s.nEnemies, loop.gen, &loop.state, loop.params);
    out.timeMs = timer.time() * 1000;
    out.traj = res.traj;
    return out;
}
MethodOut runSamplerLoop(const Scene &s, unsigned)
{
    return runLoop(loops[0], s);
}
// перебор без новшеств (гистерезис, проверка стоящего робота, две точки, ожидание, добавочный бюджет) — для сравнения
MethodOut runSamplerOld(const Scene &s, unsigned)
{
    return runLoop(loops[1], s);
}
MethodOut runNone(const Scene &, unsigned)
{
    MethodOut out;
    out.timeMs = 0;
    out.traj.reset(Point(NAN, NAN), Point(NAN, NAN));
    return out;
}

void readParams()
{
    sampling::Params &p = samplerParams;
    if (getenv("BUDGET"))
        p.budget = atoi(getenv("BUDGET"));
    if (getenv("HORIZON"))
        p.horizon = atof(getenv("HORIZON"));
    if (getenv("ACCEL"))
        p.accel = atof(getenv("ACCEL"));
    if (getenv("DECEL"))
        p.decel = atof(getenv("DECEL"));
    if (getenv("BRAKE"))
        p.emergencyBrake = atoi(getenv("BRAKE")) != 0;
    loops[1].params = p;
    loops[1].params.twoVia = false;
    loops[1].params.hardExtra = 0;
    if (getenv("TWO_VIA"))
        p.twoVia = atoi(getenv("TWO_VIA")) != 0;
    if (getenv("HARD_EXTRA"))
        p.hardExtra = atoi(getenv("HARD_EXTRA"));
    if (getenv("PENALTY"))
        p.penalty = atof(getenv("PENALTY"));
    loops[0].params = p;
}

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        fprintf(stderr, "replay <вход> <выход> [файл статистики]\n");
        return 1;
    }
    readParams();
    const int delay = getenv("DELAY") ? max(0, min(30, atoi(getenv("DELAY")))) : 0;
    const bool comp = getenv("COMP") && atoi(getenv("COMP"));
    const bool withSlsqp = !(getenv("SLSQP") && !atoi(getenv("SLSQP")));
    FILE *in = fopen(argv[1], "r");
    FILE *out = fopen(argv[2], "w");
    if (!in || !out)
        return 1;
    int nFrames, nWp;
    double dt, x, y, vx, vy;
    if (fscanf(in, "%d %lf %lf %lf %lf %lf %d", &nFrames, &dt, &x, &y, &vx, &vy, &nWp) != 7)
        return 1;
    Point pos0(x, y), vel0(vx, vy);
    vector<Point> wp(nWp), wpVel(nWp);
    for (int i = 0; i < nWp; i++)
    {
        if (fscanf(in, "%lf %lf %lf %lf", &x, &y, &vx, &vy) != 4)
            return 1;
        wp[i] = Point(x, y);
        wpVel[i] = Point(vx, vy);
    }

    // первые три метода читает replay_video.py, порядок не менять
    const Method methods[] = {
        {"slsqp", withSlsqp ? runSlsqp : runNone},
        {"перебор", runSamplerLoop},
        {"прямая без препятствий", runDirect},
        {"перебор без новшеств", runSamplerOld},
    };
    const int nMethods = sizeof(methods) / sizeof(methods[0]);
    Point pos[nMethods], vel[nMethods];
    int cur[nMethods];
    // очередь отправленных, но ещё не исполненных команд скорости (самая старая первая)
    vector<Point> pending[nMethods];
    sampling::Trajectory prevPlan[nMethods];
    bool hasPrev[nMethods];
    // статистика
    double arrive[nMethods], msSum[nMethods], msMax[nMethods], minClr[nMethods];
    int contactFrames[nMethods], events[nMethods], fastEvents[nMethods], switches[nMethods];
    bool inContact[nMethods], wasClear[nMethods];
    for (int m = 0; m < nMethods; m++)
    {
        pos[m] = pos0;
        vel[m] = vel0;
        cur[m] = 0;
        hasPrev[m] = false;
        arrive[m] = -1;
        msSum[m] = msMax[m] = 0;
        minClr[m] = 1e18;
        contactFrames[m] = events[m] = fastEvents[m] = switches[m] = 0;
        inContact[m] = wasClear[m] = false;
    }

    for (int f = 0; f < nFrames; f++)
    {
        Scene s;
        if (fscanf(in, "%d", &s.nEnemies) != 1)
            return 1;
        for (int i = 0; i < s.nEnemies; i++)
        {
            double r;
            if (fscanf(in, "%lf %lf %lf %lf %lf", &x, &y, &r, &vx, &vy) != 5)
                return 1;
            s.enemies[i] = AbsRigBody(Point(x, y), r, Point(vx, vy));
        }
        for (int m = 0; m < nMethods; m++)
        {
            // зазор до реальных роботов в этом кадре и контакты
            double clr = 1e18, relSpeed = 0;
            for (int i = 0; i < s.nEnemies; i++)
            {
                double c = (pos[m] - s.enemies[i].getPos()).mag() - s.enemies[i].getRad();
                if (c < clr)
                {
                    clr = c;
                    relSpeed = (vel[m] - s.enemies[i].getVel()).mag();
                }
            }
            minClr[m] = min(minClr[m], clr);
            if (clr < 0)
            {
                contactFrames[m]++;
                // новый контакт: до этого робот был свободен (контакт с самого старта не считается)
                if (!inContact[m] && wasClear[m])
                {
                    events[m]++;
                    fastEvents[m] += relSpeed > 1500;
                }
                inContact[m] = true;
            }
            else
            {
                inContact[m] = false;
                wasClear[m] = true;
            }
            if (arrive[m] < 0 && cur[m] == nWp - 1 && (pos[m] - wp[nWp - 1]).mag() < 50 && vel[m].mag() < 100)
                arrive[m] = f * dt;

            Scene sc = s;
            sc.start = pos[m];
            sc.vStart = vel[m];
            if (comp && delay > 0)
            {
                sampling::compensateDelay(sc.start, sc.vStart, pending[m].data(), int(pending[m].size()), dt);
                for (int i = 0; i < sc.nEnemies; i++)
                    sc.enemies[i].setPos(sc.enemies[i].getPos() + sc.enemies[i].getVel() * (pending[m].size() * dt));
            }
            sc.end = wp[cur[m]];
            sc.vEnd = wpVel[cur[m]];
            MethodOut res;
            res.timeMs = 0;
            bool hold = (sc.end - sc.start).mag() < 1 && (sc.vEnd - sc.vStart).mag() < 1;
            if (!hold)
                res = methods[m].run(sc, 42u + f);
            msSum[m] += res.timeMs;
            msMax[m] = max(msMax[m], res.timeMs);
            Point p = hold ? sc.end : res.traj.posAt(dt), v = hold ? sc.vEnd : res.traj.velAt(dt);
            double T = hold ? 0 : res.traj.duration();
            bool failed = !isfinite(p.x) || !isfinite(p.y) || !isfinite(v.x) || !isfinite(v.y) || !isfinite(T) || (!hold && res.traj.n == 0);
            if (failed)
            {
                // метод не построил траекторию: тормозим по прямой
                double sp = sc.vStart.mag(), dv = min(sp, samplerParams.decel * dt);
                v = sp > 0 ? sc.vStart * ((sp - dv) / sp) : sc.vStart;
                p = sc.start + (sc.vStart + v) * (dt / 2);
                T = NAN;
                hasPrev[m] = false;
            }
            else if (!hold)
            {
                // смена плана: новая траектория в ближайшую секунду расходится с прошлой больше чем на 150 мм
                if (hasPrev[m])
                {
                    double dev = 0;
                    for (double tau = 0.2; tau <= 1.001; tau += 0.2)
                        dev = max(dev, (res.traj.posAt(tau) - prevPlan[m].posAt(tau + dt)).mag());
                    switches[m] += dev > 150;
                }
                prevPlan[m] = res.traj;
                hasPrev[m] = true;
            }
            fprintf(out, "%d %d %.1f %.1f %.1f %.1f %.4f %d %.4f", f, m, pos[m].x, pos[m].y, vel[m].x, vel[m].y, res.timeMs, cur[m], T);
            for (int j = 0; j < 16; j++)
            {
                Point q = (hold || !isfinite(T)) ? pos[m] : res.traj.posAt(T * j / 15);
                fprintf(out, " %.1f %.1f", q.x, q.y);
            }
            fprintf(out, "\n");
            if (delay == 0)
            {
                pos[m] = p;
                vel[m] = v;
            }
            else
            {
                // команда — скорость через dt; робот исполняет ту, что выдана delay кадров назад
                pending[m].push_back(v);
                if (int(pending[m].size()) > delay)
                {
                    Point vNew = pending[m].front();
                    pending[m].erase(pending[m].begin());
                    pos[m] = pos[m] + (vel[m] + vNew) * (dt / 2);
                    vel[m] = vNew;
                }
                else
                    pos[m] = pos[m] + vel[m] * dt;
            }
            // промежуточная точка пройдена (ближе 300 мм) — к следующей. Ждать точного попадания нельзя: у точки
            // с ненулевой скоростью вблизи неё план «чуть сдать назад и разогнаться» перестраивается каждый кадр
            // одинаково, и робот замирает в ~200 мм от точки
            if (cur[m] + 1 < nWp && (pos[m] - wp[cur[m]]).mag() < 300)
                cur[m]++;
        }
    }
    fclose(out);
    if (argc > 3)
    {
        FILE *st = fopen(argv[3], "a");
        for (int m = 0; m < nMethods; m++)
            fprintf(st, "%s %d %.3f %d %d %d %d %.4f %.4f %.1f %d\n", argv[1], m, arrive[m], contactFrames[m], events[m], fastEvents[m], switches[m],
                    msSum[m] / nFrames, msMax[m], minClr[m], nFrames);
        fclose(st);
    }
    return 0;
}
