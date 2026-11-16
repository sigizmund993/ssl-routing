/*
Симуляция нескольких своих роботов: каждый кадр (60 Гц) все роботы заново строят траектории перебором и проезжают
по ним dt. Сравниваются способы учитывать друг друга:
  «не видят»      — свои роботы друг друга не учитывают (нижняя оценка времени, сколько столкновений без учёта);
  «как чужие»     — остальные свои — круги с постоянной скоростью, как чужие роботы (с горизонтом);
  «траектории»    — приоритеты по номеру: робот видит свежие траектории старших и траектории прошлого кадра младших;
  «строгий приор.» — робот видит только траектории старших, младшие обязаны уступать.
Чужих роботов нет (их проверяют bench.cpp и tools/logs). sampler.h не тянет SFML и nlopt, поэтому собирается без библиотек.

Запуск: ./multi-app [число сцен = 200] [seed = 42] [swap | circle | random | cross | zone]
  swap   — роботы стоят по кругу и меняются местами через центр (центр, радиус и цели немного случайны);
  circle — то же без случайности: круг RADIUS (2000 мм) в центре поля, цели точно напротив (одной сцены достаточно);
  random — случайные старты и цели;
  cross  — две шеренги меняются сторонами;
  zone   — старт и цель по разные стороны штрафной площади (её надо объехать), включены зоны и граница поля.
Переменные окружения: ROBOTS (6), ZONES=1 (штрафные и граница поля в любом сценарии), BUDGET, ALLY_MARGIN,
TWO_VIA, HARD_EXTRA, DUMP=файл (положения всех роботов по кадрам).
*/
#include <random>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include "include/Timer.h"
#include "include/const.h"
#include "include/sampling/sampler.h"

using namespace std;

const double DT = 1.0 / 60, T_MAX = 15, RAD = 2 * ROBOT_R;
const int MAX_BOTS = 16;

struct Bot
{
    Point pos, vel, goal;
    sampling::State state;
    sampling::Trajectory traj; // последняя построенная траектория
    mt19937 gen;
};

struct SceneDef
{
    int n;
    Point start[MAX_BOTS], goal[MAX_BOTS];
};

enum Mode
{
    BLIND,
    AS_ENEMIES,
    TRAJ,
    STRICT,
    N_MODES
};
const char *modeNames[N_MODES] = {"не видят", "как чужие", "траектории", "строгий приор."};

struct Stats
{
    vector<double> arrive, calcMs, minDist;
    int contactPairs = 0, contactFrames = 0, notArrived = 0, robots = 0, plans = 0, infeasible = 0, brakes = 0;
    int zoneFrames = 0, fieldFrames = 0, switches = 0;
};

sampling::Params params;
bool useZones = false, planZones = false; // зоны считаются в статистике / учитываются планировщиком
sampling::Rect zones[2];
Point fieldLo, fieldHi;

double mean(const vector<double> &v)
{
    double s = 0;
    for (double x : v)
        s += x;
    return v.empty() ? NAN : s / v.size();
}
double percentile(vector<double> v, double p)
{
    if (v.empty())
        return NAN;
    std::sort(v.begin(), v.end());
    return v[min(v.size() - 1, size_t(p * (v.size() - 1) + 0.5))];
}
void pad(const char *s, int width)
{
    int chars = 0;
    for (const char *c = s; *c; c++)
        if ((*c & 0xC0) != 0x80)
            chars++;
    printf("%s%*s", s, max(0, width - chars), "");
}

bool inZone(const Point &p)
{
    return useZones && (sampling::rectDistance(p, zones[0]) < 0 || sampling::rectDistance(p, zones[1]) < 0);
}

SceneDef makeScene(mt19937 &gen, const char *kind, int n)
{
    uniform_real_distribution<double> u01(0, 1), uSym(-1, 1);
    SceneDef s;
    s.n = n;
    if (strcmp(kind, "circle") == 0)
    {
        // без случайности: круг радиусом R (мм, переменная RADIUS) в центре поля, каждый едет точно в противоположную точку
        double R = getenv("RADIUS") ? atof(getenv("RADIUS")) : 2000;
        for (int i = 0; i < n; i++)
        {
            double ang = 2 * M_PI * i / n;
            s.start[i] = Point(cos(ang), sin(ang)) * R;
            s.goal[i] = Point(0, 0) - s.start[i];
        }
    }
    else if (strcmp(kind, "swap") == 0)
    {
        double R = 1200 + 1300 * u01(gen), phase = M_PI * uSym(gen);
        Point c(1500 * uSym(gen), 500 * uSym(gen));
        for (int i = 0; i < n; i++)
        {
            double ang = phase + 2 * M_PI * i / n;
            s.start[i] = c + Point(cos(ang), sin(ang)) * R;
            s.goal[i] = c - Point(cos(ang + 0.15 * uSym(gen)), sin(ang + 0.15 * uSym(gen))) * R;
        }
    }
    else if (strcmp(kind, "cross") == 0)
    {
        double gap = 450 + 300 * u01(gen), dist = 1500 + 1500 * u01(gen);
        int half = (n + 1) / 2;
        for (int i = 0; i < n; i++)
        {
            int side = i % 2, k = i / 2;
            double y = (k - (half - 1) / 2.0) * gap;
            s.start[i] = Point(side ? dist : -dist, y + 60 * uSym(gen));
            s.goal[i] = Point(side ? -dist : dist, y + 60 * uSym(gen));
        }
    }
    else
    {
        bool zone = strcmp(kind, "zone") == 0;
        auto pick = [&](Point *arr, int i, bool top)
        {
            for (;;)
            {
                Point p;
                if (zone)
                    // по разные стороны правой штрафной, недалеко от линии ворот
                    p = Point(FIELD_DX / 2.0 - ZONE_DX * (0.1 + 0.8 * u01(gen)), (top ? 1 : -1) * (ZONE_DY / 2.0 + ROBOT_R + 150 + 900 * u01(gen)));
                else
                    p = Point((FIELD_DX / 2.0 - 400) * uSym(gen), (FIELD_DY / 2.0 - 400) * uSym(gen));
                bool ok = !inZone(p);
                for (int j = 0; j < i; j++)
                    ok = ok && (arr[j] - p).mag() > 2 * RAD;
                if (ok)
                {
                    arr[i] = p;
                    return;
                }
            }
        };
        for (int i = 0; i < n; i++)
        {
            bool top = u01(gen) < 0.5;
            pick(s.start, i, top);
            do
                pick(s.goal, i, !top);
            while (!zone && (s.goal[i] - s.start[i]).mag() < 1500);
        }
    }
    return s;
}

void simulate(const SceneDef &scene, Mode mode, unsigned seed, Stats &st, FILE *dump, int sceneIdx)
{
    int n = scene.n;
    Bot bots[MAX_BOTS];
    for (int i = 0; i < n; i++)
    {
        bots[i].pos = scene.start[i];
        bots[i].vel = Point(0, 0);
        bots[i].goal = scene.goal[i];
        bots[i].traj.reset(bots[i].pos, Point(0, 0));
        bots[i].gen.seed(seed * 31u + i);
    }
    double lastAway[MAX_BOTS], pairMin[MAX_BOTS][MAX_BOTS];
    for (int i = 0; i < n; i++)
    {
        lastAway[i] = 0;
        for (int j = 0; j < n; j++)
            pairMin[i][j] = 1e18;
    }
    int nFrames = int(T_MAX / DT);
    for (int f = 0; f < nFrames; f++)
    {
        // снимок состояний на начало кадра: «как чужие» видят именно его
        Point pos0[MAX_BOTS], vel0[MAX_BOTS];
        for (int i = 0; i < n; i++)
            pos0[i] = bots[i].pos, vel0[i] = bots[i].vel;
        for (int i = 0; i < n; i++)
        {
            AbsRigBody enemies[MAX_BOTS];
            sampling::Ally allies[MAX_BOTS];
            sampling::World world;
            world.enemies = enemies;
            world.allies = allies;
            for (int j = 0; j < n; j++)
            {
                if (j == i)
                    continue;
                if (mode == AS_ENEMIES)
                    enemies[world.nEnemies++] = AbsRigBody(pos0[j], RAD, vel0[j]);
                // у старших траектория уже этого кадра, у младших — прошлого (с неё прошло dt)
                if (mode == TRAJ || (mode == STRICT && j < i))
                    allies[world.nAllies++] = {&bots[j].traj, RAD, j < i ? 0 : (f == 0 ? 0 : DT)};
            }
            if (planZones)
            {
                world.zones = zones;
                world.nZones = 2;
                world.hasField = true;
                world.fieldLo = fieldLo;
                world.fieldHi = fieldHi;
            }
            Timer timer;
            sampling::Result res = sampling::plan(pos0[i], bots[i].goal, vel0[i], Point(0, 0), world, bots[i].gen, &bots[i].state, params);
            st.calcMs.push_back(timer.time() * 1000);
            st.plans++;
            st.infeasible += !res.feasible;
            st.brakes += res.brake;
            // смена плана: новая траектория в ближайшую секунду расходится со старой больше чем на 150 мм
            if (f > 0)
            {
                double dev = 0;
                for (double tau = 0.2; tau <= 1.001; tau += 0.2)
                    dev = max(dev, (res.traj.posAt(tau) - bots[i].traj.posAt(tau + DT)).mag());
                st.switches += dev > 150;
            }
            bots[i].traj = res.traj;
            bots[i].pos = res.traj.posAt(DT);
            bots[i].vel = res.traj.velAt(DT);
        }
        double t = (f + 1) * DT;
        for (int i = 0; i < n; i++)
        {
            if ((bots[i].pos - bots[i].goal).mag() > 20 || bots[i].vel.mag() > 20)
                lastAway[i] = t;
            st.zoneFrames += inZone(bots[i].pos);
            st.fieldFrames += useZones && sampling::rectDistance(bots[i].pos, {fieldLo, fieldHi}) > 1;
            for (int j = i + 1; j < n; j++)
            {
                double d = (bots[i].pos - bots[j].pos).mag();
                pairMin[i][j] = min(pairMin[i][j], d);
                st.contactFrames += d < RAD - 1;
            }
        }
        if (dump)
        {
            fprintf(dump, "%d %d %d", sceneIdx, int(mode), f);
            for (int i = 0; i < n; i++)
                fprintf(dump, " %.1f %.1f", bots[i].pos.x, bots[i].pos.y);
            fprintf(dump, "\n");
        }
    }
    for (int i = 0; i < n; i++)
    {
        st.robots++;
        if (lastAway[i] >= T_MAX - 2 * DT)
            st.notArrived++;
        else
            st.arrive.push_back(lastAway[i]);
        for (int j = i + 1; j < n; j++)
        {
            st.minDist.push_back(pairMin[i][j] - RAD);
            st.contactPairs += pairMin[i][j] < RAD - 1;
        }
    }
}

int main(int argc, char **argv)
{
    int nScenes = argc > 1 ? atoi(argv[1]) : 200;
    unsigned seed = argc > 2 ? atoi(argv[2]) : 42;
    const char *kind = argc > 3 ? argv[3] : "swap";
    int nBots = getenv("ROBOTS") ? min(MAX_BOTS, atoi(getenv("ROBOTS"))) : 6;
    useZones = strcmp(kind, "zone") == 0 || (getenv("ZONES") && atoi(getenv("ZONES")));
    // ZONES=0 в сценарии zone: планировщик зон не знает, но заезды в них считаются (для сравнения)
    planZones = useZones && !(getenv("ZONES") && !atoi(getenv("ZONES")));
    // штрафные площади, раздутые на радиус робота, и поле с запасом 200 мм за линиями
    for (int k = 0; k < 2; k++)
    {
        double sx = k ? 1 : -1, xIn = sx * (FIELD_DX / 2.0 - ZONE_DX - ROBOT_R), xOut = sx * (FIELD_DX / 2.0 + 1000);
        zones[k] = {Point(min(xIn, xOut), -ZONE_DY / 2.0 - ROBOT_R), Point(max(xIn, xOut), ZONE_DY / 2.0 + ROBOT_R)};
    }
    fieldLo = Point(-FIELD_DX / 2.0 - 200, -FIELD_DY / 2.0 - 200);
    fieldHi = Point(FIELD_DX / 2.0 + 200, FIELD_DY / 2.0 + 200);
    if (getenv("BUDGET"))
        params.budget = atoi(getenv("BUDGET"));
    if (getenv("ALLY_MARGIN"))
        params.allyMargin = atof(getenv("ALLY_MARGIN"));
    if (getenv("TWO_VIA"))
        params.twoVia = atoi(getenv("TWO_VIA")) != 0;
    if (getenv("HARD_EXTRA"))
        params.hardExtra = atoi(getenv("HARD_EXTRA"));
    FILE *dump = getenv("DUMP") ? fopen(getenv("DUMP"), "w") : nullptr;

    Stats stats[N_MODES];
    mt19937 gen(seed);
    for (int k = 0; k < nScenes; k++)
    {
        SceneDef scene = makeScene(gen, kind, nBots);
        for (int m = 0; m < N_MODES; m++)
            simulate(scene, Mode(m), seed * 7919u + k, stats[m], dump, k);
    }
    printf("сцен %d (seed %u, сценарий %s), роботов %d, зоны и граница поля: %s\n", nScenes, seed, kind, nBots,
           planZones ? "да" : useZones ? "только в статистике" : "нет");
    pad("способ", 16);
    const char *cols[] = {"пар с контактом", "кадров контакта", "мин. зазор p5, мм", "приезд ср, с", "приезд p95, с", "не доехали", "расчёт ср, мс",
                          "расчёт p95", "расчёт max", "торможений", "смен плана", "кадров в зоне", "за полем"};
    for (const char *c : cols)
    {
        printf("  ");
        pad(c, 0);
    }
    printf("\n");
    for (int m = 0; m < N_MODES; m++)
    {
        const Stats &s = stats[m];
        pad(modeNames[m], 16);
        printf("  %15d  %15d  %17.0f  %12.3f  %13.3f  %10d  %13.3f  %10.3f  %10.3f  %10d  %10d  %13d  %8d\n", s.contactPairs, s.contactFrames,
               percentile(s.minDist, 0.05), mean(s.arrive), percentile(s.arrive, 0.95), s.notArrived, mean(s.calcMs), percentile(s.calcMs, 0.95),
               percentile(s.calcMs, 1), s.brakes, s.switches, s.zoneFrames, s.fieldFrames);
    }
    printf("пар с контактом — пары роботов, у которых расстояние между центрами хоть раз стало меньше 2·ROBOT_R (всего пар %d);\n"
           "приезд — когда робот в последний раз был не в цели; расчёт — один вызов plan() на одного робота;\n"
           "смен плана — кадры, где новая траектория в ближайшую секунду расходится с прошлой больше чем на 150 мм\n",
           nScenes * nBots * (nBots - 1) / 2);
    if (dump)
        fclose(dump);
    return 0;
}
