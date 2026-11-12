/*
Бенчмарк планировщиков на случайных сценах: минимизатор (SLSQP), перебор (sampler.h) и перебор с SLSQP-доводкой.
Все методы возвращают траекторию в общем формате sampling::Trajectory и оцениваются одной функцией
sampling::assess. Чтобы добавить метод, напиши функцию Scene → MethodOut и впиши её в массив methods в main().

runSlsqp() повторяет minimize() из include/minimizing/minimize.h, но без отрисовки и с замером времени.
Если меняешь настройку nlopt в minimize(), поменяй и здесь.

Запуск: ./bench-app [число сцен = 1000] [seed = 42] [режим = normal | tight | moving | file (сцены из файла SCENES)] [номер сцены для подробного вывода]
*/
#include <random>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <nlopt.h>
#include "include/Timer.h"
#include "include/const.h"
#include "include/metrics/metrics.h"
#include "include/minimizing/initialApprox.h"
#include "include/sampling/sampler.h"
#include "include/sampling/slsqpRefine.h"

using namespace std;

struct Scene
{
    Point start, end, vStart, vEnd;
    AbsRigBody enemies[MAX_ROBOT_COUNT];
    int nEnemies;
};

// что возвращает метод: траектория и время расчёта
struct MethodOut
{
    sampling::Trajectory traj;
    double timeMs;
};

struct Method
{
    const char *name;
    MethodOut (*run)(const Scene &, unsigned);
};

// подробности последнего запуска минимизатора
struct SlsqpInfo
{
    double tApprox, tOpt, conX, conY, collision;
    int nloptCode, nEval;
} slsqpInfo;

// подробности последнего запуска перебора и доводки
struct SamplerInfo
{
    sampling::Result result;
    sampling::RefineResult refine;
    double tSampler, tRefine;
} samplerInfo;

// параметры перебора и доводки; бюджет и лимит SLSQP можно задать переменными окружения BUDGET, PENALTY, MAXEVAL и JITTER
sampling::Params samplerParams;
sampling::RefineParams refineParams;

// защита бенчмарка от зависания минимизатора (код nlopt 5 = лимит сработал)
#define SLSQP_BENCH_MAXEVAL 3000

// счётчик вызовов целевой функции
static int evalCounter = 0;
double countingObjective(unsigned n, const double *x, double *grad, void *voidData)
{
    evalCounter++;
    return metrics::minimizing(n, x, grad, voidData);
}

// минимизатор: initialApprox + SLSQP
MethodOut runSlsqp(const Scene &s, unsigned)
{
    MethodOut out;
    Timer timer;
    MetricsData data;
    data.pos = s.start;
    data.endPos = s.end;
    data.vel = s.vStart;
    data.endVel = s.vEnd;
    data.nEnemies = s.nEnemies;
    data.n = 2 * N_SOLVER_PAIRS;
    for (int i = 0; i < s.nEnemies; i++)
        data.enemies[i] = s.enemies[i];
    for (int i = 0; i < data.n; i++)
        data.x[i] = NAN; // чтобы кэш isRepeated не сработал на мусоре

    timer.reset();
    nlopt_opt opt = nlopt_create(NLOPT_LD_SLSQP, data.n);
    double lower_bounds[data.n], x[data.n], minf;
    for (int i = 0; i < data.n; i++)
        lower_bounds[i] = (i % 2) ? 0 : -1e14;
    void *voidData = static_cast<void *>(&data);
    nlopt_set_lower_bounds(opt, lower_bounds);
    nlopt_set_min_objective(opt, countingObjective, voidData);
    double tolerances[3] = {1e-4, 1e-4};
    nlopt_add_equality_mconstraint(opt, 2, metrics::constraints, voidData, tolerances);
    nlopt_set_xtol_rel(opt, 1e-6);
    nlopt_set_ftol_rel(opt, 1e-6);
    // в minimize() этого лимита нет. Он нужен только бенчмарку: на тесных сценах SLSQP иногда не останавливается
    // вообще (сцена 113 при seed 42, режим tight). В обычных сценах максимум около 400 вызовов, лимит не срабатывает
    nlopt_set_maxeval(opt, SLSQP_BENCH_MAXEVAL);

    AbsRigBody enemiesCopy[MAX_ROBOT_COUNT];
    for (int i = 0; i < s.nEnemies; i++)
        enemiesCopy[i] = s.enemies[i];
    initialApprox(s.start, s.end, s.vStart, s.vEnd, enemiesCopy, s.nEnemies, x, data.n);
    slsqpInfo.tApprox = timer.time();

    timer.reset();
    evalCounter = 0;
    slsqpInfo.nloptCode = nlopt_optimize(opt, x, &minf);
    slsqpInfo.tOpt = timer.time();
    slsqpInfo.nEval = evalCounter;
    nlopt_destroy(opt);
    out.timeMs = (slsqpInfo.tApprox + slsqpInfo.tOpt) * 1000;

    // пересчитать метрики именно для итогового x (последний вызов nlopt мог быть в другой точке)
    metrics::mainActions(data.n, x, &data);
    slsqpInfo.conX = data.resultCon[0] * MAX_VEL * MAX_VEL / MAX_ACC; // мм
    slsqpInfo.conY = data.resultCon[1] * MAX_VEL * MAX_VEL / MAX_ACC; // мм
    slsqpInfo.collision = data.resultCon[2] * MAX_VEL / MAX_ACC;       // с внутри препятствий (без SAFE_DIST)
    out.traj = sampling::fromMetrics(data);
    return out;
}

// прямой bang-bang без учёта препятствий: ориентир для T и показатель того, насколько часто препятствия
// вообще мешают. Это не строгая нижняя оценка: прямой bang-bang при ненулевых vStart/vEnd не оптимален по
// времени, SLSQP бывает на 0.3–3 % быстрее
MethodOut runDirect(const Scene &s, unsigned)
{
    MethodOut out;
    Timer timer;
    sampling::Params params = samplerParams;
    out.traj.reset(s.start, s.vStart);
    sampling::appendBangBang(out.traj, s.vEnd, s.end, params);
    out.timeMs = timer.time() * 1000;
    return out;
}

// перебор, один вызов без тёплого старта
MethodOut runSampler(const Scene &s, unsigned seed)
{
    MethodOut out;
    mt19937 gen(seed);
    Timer timer;
    samplerInfo.result = sampling::plan(s.start, s.end, s.vStart, s.vEnd, s.enemies, s.nEnemies, gen, nullptr, samplerParams);
    samplerInfo.tSampler = out.timeMs = timer.time() * 1000;
    out.traj = samplerInfo.result.traj;
    return out;
}

// перебор на пятом кадре подряд: та же сцена, тёплый старт с прошлого вызова. Время — только последнего вызова
MethodOut runSamplerWarm(const Scene &s, unsigned seed)
{
    MethodOut out;
    mt19937 gen(seed);
    sampling::State state;
    sampling::Result res;
    Timer timer;
    for (int frame = 0; frame < 5; frame++)
    {
        timer.reset();
        res = sampling::plan(s.start, s.end, s.vStart, s.vEnd, s.enemies, s.nEnemies, gen, &state, samplerParams);
        out.timeMs = timer.time() * 1000;
    }
    out.traj = res.traj;
    return out;
}

// перебор + SLSQP-доводка лучшего кандидата
MethodOut runSamplerRefined(const Scene &s, unsigned seed)
{
    MethodOut out;
    mt19937 gen(seed);
    Timer timer;
    sampling::Result res = sampling::plan(s.start, s.end, s.vStart, s.vEnd, s.enemies, s.nEnemies, gen, nullptr, samplerParams);
    double tPlan = timer.time();
    samplerInfo.refine = sampling::refineSLSQP(res, s.start, s.end, s.vStart, s.vEnd, s.enemies, s.nEnemies, refineParams);
    out.timeMs = timer.time() * 1000;
    samplerInfo.tRefine = out.timeMs - tPlan * 1000;
    out.traj = samplerInfo.refine.traj;
    return out;
}

// обычные сцены: препятствия не ближе 2*ROBOT_R + 3*SAFE_DIST к start и end
Scene randomScene(mt19937 &gen)
{
    uniform_real_distribution<double> ux(-FIELD_DX / 2 + 500, FIELD_DX / 2 - 500), uy(-FIELD_DY / 2 + 500, FIELD_DY / 2 - 500);
    uniform_real_distribution<double> uAng(-M_PI, M_PI), uVel(0, MAX_VEL), u01(0, 1), uOff(-700, 700);
    Scene s;
    do
    {
        s.start = Point(ux(gen), uy(gen));
        s.end = Point(ux(gen), uy(gen));
    } while ((s.end - s.start).mag() < 1500);
    s.vStart = Point(1, 0) + uAng(gen);
    s.vStart *= uVel(gen);
    s.vEnd = Point(1, 0) + uAng(gen);
    s.vEnd *= uVel(gen);
    s.nEnemies = 6;
    Point dir = (s.end - s.start).unity(), normal(-dir.y, dir.x);
    for (int i = 0; i < s.nEnemies; i++)
    {
        Point p;
        do
        {
            // половина препятствий около прямой start-end, чтобы их приходилось объезжать, остальные где угодно
            if (i % 2 == 0)
                p = s.start + (s.end - s.start) * (0.15 + 0.7 * u01(gen)) + normal * uOff(gen);
            else
                p = Point(ux(gen), uy(gen));
        } while ((p - s.start).mag() < 2 * ROBOT_R + 3 * SAFE_DIST || (p - s.end).mag() < 2 * ROBOT_R + 3 * SAFE_DIST);
        s.enemies[i] = AbsRigBody(p, 2 * ROBOT_R, Point(0, 0));
    }
    return s;
}

// тесные сцены: start или end стоит вплотную к препятствию (зазор 1..30 мм, но не внутри), препятствия слиплись
// в две группы по три: одна у этой точки, вторая на пути
Scene tightScene(mt19937 &gen)
{
    uniform_real_distribution<double> ux(-FIELD_DX / 2 + 500, FIELD_DX / 2 - 500), uy(-FIELD_DY / 2 + 500, FIELD_DY / 2 - 500);
    uniform_real_distribution<double> uAng(-M_PI, M_PI), uVel(0, MAX_VEL), u01(0, 1), uOff(-400, 400), uGap(1, 30), uLink(0.6, 0.95);
    const double rad = 2 * ROBOT_R;
    Scene s;
    do
    {
        s.start = Point(ux(gen), uy(gen));
        s.end = Point(ux(gen), uy(gen));
    } while ((s.end - s.start).mag() < 1500);
    s.vStart = Point(1, 0) + uAng(gen);
    s.vStart *= uVel(gen);
    s.vEnd = Point(1, 0) + uAng(gen);
    s.vEnd *= uVel(gen);
    s.nEnemies = 6;
    bool atStart = u01(gen) < 0.5;
    Point anchor = atStart ? s.start : s.end;
    Point dir = (s.end - s.start).unity(), normal(-dir.y, dir.x);
    for (int i = 0; i < s.nEnemies; i++)
    {
        Point p;
        do
        {
            if (i == 0)
                p = anchor + (Point(1, 0) + uAng(gen)) * (rad + uGap(gen)); // вплотную к start или end
            else if (i == 3)
                p = s.start + (s.end - s.start) * (0.25 + 0.5 * u01(gen)) + normal * uOff(gen); // вторая группа на пути
            else
            {
                // прилипает к случайному препятствию своей группы
                int first = i < 3 ? 0 : 3;
                int base = first + int(u01(gen) * (i - first));
                p = s.enemies[base].getPos() + (Point(1, 0) + uAng(gen)) * (2 * rad * uLink(gen));
            }
        } while ((p - s.start).mag() < rad + 1 || (p - s.end).mag() < rad + 1);
        s.enemies[i] = AbsRigBody(p, rad, Point(0, 0));
    }
    // скорость в точке у препятствий: не больше половины MAX_VEL, на старте направлена не в ближние препятствия,
    // на финише не из них (иначе столкновение неизбежно при любой траектории). Если угодить всем не вышло, скорость нулевая
    Point &vel = atStart ? s.vStart : s.vEnd;
    double sign = atStart ? 1 : -1;
    vel = vel * 0.5;
    bool bad = true;
    for (int iter = 0; iter < 4 && bad; iter++)
    {
        bad = false;
        for (int i = 0; i < s.nEnemies; i++)
        {
            Point out = anchor - s.enemies[i].getPos();
            double dist = out.mag();
            out = out / dist;
            if (dist - rad < 150 && sign * (vel ^ out) < -1e-9)
            {
                vel = vel - out * (2 * (vel ^ out));
                bad = true;
            }
        }
    }
    if (bad)
        vel = Point(0, 0);
    return s;
}

// сцены с движущимися препятствиями (модель: постоянная скорость). Три препятствия пересекают путь:
// каждое приходит в точку на отрезке start→end примерно тогда же, когда туда доехал бы робот (±0.5 с),
// поперёк пути (±0.5 рад) со скоростью 0.3..1 MAX_VEL. Остальные три — где угодно, скорость до MAX_VEL
// в любую сторону. Препятствие не стоит у start и в первые 0.3 с не подъезжает к нему ближе rad + 50 мм,
// иначе столкновение было бы неизбежным при любой траектории
Scene movingScene(mt19937 &gen)
{
    uniform_real_distribution<double> ux(-FIELD_DX / 2 + 500, FIELD_DX / 2 - 500), uy(-FIELD_DY / 2 + 500, FIELD_DY / 2 - 500);
    uniform_real_distribution<double> uAng(-M_PI, M_PI), uVel(0, MAX_VEL), u01(0, 1), uSym(-1, 1);
    const double rad = 2 * ROBOT_R;
    Scene s;
    do
    {
        s.start = Point(ux(gen), uy(gen));
        s.end = Point(ux(gen), uy(gen));
    } while ((s.end - s.start).mag() < 1500);
    s.vStart = Point(1, 0) + uAng(gen);
    s.vStart *= uVel(gen);
    s.vEnd = Point(1, 0) + uAng(gen);
    s.vEnd *= uVel(gen);
    s.nEnemies = 6;
    Point line = s.end - s.start, dir = line.unity(), normal(-dir.y, dir.x);
    double len = line.mag();
    for (int i = 0; i < s.nEnemies; i++)
    {
        Point p, v;
        for (int attempt = 0;; attempt++)
        {
            if (i % 2 == 0)
            {
                double f = 0.2 + 0.6 * u01(gen);
                Point cross = s.start + line * f;
                double tArrive = f * len / (0.8 * MAX_VEL) + 0.5 * uSym(gen);
                Point side = (u01(gen) < 0.5 ? normal : -normal) + 0.5 * uSym(gen);
                v = side * (MAX_VEL * (0.3 + 0.7 * u01(gen)));
                p = cross - v * max(0.0, tArrive);
            }
            else
            {
                p = Point(ux(gen), uy(gen));
                v = (Point(1, 0) + uAng(gen)) * uVel(gen);
            }
            // ближайшее сближение со start за первые 0.3 с
            Point rel = s.start - p;
            double tc = v.mag2() > 0 ? max(0.0, min(0.3, (rel ^ v) / v.mag2())) : 0;
            bool nearStart = (p + v * tc - s.start).mag() < rad + 50;
            bool inField = abs(p.x) < FIELD_DX / 2 + 1000 && abs(p.y) < FIELD_DY / 2 + 1000;
            if ((!nearStart && inField && (p - s.start).mag() >= rad + 3 * SAFE_DIST) || attempt > 100)
                break;
        }
        s.enemies[i] = AbsRigBody(p, rad, v);
    }
    return s;
}

// сцены из файла (например, нарезанные из логов игр): по строке на сцену
// start.x start.y vStart.x vStart.y end.x end.y vEnd.x vEnd.y n, затем n раз x y rad vx vy. Единицы мм, мм/с
vector<Scene> loadScenes(const char *path)
{
    vector<Scene> scenes;
    FILE *f = fopen(path, "r");
    if (!f)
    {
        fprintf(stderr, "не открыть файл сцен %s\n", path);
        exit(1);
    }
    Scene s;
    double v[8];
    while (fscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf %d", v, v + 1, v + 2, v + 3, v + 4, v + 5, v + 6, v + 7, &s.nEnemies) == 9)
    {
        s.start = Point(v[0], v[1]);
        s.vStart = Point(v[2], v[3]);
        s.end = Point(v[4], v[5]);
        s.vEnd = Point(v[6], v[7]);
        if (s.nEnemies > MAX_ROBOT_COUNT)
            s.nEnemies = MAX_ROBOT_COUNT;
        for (int i = 0; i < s.nEnemies; i++)
        {
            double x, y, r, vx, vy;
            if (fscanf(f, "%lf %lf %lf %lf %lf", &x, &y, &r, &vx, &vy) != 5)
                break;
            s.enemies[i] = AbsRigBody(Point(x, y), r, Point(vx, vy));
        }
        scenes.push_back(s);
    }
    fclose(f);
    return scenes;
}

bool straightBlocked(const Scene &s)
{
    Point prod[2];
    for (int i = 0; i < s.nEnemies; i++)
        if (vecAux::lineCircleIntersect(prod, s.start, s.end, s.enemies[i].getPos(), s.enemies[i].getRad()))
            return true;
    return false;
}

double percentile(vector<double> v, double p)
{
    if (v.empty())
        return NAN;
    std::sort(v.begin(), v.end());
    return v[min(v.size() - 1, size_t(p * (v.size() - 1) + 0.5))];
}

double mean(const vector<double> &v)
{
    double sum = 0;
    for (double x : v)
        sum += x;
    return v.empty() ? NAN : sum / v.size();
}

// printf считает ширину в байтах, а кириллица в UTF-8 занимает 2 байта на символ, поэтому выравниваем вручную
void pad(const char *s, int width, bool right = false)
{
    int chars = 0;
    for (const char *c = s; *c; c++)
        if ((*c & 0xC0) != 0x80)
            chars++;
    if (right)
        printf("%*s%s", max(0, width - chars), "", s);
    else
        printf("%s%*s", s, max(0, width - chars), "");
}

// недопустимая траектория: промах больше 50 мм или заход в препятствие
bool isInvalid(const sampling::Quality &q)
{
    return q.miss > 50 || q.clearance < 0;
}

// одна строка JSON на пару (сцена, метод): сцена, точки траектории через 10 мс, время расчёта и качество
void dumpMethod(FILE *f, const Scene &s, int k, int m, const char *name, const MethodOut &out, const sampling::Quality &q)
{
    fprintf(f, "{\"scene\":%d,\"method\":%d,\"name\":\"%s\",\"timeMs\":%.4f,\"T\":%.4f,\"clearance\":%.2f,\"miss\":%.2f,", k, m, name, out.timeMs, q.T,
            q.clearance, q.miss);
    fprintf(f, "\"start\":[%.1f,%.1f],\"end\":[%.1f,%.1f],\"vStart\":[%.1f,%.1f],\"vEnd\":[%.1f,%.1f],\"enemies\":[", s.start.x, s.start.y, s.end.x,
            s.end.y, s.vStart.x, s.vStart.y, s.vEnd.x, s.vEnd.y);
    for (int i = 0; i < s.nEnemies; i++)
        fprintf(f, "%s[%.1f,%.1f,%.1f,%.1f,%.1f]", i ? "," : "", s.enemies[i].getPos().x, s.enemies[i].getPos().y, s.enemies[i].getRad(),
                s.enemies[i].getVel().x, s.enemies[i].getVel().y);
    fprintf(f, "],\"path\":[");
    double T = out.traj.duration();
    int nPts = max(2, int(T / 0.01) + 1);
    for (int i = 0; i <= nPts; i++)
    {
        Point p = out.traj.posAt(T * i / nPts);
        fprintf(f, "%s[%.1f,%.1f]", i ? "," : "", p.x, p.y);
    }
    fprintf(f, "]}\n");
}

int main(int argc, char **argv)
{
    int nScenes = argc > 1 ? atoi(argv[1]) : 1000;
    unsigned seed = argc > 2 ? atoi(argv[2]) : 42;
    const char *mode = argc > 3 ? argv[3] : "normal";
    bool tight = strcmp(mode, "tight") == 0, moving = strcmp(mode, "moving") == 0;
    // режим file: сцены из файла, путь в переменной окружения SCENES
    bool fromFile = strcmp(mode, "file") == 0;
    vector<Scene> fileScenes;
    if (fromFile)
    {
        if (!getenv("SCENES"))
        {
            fprintf(stderr, "режим file: задай путь к сценам в SCENES\n");
            return 1;
        }
        fileScenes = loadScenes(getenv("SCENES"));
        nScenes = min(nScenes, int(fileScenes.size()));
    }
    int detailIdx = argc > 4 ? atoi(argv[4]) : -1;
    mt19937 gen(seed);
    if (getenv("BUDGET"))
        samplerParams.budget = atoi(getenv("BUDGET"));
    if (getenv("PENALTY"))
        samplerParams.penalty = atof(getenv("PENALTY"));
    if (getenv("MARGIN_BASE"))
        samplerParams.marginBase = atof(getenv("MARGIN_BASE"));
    if (getenv("MARGIN_VREF"))
        samplerParams.marginVRef = atof(getenv("MARGIN_VREF"));
    if (getenv("RANDOM_SHARE"))
        samplerParams.randomShare = atof(getenv("RANDOM_SHARE"));
    if (getenv("PREDICT"))
        samplerParams.predictTangents = atoi(getenv("PREDICT")) != 0;
    if (getenv("HORIZON"))
        samplerParams.horizon = atof(getenv("HORIZON"));
    if (getenv("ACCEL"))
        samplerParams.accel = atof(getenv("ACCEL"));
    if (getenv("DECEL"))
        samplerParams.decel = atof(getenv("DECEL"));
    if (getenv("BRAKE"))
        samplerParams.emergencyBrake = atoi(getenv("BRAKE")) != 0;
    if (getenv("NM"))
        samplerParams.nelderMead = atoi(getenv("NM")) != 0;
    if (getenv("NM_STEP_POS"))
        samplerParams.nmStepPos = atof(getenv("NM_STEP_POS"));
    if (getenv("NM_STEP_VEL"))
        samplerParams.nmStepVel = atof(getenv("NM_STEP_VEL"));
    if (getenv("DIRECT_SHARE"))
        samplerParams.directShare = atoi(getenv("DIRECT_SHARE"));
    if (getenv("JITTER"))
        refineParams.jitter = atof(getenv("JITTER"));
    if (getenv("MAXEVAL"))
        refineParams.maxEval = atoi(getenv("MAXEVAL"));

    // сюда добавляются новые методы. Первый метод — база для сравнения
    const Method methods[] = {
        {"slsqp", runSlsqp},
        {"перебор", runSampler},
        {"перебор+slsqp", runSamplerRefined},
        {"перебор, кадр 5", runSamplerWarm},
        {"прямая без препятствий", runDirect},
    };
    const int nMethods = sizeof(methods) / sizeof(methods[0]);
    // DUMP=файл: выгрузить сцены и траектории всех методов в JSON Lines (для графиков)
    FILE *dump = getenv("DUMP") ? fopen(getenv("DUMP"), "w") : nullptr;

    vector<vector<double>> times(nMethods);
    vector<vector<sampling::Quality>> quality(nMethods);
    // старая статистика минимизатора
    vector<double> tApprox, tOpt, evals;
    int codes[10] = {0}, negCodes = 0, blocked = 0, missed = 0, collided = 0;
    double worstMiss = 0;
    int worstIdx = -1;
    // статистика перебора и доводки
    int samplerInfeasible = 0, samplerDirect = 0, samplerNotFound = 0, refineAccepted = 0, samplerBrake = 0;
    vector<double> refineGain, refineTime, refineEvals, samplerTried;
    int rejectMiss = 0, rejectClearance = 0, rejectClearanceSmall = 0, rejectNoGain = 0;
    double maxConvMiss = 0, maxConvDT = 0;

    for (int k = 0; k < nScenes; k++)
    {
        Scene s = fromFile ? fileScenes[k] : tight ? tightScene(gen) : moving ? movingScene(gen) : randomScene(gen);
        blocked += straightBlocked(s);
        for (int m = 0; m < nMethods; m++)
        {
            // у каждой сцены свой seed, одинаковый для всех методов
            MethodOut out = methods[m].run(s, seed * 7919u + k);
            times[m].push_back(out.timeMs);
            quality[m].push_back(sampling::assess(out.traj, s.end, s.enemies, s.nEnemies));
            if (dump)
                dumpMethod(dump, s, k, m, methods[m].name, out, quality[m].back());
            if (methods[m].run == runSlsqp)
            {
                tApprox.push_back(slsqpInfo.tApprox * 1000);
                tOpt.push_back(slsqpInfo.tOpt * 1000);
                evals.push_back(slsqpInfo.nEval);
                if (slsqpInfo.nloptCode < 0)
                    negCodes++;
                else if (slsqpInfo.nloptCode < 10)
                    codes[slsqpInfo.nloptCode]++;
                double miss = hypot(slsqpInfo.conX, slsqpInfo.conY);
                if (miss > 50)
                    missed++;
                if (miss > worstMiss)
                {
                    worstMiss = miss;
                    worstIdx = k;
                }
                if (slsqpInfo.collision > 1e-3)
                    collided++;
            }
            if (methods[m].run == runSampler)
            {
                samplerInfeasible += !samplerInfo.result.feasible;
                samplerBrake += samplerInfo.result.brake;
                samplerDirect += samplerInfo.result.direct;
                samplerNotFound += !samplerInfo.result.found;
                samplerTried.push_back(samplerInfo.result.nTried);
            }
            if (methods[m].run == runSamplerRefined)
            {
                const sampling::RefineResult &r = samplerInfo.refine;
                refineTime.push_back(samplerInfo.tRefine);
                refineEvals.push_back(r.nEval);
                maxConvMiss = max(maxConvMiss, r.convMiss);
                maxConvDT = max(maxConvDT, abs(r.convDT));
                if (!r.accepted)
                {
                    // причина отказа (первая подходящая)
                    if (r.refined.miss > refineParams.maxMiss)
                        rejectMiss++;
                    else if (r.refined.violation > 1e-6)
                    {
                        rejectClearance++;
                        rejectClearanceSmall += r.refined.violation < 1;
                    }
                    else
                        rejectNoGain++;
                }
                if (r.accepted)
                {
                    refineAccepted++;
                    refineGain.push_back((1 - r.quality.T / samplerInfo.result.traj.duration()) * 100);
                }
            }
            if (k == detailIdx)
            {
                const sampling::Quality &q = quality[m].back();
                if (m == 0)
                {
                    printf("сцена %d: start (%.0f, %.0f) v (%.0f, %.0f) → end (%.0f, %.0f) v (%.0f, %.0f)\n", k, s.start.x, s.start.y,
                           s.vStart.x, s.vStart.y, s.end.x, s.end.y, s.vEnd.x, s.vEnd.y);
                    for (int i = 0; i < s.nEnemies; i++)
                        printf("  препятствие %d: (%.0f, %.0f) r %.0f\n", i, s.enemies[i].getPos().x, s.enemies[i].getPos().y, s.enemies[i].getRad());
                }
                pad(methods[m].name, 18);
                printf("T %.3f с, зазор %.1f мм, промах %.2f мм, участков %d\n", q.T, q.clearance, q.miss, out.traj.n);
                if (methods[m].run == runSampler && !samplerInfo.result.direct)
                    printf("  P (%.0f, %.0f), vP (%.0f, %.0f)\n", samplerInfo.result.P.x, samplerInfo.result.P.y, samplerInfo.result.vP.x,
                           samplerInfo.result.vP.y);
                for (int i = 0; i <= 20; i++)
                {
                    Point p = out.traj.posAt(q.T * i / 20);
                    printf("%s(%.0f, %.0f)", i ? " " : "  путь: ", p.x, p.y);
                }
                printf("\n");
            }
        }
    }

    printf("сцен: %d (seed %u, режим %s), прямая start-end перекрыта препятствием в %d\n", nScenes, seed, mode, blocked);

    // ---- минимизатор, как раньше ----
    auto row = [&](const char *name, const vector<double> &v)
    {
        pad(name, 16);
        printf("%9.3f%9.3f%9.3f%9.3f%9.3f\n", mean(v), percentile(v, .5), percentile(v, .95), percentile(v, 1), percentile(v, 0));
    };
    printf("\n== минимизатор (SLSQP) ==\n");
    pad("время, мс", 16);
    const char *headers[] = {"среднее", "медиана", "p95", "max", "min"};
    for (const char *h : headers)
        pad(h, 9, true);
    printf("\n");
    row("initialApprox", tApprox);
    row("nlopt_optimize", tOpt);
    row("всего", times[0]);
    printf("вызовов цели: среднее %.1f, медиана %.0f, max %.0f\n", mean(evals), percentile(evals, .5), percentile(evals, 1));
    printf("коды nlopt:");
    for (int c = 1; c < 10; c++)
        if (codes[c])
            printf(" %d→%d", c, codes[c]);
    printf(", ошибки (<0)→%d\n", negCodes);
    printf("промах по конечной точке > 50 мм: %d (худший %.1f мм, сцена %d)\n", missed, worstMiss, worstIdx);
    printf("траектория заходит в препятствие (по metrics, > 1 мс внутри): %d\n", collided);

    // ---- все методы, общая оценка ----
    printf("\n== все методы, общая оценка (sampling::assess) ==\n");
    pad("метод", 18);
    const char *cols[] = {"t ср, мс", "медиана", "p95", "max", "T ср, с", "недопуст.", "промах>50", "зазор<0", "зазор<SAFE", "<0 за 1 с"};
    for (const char *h : cols)
        pad(h, 11, true);
    printf("\n");
    for (int m = 0; m < nMethods; m++)
    {
        int invalid = 0, miss = 0, inside = 0, unsafe = 0, insideH = 0;
        vector<double> T;
        for (const sampling::Quality &q : quality[m])
        {
            invalid += isInvalid(q);
            miss += q.miss > 50;
            inside += q.clearance < 0;
            unsafe += q.violation > 1e-6;
            insideH += q.clearanceH < 0;
            T.push_back(q.T);
        }
        pad(methods[m].name, 18);
        printf("%11.3f%11.3f%11.3f%11.3f%11.3f%11d%11d%11d%11d%11d\n", mean(times[m]), percentile(times[m], .5), percentile(times[m], .95),
               percentile(times[m], 1), mean(T), invalid, miss, inside, unsafe, insideH);
    }
    printf("недопуст. = промах > 50 мм или зазор < 0; зазор<SAFE = зазор меньше требуемого (SAFE_DIST либо зазор в start/end, если он меньше);\n"
           "<0 за 1 с = зазор < 0, если движущиеся препятствия проверять только первую секунду (горизонт перебора)\n");

    // ---- сравнение T с базой на сценах, где оба метода допустимы ----
    printf("\n== T относительно %s (только сцены, где оба допустимы; «быстрее» = разница T больше 0.5 %%) ==\n", methods[0].name);
    pad("метод", 18);
    const char *cmpCols[] = {"сцен", "ratio ср", "медиана", "p95", "max", "быстрее", "медленнее", "наравне"};
    for (const char *h : cmpCols)
        pad(h, 11, true);
    printf("\n");
    for (int m = 1; m < nMethods; m++)
    {
        vector<double> ratio;
        int faster = 0, slower = 0;
        for (int k = 0; k < nScenes; k++)
        {
            if (isInvalid(quality[0][k]) || isInvalid(quality[m][k]))
                continue;
            double r = quality[m][k].T / quality[0][k].T;
            ratio.push_back(r);
            faster += r < 0.995;
            slower += r > 1.005;
        }
        pad(methods[m].name, 18);
        printf("%11d%11.4f%11.4f%11.4f%11.4f%11d%11d%11d\n", int(ratio.size()), mean(ratio), percentile(ratio, .5), percentile(ratio, .95),
               percentile(ratio, 1), faster, slower, int(ratio.size()) - faster - slower);
    }

    // ---- перебор ----
    // T относительно прямой без препятствий (последний метод): ориентир, а не строгая нижняя оценка
    {
        printf("\n== T относительно прямой без препятствий (ориентир, не строгая нижняя оценка; сцены, где метод допустим) ==\n");
        printf("прямая без препятствий заходит в препятствие в %d сценах из %d\n",
               int(std::count_if(quality[nMethods - 1].begin(), quality[nMethods - 1].end(), [](const sampling::Quality &q)
                                 { return q.clearance < 0; })),
               nScenes);
        for (int m = 0; m < nMethods - 1; m++)
        {
            vector<double> r;
            for (int k = 0; k < nScenes; k++)
                if (!isInvalid(quality[m][k]))
                    r.push_back(quality[m][k].T / quality[nMethods - 1][k].T);
            pad(methods[m].name, 24);
            printf("сцен %4d, ratio ср %.4f, медиана %.4f, p95 %.4f, max %.4f\n", int(r.size()), mean(r), percentile(r, .5), percentile(r, .95),
                   percentile(r, 1));
        }
    }
    printf("\n== перебор ==\n");
    printf("прямая траектория свободна: %d, недопустимо по собственной оценке: %d, ничего не построилось: %d, аварийное торможение: %d\n",
           samplerDirect, samplerInfeasible, samplerNotFound, samplerBrake);
    printf("бюджет %d; кандидатов за вызов: среднее %.1f, max %.0f\n", samplerParams.budget, mean(samplerTried), percentile(samplerTried, 1));
    printf("сцены, где у перебора зазор меньше требуемого:");
    for (int k = 0, shown = 0; k < nScenes && shown < 20; k++)
        if (quality[1][k].violation > 1e-6)
        {
            printf(" %d (%.0f мм)", k, quality[1][k].clearance);
            shown++;
        }
    printf("\n");
    // сцены, где перебор сильнее всего проигрывает базе
    vector<pair<double, int>> worst;
    for (int k = 0; k < nScenes; k++)
        if (!isInvalid(quality[0][k]) && !isInvalid(quality[1][k]))
            worst.push_back({quality[1][k].T / quality[0][k].T, k});
    std::sort(worst.rbegin(), worst.rend());
    printf("худшие сцены перебора относительно %s:\n", methods[0].name);
    for (int i = 0; i < min(5, int(worst.size())); i++)
    {
        int k = worst[i].second;
        printf("  сцена %4d: ratio %.3f, T %.3f против %.3f с, зазор %.0f против %.0f мм, промах базы %.1f мм\n", k, worst[i].first, quality[1][k].T,
               quality[0][k].T, quality[1][k].clearance, quality[0][k].clearance, quality[0][k].miss);
    }

    // ---- доводка ----
    printf("\n== SLSQP-доводка лучшего кандидата перебора (maxeval %d) ==\n", refineParams.maxEval);
    double gainSum = 0;
    for (double g : refineGain)
        gainSum += g;
    printf("принята в %d сценах из %d; выигрыш по T на принятых: среднее %.2f %%, медиана %.2f %%, max %.2f %%; по всем сценам %.2f %%\n",
           refineAccepted, nScenes, mean(refineGain), percentile(refineGain, .5), percentile(refineGain, 1), gainSum / nScenes);
    printf("отклонена: промах > %.0f мм — %d, зазор меньше требуемого — %d (из них меньше чем на 1 мм — %d), T не уменьшилось — %d\n",
           refineParams.maxMiss, rejectMiss, rejectClearance, rejectClearanceSmall, rejectNoGain);
    printf("цена: среднее %.3f мс, медиана %.3f, p95 %.3f, max %.3f (перебор сам: среднее %.3f мс); вызовов цели в среднем %.1f\n",
           mean(refineTime), percentile(refineTime, .5), percentile(refineTime, .95), percentile(refineTime, 1), mean(times[1]), mean(refineEvals));
    printf("точность перевода в пары: худший промах %.2e мм, худшая разница T %.2e с\n", maxConvMiss, maxConvDT);
    if (dump)
        fclose(dump);
    return 0;
}
