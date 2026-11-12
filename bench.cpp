/*
Бенчмарк минимизации на случайных сценах.
Повторяет minimize() из include/minimizing/minimize.h, но без отрисовки и с замером времени.
Если меняешь настройку nlopt в minimize(), поменяй и здесь.

Запуск: ./bench [число сцен = 1000] [seed = 42]
*/
#include <random>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <nlopt.h>
#include "include/Timer.h"
#include "include/const.h"
#include "include/metrics/metrics.h"
#include "include/minimizing/initialApprox.h"

using namespace std;

struct Scene
{
    Point start, end, vStart, vEnd;
    AbsRigBody enemies[MAX_ROBOT_COUNT];
    int nEnemies;
};

struct RunResult
{
    double tApprox, tOpt, minf, conX, conY, collision, pathTime;
    int nloptCode, nEval;
};

// счётчик вызовов целевой функции
static int evalCounter = 0;
double countingObjective(unsigned n, const double *x, double *grad, void *voidData)
{
    evalCounter++;
    return metrics::minimizing(n, x, grad, voidData);
}

RunResult runOne(const Scene &s)
{
    RunResult res;
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
    double lower_bounds[data.n], x[data.n];
    for (int i = 0; i < data.n; i++)
        lower_bounds[i] = (i % 2) ? 0 : -1e14;
    void *voidData = static_cast<void *>(&data);
    nlopt_set_lower_bounds(opt, lower_bounds);
    nlopt_set_min_objective(opt, countingObjective, voidData);
    double tolerances[3] = {1e-4, 1e-4};
    nlopt_add_equality_mconstraint(opt, 2, metrics::constraints, voidData, tolerances);
    nlopt_set_xtol_rel(opt, 1e-6);
    nlopt_set_ftol_rel(opt, 1e-6);

    AbsRigBody enemiesCopy[MAX_ROBOT_COUNT];
    for (int i = 0; i < s.nEnemies; i++)
        enemiesCopy[i] = s.enemies[i];
    initialApprox(s.start, s.end, s.vStart, s.vEnd, enemiesCopy, s.nEnemies, x, data.n);
    res.tApprox = timer.time();

    timer.reset();
    evalCounter = 0;
    res.nloptCode = nlopt_optimize(opt, x, &res.minf);
    res.tOpt = timer.time();
    res.nEval = evalCounter;
    nlopt_destroy(opt);

    // пересчитать метрики именно для итогового x (последний вызов nlopt мог быть в другой точке)
    metrics::mainActions(data.n, x, &data);
    res.conX = data.resultCon[0] * MAX_VEL * MAX_VEL / MAX_ACC; // мм
    res.conY = data.resultCon[1] * MAX_VEL * MAX_VEL / MAX_ACC; // мм
    res.collision = data.resultCon[2] * MAX_VEL / MAX_ACC;       // с внутри препятствий (без SAFE_DIST)
    res.pathTime = data.t[data.n + 2];                           // с
    return res;
}

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
    std::sort(v.begin(), v.end());
    return v[min(v.size() - 1, size_t(p * (v.size() - 1) + 0.5))];
}

int main(int argc, char **argv)
{
    int nScenes = argc > 1 ? atoi(argv[1]) : 1000;
    unsigned seed = argc > 2 ? atoi(argv[2]) : 42;
    mt19937 gen(seed);

    vector<double> tTotal, tApprox, tOpt, evals;
    int codes[10] = {0}, negCodes = 0, blocked = 0, missed = 0, collided = 0;
    double worstMiss = 0;
    int worstIdx = -1;
    for (int k = 0; k < nScenes; k++)
    {
        Scene s = randomScene(gen);
        blocked += straightBlocked(s);
        RunResult r = runOne(s);
        tApprox.push_back(r.tApprox * 1000);
        tOpt.push_back(r.tOpt * 1000);
        tTotal.push_back((r.tApprox + r.tOpt) * 1000);
        evals.push_back(r.nEval);
        if (r.nloptCode < 0)
            negCodes++;
        else if (r.nloptCode < 10)
            codes[r.nloptCode]++;
        double miss = hypot(r.conX, r.conY);
        if (miss > 50)
            missed++;
        if (miss > worstMiss)
        {
            worstMiss = miss;
            worstIdx = k;
        }
        if (r.collision > 1e-3)
            collided++;
    }

    auto mean = [](const vector<double> &v)
    {
        double sum = 0;
        for (double x : v)
            sum += x;
        return sum / v.size();
    };
    printf("сцен: %d (seed %u), прямая start-end перекрыта препятствием в %d\n", nScenes, seed, blocked);
    // printf считает ширину в байтах, а кириллица в UTF-8 занимает 2 байта на символ, поэтому выравниваем вручную
    auto pad = [](const char *s, int width, bool right = false)
    {
        int chars = 0;
        for (const char *c = s; *c; c++)
            if ((*c & 0xC0) != 0x80)
                chars++;
        if (right)
            printf("%*s%s", max(0, width - chars), "", s);
        else
            printf("%s%*s", s, max(0, width - chars), "");
    };
    auto row = [&](const char *name, const vector<double> &v)
    {
        pad(name, 16);
        printf("%9.3f%9.3f%9.3f%9.3f%9.3f\n", mean(v), percentile(v, .5), percentile(v, .95), percentile(v, 1), percentile(v, 0));
    };
    pad("время, мс", 16);
    const char *headers[] = {"среднее", "медиана", "p95", "max", "min"};
    for (const char *h : headers)
        pad(h, 9, true);
    printf("\n");
    row("initialApprox", tApprox);
    row("nlopt_optimize", tOpt);
    row("всего", tTotal);
    printf("вызовов цели: среднее %.1f, медиана %.0f, max %.0f\n", mean(evals), percentile(evals, .5), percentile(evals, 1));
    printf("коды nlopt:");
    for (int c = 1; c < 10; c++)
        if (codes[c])
            printf(" %d→%d", c, codes[c]);
    printf(", ошибки (<0)→%d\n", negCodes);
    printf("промах по конечной точке > 50 мм: %d (худший %.1f мм, сцена %d)\n", missed, worstMiss, worstIdx);
    printf("траектория заходит в препятствие: %d\n", collided);
    return 0;
}
