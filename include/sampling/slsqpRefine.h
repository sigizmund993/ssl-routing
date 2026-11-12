/*
Связь перебора с минимизатором: перевод траектории минимизатора в общий формат и доводка
лучшего кандидата перебора через SLSQP.
Подключает metrics.h, а с ним drawer.h: программа с этим заголовком при старте открывает окно SFML.
*/
#pragma once

#include <nlopt.h>
#include "sampler.h"
#include "../metrics/metrics.h"

namespace sampling
{
    // траектория минимизатора (после metrics::mainActions) в общем формате.
    // Участок i: разгон v[i]→ с ускорением a[i], у длинного ещё РПД; в конце равноускоренный переход к endVel
    Trajectory fromMetrics(const MetricsData &data)
    {
        Trajectory tr;
        int n = data.n;
        tr.reset(data.pos, data.vel);
        for (int i = 0; i < n / 2; i++)
        {
            tr.append(data.t[2 * i + 1] - data.t[2 * i], data.a[i]);
            if (data.isLong[i])
                tr.append(data.t[2 * i + 2] - data.t[2 * i + 1], Point(0, 0));
        }
        tr.append(data.t[n + 1] - data.t[n], data.a[n / 2]);
        return tr;
    }

    // угол целевой скорости минимизатора для разгона from→to: точка, где луч from→to пересекает окружность MAX_VEL
    double targetAngle(const Point &from, const Point &to)
    {
        Point d = to - from;
        double dm = d.mag();
        if (dm < 1e-9)
            return to.mag() > 1e-9 ? to.arg() : 0;
        d = d / dm;
        double b = from ^ d, disc = b * b - (from.mag2() - double(MAX_VEL) * MAX_VEL);
        double l = -b + sqrt(max(0.0, disc));
        return (from + d * max(l, dm)).arg();
    }

    // перевести кандидата перебора в n/2 пар (угол, длительность) минимизатора.
    // Каждый bang-bang даёт две фазы: «разгон + РПД» и «торможение». Обе точно ложатся в модель минимизатора:
    // разгон по прямой в пространстве скоростей — это движение к точке на окружности MAX_VEL, РПД — её удержание.
    // Пары делятся между фазами поровну (одна фаза = несколько пар с тем же углом). Последняя длительность
    // делится пополам: вторую половину торможения минимизатор проезжает сам на финальном участке.
    // При jitter = 0 перевод точный (промах ~1e-4 мм), но SLSQP из такой точки не уходит; см. комментарий ниже.
    void toPairs(const Result &res, double *x, int n, double jitter = 0)
    {
        int nPhases = 2 * res.nParts, nPairs = n / 2, j = 0;
        for (int p = 0; p < nPhases; p++)
        {
            const BangPart &part = res.parts[p / 2];
            double angle, dur;
            if (p % 2 == 0)
            {
                angle = part.tc > 0 ? part.Vm.arg() : targetAngle(part.v0, part.Vm);
                dur = part.ta + part.tc;
            }
            else
            {
                angle = targetAngle(part.Vm, part.v1);
                dur = part.td;
            }
            int count = nPairs / nPhases + (p < nPairs % nPhases ? 1 : 0);
            for (int k = 0; k < count; k++, j++)
            {
                // jitter разводит углы соседних пар одной фазы: при равных углах у следующей пары tMax = 0,
                // а там градиент минимизатора по углу имеет излом, и SLSQP не может сдвинуться с места
                x[2 * j] = angle + jitter * (k - (count - 1) / 2.0);
                x[2 * j + 1] = dur / count;
            }
        }
        x[n - 1] /= 2;
    }

    struct RefineParams
    {
        int maxEval = 40;      // лимит вызовов цели в SLSQP
        double jitter = 1e-3;  // рад: на сколько развести углы соседних пар одной фазы (0 — точный перевод)
        double maxMiss = 5;    // мм: с каким промахом результат ещё принимается
        double minGain = 1e-4; // доля T: меньший выигрыш улучшением не считается
    };

    struct RefineResult
    {
        Trajectory traj;      // итог: доведённая траектория, если принята, иначе исходная
        bool accepted = false;
        Quality quality;      // качество итога
        Quality refined;      // качество того, что выдал SLSQP (принято или нет)
        double convMiss = 0;  // мм: промах сразу после перевода в пары (точность перевода)
        double convDT = 0;    // с: разница T после перевода в пары
        int nloptCode = 0, nEval = 0;
    };

    // доводка: SLSQP из лучшего кандидата перебора. Результат принимается, только если он допустим
    // по общей оценке (промах <= maxMiss, зазор не нарушен) и T стало меньше
    RefineResult refineSLSQP(const Result &res, const Point &start, const Point &end, const Point &vStart, const Point &vEnd,
                             const AbsRigBody *enemies, int nEnemies, const RefineParams &params = RefineParams())
    {
        RefineResult out;
        out.traj = res.traj;
        out.quality = out.refined = assess(res.traj, end, enemies, nEnemies);
        if (!res.found || res.nParts == 0)
            return out;

        MetricsData data;
        data.pos = start;
        data.endPos = end;
        data.vel = vStart;
        data.endVel = vEnd;
        data.nEnemies = nEnemies;
        data.n = 2 * N_SOLVER_PAIRS;
        for (int i = 0; i < nEnemies; i++)
            data.enemies[i] = enemies[i];
        for (int i = 0; i < data.n; i++)
            data.x[i] = NAN; // чтобы кэш isRepeated не сработал на мусоре

        double x[data.n], lowerBounds[data.n], minf;
        toPairs(res, x, data.n, params.jitter);
        for (int i = 0; i < data.n; i++)
        {
            lowerBounds[i] = (i % 2) ? 0 : -1e14;
            if (!std::isfinite(x[i]))
                return out;
        }
        // точность перевода: траектория из пар должна совпасть с кандидатом
        metrics::mainActions(data.n, x, &data);
        Quality conv = assess(fromMetrics(data), end, enemies, nEnemies);
        out.convMiss = conv.miss;
        out.convDT = conv.T - res.traj.duration();

        nlopt_opt opt = nlopt_create(NLOPT_LD_SLSQP, data.n);
        void *voidData = static_cast<void *>(&data);
        nlopt_set_lower_bounds(opt, lowerBounds);
        nlopt_set_min_objective(opt, metrics::minimizing, voidData);
        double tolerances[2] = {1e-4, 1e-4};
        nlopt_add_equality_mconstraint(opt, 2, metrics::constraints, voidData, tolerances);
        nlopt_set_xtol_rel(opt, 1e-6);
        nlopt_set_ftol_rel(opt, 1e-6);
        nlopt_set_maxeval(opt, params.maxEval);
        out.nloptCode = nlopt_optimize(opt, x, &minf);
        out.nEval = nlopt_get_numevals(opt);
        nlopt_destroy(opt);

        for (int i = 0; i < data.n; i++)
            if (!std::isfinite(x[i]))
                return out;
        metrics::mainActions(data.n, x, &data);
        Trajectory refined = fromMetrics(data);
        out.refined = assess(refined, end, enemies, nEnemies);
        if (out.refined.miss <= params.maxMiss && out.refined.violation <= 1e-6 && std::isfinite(out.refined.T) &&
            out.refined.T < out.quality.T * (1 - params.minGain))
        {
            out.accepted = true;
            out.traj = refined;
            out.quality = out.refined;
        }
        return out;
    }
}
