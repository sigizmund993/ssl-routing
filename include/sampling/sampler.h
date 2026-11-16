/*
Планировщик перебором: bang-bang траектории через одно промежуточное состояние (P, vP).
Попадание в цель обеспечено построением, зазор до препятствий считается аналитически.
Заголовок не подключает drawer.h (и metrics.h), поэтому окно SFML не открывает.
*/
#pragma once

#include <cmath>
#include <random>
#include "../const.h"
#include "../auxiliary/vecAux.h"
#include "../objects/AbsRigBody.h"
#include "../minimizing/bangBang.h"

// хватает и на траекторию минимизатора: N_SOLVER_PAIRS разгонов + N_SOLVER_PAIRS РПД + финальный участок
#define SAMPLER_MAX_SEGMENTS (2 * N_SOLVER_PAIRS + 2)
#define SAMPLER_MAX_WAYPOINTS 96

namespace sampling
{
    // равноускоренный участок
    struct Segment
    {
        double t0, dur;
        Point r0, v0, a;
    };

    // кусочно-равноускоренная траектория
    struct Trajectory
    {
        Segment seg[SAMPLER_MAX_SEGMENTS];
        int n = 0;
        Point startPos, startVel; // состояние в t = 0
        Point rEnd, vEnd;         // состояние в конце (получено интегрированием участков)
        double tEnd = 0;

        void reset(const Point &r, const Point &v)
        {
            n = 0;
            startPos = rEnd = r;
            startVel = vEnd = v;
            tEnd = 0;
        }
        // добавить участок, продолжающий траекторию; участок нулевой длины пропускается
        bool append(double dur, const Point &a)
        {
            if (!(dur > 1e-12))
                return true;
            if (n >= SAMPLER_MAX_SEGMENTS)
                return false;
            seg[n].t0 = tEnd;
            seg[n].dur = dur;
            seg[n].r0 = rEnd;
            seg[n].v0 = vEnd;
            seg[n].a = a;
            n++;
            rEnd = rEnd + vEnd * dur + a * (dur * dur / 2);
            vEnd = vEnd + a * dur;
            tEnd += dur;
            return true;
        }
        double duration() const
        {
            return tEnd;
        }
        // номер участка, в который попадает момент t
        int find(double t) const
        {
            int lo = 0, hi = n - 1;
            while (lo < hi)
            {
                int mid = (lo + hi + 1) / 2;
                if (seg[mid].t0 <= t)
                    lo = mid;
                else
                    hi = mid - 1;
            }
            return lo;
        }
        Point posAt(double t) const
        {
            if (n == 0 || t <= 0)
                return startPos;
            if (t >= tEnd)
                return rEnd;
            const Segment &s = seg[find(t)];
            double dt = t - s.t0;
            return s.r0 + s.v0 * dt + s.a * (dt * dt / 2);
        }
        Point velAt(double t) const
        {
            if (n == 0 || t <= 0)
                return startVel;
            if (t >= tEnd)
                return vEnd;
            const Segment &s = seg[find(t)];
            return s.v0 + s.a * (t - s.t0);
        }
    };

    // прямоугольная запрещённая зона (штрафная площадь и т. п.). Задаётся уже раздутой на радиус робота:
    // внутри не должен оказаться центр робота
    struct Rect
    {
        Point lo, hi;
    };

    // свой робот с известной траекторией. tOffset — сколько времени прошло с начала этой траектории
    // (0 — построена в этом кадре, dt — в прошлом). После конца траектории робот едет с её конечной скоростью
    struct Ally
    {
        const Trajectory *traj;
        double rad; // сумма радиусов, как у AbsRigBody
        double tOffset;
    };

    // всё, что надо объезжать. Мяч задаётся как обычное препятствие в enemies (радиус = нужное расстояние до мяча)
    struct World
    {
        const AbsRigBody *enemies = nullptr; // чужие роботы и прочие круги: прогноз «постоянная скорость»
        int nEnemies = 0;
        const Ally *allies = nullptr; // свои роботы: траектории известны
        int nAllies = 0;
        const Rect *zones = nullptr; // запрещённые прямоугольники
        int nZones = 0;
        bool hasField = false; // центр робота должен оставаться в прямоугольнике fieldLo..fieldHi
        Point fieldLo, fieldHi;
    };

    // один bang-bang: разгон v0→Vm (ta), РПД с Vm (tc), торможение Vm→v1 (td)
    struct BangPart
    {
        Point v0, Vm, v1;
        double ta, tc, td;
    };

    // состояние между кадрами (тёплый старт)
    // вид кандидата: 0 — прямая, 1 — через (P, vP), 2 — через (P, vP) и (P2, vP2)
    struct State
    {
        bool valid = false;
        Point P, vP;
        int kind = 1;
        Point P2, vP2;
    };

    struct Params
    {
        int budget = 100;            // сколько кандидатов можно построить за вызов
        double margin = 100;         // мм: запас касательных точек обхода
        double tightMargin = 2 * SAFE_DIST; // мм: запас для второго набора касательных точек (узкие проходы)
        // мягкий запас до препятствия зависит от скорости, как у TIGERs (ETDP 2024, формула 4):
        // m(v) = marginBase * (min(|v|, marginVRef) / marginVRef)^2. Медленно можно ехать вплотную, быстро — нет
        double marginBase = 200;     // мм: запас на скорости marginVRef
        double marginVRef = 3000;    // мм/с
        // стоимость = T * (1 + penalty * clamp(дефицит / marginBase, 0, 1)),
        // дефицит = max по времени (SAFE_DIST + m(|v(t)|) - зазор(t)), то есть насколько нарушен мягкий запас
        double penalty = 0.2;
        int directShare = 20;        // сколько кандидатов потратить на поиск рядом со свободной прямой траекторией
        bool nelderMead = true;      // уточнение Нелдером–Мидом по (P, vP) вместо случайного шума
        bool predictTangents = true; // касательные точки строятся к прогнозу положения движущихся препятствий
        // с: движущиеся препятствия проверяются только до этого момента траектории. Прогноз «постоянная скорость»
        // дальше ~0.5–1 с почти бесполезен (логи игр: ошибка через 1 с — 0.3 м в медиане, 1 м в p90),
        // а план всё равно пересчитывается каждый кадр. Неподвижные препятствия проверяются на всём пути
        double horizon = 1.0;
        // мм/с²: ускорение на разгоне и на торможении. Реальные роботы тормозят в 1.5–2 раза резче, чем
        // разгоняются (логи игр). По умолчанию оба равны MAX_ACC — симметричный bang-bang, как у минимизатора
        double accel = MAX_ACC;
        double decel = MAX_ACC;
        bool emergencyBrake = true;  // нет допустимой траектории — тормозить, если это не хуже лучшего кандидата
        double nmStepPos = 200;      // мм: начальный шаг симплекса по P (у свободной прямой вдвое меньше)
        double nmStepVel = 400;      // мм/с: начальный шаг симплекса по vP
        double maxResidual = 1;      // мм: допустимая невязка конечной точки bang-bang
        double tangentShare = 0.3;   // доля бюджета на касательные точки
        double randomShare = 0.4;    // доля бюджета на случайные точки, остальное идёт на уточнение
        double refineStepPos = 300;  // мм: начальный шаг уточнения по P
        double refineStepVel = 300;  // мм/с: начальный шаг уточнения по vP
        int barrier = 160;           // предел сетки запасного перебора углов в bangBang (в initialApprox он 10000)
        bool twoVia = true;          // нет допустимого кандидата через одну точку — пробовать две
        double twoViaShare = 0.2;    // доля бюджета на них (берётся из случайных точек)
        // трудная сцена (допустимого кандидата так и нет): добавочный бюджет на пары точек, случайные точки и уточнение
        int hardExtra = 40;
        double allyMargin = 0;       // мм: добавка к SAFE_DIST для своих роботов (ошибка слежения за траекторией)
        bool analyticJacobian = true; // якобиан Ньютона для Vm при accel != decel: аналитический или численный
    };

    // качество траектории, общее для всех методов
    struct Quality
    {
        double T;         // полное время, с
        double clearance; // минимальный зазор до препятствий (расстояние до центра минус радиус), мм
        double miss;      // промах мимо конечной точки, мм
        double violation; // на сколько мм нарушен требуемый зазор (<= 0, если всё в порядке)
        double clearanceH; // зазор, если движущиеся препятствия проверять только до горизонта (неподвижные — весь путь)
    };

    struct Result
    {
        Trajectory traj;
        bool found = false;    // построилась хоть одна траектория
        bool feasible = false; // зазор не меньше требуемого
        bool direct = false;   // прямая траектория без промежуточной точки
        bool brake = false;    // допустимой траектории нет, вместо неё аварийное торможение
        Point P, vP;           // промежуточное состояние (если !direct)
        int kind = 0;          // вид кандидата, см. State
        Point P2, vP2;         // второе промежуточное состояние (kind == 2)
        BangPart parts[3];
        int nParts = 0;
        double T = 0, clearance = 0, cost = 0, violation = 0;
        double deficit = 0; // насколько нарушен мягкий запас m(v), мм (0 — не нарушен)
        int nTried = 0; // сколько кандидатов построено
    };

    // минимум расстояния от участка до центра препятствия (минус радиус).
    // Минимум |a t^2/2 + v t + r| ищется по корням производной квадрата расстояния и по концам участка
    double segmentClearance(const Segment &s, const Point &pos, const Point &vel, double rad)
    {
        Point r = s.r0 - (pos + vel * s.t0), v = s.v0 - vel;
        auto d2 = [&](double t)
        {
            return (r + v * t + s.a * (t * t / 2)).mag2();
        };
        double best = min(d2(0), d2(s.dur)), roots[3];
        int nRoots = numAux::solveEq(roots, s.a.mag2() / 2, 1.5 * (s.a ^ v), v.mag2() + (s.a ^ r), v ^ r);
        bool broken = false;
        for (int i = 0; i < nRoots; i++)
        {
            if (!std::isfinite(roots[i]))
                broken = true;
            else if (roots[i] > 0 && roots[i] < s.dur)
                best = min(best, d2(roots[i]));
        }
        if (broken)
        {
            // решатель кубического уравнения вернул NaN: проверяем участок по точкам, чтобы не пропустить столкновение
            const int nSteps = 32;
            for (int i = 1; i < nSteps; i++)
                best = min(best, d2(s.dur * i / nSteps));
        }
        return sqrt(best) - rad;
    }

    // кусок участка s от момента ta до tb (времена абсолютные)
    Segment subSegment(const Segment &s, double ta, double tb)
    {
        double dt = ta - s.t0;
        return {ta, tb - ta, s.r0 + s.v0 * dt + s.a * (dt * dt / 2), s.v0 + s.a * dt, s.a};
    }

    // зазор участка с учётом горизонта: движущееся препятствие проверяется только до момента horizon
    double horizonClearance(const Segment &s, const Point &pos, const Point &vel, double rad, double horizon)
    {
        if (vel.mag2() == 0 || s.t0 + s.dur <= horizon)
            return segmentClearance(s, pos, vel, rad);
        if (s.t0 >= horizon)
            return 1e18;
        Segment cut = s;
        cut.dur = horizon - s.t0;
        return segmentClearance(cut, pos, vel, rad);
    }

    // требуемый зазор до препятствия: SAFE_DIST, но не больше, чем зазор в начальной и конечной точках.
    // Иначе при старте или цели вплотную к препятствию допустимых траекторий не было бы вообще.
    // Положение препятствия для конечной точки берётся на t = 0.
    // Зазор в end учитывается только для неподвижных препятствий: где движущееся будет к приезду, заранее
    // неизвестно, и ослаблять из-за него требование по всей траектории нельзя
    double requiredClearance(const AbsRigBody &enemy, const Point &start, const Point &end)
    {
        double cs = (start - enemy.getPos()).mag() - enemy.getRad();
        double ce = enemy.getVel().mag2() > 0 ? 1e18 : (end - enemy.getPos()).mag() - enemy.getRad();
        return min(double(SAFE_DIST), min(cs, ce));
    }

    // препятствия, сдвинутые туда, где они будут, когда робот поравняется с ними на отрезке start→end.
    // Время оценивается по проекции на отрезок и средней скорости 0.7·MAX_VEL (три итерации, не больше 3 с).
    // Для неподвижных препятствий это их текущее положение
    void predictObstacles(const Point &start, const Point &end, const AbsRigBody *enemies, int nEnemies, AbsRigBody *out, double horizon = 3)
    {
        Point line = end - start;
        double len = line.mag();
        Point dir = len > 1e-6 ? line / len : Point(1, 0);
        for (int k = 0; k < nEnemies; k++)
        {
            Point pos = enemies[k].getPos(), vel = enemies[k].getVel(), p = pos;
            if (vel.mag2() > 0)
                for (int it = 0; it < 3; it++)
                {
                    double along = max(0.0, min(len, (p - start) ^ dir));
                    double t = min(min(3.0, horizon), along / (0.7 * MAX_VEL));
                    p = pos + vel * t;
                }
            out[k] = AbsRigBody(p, enemies[k].getRad(), vel);
        }
    }

    // зазор и нарушение по всей траектории. Если cap > 0, препятствия заведомо дальше cap пропускаются,
    // а зазор обрезается сверху значением cap (для перебора точнее не надо)
    void clearanceOf(const Trajectory &tr, const AbsRigBody *enemies, int nEnemies, const double *required, double cap, double &clearance, double &violation,
                     double horizon = 1e18)
    {
        clearance = cap > 0 ? cap : 1e18;
        violation = -1e18;
        for (int k = 0; k < nEnemies; k++)
        {
            Point pos = enemies[k].getPos(), vel = enemies[k].getVel();
            double rad = enemies[k].getRad(), vMaxRel = MAX_VEL * 1.001 + vel.mag(), best = cap > 0 ? cap : 1e18;
            for (int i = 0; i < tr.n; i++)
            {
                const Segment &s = tr.seg[i];
                if (cap > 0)
                {
                    // грубая оценка снизу: за время участка робот не подойдёт ближе
                    double low = (s.r0 - (pos + vel * s.t0)).mag() - rad - vMaxRel * s.dur;
                    if (low >= best)
                        continue;
                }
                best = min(best, horizonClearance(s, pos, vel, rad, horizon));
            }
            if (tr.n == 0)
                best = min(best, (tr.startPos - pos).mag() - rad);
            clearance = min(clearance, best);
            violation = max(violation, required[k] - best);
        }
        if (nEnemies == 0)
            violation = -1e18;
    }

    // мягкий запас до препятствия на скорости speed
    double speedMargin(double speed, const Params &params)
    {
        double k = min(speed, params.marginVRef) / params.marginVRef;
        return params.marginBase * k * k;
    }

    // дефицит мягкого запаса: max по времени (SAFE_DIST + m(|v(t)|) - зазор(t)), но не меньше 0.
    // Считается по участкам с оценкой сверху: SAFE_DIST + m(max |v| на участке) - минимальный зазор участка.
    // |v| на равноускоренном участке выпукла, поэтому её максимум на концах. Оценка завышает дефицит, если
    // робот подходит к препятствию медленнее, чем едет на краю участка; для мягкого штрафа это допустимо
    double marginDeficit(const Trajectory &tr, const AbsRigBody *enemies, int nEnemies, const Params &params)
    {
        double deficit = 0;
        for (int i = 0; i < tr.n; i++)
        {
            const Segment &s = tr.seg[i];
            double vMax = max(s.v0.mag(), (s.v0 + s.a * s.dur).mag());
            double need = SAFE_DIST + speedMargin(vMax, params);
            for (int k = 0; k < nEnemies; k++)
            {
                Point pos = enemies[k].getPos(), vel = enemies[k].getVel();
                double rad = enemies[k].getRad();
                // грубая оценка снизу расстояния: если и она дальше нужного, участок дефицита не даст
                double low = (s.r0 - (pos + vel * s.t0)).mag() - rad - (vMax + vel.mag()) * s.dur;
                if (need - low <= deficit)
                    continue;
                deficit = max(deficit, need - horizonClearance(s, pos, vel, rad, params.horizon));
            }
        }
        return deficit;
    }

    // положение своего робота в момент t (после конца траектории он едет с конечной скоростью)
    Point allyPosAt(const Ally &al, double t)
    {
        const Trajectory &tr = *al.traj;
        double ta = t + al.tOffset;
        if (ta >= tr.tEnd)
            return tr.rEnd + tr.vEnd * (ta - tr.tEnd);
        return tr.posAt(ta);
    }

    // наибольшая скорость своего робота на его траектории
    double allyMaxSpeed(const Ally &al)
    {
        const Trajectory &tr = *al.traj;
        double v = max(tr.startVel.mag(), tr.vEnd.mag());
        for (int i = 0; i < tr.n; i++)
            v = max(v, tr.seg[i].v0.mag());
        return v;
    }

    // свой робот уже стоит (траектория кончилась, конечная скорость нулевая)
    bool allyStanding(const Ally &al)
    {
        return al.tOffset >= al.traj->tEnd && al.traj->vEnd.mag2() < 1;
    }

    // минимум расстояния между участком s и своим роботом на его траектории (минус радиус). Оба едут с постоянным
    // ускорением, поэтому относительное движение тоже равноускоренное: участок режется по границам участков
    // чужой траектории, на каждом куске зазор считает segmentClearance в относительных координатах
    double allyClearance(const Segment &s, const Ally &al)
    {
        const Trajectory &tr = *al.traj;
        double t = s.t0, tEnd = s.t0 + s.dur, best = 1e18;
        int i = tr.n > 0 ? tr.find(t + al.tOffset) : 0;
        while (t < tEnd - 1e-12)
        {
            double ta = t + al.tOffset, tb = tEnd;
            Point ar, av, aa;
            if (i >= tr.n || ta >= tr.tEnd)
            {
                av = tr.vEnd;
                ar = tr.rEnd + av * (ta - tr.tEnd);
                i = tr.n;
            }
            else
            {
                const Segment &q = tr.seg[i];
                double dq = max(0.0, ta - q.t0);
                ar = q.r0 + q.v0 * dq + q.a * (dq * dq / 2);
                av = q.v0 + q.a * dq;
                aa = q.a;
                tb = min(tEnd, q.t0 + q.dur - al.tOffset);
                i++;
            }
            if (tb > t)
            {
                Segment our = subSegment(s, t, tb);
                Segment rel = {0, tb - t, our.r0 - ar, our.v0 - av, s.a - aa};
                best = min(best, segmentClearance(rel, Point(0, 0), Point(0, 0), al.rad));
                t = tb;
            }
        }
        return best;
    }

    // расстояние от точки до прямоугольника: снаружи положительное, внутри — минус глубина
    double rectDistance(const Point &p, const Rect &z)
    {
        double dx = max(max(z.lo.x - p.x, p.x - z.hi.x), 0.0), dy = max(max(z.lo.y - p.y, p.y - z.hi.y), 0.0);
        if (dx > 0 || dy > 0)
            return sqrt(dx * dx + dy * dy);
        return -min(min(p.x - z.lo.x, z.hi.x - p.x), min(p.y - z.lo.y, z.hi.y - p.y));
    }

    // минимум rectDistance по участку. Проверяются концы, вершины парабол x(t) и y(t), моменты пересечения четырёх
    // прямых-сторон и точки между ними (глубина захода, приближённо), а снаружи ещё ближайший подход к каждому углу:
    // расстояние снаружи и сам факт захода точные.
    // Если cap > 0 и участок заведомо дальше cap, возвращается cap
    double rectClearance(const Segment &s, const Rect &z, double cap = -1)
    {
        double times[12], roots[2];
        int n = 0;
        times[n++] = 0;
        times[n++] = s.dur;
        Point e = s.r0 + s.v0 * s.dur + s.a * (s.dur * s.dur / 2);
        double xMin = min(s.r0.x, e.x), xMax = max(s.r0.x, e.x), yMin = min(s.r0.y, e.y), yMax = max(s.r0.y, e.y);
        if (s.a.x != 0)
        {
            double t = -s.v0.x / s.a.x;
            if (t > 0 && t < s.dur)
            {
                double x = s.r0.x + s.v0.x * t / 2;
                xMin = min(xMin, x), xMax = max(xMax, x);
                times[n++] = t;
            }
        }
        if (s.a.y != 0)
        {
            double t = -s.v0.y / s.a.y;
            if (t > 0 && t < s.dur)
            {
                double y = s.r0.y + s.v0.y * t / 2;
                yMin = min(yMin, y), yMax = max(yMax, y);
                times[n++] = t;
            }
        }
        double gap = max(max(z.lo.x - xMax, xMin - z.hi.x), max(z.lo.y - yMax, yMin - z.hi.y));
        if (cap > 0 && gap >= cap)
            return cap;
        if (gap <= 0)
        {
            // рамка участка задевает зону: ищем моменты пересечения сторон
            const double lines[4] = {z.lo.x, z.hi.x, z.lo.y, z.hi.y};
            for (int k = 0; k < 4; k++)
            {
                bool isX = k < 2;
                int nr = numAux::solveEq(roots, (isX ? s.a.x : s.a.y) / 2, isX ? s.v0.x : s.v0.y, (isX ? s.r0.x : s.r0.y) - lines[k]);
                for (int i = 0; i < nr; i++)
                    if (roots[i] > 0 && roots[i] < s.dur)
                        times[n++] = roots[i];
            }
        }
        for (int i = 1; i < n; i++)
            for (int j = i; j > 0 && times[j] < times[j - 1]; j--)
                std::swap(times[j], times[j - 1]);
        auto at = [&](double t)
        {
            return rectDistance(s.r0 + s.v0 * t + s.a * (t * t / 2), z);
        };
        double best = at(times[0]);
        for (int i = 1; i < n; i++)
        {
            double mid = at((times[i] + times[i - 1]) / 2);
            best = min(best, min(at(times[i]), mid));
            // кусок внутри зоны: глубина захода — минимум из четырёх парабол, её максимум не обязан быть в середине.
            // Глубина нужна только чтобы сравнивать недопустимых кандидатов, поэтому хватает нескольких точек
            if (mid < 0)
                for (int j = 1; j < 8; j++)
                    best = min(best, at(times[i - 1] + (times[i] - times[i - 1]) * j / 8));
        }
        if (best > 0)
        {
            Segment local = s;
            local.t0 = 0;
            const Point corners[4] = {z.lo, z.hi, Point(z.lo.x, z.hi.y), Point(z.hi.x, z.lo.y)};
            for (const Point &c : corners)
                best = min(best, segmentClearance(local, c, Point(0, 0), 0));
        }
        return best;
    }

    // запас до границы поля на участке (внутри положительный): минимум по концам и вершинам парабол x(t), y(t)
    double fieldClearance(const Segment &s, const Point &lo, const Point &hi)
    {
        double best = 1e18;
        auto at = [&](double t)
        {
            Point p = s.r0 + s.v0 * t + s.a * (t * t / 2);
            best = min(best, min(min(p.x - lo.x, hi.x - p.x), min(p.y - lo.y, hi.y - p.y)));
        };
        at(0);
        at(s.dur);
        if (s.a.x != 0 && -s.v0.x / s.a.x > 0 && -s.v0.x / s.a.x < s.dur)
            at(-s.v0.x / s.a.x);
        if (s.a.y != 0 && -s.v0.y / s.a.y > 0 && -s.v0.y / s.a.y < s.dur)
            at(-s.v0.y / s.a.y);
        return best;
    }

    int worldCount(const World &w)
    {
        return w.nEnemies + w.nAllies + w.nZones + (w.hasField ? 1 : 0);
    }

    // требуемые зазоры для всех объектов мира: сначала чужие, потом свои, зоны, последней — граница поля.
    // Правило то же, что в requiredClearance: требование не строже, чем зазор в start и end. Для зон и поля
    // требуемый зазор 0 (или отрицательный, если робот уже внутри зоны или цель задана внутри)
    void worldRequired(const World &w, const Point &start, const Point &end, const Params &params, double *required)
    {
        int j = 0;
        for (int k = 0; k < w.nEnemies; k++)
            required[j++] = requiredClearance(w.enemies[k], start, end);
        for (int k = 0; k < w.nAllies; k++)
        {
            const Ally &al = w.allies[k];
            double cs = (start - allyPosAt(al, 0)).mag() - al.rad;
            double ce = allyStanding(al) ? (end - al.traj->rEnd).mag() - al.rad : 1e18;
            required[j++] = min(SAFE_DIST + params.allyMargin, min(cs, ce));
        }
        for (int k = 0; k < w.nZones; k++)
            required[j++] = min(0.0, min(rectDistance(start, w.zones[k]), rectDistance(end, w.zones[k])));
        if (w.hasField)
        {
            Rect f = {w.fieldLo, w.fieldHi};
            required[j++] = min(0.0, min(-rectDistance(start, f), -rectDistance(end, f)));
        }
    }

    // зазор и нарушение траектории по всем объектам мира (см. clearanceOf). allyVmax — наибольшие скорости своих
    void worldClearance(const Trajectory &tr, const World &w, const double *required, const double *allyVmax, double cap, double &clearance,
                        double &violation, const Params &params, double horizon)
    {
        clearanceOf(tr, w.enemies, w.nEnemies, required, cap, clearance, violation, horizon);
        if (w.nAllies + w.nZones == 0 && !w.hasField)
            return;
        int j = w.nEnemies;
        auto account = [&](double best)
        {
            clearance = min(clearance, best);
            violation = max(violation, required[j++] - best);
        };
        for (int k = 0; k < w.nAllies; k++)
        {
            const Ally &al = w.allies[k];
            double best = cap > 0 ? cap : 1e18, vMaxRel = MAX_VEL * 1.001 + allyVmax[k];
            for (int i = 0; i < tr.n; i++)
            {
                const Segment &s = tr.seg[i];
                if (cap > 0 && (s.r0 - allyPosAt(al, s.t0)).mag() - al.rad - vMaxRel * s.dur >= best)
                    continue;
                best = min(best, allyClearance(s, al));
            }
            if (tr.n == 0)
                best = min(best, (tr.startPos - allyPosAt(al, 0)).mag() - al.rad);
            account(best);
        }
        for (int k = 0; k < w.nZones; k++)
        {
            double best = cap > 0 ? cap : 1e18;
            for (int i = 0; i < tr.n; i++)
                best = min(best, rectClearance(tr.seg[i], w.zones[k], cap > 0 ? best : -1));
            if (tr.n == 0)
                best = min(best, rectDistance(tr.startPos, w.zones[k]));
            account(best);
        }
        if (w.hasField)
        {
            double best = 1e18;
            for (int i = 0; i < tr.n; i++)
                best = min(best, fieldClearance(tr.seg[i], w.fieldLo, w.fieldHi));
            if (tr.n == 0)
                best = -rectDistance(tr.startPos, {w.fieldLo, w.fieldHi});
            // запас до границы поля в общий зазор не идёт (он бывает большим и ничего не говорит о препятствиях)
            violation = max(violation, required[j++] - best);
            if (best < 0)
                clearance = min(clearance, best);
        }
    }

    // дефицит мягкого запаса до своих роботов (см. marginDeficit)
    double allyDeficit(const Trajectory &tr, const World &w, const double *allyVmax, const Params &params)
    {
        double deficit = 0;
        for (int i = 0; i < tr.n; i++)
        {
            const Segment &s = tr.seg[i];
            double vMax = max(s.v0.mag(), (s.v0 + s.a * s.dur).mag());
            double need = SAFE_DIST + params.allyMargin + speedMargin(vMax, params);
            for (int k = 0; k < w.nAllies; k++)
            {
                const Ally &al = w.allies[k];
                double low = (s.r0 - allyPosAt(al, s.t0)).mag() - al.rad - (vMax + allyVmax[k]) * s.dur;
                if (need - low <= deficit)
                    continue;
                deficit = max(deficit, need - allyClearance(s, al));
            }
        }
        return deficit;
    }

    // оценка траектории любого метода: время, точный зазор, промах (интегрированием участков), нарушение зазора
    Quality assess(const Trajectory &tr, const Point &endPos, const AbsRigBody *enemies, int nEnemies, double horizon = 1.0)
    {
        Quality q;
        double required[nEnemies + 1];
        for (int k = 0; k < nEnemies; k++)
            required[k] = requiredClearance(enemies[k], tr.startPos, endPos);
        Point r = tr.startPos, v = tr.startVel;
        for (int i = 0; i < tr.n; i++)
        {
            r = r + v * tr.seg[i].dur + tr.seg[i].a * (tr.seg[i].dur * tr.seg[i].dur / 2);
            v = v + tr.seg[i].a * tr.seg[i].dur;
        }
        q.T = tr.duration();
        q.miss = (r - endPos).mag();
        if (!std::isfinite(q.miss))
            q.miss = 1e18;
        clearanceOf(tr, enemies, nEnemies, required, -1, q.clearance, q.violation);
        double violationH;
        clearanceOf(tr, enemies, nEnemies, required, -1, q.clearanceH, violationH, horizon);
        return q;
    }

    // дописать к траектории bang-bang из её конца в точку target со скоростью v1.
    // false, если bang-bang не построился или не попал в target с точностью maxResidual
    bool appendBangBang(Trajectory &tr, const Point &v1, const Point &target, const Params &params, BangPart *part = nullptr)
    {
        Point v0 = tr.vEnd, dr = target - tr.rEnd, Vm, rem, u;
        double ta, tc = 0, td;
        if (dr.mag() < 1e-3 && (v1 - v0).mag() < 1e-6)
        {
            // уже на месте с нужной скоростью, bangBang на |dr| = 0 делить не заставляем
            if (part)
                *part = {v0, v0, v1, 0, 0, 0};
            return true;
        }
        const bool asym = params.decel != params.accel;
        // ускорение фазы from→to (скорость меняется по прямой): чистый разгон — accel, чистое торможение —
        // decel, между ними плавно по тому, насколько фаза меняет модуль скорости. Плавность нужна Ньютону
        auto phaseAcc = [&](const Point &from, const Point &to)
        {
            if (!asym)
                return params.accel;
            double dv = (to - from).mag();
            if (dv < 1e-9)
                return params.accel;
            double c = max(-1.0, min(1.0, (to.mag() - from.mag()) / dv));
            return params.decel + (params.accel - params.decel) * (c + 1) / 2;
        };
        Vm = bangBang::bangBang(v0, v1, dr, params.accel, MAX_VEL, 10, 10, 2, params.barrier);
        if (!std::isfinite(Vm.x) || !std::isfinite(Vm.y))
            return false;
        double vm = Vm.mag();
        if (vm > MAX_VEL * 1.002)
            return false;
        auto remainder = [&](const Point &V)
        {
            ta = (V - v0).mag() / phaseAcc(v0, V);
            td = (v1 - V).mag() / phaseAcc(V, v1);
            return dr - (v0 + V) * (ta / 2) - (v1 + V) * (td / 2);
        };
        rem = remainder(Vm);
        if (asym && rem.mag() > 1e-3 && vm < MAX_VEL * 0.999)
        {
            // bangBang решал симметричную задачу; с разными ускорениями доводим Vm Ньютоном с численным якобианом.
            // Если Vm вышла за MAX_VEL, дальше работает ветка с РПД
            // якобиан remainder по V аналитически: fx = d rem / dV.x, fy = d rem / dV.y.
            // false в вырожденных точках (нулевая фаза или V = 0), там якобиан берётся численно
            auto jacobian = [&](const Point &V, Point &fx, Point &fy)
            {
                if (!params.analyticJacobian)
                    return false;
                Point d0 = V - v0, d1 = v1 - V;
                double m0 = d0.mag(), m1 = d1.mag(), mv = V.mag(), s0 = v0.mag(), s1 = v1.mag();
                if (m0 < 1e-6 || m1 < 1e-6 || mv < 1e-6)
                    return false;
                Point uV = V / mv, dc0, dc1;
                double c0 = (mv - s0) / m0, c1 = (s1 - mv) / m1, k = (params.accel - params.decel) / 2;
                // c обрезано до [-1, 1]: на границе производная нулевая
                if (c0 > -1 && c0 < 1)
                    dc0 = uV / m0 - d0 * ((mv - s0) / (m0 * m0 * m0));
                if (c1 > -1 && c1 < 1)
                    dc1 = d1 * ((s1 - mv) / (m1 * m1 * m1)) - uV / m1;
                double A0 = phaseAcc(v0, V), A1 = phaseAcc(V, v1);
                Point dta = d0 / (m0 * A0) - dc0 * (k * m0 / (A0 * A0));
                Point dtd = d1 / (-m1 * A1) - dc1 * (k * m1 / (A1 * A1));
                Point p0 = (v0 + V) * 0.5, p1 = (v1 + V) * 0.5;
                double half = (m0 / A0 + m1 / A1) / 2;
                fx = Point(-half - p0.x * dta.x - p1.x * dtd.x, -p0.y * dta.x - p1.y * dtd.x);
                fy = Point(-p0.x * dta.y - p1.x * dtd.y, -half - p0.y * dta.y - p1.y * dtd.y);
                return true;
            };
            auto newton = [&](Point V, Point &rV)
            {
                rV = remainder(V);
                for (int it = 0; it < 30 && rV.mag() > 1e-3; it++)
                {
                    Point fx, fy;
                    if (!jacobian(V, fx, fy))
                    {
                        const double h = 1e-2;
                        fx = (remainder(V + Point(h, 0)) - remainder(V - Point(h, 0))) / (2 * h);
                        fy = (remainder(V + Point(0, h)) - remainder(V - Point(0, h))) / (2 * h);
                    }
                    double det = fx.x * fy.y - fy.x * fx.y;
                    if (!(abs(det) > 1e-12))
                        break;
                    Point d((-rV.x * fy.y + fy.x * rV.y) / det, (-fx.x * rV.y + rV.x * fx.y) / det);
                    if (d.mag() > 500)
                        d = d * (500 / d.mag());
                    V = V + d;
                    if (V.mag() > MAX_VEL)
                    {
                        V = V * (MAX_VEL / V.mag());
                        rV = remainder(V);
                        break;
                    }
                    rV = remainder(V);
                }
                return V;
            };
            Point V = newton(Vm, rem);
            // из симметричного решения Ньютон сходится не всегда: оно бывает направлено против нужного (симметричный
            // робот проскакивает цель и возвращается, а с резким торможением успевает остановиться). Тогда пробуем
            // другие начальные точки. Решение на окружности MAX_VEL — тоже годится, его доведёт ветка с РПД
            if (rem.mag() > 1e-3 && V.mag() < MAX_VEL * 0.999)
            {
                Point dir = dr * (1 / dr.mag());
                const Point starts[] = {v0, v0 * 1.2, (v0 + v1) * 0.5, dir * max(v0.mag(), 300.0),
                                        dir * (MAX_VEL * 0.5), dir * (MAX_VEL * 0.9)};
                for (const Point &S : starts)
                {
                    Point r2, V2 = newton(S, r2);
                    if (r2.mag() <= 1e-3 || V2.mag() >= MAX_VEL * 0.999)
                    {
                        V = V2;
                        rem = r2;
                        break;
                    }
                }
            }
            Vm = V;
            vm = Vm.mag();
        }
        if (rem.mag() > 1e-3)
        {
            // остаток пути проезжается на РПД, для этого Vm должна лежать на окружности Vmax и смотреть вдоль остатка.
            // bangBang останавливается по 1 - cos < 1e-7, это до 0.5 мрад ошибки по углу, поэтому угол доводим сами
            if (vm < MAX_VEL * 0.999)
                return false;
            double th = Vm.arg();
            auto cross = [&](double angle)
            {
                Point dir(cos(angle), sin(angle));
                Point left = remainder(dir * MAX_VEL);
                return dir.x * left.y - dir.y * left.x;
            };
            for (int it = 0; it < 8; it++)
            {
                double g = cross(th);
                if (abs(g) < 1e-4)
                    break;
                const double h = 1e-6;
                double dg = (cross(th + h) - cross(th - h)) / (2 * h);
                if (dg == 0 || !std::isfinite(dg))
                    break;
                th -= max(-0.2, min(0.2, g / dg));
            }
            u = Point(cos(th), sin(th));
            Vm = u * MAX_VEL;
            rem = remainder(Vm);
            tc = (rem ^ u) / MAX_VEL;
            if (!(tc >= 0))
                return false;
        }
        if (!std::isfinite(ta) || !std::isfinite(td))
            return false;
        if (ta > 1e-12 && !tr.append(ta, (Vm - v0) / ta))
            return false;
        if (!tr.append(tc, Point(0, 0)))
            return false;
        if (td > 1e-12 && !tr.append(td, (v1 - Vm) / td))
            return false;
        // невязка конечной точки по факту (rEnd получен интегрированием участков)
        double residual = (tr.rEnd - target).mag();
        if (!(residual <= params.maxResidual))
            return false;
        if (part)
            *part = {v0, Vm, v1, ta, tc, td};
        return true;
    }

    // крайние левая и правая касательные из точки from к группе кругов (если смотреть в сторону toward).
    // false, если from лежит внутри одного из кругов
    bool extremeTangents(const Point &from, const Point &toward, const AbsRigBody *enemies, const int *members, int nMembers, double margin, Point &left, Point &right)
    {
        Point prod[2];
        double base = (toward - from).arg(), angMax = -1e9, angMin = 1e9;
        for (int i = 0; i < nMembers; i++)
        {
            const AbsRigBody &e = enemies[members[i]];
            double R = e.getRad() + margin;
            if ((from - e.getPos()).mag() <= R * 1.001)
                return false;
            if (vecAux::getTangentPoints(prod, from, e.getPos(), R) != 2)
                return false;
            for (int j = 0; j < 2; j++)
            {
                if (!std::isfinite(prod[j].x) || !std::isfinite(prod[j].y))
                    return false;
                double ang = numAux::windDownAngle((prod[j] - from).arg() - base);
                if (ang > angMax)
                {
                    angMax = ang;
                    left = prod[j];
                }
                if (ang < angMin)
                {
                    angMin = ang;
                    right = prod[j];
                }
            }
        }
        return nMembers > 0;
    }

    // точки обхода препятствий, которые перекрывают отрезок start→end. Пересекающиеся круги (rad + margin)
    // объединяются в группы. Порядок: углы обхода группы (пересечение касательных из start и из end),
    // касательные точки из start, из end, затем точки сбоку от каждого перекрывающего препятствия
    int tangentWaypoints(const Point &start, const Point &end, const AbsRigBody *enemies, int nEnemies, double margin, Point *out, int maxOut)
    {
        int group[nEnemies + 1], members[nEnemies + 1], nOut = 0;
        bool blocking[nEnemies + 1];
        Point corners[2 * nEnemies + 1], fromStart[2 * nEnemies + 1], fromEnd[2 * nEnemies + 1], sides[2 * nEnemies + 1];
        int nCorners = 0, nFromStart = 0, nFromEnd = 0, nSides = 0;
        Point dir = end - start;
        double len = dir.mag();
        if (len < 1e-6)
            return 0;
        dir = dir / len;
        Point normal(-dir.y, dir.x);
        for (int i = 0; i < nEnemies; i++)
        {
            group[i] = i;
            Point rel = enemies[i].getPos() - start;
            double along = max(0.0, min(len, rel ^ dir));
            blocking[i] = (rel - dir * along).mag() < enemies[i].getRad() + margin;
        }
        // группы: перекрашиваем, пока есть пересекающиеся круги с разными номерами
        for (bool changed = true; changed;)
        {
            changed = false;
            for (int i = 0; i < nEnemies; i++)
                for (int j = i + 1; j < nEnemies; j++)
                    if (group[i] != group[j] && (enemies[i].getPos() - enemies[j].getPos()).mag() < enemies[i].getRad() + enemies[j].getRad() + 2 * margin)
                    {
                        int from = max(group[i], group[j]), to = min(group[i], group[j]);
                        for (int k = 0; k < nEnemies; k++)
                            if (group[k] == from)
                                group[k] = to;
                        changed = true;
                    }
        }
        for (int g = 0; g < nEnemies; g++)
        {
            int nMembers = 0;
            bool isBlocking = false;
            for (int i = 0; i < nEnemies; i++)
                if (group[i] == g)
                {
                    members[nMembers++] = i;
                    isBlocking = isBlocking || blocking[i];
                }
            if (!isBlocking)
                continue;
            Point sl, sr, el, er, corner;
            bool okS = extremeTangents(start, end, enemies, members, nMembers, margin, sl, sr);
            bool okE = extremeTangents(end, start, enemies, members, nMembers, margin, el, er);
            if (okS)
            {
                fromStart[nFromStart++] = sl;
                fromStart[nFromStart++] = sr;
            }
            if (okE)
            {
                fromEnd[nFromEnd++] = el;
                fromEnd[nFromEnd++] = er;
            }
            if (okS && okE)
            {
                // левая сторона из start — это правая, если смотреть из end
                if (vecAux::getLineIntersection(corner, start, sl, end, er, 'R', 'R'))
                    corners[nCorners++] = corner;
                if (vecAux::getLineIntersection(corner, start, sr, end, el, 'R', 'R'))
                    corners[nCorners++] = corner;
            }
        }
        for (int i = 0; i < nEnemies; i++)
            if (blocking[i])
            {
                sides[nSides++] = enemies[i].getPos() + normal * (enemies[i].getRad() + margin + 30);
                sides[nSides++] = enemies[i].getPos() - normal * (enemies[i].getRad() + margin + 30);
            }
        for (int i = 0; i < nCorners && nOut < maxOut; i++)
            out[nOut++] = corners[i];
        for (int i = 0; i < nFromStart && nOut < maxOut; i++)
            out[nOut++] = fromStart[i];
        for (int i = 0; i < nFromEnd && nOut < maxOut; i++)
            out[nOut++] = fromEnd[i];
        for (int i = 0; i < nSides && nOut < maxOut; i++)
            out[nOut++] = sides[i];
        return nOut;
    }

    // точки обхода для отрезка from→to: касательные к кругам (tangentWaypoints) и углы запрещённых зон,
    // которые отрезок задевает (угол отодвинут наружу на margin по обеим осям)
    int collectWaypoints(const Point &from, const Point &to, const AbsRigBody *circles, int nCircles, const World &world, double margin, Point *out,
                         int maxOut)
    {
        int nOut = tangentWaypoints(from, to, circles, nCircles, margin, out, maxOut);
        Segment line = {0, 1, from, to - from, Point(0, 0)};
        for (int k = 0; k < world.nZones; k++)
        {
            const Rect &z = world.zones[k];
            if (rectClearance(line, z) >= margin)
                continue;
            const Point corners[4] = {Point(z.lo.x - margin, z.lo.y - margin), Point(z.hi.x + margin, z.hi.y + margin),
                                      Point(z.lo.x - margin, z.hi.y + margin), Point(z.hi.x + margin, z.lo.y - margin)};
            for (const Point &c : corners)
            {
                // угол за границей поля бесполезен
                if (world.hasField && rectDistance(c, {world.fieldLo, world.fieldHi}) > 0)
                    continue;
                if (nOut < maxOut)
                    out[nOut++] = c;
            }
        }
        return nOut;
    }

    // построить траекторию из (start, vStart) в (end, vEnd) мимо всего, что есть в world.
    // gen — генератор случайных чисел (детерминированный, передаётся снаружи),
    // state — план с прошлого кадра (тёплый старт), туда же записывается новый (можно nullptr)
    Result plan(const Point &start, const Point &end, const Point &vStart, const Point &vEnd, const World &world, std::mt19937 &gen,
                State *state = nullptr, const Params &params = Params())
    {
        Result best;
        Trajectory traj;
        BangPart parts[3];
        const AbsRigBody *enemies = world.enemies;
        const int nEnemies = world.nEnemies, nObs = worldCount(world);
        double required[nObs + 1], allyVmax[world.nAllies + 1];
        int tried = 0, budget = params.budget;
        worldRequired(world, start, end, params, required);
        for (int k = 0; k < world.nAllies; k++)
            allyVmax[k] = allyMaxSpeed(world.allies[k]);
        const bool warm = state && state->valid;

        // размерность кандидата вида kind и его запись в массив: (P, vP) или (P, vP, P2, vP2)
        const int dims[3] = {0, 4, 8};
        auto resultValue = [](const Result &r)
        {
            return r.feasible ? r.cost : r.cost + 100 + r.violation;
        };
        // значение последнего построенного кандидата для Нелдера–Мида: стоимость, если допустим; если нет —
        // стоимость + 100 + нарушение зазора (хуже любого допустимого); если не построился — 1e6
        double lastValue = 1e6;
        // оценить построенную траекторию и запомнить, если она лучше текущей лучшей
        auto consider = [&](int kind, const double *x)
        {
            double clearance, violation;
            worldClearance(traj, world, required, allyVmax, params.margin, clearance, violation, params, params.horizon);
            bool feasible = violation <= 1e-6;
            double T = traj.duration();
            double deficit = params.penalty > 0 ? marginDeficit(traj, enemies, nEnemies, params) : 0;
            if (params.penalty > 0 && world.nAllies > 0)
                deficit = max(deficit, allyDeficit(traj, world, allyVmax, params));
            double cost = T * (1 + params.penalty * min(1.0, deficit / params.marginBase));
            lastValue = feasible ? cost : cost + 100 + violation;
            auto betterThan = [&](const Result &b)
            {
                if (!b.found)
                    return true;
                if (feasible != b.feasible)
                    return feasible;
                if (feasible)
                    return cost < b.cost;
                return violation < b.violation - 1e-9 || (violation <= b.violation + 1e-9 && cost < b.cost);
            };
            if (!betterThan(best))
                return false;
            best.traj = traj;
            best.found = true;
            best.feasible = feasible;
            best.direct = kind == 0;
            best.kind = kind;
            best.P = best.vP = best.P2 = best.vP2 = Point();
            if (kind >= 1)
                best.P = Point(x[0], x[1]), best.vP = Point(x[2], x[3]);
            if (kind == 2)
                best.P2 = Point(x[4], x[5]), best.vP2 = Point(x[6], x[7]);
            best.nParts = kind + 1;
            for (int i = 0; i < 3; i++)
                best.parts[i] = parts[i];
            best.T = T;
            best.clearance = clearance;
            best.cost = cost;
            best.violation = violation;
            best.deficit = deficit;
            return true;
        };
        // построить и оценить кандидата вида kind. prune = false отключает досрочный отказ:
        // Нелдеру–Миду нужна настоящая стоимость каждого кандидата, а не только «хуже лучшего»
        auto tryCand = [&](int kind, const double *x, bool prune = true)
        {
            tried++;
            lastValue = 1e6;
            double y[8];
            for (int i = 0; i < dims[kind]; i++)
            {
                if (!std::isfinite(x[i]))
                    return false;
                y[i] = x[i];
            }
            // стоимость не меньше времени, поэтому начало длиннее лучшей стоимости — дальше не считаем
            auto tooLong = [&]()
            {
                return prune && best.found && best.feasible && traj.duration() >= best.cost;
            };
            auto clampVel = [&](int i)
            {
                double speed = hypot(y[i], y[i + 1]);
                if (speed > MAX_VEL)
                    y[i] *= MAX_VEL / speed, y[i + 1] *= MAX_VEL / speed;
            };
            traj.reset(start, vStart);
            clampVel(2);
            if (!appendBangBang(traj, Point(y[2], y[3]), Point(y[0], y[1]), params, &parts[0]) || tooLong())
                return false;
            if (kind == 2)
            {
                clampVel(6);
                if (!appendBangBang(traj, Point(y[6], y[7]), Point(y[4], y[5]), params, &parts[1]) || tooLong())
                    return false;
            }
            if (!appendBangBang(traj, vEnd, end, params, &parts[kind == 2 ? 2 : 1]))
                return false;
            return consider(kind, y);
        };
        auto tryVia = [&](const Point &P, const Point &vP, bool prune = true)
        {
            const double x[4] = {P.x, P.y, vP.x, vP.y};
            return tryCand(1, x, prune);
        };
        // записать кандидата (лучшего или из state) в массив
        auto pack = [](int kind, const Point &P, const Point &vP, const Point &P2, const Point &vP2, double *x)
        {
            x[0] = P.x, x[1] = P.y, x[2] = vP.x, x[3] = vP.y;
            if (kind == 2)
                x[4] = P2.x, x[5] = P2.y, x[6] = vP2.x, x[7] = vP2.y;
        };
        auto tryWarm = [&]()
        {
            double x[8];
            pack(state->kind, state->P, state->vP, state->P2, state->vP2, x);
            tryCand(state->kind, x);
        };

        // 1. прямая траектория
        tried++;
        traj.reset(start, vStart);
        if (appendBangBang(traj, vEnd, end, params, &parts[0]))
        {
            consider(0, nullptr);
        }
        Trajectory directTraj = traj;
        bool hasDirect = best.found;
        std::uniform_real_distribution<double> u01(0, 1), uSym(-1, 1);
        std::normal_distribution<double> gauss(0, 1);
        // кандидат рядом с прямой траекторией: промежуточное состояние = её состояние в случайный момент плюс шум.
        // Прямой bang-bang при ненулевых vStart/vEnd не оптимален по времени, изогнутая траектория бывает быстрее
        auto tryNearDirect = [&](double scale)
        {
            double t = directTraj.duration() * (0.15 + 0.7 * u01(gen));
            tryVia(directTraj.posAt(t) + Point(gauss(gen), gauss(gen)) * (params.refineStepPos * scale),
                   directTraj.velAt(t) + Point(gauss(gen), gauss(gen)) * (params.refineStepVel * scale));
        };

        // Нелдер–Мид по параметрам кандидата вида kind из точки x0 со значением f0, пока не кончится бюджет.
        // Лучший кандидат запоминается в consider, как и при обычном переборе. Если симплекс сжался,
        // поиск перезапускается из лучшего с шагом втрое меньше
        auto nelderMead = [&](int kind, const double *x0, double f0, double stepPos, double stepVel, int limit)
        {
            const int n = dims[kind];
            double xs[9][8], fs[9], c[8], xr[8], xe[8], xc[8], start0[8], steps[8];
            auto eval = [&](const double *x)
            {
                tryCand(kind, x, false);
                return lastValue;
            };
            auto out = [&]()
            {
                return tried >= limit || tried >= budget;
            };
            for (int j = 0; j < n; j++)
            {
                start0[j] = x0[j];
                // координаты точек — stepPos, скорости — stepVel
                steps[j] = j % 4 < 2 ? stepPos : stepVel;
            }
            double fStart = f0, shrink = 1;
            while (!out())
            {
                for (int i = 0; i <= n; i++)
                    for (int j = 0; j < n; j++)
                        xs[i][j] = start0[j] + (i > 0 && j == i - 1 ? steps[j] * shrink : 0);
                fs[0] = fStart;
                for (int i = 1; i <= n && !out(); i++)
                    fs[i] = eval(xs[i]);
                if (out())
                    return;
                bool converged = false;
                while (!out() && !converged)
                {
                    // сортировка вершин по значению
                    for (int i = 1; i <= n; i++)
                        for (int j = i; j > 0 && fs[j] < fs[j - 1]; j--)
                        {
                            std::swap(fs[j], fs[j - 1]);
                            for (int k = 0; k < n; k++)
                                std::swap(xs[j][k], xs[j - 1][k]);
                        }
                    double spread = 0;
                    for (int i = 1; i <= n; i++)
                        for (int k = 0; k < n; k++)
                            spread = max(spread, abs(xs[i][k] - xs[0][k]));
                    if (spread < 1 || fs[n] - fs[0] < 1e-7 * max(1.0, abs(fs[0])))
                    {
                        converged = true;
                        break;
                    }
                    for (int k = 0; k < n; k++)
                    {
                        c[k] = 0;
                        for (int i = 0; i < n; i++)
                            c[k] += xs[i][k] / n;
                        xr[k] = c[k] + (c[k] - xs[n][k]);
                    }
                    double fr = eval(xr);
                    if (fr < fs[0])
                    {
                        double fe = 1e18;
                        if (!out())
                        {
                            for (int k = 0; k < n; k++)
                                xe[k] = c[k] + 2 * (c[k] - xs[n][k]);
                            fe = eval(xe);
                        }
                        for (int k = 0; k < n; k++)
                            xs[n][k] = fe < fr ? xe[k] : xr[k];
                        fs[n] = min(fe, fr);
                    }
                    else if (fr < fs[n - 1])
                    {
                        for (int k = 0; k < n; k++)
                            xs[n][k] = xr[k];
                        fs[n] = fr;
                    }
                    else
                    {
                        if (out())
                            return;
                        bool outside = fr < fs[n];
                        for (int k = 0; k < n; k++)
                            xc[k] = outside ? c[k] + 0.5 * (xr[k] - c[k]) : c[k] + 0.5 * (xs[n][k] - c[k]);
                        double fc = eval(xc);
                        if (fc < min(fr, fs[n]))
                        {
                            for (int k = 0; k < n; k++)
                                xs[n][k] = xc[k];
                            fs[n] = fc;
                        }
                        else
                        {
                            // сжатие к лучшей вершине
                            for (int i = 1; i <= n && !out(); i++)
                            {
                                for (int k = 0; k < n; k++)
                                    xs[i][k] = xs[0][k] + 0.5 * (xs[i][k] - xs[0][k]);
                                fs[i] = eval(xs[i]);
                            }
                        }
                    }
                }
                // перезапуск из лучшего найденного с меньшим шагом
                if (!best.found || best.kind != kind)
                    return;
                pack(kind, best.P, best.vP, best.P2, best.vP2, start0);
                fStart = resultValue(best);
                shrink /= 3;
            }
        };
        // старт Нелдера–Мида: лучший кандидат или, если лучшая — прямая, её середина
        auto runNelderMead = [&](double stepPos, double stepVel, int limit)
        {
            double x0[8];
            if (best.direct)
            {
                double tm = directTraj.duration() / 2;
                Point P0 = directTraj.posAt(tm), vP0 = directTraj.velAt(tm);
                tryVia(P0, vP0, false);
                pack(1, P0, vP0, Point(), Point(), x0);
                nelderMead(1, x0, lastValue, stepPos, stepVel, limit);
            }
            else
            {
                pack(best.kind, best.P, best.vP, best.P2, best.vP2, x0);
                nelderMead(best.kind, x0, resultValue(best), stepPos, stepVel, limit);
            }
        };
        // случайный шаг вокруг лучшего кандидата (не прямой)
        auto tryNearBest = [&](double scale)
        {
            if (best.kind == 1)
            {
                Point P = best.P + Point(gauss(gen), gauss(gen)) * (params.refineStepPos * scale);
                Point vP = best.vP + Point(gauss(gen), gauss(gen)) * (params.refineStepVel * scale);
                tryVia(P, vP);
                return;
            }
            double x[8];
            int kind = best.kind;
            pack(kind, best.P, best.vP, best.P2, best.vP2, x);
            for (int j = 0; j < dims[kind]; j++)
                x[j] += gauss(gen) * scale * (j % 4 < 2 ? params.refineStepPos : params.refineStepVel);
            tryCand(kind, x);
        };
        // итог: точный зазор, тёплый старт на следующий кадр
        auto finish = [&]()
        {
            best.nTried = tried;
            if (best.found)
            {
                // точный зазор итоговой траектории (при переборе он обрезан сверху значением margin)
                double violation;
                worldClearance(best.traj, world, required, allyVmax, -1, best.clearance, violation, params, params.horizon);
            }
            else
                best.traj.reset(start, vStart);
            if (state)
            {
                state->valid = best.found && !best.direct;
                state->kind = best.kind;
                state->P = best.P;
                state->vP = best.vP;
                state->P2 = best.P2;
                state->vP2 = best.vP2;
            }
        };

        // прямая свободна и мягкий запас не нарушен: обходить нечего, только ищем более быструю траекторию рядом
        if (best.feasible && best.deficit <= 0)
        {
            if (warm)
                tryWarm();
            int limit = tried + params.directShare;
            if (params.nelderMead)
                runNelderMead(params.nmStepPos / 2, params.nmStepVel / 2, limit);
            double scale = 0.5, decay = pow(0.1, 1.0 / max(1, params.directShare));
            while (tried < limit && tried < budget)
            {
                if (best.direct)
                    tryNearDirect(scale);
                else
                    tryNearBest(scale);
                scale *= decay;
            }
            finish();
            return best;
        }

        // 2. тёплый старт
        if (warm && tried < budget)
            tryWarm();

        // 3. касательные точки
        Point waypoints[SAMPLER_MAX_WAYPOINTS];
        // движущиеся препятствия берутся там, где они будут, когда робот поравняется с ними. Свои роботы тоже
        // идут в этот список кругами: положение берётся с их траектории в тот же момент
        const int nCircles = nEnemies + world.nAllies;
        AbsRigBody predicted[nCircles + 1];
        if (params.predictTangents)
            predictObstacles(start, end, enemies, nEnemies, predicted, params.horizon);
        else
            for (int k = 0; k < nEnemies; k++)
                predicted[k] = enemies[k];
        Point line = end - start;
        double len = line.mag();
        for (int k = 0; k < world.nAllies; k++)
        {
            const Ally &al = world.allies[k];
            Point p = allyPosAt(al, 0);
            if (params.predictTangents && len > 1e-6)
                for (int it = 0; it < 3; it++)
                {
                    double along = max(0.0, min(len, (p - start) ^ line) / len);
                    p = allyPosAt(al, min(3.0, along / (0.7 * MAX_VEL)));
                }
            predicted[nEnemies + k] = AbsRigBody(p, al.rad, Point(0, 0));
        }
        int nWaypoints = collectWaypoints(start, end, predicted, nCircles, world, params.margin, waypoints, SAMPLER_MAX_WAYPOINTS / 2);
        // те же точки с малым запасом: они открывают узкие проходы, куда с запасом margin не пролезть
        nWaypoints += collectWaypoints(start, end, predicted, nCircles, world, params.tightMargin, waypoints + nWaypoints, SAMPLER_MAX_WAYPOINTS / 2);
        int tangentLimit = tried + int(budget * params.tangentShare);
        const double speedFactors[] = {1.0, 0.6, 0.3};
        for (double factor : speedFactors)
            for (int i = 0; i < nWaypoints && tried < tangentLimit && tried < budget; i++)
            {
                Point toEnd = end - waypoints[i];
                double dist = toEnd.mag();
                if (dist < 1e-6)
                    continue;
                tryVia(waypoints[i], toEnd * (MAX_VEL * factor / dist));
            }
        // середина прямой траектории как отправная точка для уточнения
        if (hasDirect && tried < budget)
            tryVia(directTraj.posAt(directTraj.duration() / 2), directTraj.velAt(directTraj.duration() / 2));

        int extraStart = tried;
        // две промежуточные точки, если через одну допустимого кандидата нет (препятствия и у старта, и у цели,
        // зона, которую надо обходить через два угла). Вторая точка — обход для отрезка P1→end, либо первая —
        // обход для отрезка start→P2
        auto runTwoVia = [&](int limit)
        {
            Point extra[4];
            auto tryPair = [&](const Point &P1, const Point &P2, double factor)
            {
                Point d1 = P2 - P1, d2 = end - P2;
                if (d1.mag() < 1e-6 || d2.mag() < 1e-6)
                    return;
                Point v1 = d1 * (MAX_VEL * factor / d1.mag()), v2 = d2 * (MAX_VEL * factor / d2.mag());
                const double x[8] = {P1.x, P1.y, v1.x, v1.y, P2.x, P2.y, v2.x, v2.y};
                tryCand(2, x);
            };
            for (int i = 0; i < nWaypoints && tried < limit && tried < budget; i++)
            {
                int n2 = collectWaypoints(waypoints[i], end, predicted, nCircles, world, params.margin, extra, 4);
                for (int j = 0; j < min(n2, 2) && tried < limit; j++)
                    tryPair(waypoints[i], extra[j], 0.6);
                n2 = collectWaypoints(start, waypoints[i], predicted, nCircles, world, params.margin, extra, 4);
                for (int j = 0; j < min(n2, 2) && tried < limit; j++)
                    tryPair(extra[j], waypoints[i], 0.6);
            }
        };
        if (params.twoVia && !best.feasible)
            runTwoVia(tried + int(budget * params.twoViaShare));

        // 4. случайные точки: чётные рядом с отрезком start→end, нечётные по всему полю
        Point normal = len > 1e-6 ? Point(-line.y, line.x) / len : Point(0, 1);
        double spread = max(500.0, 0.3 * len);
        // пары точек тратят бюджет случайных, но не больше его половины
        int randomLimit = max(tried, extraStart + int(budget * params.randomShare) - min(tried - extraStart, int(budget * params.randomShare / 2)));
        auto randomPoints = [&](int randomLimit)
        {
        for (int i = 0; tried < randomLimit && tried < budget; i++)
        {
            Point P, vP;
            if (i % 2 == 0)
            {
                P = start + line * u01(gen) + normal * (spread * uSym(gen));
                Point toEnd = end - P;
                double ang = toEnd.arg() + uSym(gen);
                vP = Point(cos(ang), sin(ang)) * (MAX_VEL * (0.2 + 0.8 * u01(gen)));
            }
            else
            {
                P = Point(FIELD_DX / 2.0 * uSym(gen), FIELD_DY / 2.0 * uSym(gen));
                double ang = M_PI * uSym(gen);
                vP = Point(cos(ang), sin(ang)) * (MAX_VEL * sqrt(u01(gen)));
            }
            tryVia(P, vP);
        }
        };
        randomPoints(randomLimit);
        // трудная сцена (допустимого кандидата всё ещё нет): добавочный бюджет на пары точек и случайные точки
        if (params.hardExtra > 0 && !best.feasible)
        {
            budget += params.hardExtra;
            if (params.twoVia)
                runTwoVia(tried + params.hardExtra / 3);
            randomPoints(tried + params.hardExtra / 3);
        }

        // 5. уточнение вокруг лучшего кандидата: Нелдер–Мид на весь остаток бюджета. Если он вернул
        // бюджет (лучшая так и осталась прямой), остаток тратится на случайное уточнение с убывающим шагом
        if (params.nelderMead && best.found)
            runNelderMead(params.nmStepPos, params.nmStepVel, budget);
        int nRefine = budget - tried;
        double scale = 1, decay = nRefine > 1 ? pow(0.05, 1.0 / nRefine) : 1;
        while (tried < budget)
        {
            if (!best.found)
            {
                // уточнять нечего: тратим попытку на случайную точку у отрезка
                tryVia(start + line * u01(gen) + normal * (spread * uSym(gen)), Point(gauss(gen), gauss(gen)) * (MAX_VEL / 2));
                continue;
            }
            if (best.direct)
                tryNearDirect(scale); // лучшая пока прямая: ищем рядом с ней
            else
                tryNearBest(scale);
            scale *= decay;
        }

        finish();
        // допустимой траектории нет: аварийное торможение с decel по прямой до остановки, если оно нарушает зазор
        // не сильнее лучшего кандидата. Тёплый старт остаётся от лучшего кандидата: в следующем кадре путь может
        // найтись. Торможение не доезжает до цели — это осознанный отказ
        if (params.emergencyBrake && (!best.found || !best.feasible))
        {
            Trajectory brake;
            brake.reset(start, vStart);
            double speed = vStart.mag();
            if (speed > 1e-6)
                brake.append(speed / params.decel, vStart * (-params.decel / speed));
            double clearance, violation;
            worldClearance(brake, world, required, allyVmax, -1, clearance, violation, params, params.horizon);
            // тормозим, только если лучший кандидат действительно задевает препятствие (зазор < 0), а не просто
            // проходит ближе SAFE_DIST, и торможение нарушает зазор не сильнее
            if (!best.found || (best.clearance < 0 && violation <= best.violation + 1e-9))
            {
                best.traj = brake;
                best.found = true;
                best.brake = true;
                best.direct = false;
                best.feasible = violation <= 1e-6;
                best.nParts = 0;
                best.T = brake.duration();
                best.clearance = clearance;
                best.violation = violation;
                best.cost = best.T;
            }
        }
        return best;
    }

    // компенсация задержки связи: состояние робота после того, как он отработает уже отправленные, но ещё не
    // исполненные команды скорости pending[0..n) (по одной на кадр dt, самая старая первая). Планировать надо из
    // этого состояния, а препятствия сдвинуть на их скорость, умноженную на ту же задержку n·dt
    void compensateDelay(Point &pos, Point &vel, const Point *pending, int n, double dt)
    {
        for (int i = 0; i < n; i++)
        {
            pos = pos + (vel + pending[i]) * (dt / 2);
            vel = pending[i];
        }
    }

    // то же, но препятствия — только круги с постоянной скоростью
    Result plan(const Point &start, const Point &end, const Point &vStart, const Point &vEnd, const AbsRigBody *enemies, int nEnemies,
                std::mt19937 &gen, State *state = nullptr, const Params &params = Params())
    {
        World world;
        world.enemies = enemies;
        world.nEnemies = nEnemies;
        return plan(start, end, vStart, vEnd, world, gen, state, params);
    }
}
