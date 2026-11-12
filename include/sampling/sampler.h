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

    // один bang-bang: разгон v0→Vm (ta), РПД с Vm (tc), торможение Vm→v1 (td)
    struct BangPart
    {
        Point v0, Vm, v1;
        double ta, tc, td;
    };

    // состояние между кадрами (тёплый старт)
    struct State
    {
        bool valid = false;
        Point P, vP;
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
        double nmStepPos = 200;      // мм: начальный шаг симплекса по P (у свободной прямой вдвое меньше)
        double nmStepVel = 400;      // мм/с: начальный шаг симплекса по vP
        double maxResidual = 1;      // мм: допустимая невязка конечной точки bang-bang
        double tangentShare = 0.3;   // доля бюджета на касательные точки
        double randomShare = 0.4;    // доля бюджета на случайные точки, остальное идёт на уточнение
        double refineStepPos = 300;  // мм: начальный шаг уточнения по P
        double refineStepVel = 300;  // мм/с: начальный шаг уточнения по vP
        int barrier = 160;           // предел сетки запасного перебора углов в bangBang (в initialApprox он 10000)
    };

    // качество траектории, общее для всех методов
    struct Quality
    {
        double T;         // полное время, с
        double clearance; // минимальный зазор до препятствий (расстояние до центра минус радиус), мм
        double miss;      // промах мимо конечной точки, мм
        double violation; // на сколько мм нарушен требуемый зазор (<= 0, если всё в порядке)
    };

    struct Result
    {
        Trajectory traj;
        bool found = false;    // построилась хоть одна траектория
        bool feasible = false; // зазор не меньше требуемого
        bool direct = false;   // прямая траектория без промежуточной точки
        Point P, vP;           // промежуточное состояние (если !direct)
        BangPart parts[2];
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
    void predictObstacles(const Point &start, const Point &end, const AbsRigBody *enemies, int nEnemies, AbsRigBody *out)
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
                    double t = min(3.0, along / (0.7 * MAX_VEL));
                    p = pos + vel * t;
                }
            out[k] = AbsRigBody(p, enemies[k].getRad(), vel);
        }
    }

    // зазор и нарушение по всей траектории. Если cap > 0, препятствия заведомо дальше cap пропускаются,
    // а зазор обрезается сверху значением cap (для перебора точнее не надо)
    void clearanceOf(const Trajectory &tr, const AbsRigBody *enemies, int nEnemies, const double *required, double cap, double &clearance, double &violation)
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
                best = min(best, segmentClearance(s, pos, vel, rad));
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
                deficit = max(deficit, need - segmentClearance(s, pos, vel, rad));
            }
        }
        return deficit;
    }

    // оценка траектории любого метода: время, точный зазор, промах (интегрированием участков), нарушение зазора
    Quality assess(const Trajectory &tr, const Point &endPos, const AbsRigBody *enemies, int nEnemies)
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
        Vm = bangBang::bangBang(v0, v1, dr, MAX_ACC, MAX_VEL, 10, 10, 2, params.barrier);
        if (!std::isfinite(Vm.x) || !std::isfinite(Vm.y))
            return false;
        double vm = Vm.mag();
        if (vm > MAX_VEL * 1.002)
            return false;
        auto remainder = [&](const Point &V)
        {
            ta = (V - v0).mag() / MAX_ACC;
            td = (v1 - V).mag() / MAX_ACC;
            return dr - (v0 + V) * (ta / 2) - (v1 + V) * (td / 2);
        };
        rem = remainder(Vm);
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

    // построить траекторию из (start, vStart) в (end, vEnd) мимо препятствий enemies.
    // gen — генератор случайных чисел (детерминированный, передаётся снаружи),
    // state — промежуточное состояние с прошлого кадра, туда же записывается новое (можно nullptr)
    Result plan(const Point &start, const Point &end, const Point &vStart, const Point &vEnd, const AbsRigBody *enemies, int nEnemies,
                std::mt19937 &gen, State *state = nullptr, const Params &params = Params())
    {
        Result best;
        Trajectory traj;
        BangPart parts[2];
        double required[nEnemies + 1];
        int tried = 0;
        for (int k = 0; k < nEnemies; k++)
            required[k] = requiredClearance(enemies[k], start, end);

        // значение последнего построенного кандидата для Нелдера–Мида: стоимость, если допустим; если нет —
        // стоимость + 100 + нарушение зазора (хуже любого допустимого); если не построился — 1e6
        double lastValue = 1e6;
        // оценить построенную траекторию и запомнить, если она лучше текущей лучшей
        auto consider = [&](bool direct, const Point &P, const Point &vP)
        {
            double clearance, violation;
            clearanceOf(traj, enemies, nEnemies, required, params.margin, clearance, violation);
            bool feasible = violation <= 1e-6;
            double T = traj.duration();
            double deficit = params.penalty > 0 ? marginDeficit(traj, enemies, nEnemies, params) : 0;
            double cost = T * (1 + params.penalty * min(1.0, deficit / params.marginBase));
            lastValue = feasible ? cost : cost + 100 + violation;
            bool better;
            if (!best.found)
                better = true;
            else if (feasible != best.feasible)
                better = feasible;
            else if (feasible)
                better = cost < best.cost;
            else
                better = violation < best.violation - 1e-9 || (violation <= best.violation + 1e-9 && cost < best.cost);
            if (!better)
                return false;
            best.traj = traj;
            best.found = true;
            best.feasible = feasible;
            best.direct = direct;
            best.P = P;
            best.vP = vP;
            best.nParts = direct ? 1 : 2;
            best.parts[0] = parts[0];
            best.parts[1] = parts[1];
            best.T = T;
            best.clearance = clearance;
            best.cost = cost;
            best.violation = violation;
            best.deficit = deficit;
            return true;
        };
        // кандидат через промежуточное состояние (P, vP). prune = false отключает досрочный отказ:
        // Нелдеру–Миду нужна настоящая стоимость каждого кандидата, а не только «хуже лучшего»
        auto tryVia = [&](const Point &P, Point vP, bool prune = true)
        {
            tried++;
            lastValue = 1e6;
            if (!std::isfinite(P.x) || !std::isfinite(P.y) || !std::isfinite(vP.x) || !std::isfinite(vP.y))
                return false;
            double speed = vP.mag();
            if (speed > MAX_VEL)
                vP = vP * (MAX_VEL / speed);
            traj.reset(start, vStart);
            if (!appendBangBang(traj, vP, P, params, &parts[0]))
                return false;
            // стоимость не меньше времени, поэтому первая часть длиннее лучшей стоимости — дальше не считаем
            if (prune && best.found && best.feasible && traj.duration() >= best.cost)
                return false;
            if (!appendBangBang(traj, vEnd, end, params, &parts[1]))
                return false;
            return consider(false, P, vP);
        };

        // 1. прямая траектория
        tried++;
        traj.reset(start, vStart);
        if (appendBangBang(traj, vEnd, end, params, &parts[0]))
        {
            consider(true, Point(), Point());
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

        // Нелдер–Мид по x = (P.x, P.y, vP.x, vP.y) из точки x0 со значением f0, пока не кончится бюджет.
        // Лучший кандидат запоминается в consider, как и при обычном переборе. Если симплекс сжался,
        // поиск перезапускается из лучшего с шагом втрое меньше
        auto nelderMead = [&](const Point &P0, const Point &vP0, double f0, double stepPos, double stepVel, int limit)
        {
            const int n = 4;
            double xs[n + 1][n], fs[n + 1], c[n], xr[n], xe[n], xc[n];
            auto eval = [&](const double *x)
            {
                tryVia(Point(x[0], x[1]), Point(x[2], x[3]), false);
                return lastValue;
            };
            auto out = [&]()
            {
                return tried >= limit || tried >= params.budget;
            };
            double start0[n] = {P0.x, P0.y, vP0.x, vP0.y};
            double fStart = f0;
            while (!out())
            {
                for (int i = 0; i <= n; i++)
                    for (int j = 0; j < n; j++)
                        xs[i][j] = start0[j] + (i > 0 && j == i - 1 ? (j < 2 ? stepPos : stepVel) : 0);
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
                if (!best.found || best.direct)
                    return;
                start0[0] = best.P.x, start0[1] = best.P.y, start0[2] = best.vP.x, start0[3] = best.vP.y;
                fStart = best.feasible ? best.cost : best.cost + 100 + best.violation;
                stepPos /= 3;
                stepVel /= 3;
            }
        };
        // значение лучшего кандидата в шкале lastValue
        auto bestValue = [&]()
        {
            return best.feasible ? best.cost : best.cost + 100 + best.violation;
        };
        // старт Нелдера–Мида: лучший кандидат через (P, vP) или, если лучшая — прямая, её середина
        auto runNelderMead = [&](double stepPos, double stepVel, int limit)
        {
            if (best.direct)
            {
                double tm = directTraj.duration() / 2;
                Point P0 = directTraj.posAt(tm), vP0 = directTraj.velAt(tm);
                tryVia(P0, vP0, false);
                nelderMead(P0, vP0, lastValue, stepPos, stepVel, limit);
            }
            else
                nelderMead(best.P, best.vP, bestValue(), stepPos, stepVel, limit);
        };

        // прямая свободна и мягкий запас не нарушен: обходить нечего, только ищем более быструю траекторию рядом
        if (best.feasible && best.deficit <= 0)
        {
            if (state && state->valid)
                tryVia(state->P, state->vP);
            int limit = tried + params.directShare;
            if (params.nelderMead)
                runNelderMead(params.nmStepPos / 2, params.nmStepVel / 2, limit);
            double scale = 0.5, decay = pow(0.1, 1.0 / max(1, params.directShare));
            while (tried < limit && tried < params.budget)
            {
                if (best.direct)
                    tryNearDirect(scale);
                else
                    tryVia(best.P + Point(gauss(gen), gauss(gen)) * (params.refineStepPos * scale),
                           best.vP + Point(gauss(gen), gauss(gen)) * (params.refineStepVel * scale));
                scale *= decay;
            }
            best.nTried = tried;
            double violation;
            clearanceOf(best.traj, enemies, nEnemies, required, -1, best.clearance, violation);
            if (state)
            {
                state->valid = !best.direct;
                state->P = best.P;
                state->vP = best.vP;
            }
            return best;
        }

        // 2. тёплый старт
        if (state && state->valid && tried < params.budget)
            tryVia(state->P, state->vP);

        // 3. касательные точки
        Point waypoints[SAMPLER_MAX_WAYPOINTS];
        // движущиеся препятствия берутся там, где они будут, когда робот поравняется с ними
        AbsRigBody predicted[nEnemies + 1];
        if (params.predictTangents)
            predictObstacles(start, end, enemies, nEnemies, predicted);
        else
            for (int k = 0; k < nEnemies; k++)
                predicted[k] = enemies[k];
        int nWaypoints = tangentWaypoints(start, end, predicted, nEnemies, params.margin, waypoints, SAMPLER_MAX_WAYPOINTS / 2);
        // те же точки с малым запасом: они открывают узкие проходы, куда с запасом margin не пролезть
        nWaypoints += tangentWaypoints(start, end, predicted, nEnemies, params.tightMargin, waypoints + nWaypoints, SAMPLER_MAX_WAYPOINTS / 2);
        int tangentLimit = tried + int(params.budget * params.tangentShare);
        const double speedFactors[] = {1.0, 0.6, 0.3};
        for (double factor : speedFactors)
            for (int i = 0; i < nWaypoints && tried < tangentLimit && tried < params.budget; i++)
            {
                Point toEnd = end - waypoints[i];
                double dist = toEnd.mag();
                if (dist < 1e-6)
                    continue;
                tryVia(waypoints[i], toEnd * (MAX_VEL * factor / dist));
            }
        // середина прямой траектории как отправная точка для уточнения
        if (hasDirect && tried < params.budget)
            tryVia(directTraj.posAt(directTraj.duration() / 2), directTraj.velAt(directTraj.duration() / 2));

        // 4. случайные точки: чётные рядом с отрезком start→end, нечётные по всему полю
        Point line = end - start;
        double len = line.mag();
        Point normal = len > 1e-6 ? Point(-line.y, line.x) / len : Point(0, 1);
        double spread = max(500.0, 0.3 * len);
        int randomLimit = tried + int(params.budget * params.randomShare);
        for (int i = 0; tried < randomLimit && tried < params.budget; i++)
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

        // 5. уточнение вокруг лучшего кандидата: Нелдер–Мид на весь остаток бюджета. Если он вернул
        // бюджет (лучшая так и осталась прямой), остаток тратится на случайное уточнение с убывающим шагом
        if (params.nelderMead && best.found)
            runNelderMead(params.nmStepPos, params.nmStepVel, params.budget);
        int nRefine = params.budget - tried;
        double scale = 1, decay = nRefine > 1 ? pow(0.05, 1.0 / nRefine) : 1;
        while (tried < params.budget)
        {
            if (!best.found)
            {
                // уточнять нечего: тратим попытку на случайную точку у отрезка
                tryVia(start + line * u01(gen) + normal * (spread * uSym(gen)), Point(gauss(gen), gauss(gen)) * (MAX_VEL / 2));
                continue;
            }
            if (best.direct)
            {
                // лучшая пока прямая: ищем рядом с ней
                tryNearDirect(scale);
                scale *= decay;
                continue;
            }
            Point P = best.P + Point(gauss(gen), gauss(gen)) * (params.refineStepPos * scale);
            Point vP = best.vP + Point(gauss(gen), gauss(gen)) * (params.refineStepVel * scale);
            tryVia(P, vP);
            scale *= decay;
        }

        best.nTried = tried;
        if (best.found)
        {
            // точный зазор итоговой траектории (при переборе он обрезан сверху значением margin)
            double violation;
            clearanceOf(best.traj, enemies, nEnemies, required, -1, best.clearance, violation);
        }
        else
        {
            best.traj.reset(start, vStart);
        }
        if (state)
        {
            state->valid = best.found && !best.direct;
            state->P = best.P;
            state->vP = best.vP;
        }
        return best;
    }
}
