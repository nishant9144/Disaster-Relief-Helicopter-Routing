#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <tuple>
#include <numeric>
#include <random>
#include <chrono>
#include <functional>

#include "structures.h"
using namespace std;

// ---------- TimeBudget helper ----------
struct TimeBudget {
    chrono::steady_clock::time_point start;
    chrono::milliseconds limit_ms; // zero = disabled

    TimeBudget() : start(chrono::steady_clock::now()), limit_ms(0) {}
    TimeBudget(double minutes) : start(chrono::steady_clock::now()) {
        if (minutes > 1e-12) limit_ms = chrono::duration_cast<chrono::milliseconds>(chrono::duration<double>(minutes * 60.0));
        else limit_ms = chrono::milliseconds(0);
    }
    inline bool enabled() const { return limit_ms.count() > 0; }
    inline chrono::milliseconds elapsed() const {
        return chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - start);
    }
    inline bool exceeded() const {
        if (!enabled()) return false;
        return elapsed() > limit_ms;
    }
    // near end: remaining <= margin_ms
    inline bool almost_up(int margin_ms = 50) const {
        if (!enabled()) return false;
        auto rem = limit_ms - elapsed();
        return rem.count() <= margin_ms;
    }
};

// ---------- Helpers & core routines ----------
namespace relief_impl {

static constexpr double EPS_DBL = 1e-9;

// Euclidean distance
static double euclid(const Point &a, const Point &b) {
    double dx = a.x - b.x, dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

// route distance starting/ending at depot city, visiting villages in order
static double route_distance_from_city(const Point &city,
                                       const vector<Point> &vcoords,
                                       const vector<int> &route) {
    if (route.empty()) return 0.0;
    double total = 0.0;
    total += euclid(city, vcoords[route.front()]);
    for (size_t i = 1; i < route.size(); ++i)
        total += euclid(vcoords[route[i-1]], vcoords[route[i]]);
    total += euclid(vcoords[route.back()], city);
    return total;
}

// Exact 3-type packing for doubles by enumerating two types and computing the third.
// Now accepts a TimeBudget pointer to bail out early and use greedy fallback.
// Returns (xp, xd, xo, value) where xp=perishable, xd=dry, xo=other
static tuple<int,int,int,double> solve_packing_3types_double(
    double wcap,
    int M_R, int O_R,
    double w_d, double v_d,
    double w_p, double v_p,
    double w_o, double v_o,
    const TimeBudget *tb = nullptr)
{
    int max_p = (w_p <= 0.0) ? M_R : min(M_R, (int)floor((wcap + EPS_DBL) / w_p));
    int max_d = (w_d <= 0.0) ? M_R : min(M_R, (int)floor((wcap + EPS_DBL) / w_d));
    int max_o = (w_o <= 0.0) ? O_R : min(O_R, (int)floor((wcap + EPS_DBL) / w_o));

    const long long ENUMERATE_LOOP_LIMIT = 200000; // cap on product of two loops
    const int CHECK_TIME_FREQ = 1000; // check time every this many inner iter

    struct PairChoice { int a,b; long long prod; };
    PairChoice choices[3] = {
        {0,1, (long long)max(1,max_p) * max(1,max_d)},
        {0,2, (long long)max(1,max_p) * max(1,max_o)},
        {1,2, (long long)max(1,max_d) * max(1,max_o)}
    };
    sort(begin(choices), end(choices), [](const PairChoice &A, const PairChoice &B){
        return A.prod < B.prod;
    });

    double best_value = -numeric_limits<double>::infinity();
    int best_p = 0, best_d = 0, best_o = 0;

    auto eval = [&](int xp, int xd, int xo){
        double val = xp * v_p + xd * v_d + xo * v_o;
        if (val > best_value + 1e-12) {
            best_value = val; best_p = xp; best_d = xd; best_o = xo;
        }
    };

    auto greedy_fallback = [&](){
        struct Type { double ratio; int id; double w; double v; int demand; };
        vector<Type> types;
        types.push_back({ (v_p/(w_p+EPS_DBL)), 0, w_p, v_p, M_R });
        types.push_back({ (v_d/(w_d+EPS_DBL)), 1, w_d, v_d, M_R });
        types.push_back({ (v_o/(w_o+EPS_DBL)), 2, w_o, v_o, O_R });
        sort(types.begin(), types.end(), [](const Type &A, const Type &B){ return A.ratio > B.ratio; });
        int xp=0, xd=0, xo=0;
        double remw = wcap;
        for (auto &t : types) {
            if (t.w <= 0.0) continue;
            int can_take = (int)floor((remw + EPS_DBL) / t.w);
            int take = min(can_take, t.demand);
            if (t.id == 0) xp = take;
            else if (t.id == 1) xd = take;
            else xo = take;
            remw -= take * t.w;
        }
        eval(xp, xd, xo);
    };

    // If the best pair product is huge, immediately fallback to greedy
    if (choices[0].prod > ENUMERATE_LOOP_LIMIT) {
        greedy_fallback();
        return make_tuple(best_p, best_d, best_o, best_value);
    }

    long long inner_counter = 0;
    for (int ci = 0; ci < 3; ++ci) {
        int A = choices[ci].a, B = choices[ci].b;
        int bounds[3] = { max_p, max_d, max_o };
        double weights[3] = { w_p, w_d, w_o };
        if (choices[ci].prod > ENUMERATE_LOOP_LIMIT) continue;
        for (int a = 0; a <= bounds[A]; ++a) {
            double rem_after_a = wcap - a * weights[A];
            if (rem_after_a + EPS_DBL < 0.0) break;
            for (int b = 0; b <= bounds[B]; ++b) {
                double weight_used = a*weights[A] + b*weights[B];
                if (weight_used > wcap + EPS_DBL) break;
                double rem_weight = wcap - weight_used;
                int C = 3 - (A + B);
                int max_by_weight = (weights[C] <= 0.0) ? (C==2 ? O_R : M_R) : (int)floor((rem_weight + EPS_DBL) / weights[C]);
                int max_by_demand = 0;
                if (C == 2) max_by_demand = O_R;
                else {
                    int already_meals = 0;
                    if (A == 0) already_meals += a;
                    if (B == 0) already_meals += b;
                    max_by_demand = max(0, M_R - already_meals);
                }
                int c = min(max_by_weight, max_by_demand);
                if (c < 0) c = 0;
                int xp = 0, xd = 0, xo = 0;
                for (int idx = 0; idx < 3; ++idx) {
                    int cnt = (idx == A) ? a : ((idx == B) ? b : c);
                    if (idx == 0) xp = cnt;
                    else if (idx == 1) xd = cnt;
                    else xo = cnt;
                }
                if (xp + xd > M_R) continue;
                eval(xp, xd, xo);

                // periodic time check
                if (tb && (++inner_counter % CHECK_TIME_FREQ == 0)) {
                    if (tb->exceeded() || tb->almost_up(30)) {
                        // time nearly up -> fallback to greedy and return what we have
                        greedy_fallback();
                        return make_tuple(best_p, best_d, best_o, best_value);
                    }
                }
            }
        }
    }
    if (!isfinite(best_value)) greedy_fallback();
    return make_tuple(best_p, best_d, best_o, best_value);
}

// Distribute chosen xp, xd, xo along route in order (perishable then dry then other)
static vector<Drop> distribute_on_route(const vector<int> &route,
                                        vector<int> &M_rem_by_v,
                                        vector<int> &O_rem_by_v,
                                        int xp, int xd, int xo,
                                        const vector<int> &village_ids_1based)
{
    vector<Drop> drops;
    drops.reserve(route.size());
    int perishable_left = xp, dry_left = xd, other_left = xo;
    for (size_t i = 0; i < route.size(); ++i) {
        int vidx = route[i];
        int haveMeals = M_rem_by_v[vidx];
        int haveOther = O_rem_by_v[vidx];
        int give_p = 0, give_d = 0, give_o = 0;
        if (perishable_left > 0 && haveMeals > 0) {
            give_p = min(perishable_left, haveMeals);
            perishable_left -= give_p; haveMeals -= give_p;
        }
        if (dry_left > 0 && haveMeals > 0) {
            give_d = min(dry_left, haveMeals);
            dry_left -= give_d; haveMeals -= give_d;
        }
        if (other_left > 0 && haveOther > 0) {
            give_o = min(other_left, haveOther);
            other_left -= give_o; haveOther -= give_o;
        }
        M_rem_by_v[vidx] = haveMeals;
        O_rem_by_v[vidx] = haveOther;
        Drop drop;
        drop.village_id = village_ids_1based[vidx];
        drop.dry_food = give_d;
        drop.perishable_food = give_p;
        drop.other_supplies = give_o;
        drops.push_back(drop);
    }
    return drops;
}

// Clarke-Wright savings candidate generator (returns routes as vectors of 0-based village indices)
static vector<vector<int>> generate_candidates_clarke_wright(
    const Point &city,
    const vector<Point> &vcoords,
    double dcap,
    int max_candidates,
    std::mt19937_64 &rng,
    const TimeBudget *tb = nullptr)
{
    int V = (int)vcoords.size();
    vector<vector<int>> result;
    if (V == 0) return result;

    // periodic time check while building distances
    for (int i = 0; i < V; ++i) {
        if (tb && (i % 50 == 0) && (tb->exceeded() || tb->almost_up(30))) return result;
    }

    // precompute distances
    vector<double> d_depot(V);
    for (int i = 0; i < V; ++i) d_depot[i] = euclid(city, vcoords[i]);
    vector<vector<double>> dvv(V, vector<double>(V, 0.0));
    for (int i = 0; i < V; ++i)
        for (int j = i+1; j < V; ++j) {
            dvv[i][j] = dvv[j][i] = euclid(vcoords[i], vcoords[j]);
        }

    struct Saving { double s; int i, j; };
    vector<Saving> savings; savings.reserve((size_t)V*(V-1)/2);
    for (int i = 0; i < V; ++i)
        for (int j = i+1; j < V; ++j)
            savings.push_back({ d_depot[i] + d_depot[j] - dvv[i][j], i, j });

    // randomize then sort descending
    shuffle(savings.begin(), savings.end(), rng);
    sort(savings.begin(), savings.end(), [](const Saving &A, const Saving &B){ return A.s > B.s; });

    // initial routes (each node alone)
    vector<vector<int>> routes;
    routes.reserve(V*2);
    vector<int> node_route_id(V);
    for (int i = 0; i < V; ++i) {
        routes.push_back(vector<int>{i});
        node_route_id[i] = i;
    }

    auto route_dist = [&](const vector<int> &r)->double {
        return route_distance_from_city(city, vcoords, r);
    };

    int count_processed = 0;
    for (const auto &sv : savings) {
        if (tb && (++count_processed % 1000 == 0) && (tb->exceeded() || tb->almost_up(30))) break;
        if ((int)routes.size() >= max_candidates) break;
        int i = sv.i, j = sv.j;
        int ri = node_route_id[i], rj = node_route_id[j];
        if (ri == rj) continue;
        vector<int> &ri_route = routes[ri];
        vector<int> &rj_route = routes[rj];
        if (ri_route.empty() || rj_route.empty()) continue;

        bool i_head = (ri_route.front() == i), i_tail = (ri_route.back() == i);
        bool j_head = (rj_route.front() == j), j_tail = (rj_route.back() == j);

        vector<vector<int>> try_list;
        if (i_tail && j_head) {
            vector<int> merged = ri_route;
            merged.insert(merged.end(), rj_route.begin(), rj_route.end());
            try_list.push_back(move(merged));
        }
        if (j_tail && i_head) {
            vector<int> merged = rj_route;
            merged.insert(merged.end(), ri_route.begin(), ri_route.end());
            try_list.push_back(move(merged));
        }
        if (i_head && j_head) {
            vector<int> ri_rev = ri_route; reverse(ri_rev.begin(), ri_rev.end());
            vector<int> merged = ri_rev;
            merged.insert(merged.end(), rj_route.begin(), rj_route.end());
            try_list.push_back(move(merged));
        }
        if (i_tail && j_tail) {
            vector<int> rj_rev = rj_route; reverse(rj_rev.begin(), rj_rev.end());
            vector<int> merged = ri_route;
            merged.insert(merged.end(), rj_rev.begin(), rj_rev.end());
            try_list.push_back(move(merged));
        }

        for (auto &cand : try_list) {
            double d = route_dist(cand);
            if (d <= dcap + 1e-9) {
                routes[ri].clear(); routes[rj].clear();
                int new_id = (int)routes.size();
                routes.push_back(cand);
                for (int node : routes.back()) node_route_id[node] = new_id;
                break;
            }
        }
    }

    // collect non-empty routes
    for (const auto &r : routes) {
        if (!r.empty()) {
            double d = route_distance_from_city(city, vcoords, r);
            if (d <= dcap + 1e-9) result.push_back(r);
            if ((int)result.size() >= max_candidates) break;
        }
    }
    return result;
}

} // namespace relief_impl


// -------------------- Complete solve(...) with robust time checks --------------------

Solution solve(const ProblemData& problem) {
    using namespace relief_impl;

    int V = (int)problem.villages.size();
    int H = (int)problem.helicopters.size();

    // village coords and original 1-based ids
    vector<Point> vcoords; vcoords.reserve(V);
    vector<int> village_ids_1based; village_ids_1based.reserve(V);
    for (int i = 0; i < V; ++i) {
        vcoords.push_back(problem.villages[i].coords);
        village_ids_1based.push_back(problem.villages[i].id);
    }

    // cities array (depots)
    vector<Point> cities = problem.cities;

    // packages: assume order dry(0), perishable(1), other(2)
    Solution empty_sol;
    empty_sol.resize(H);
    for (int h = 0; h < H; ++h) empty_sol[h].helicopter_id = problem.helicopters[h].id;

    if (problem.packages.size() < 3) return empty_sol;
    auto pkg_d = problem.packages[0];
    auto pkg_p = problem.packages[1];
    auto pkg_o = problem.packages[2];

    // initial per-village demand
    vector<int> M_init(V), O_init(V);
    for (int i = 0; i < V; ++i) {
        long long pop = problem.villages[i].population;
        long long meals = 9LL * pop;
        M_init[i] = (int)min<long long>(meals, numeric_limits<int>::max());
        long long others = pop;
        O_init[i] = (int)min<long long>(others, numeric_limits<int>::max());
    }

    // precompute neighbor lists for quick candidate building
    vector<vector<pair<double,int>>> v_nearest(V);
    for (int i = 0; i < V; ++i) {
        v_nearest[i].reserve(max(0, V-1));
        for (int j = 0; j < V; ++j) if (i != j)
            v_nearest[i].push_back({ euclid(vcoords[i], vcoords[j]), j });
        sort(v_nearest[i].begin(), v_nearest[i].end());
    }

    // RNG and time control
    mt19937_64 rng((unsigned)chrono::high_resolution_clock::now().time_since_epoch().count());
    TimeBudget tb(problem.time_limit_minutes);

    // restart parameters
    int MAX_RESTARTS = 12;
    if (tb.enabled()) {
        MAX_RESTARTS = max(3, min(60, (int)(5 + V / 20)));
    }

    const int K_NEAREST = 6;
    const int MAX_CANDIDATES = max(500, V * 6);

    double best_overall_obj = -numeric_limits<double>::infinity();
    Solution best_overall_solution = empty_sol;

    // random restarts
    for (int restart = 0; restart < MAX_RESTARTS; ++restart) {
        // check wall-clock
        if (tb.enabled() && tb.exceeded()) break;
        if (tb.enabled() && tb.almost_up(50)) break; // safety margin

        // per-restart state
        vector<int> M_rem = M_init;
        vector<int> O_rem = O_init;
        vector<double> heli_remaining_total(H, problem.d_max);
        Solution sol; sol.resize(H);
        for (int h = 0; h < H; ++h) sol[h].helicopter_id = problem.helicopters[h].id;

        double current_obj = 0.0;

        // randomize helicopter ordering for this restart
        vector<int> heli_order(H); iota(heli_order.begin(), heli_order.end(), 0);
        shuffle(heli_order.begin(), heli_order.end(), rng);

        // greedy loop for this restart
        bool progress = true;
        int iter = 0;
        const int MAX_ITERS = 5000;
        uniform_real_distribution<double> unif01(0.0, 1.0);

        while (progress && iter < MAX_ITERS) {
            ++iter;
            // frequent time checks
            if (tb.enabled() && tb.exceeded()) { progress = false; break; }
            if (tb.enabled() && tb.almost_up(50)) { progress = false; break; }

            progress = false;

            double best_gain = 1e-9;
            int best_h = -1;
            vector<int> best_route;
            int best_xp=0, best_xd=0, best_xo=0;
            double best_trip_dist = 0.0;
            double best_trip_value = 0.0;
            double best_trip_cost = 0.0;

            // build candidate pool: Clarke-Wright + randomized neighbor windows + singletons
            vector<vector<int>> candidates;
            candidates.reserve(min(MAX_CANDIDATES, max(1, V*3)));

            // add single-village candidates (sampled randomly)
            vector<int> vv(V); iota(vv.begin(), vv.end(), 0);
            shuffle(vv.begin(), vv.end(), rng);
            for (int idx = 0; idx < V && (int)candidates.size() < MAX_CANDIDATES; ++idx) {
                if (tb.enabled() && (idx % 100 == 0) && tb.almost_up(40)) break;
                int vi = vv[idx];
                if (M_rem[vi] > 0 || O_rem[vi] > 0) {
                    if (unif01(rng) < 0.9) candidates.push_back(vector<int>{vi});
                }
            }

            // neighbor windows
            for (int base = 0; base < V && (int)candidates.size() < MAX_CANDIDATES; ++base) {
                if (tb.enabled() && (base % 100 == 0) && tb.almost_up(40)) break;
                int vi = vv[base];
                if (M_rem[vi] == 0 && O_rem[vi] == 0) continue;
                vector<int> route; route.push_back(vi);
                for (int k = 0; k < (int)min((int)v_nearest[vi].size(), K_NEAREST-1); ++k) {
                    int nb = v_nearest[vi][k].second;
                    if (M_rem[nb] == 0 && O_rem[nb] == 0) continue;
                    route.push_back(nb);
                    candidates.push_back(route);
                    if ((int)candidates.size() >= MAX_CANDIDATES) break;
                }
            }

            // For each helicopter (in randomized order), add Clarke-Wright candidates based on that helicopter's depot
            for (int hid_idx = 0; hid_idx < H && (int)candidates.size() < MAX_CANDIDATES; ++hid_idx) {
                if (tb.enabled() && (hid_idx % 10 == 0) && tb.almost_up(40)) break;
                int h = heli_order[hid_idx];
                const Helicopter &hel = problem.helicopters[h];
                int city_idx = max(0, hel.home_city_id - 1);
                const Point &home_city = cities[city_idx];
                int cw_cap = min(60, MAX_CANDIDATES / max(1, H));
                auto cw = generate_candidates_clarke_wright(home_city, vcoords, hel.distance_capacity, cw_cap, rng, &tb);
                for (auto &r : cw) {
                    bool useful = false;
                    for (int vid : r) if (M_rem[vid] > 0 || O_rem[vid] > 0) { useful = true; break; }
                    if (useful && (int)candidates.size() < MAX_CANDIDATES) candidates.push_back(r);
                }
            }

            if ((int)candidates.size() > MAX_CANDIDATES) candidates.resize(MAX_CANDIDATES);

            // Evaluate candidates across helicopters in randomized order
            int eval_count = 0;
            for (int hid_idx = 0; hid_idx < H; ++hid_idx) {
                if (tb.enabled() && (eval_count % 500 == 0) && tb.almost_up(40)) break;
                int h = heli_order[hid_idx];
                const Helicopter &hel = problem.helicopters[h];
                double heli_total_rem = heli_remaining_total[h];
                if (heli_total_rem <= 1e-9) continue;

                int city_idx = max(0, hel.home_city_id - 1);
                const Point &home_city = cities[city_idx];
                double dcap = hel.distance_capacity;
                double wcap = hel.weight_capacity;
                double fixed = hel.fixed_cost;
                double alpha = hel.alpha;

                // shuffle candidate order slightly
                shuffle(candidates.begin(), candidates.end(), rng);

                for (const auto &route : candidates) {
                    if (tb.enabled() && (++eval_count % 500 == 0) && tb.almost_up(30)) break;
                    if (route.empty()) continue;
                    int M_R = 0, O_Rt = 0;
                    for (int vid : route) { M_R += M_rem[vid]; O_Rt += O_rem[vid]; }
                    if (M_R == 0 && O_Rt == 0) continue;

                    double trip_dist = route_distance_from_city(home_city, vcoords, route);
                    if (trip_dist > dcap + 1e-9) continue;
                    if (trip_dist > heli_total_rem + 1e-9) continue;

                    int xp, xd, xo; double trip_value_items;
                    tie(xp, xd, xo, trip_value_items) =
                        solve_packing_3types_double(wcap, M_R, O_Rt,
                                                    pkg_d.weight, pkg_d.value,
                                                    pkg_p.weight, pkg_p.value,
                                                    pkg_o.weight, pkg_o.value,
                                                    &tb);
                    if (xp + xd + xo == 0) continue;

                    double trip_cost = fixed + alpha * trip_dist;
                    double gain = trip_value_items - trip_cost;
                    double noise = (unif01(rng) - 0.5) * 1e-8 * fabs(gain);
                    double gain_noisy = gain + noise;

                    if (gain_noisy > best_gain + 1e-12) {
                        best_gain = gain_noisy;
                        best_h = h;
                        best_route = route;
                        best_xp = xp; best_xd = xd; best_xo = xo;
                        best_trip_dist = trip_dist;
                        best_trip_value = trip_value_items;
                        best_trip_cost = trip_cost;
                        progress = true;
                    }
                } // candidates
                if (tb.enabled() && tb.almost_up(40)) break;
            } // helicopters

            if (!progress || best_h == -1) break;

            // commit best trip
            vector<Drop> drops = distribute_on_route(best_route, M_rem, O_rem,
                                                     best_xp, best_xd, best_xo, village_ids_1based);
            Trip trip;
            trip.dry_food_pickup = best_xd;
            trip.perishable_food_pickup = best_xp;
            trip.other_supplies_pickup = best_xo;
            trip.drops = drops;
            sol[best_h].trips.push_back(trip);
            heli_remaining_total[best_h] -= best_trip_dist;
            current_obj += (best_trip_value - best_trip_cost);

            // break early if time nearly up
            if (tb.enabled() && tb.almost_up(40)) break;
        } // greedy while

        // update best overall if improved
        if (current_obj > best_overall_obj + 1e-12) {
            best_overall_obj = current_obj;
            best_overall_solution = sol;
        }

        // stop restarts if time nearly up
        if (tb.enabled() && tb.almost_up(40)) break;
    } // restarts

    // If no solution found, return empty plan with ids set
    if (!isfinite(best_overall_obj) && best_overall_solution.size() == 0) {
        return empty_sol;
    }
    return best_overall_solution;
}
