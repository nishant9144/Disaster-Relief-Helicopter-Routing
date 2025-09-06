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

// --- ASSUMES the same Point, PackageInfo, Village, Helicopter, ProblemData,
//     Drop, Trip, HelicopterPlan, Solution types as you provided earlier. ---

namespace relief_impl {

// numeric eps
static constexpr double EPS_DBL = 1e-9;

// Euclidean distance
static double euclid(const Point &a, const Point &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

// Clarke-Wright savings candidate generator
// Inputs:
//   city: depot Point
//   vcoords: vector<Point> of villages (0-based indices)
//   dcap: per-trip distance capacity (we will enforce this when merging)
//   max_candidates: cap on number of returned candidate routes
//   rng: optional random engine to break ties/stabilize randomness
// Returns:
//   vector<vector<int>>  (each inner vector is ordered list of 0-based village indices)
static vector<vector<int>> generate_candidates_clarke_wright(
    const Point &city,
    const vector<Point> &vcoords,
    double dcap,
    int max_candidates,
    std::mt19937_64 &rng)
{
    int V = (int)vcoords.size();
    vector<vector<int>> result;
    if (V == 0) return result;

    // Precompute distances
    vector<double> d_depot(V);
    for (int i = 0; i < V; ++i) d_depot[i] = euclid(city, vcoords[i]);
    vector<vector<double>> dvv(V, vector<double>(V, 0.0));
    for (int i = 0; i < V; ++i)
        for (int j = i+1; j < V; ++j) {
            double d = euclid(vcoords[i], vcoords[j]);
            dvv[i][j] = dvv[j][i] = d;
        }

    // savings list: (s, i, j) for i<j
    struct Saving { double s; int i, j; };
    vector<Saving> savings;
    savings.reserve((size_t)V*(V-1)/2);
    for (int i = 0; i < V; ++i) {
        for (int j = i+1; j < V; ++j) {
            double s = d_depot[i] + d_depot[j] - dvv[i][j];
            savings.push_back({s, i, j});
        }
    }

    // sort descending by saving; tie-break randomly to diversify
    std::shuffle(savings.begin(), savings.end(), rng);
    sort(savings.begin(), savings.end(), [](const Saving &A, const Saving &B){
        return A.s > B.s;
    });

    // initial routes: each node in its own route (route_id -> vector<int>)
    vector<vector<int>> route_of_node(V);
    for (int i = 0; i < V; ++i) route_of_node[i] = vector<int>{i};
    // map node -> route index (index in "routes" vector)
    vector<int> node_route_id(V);
    vector<vector<int>> routes; routes.reserve(V);
    for (int i = 0; i < V; ++i) { routes.push_back(vector<int>{i}); node_route_id[i] = i; }

    auto route_dist = [&](const vector<int> &r)->double {
        return route_distance_from_city(city, vcoords, r);
    };

    // Merge according to savings list
    for (const auto &sv : savings) {
        if ((int)routes.size() >= max_candidates) break;
        int i = sv.i, j = sv.j;
        int ri = node_route_id[i];
        int rj = node_route_id[j];
        if (ri == rj) continue; // same route -> skip

        vector<int> &route_i = routes[ri];
        vector<int> &route_j = routes[rj];

        // only allow merges where i is at an end of route_i and j is at an end of route_j
        bool i_is_head = (route_i.front() == i);
        bool i_is_tail = (route_i.back() == i);
        bool j_is_head = (route_j.front() == j);
        bool j_is_tail = (route_j.back() == j);

        // possible merges: (tail_i + head_j) or (tail_j + head_i) depending on ends
        // We'll attempt tail_i + head_j (i at tail, j at head) first, then other orientation
        vector<vector<int>> candidates_to_try;
        if (i_is_tail && j_is_head) {
            vector<int> merged = route_i;
            merged.insert(merged.end(), route_j.begin(), route_j.end());
            candidates_to_try.push_back(std::move(merged));
        }
        if (j_is_tail && i_is_head) {
            vector<int> merged = route_j;
            merged.insert(merged.end(), route_i.begin(), route_i.end());
            candidates_to_try.push_back(std::move(merged));
        }
        // also try other two orientations (reversing one route) if helpful
        if (i_is_head && j_is_head) {
            // reverse route_i then merge
            vector<int> ri_rev = route_i; reverse(ri_rev.begin(), ri_rev.end());
            vector<int> merged = ri_rev;
            merged.insert(merged.end(), route_j.begin(), route_j.end());
            candidates_to_try.push_back(std::move(merged));
        }
        if (i_is_tail && j_is_tail) {
            // reverse route_j then merge
            vector<int> rj_rev = route_j; reverse(rj_rev.begin(), rj_rev.end());
            vector<int> merged = route_i;
            merged.insert(merged.end(), rj_rev.begin(), rj_rev.end());
            candidates_to_try.push_back(std::move(merged));
        }

        bool merged_done = false;
        for (auto &cand : candidates_to_try) {
            double d = route_dist(cand);
            if (d <= dcap + 1e-9) {
                // accept merge: create new route, invalidate old ones
                vector<int> newroute = std::move(cand);
                // mark old routes as empty (we'll reuse their spots)
                routes[ri].clear();
                routes[rj].clear();
                // add new route at end
                int new_id = (int)routes.size();
                routes.push_back(newroute);
                // update node_route_id for nodes in new route
                for (int node : newroute) node_route_id[node] = new_id;
                merged_done = true;
                break;
            }
        }
        // continue to next savings
    }

    // collect non-empty routes into result
    for (const auto &r : routes) if (!r.empty()) {
        // optionally prune routes that exceed dcap (shouldn't happen) or are too large
        double d = route_distance_from_city(city, vcoords, r);
        if (d <= dcap + 1e-9) result.push_back(r);
        if ((int)result.size() >= max_candidates) break;
    }

    return result;
}


// route distance from a city to an ordered list of villages (vcoords indexed by 0-based village indices)
static double route_distance_from_city(const Point &city,
                                       const vector<Point> &vcoords,
                                       const vector<int> &route) {
    if (route.empty()) return 0.0;
    double total = 0.0;
    total += euclid(city, vcoords[route[0]]);
    for (size_t i = 1; i < route.size(); ++i)
        total += euclid(vcoords[route[i-1]], vcoords[route[i]]);
    total += euclid(vcoords[route.back()], city);
    return total;
}

// Exact 3-type packing (doubles) by enumerating two types and computing third.
// returns tuple (xp, xd, xo, value)
static tuple<int,int,int,double> solve_packing_3types_double(
    double wcap,
    int M_R, int O_R,
    double w_d, double v_d,
    double w_p, double v_p,
    double w_o, double v_o)
{
    int max_p = (w_p <= 0.0) ? M_R : min(M_R, (int)floor((wcap + EPS_DBL) / w_p));
    int max_d = (w_d <= 0.0) ? M_R : min(M_R, (int)floor((wcap + EPS_DBL) / w_d));
    int max_o = (w_o <= 0.0) ? O_R : min(O_R, (int)floor((wcap + EPS_DBL) / w_o));

    const long long ENUMERATE_LOOP_LIMIT = 200000; // threshold for switching to greedy fallback

    // choose pair to enumerate (p,d), (p,o), (d,o) by smallest product
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

    if (choices[0].prod > ENUMERATE_LOOP_LIMIT) {
        greedy_fallback();
        return make_tuple(best_p, best_d, best_o, best_value);
    }

    for (int ci = 0; ci < 3; ++ci) {
        int A = choices[ci].a, B = choices[ci].b;
        int bounds[3] = { max_p, max_d, max_o };
        double weights[3] = { w_p, w_d, w_o };
        // if this pair product huge skip
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
                    if (A == 1) { /* a is dry */ }
                    if (B == 0) already_meals += b;
                    if (B == 1) { /* b is dry */ }
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
            }
        }
    }
    if (!isfinite(best_value)) greedy_fallback();
    return make_tuple(best_p, best_d, best_o, best_value);
}


// distribute items on route (p then d then o) and update M_rem_by_v / O_rem_by_v in-place
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

} // namespace relief_impl


// -------------------- SOLVE with Random Restart --------------------

Solution solve(const ProblemData& problem) {
    using namespace relief_impl;

    // prepare static inputs
    int V = (int)problem.villages.size();
    int H = (int)problem.helicopters.size();
    vector<Point> vcoords; vcoords.reserve(V);
    vector<int> village_ids_1based; village_ids_1based.reserve(V);
    for (int i = 0; i < V; ++i) {
        vcoords.push_back(problem.villages[i].coords);
        village_ids_1based.push_back(problem.villages[i].id);
    }
    vector<Point> cities = problem.cities;
    // packages: assume order dry(0), perishable(1), other(2)
    auto pkg_d = problem.packages[0];
    auto pkg_p = problem.packages[1];
    auto pkg_o = problem.packages[2];

    // initial demand
    vector<int> M_init(V), O_init(V);
    for (int i = 0; i < V; ++i) {
        long long pop = problem.villages[i].population;
        long long meals = 9LL * pop;
        M_init[i] = (int)min<long long>(meals, numeric_limits<int>::max());
        long long others = pop;
        O_init[i] = (int)min<long long>(others, numeric_limits<int>::max());
    }

    // precompute neighbors
    vector<vector<pair<double,int>>> v_nearest(V);
    for (int i = 0; i < V; ++i) {
        v_nearest[i].reserve(V-1);
        for (int j = 0; j < V; ++j) if (i!=j) {
            v_nearest[i].push_back({ euclid(vcoords[i], vcoords[j]), j });
        }
        sort(v_nearest[i].begin(), v_nearest[i].end());
    }

    // RNG and timing
    std::mt19937_64 rng((unsigned)chrono::high_resolution_clock::now().time_since_epoch().count());
    double time_limit_minutes = problem.time_limit_minutes;
    auto wall_start = chrono::steady_clock::now();
    chrono::milliseconds time_limit_ms = chrono::milliseconds(0);
    if (time_limit_minutes > 1e-9) {
        time_limit_ms = chrono::duration_cast<chrono::milliseconds>(chrono::duration<double>(time_limit_minutes*60.0));
    }
    // restart params
    int MAX_RESTARTS = 12;
    if (time_limit_ms.count() > 0) {
        // if time given, compute approximate restarts based on V (heuristic)
        MAX_RESTARTS = max(3, min(50, (int)(5 + V / 20)));
    }

    // candidate generation params (we'll randomize inside restarts)
    const int K_NEAREST = 6;
    const int MAX_CANDIDATES = max(500, V * 6);

    // storage for best overall solution across restarts
    double best_overall_obj = -numeric_limits<double>::infinity();
    Solution best_overall_solution;
    best_overall_solution.resize(H);

    // main restart loop
    for (int restart = 0; restart < MAX_RESTARTS; ++restart) {
        // check time limit
        if (time_limit_ms.count() > 0) {
            auto elapsed = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - wall_start);
            if (elapsed > time_limit_ms) break;
        }

        // initialize state for this restart
        vector<int> M_rem = M_init;
        vector<int> O_rem = O_init;
        vector<double> heli_remaining_total(H, problem.d_max);
        Solution sol;
        sol.resize(H);
        for (int h = 0; h < H; ++h) sol[h].helicopter_id = problem.helicopters[h].id;

        double current_obj = 0.0; // cumulative delivered value - trip costs for this restart

        // slight randomization seeds
        std::uniform_real_distribution<double> unif01(0.0, 1.0);
        // random shuffle helicopter order to vary assignment patterns
        vector<int> heli_order(H); iota(heli_order.begin(), heli_order.end(), 0);
        shuffle(heli_order.begin(), heli_order.end(), rng);

        bool progress = true;
        int global_iter = 0;
        const int MAX_GLOBAL_ITERS = 5000;

        while (progress && global_iter < MAX_GLOBAL_ITERS) {
            ++global_iter;
            progress = false;

            double best_gain = 1e-9;
            int best_h = -1;
            vector<int> best_route;
            int best_xp=0, best_xd=0, best_xo=0;
            double best_trip_dist = 0.0;
            double best_trip_value = 0.0;
            double best_trip_cost = 0.0;

            // Build candidate set with randomness: shuffle villages and sample neighbors randomly
            vector<int> vv(V); iota(vv.begin(), vv.end(), 0);
            shuffle(vv.begin(), vv.end(), rng);

            vector<vector<int>> candidates;
            candidates.reserve(min(MAX_CANDIDATES, max(1, V*3)));

            // include some random single-village candidates and high-demand ones
            for (int idx = 0; idx < V && (int)candidates.size() < MAX_CANDIDATES; ++idx) {
                int vi = vv[idx];
                if (M_rem[vi] > 0 || O_rem[vi] > 0) {
                    // randomly skip some singles to diversify restarts
                    if (unif01(rng) < 0.85) candidates.push_back(vector<int>{vi});
                }
            }

            // neighbor windows with slight randomness in selection
            for (int base_i = 0; base_i < V && (int)candidates.size() < MAX_CANDIDATES; ++base_i) {
                int vi = vv[base_i];
                if (M_rem[vi] == 0 && O_rem[vi] == 0) continue;
                vector<int> route; route.push_back(vi);
                int added = 0;
                for (int k = 0; k < (int)min((int)v_nearest[vi].size(), K_NEAREST-1); ++k) {
                    int nb = v_nearest[vi][k].second;
                    if (M_rem[nb] == 0 && O_rem[nb] == 0) continue;
                    // random chance to include neighbor to diversify
                    if (unif01(rng) < 0.9) {
                        route.push_back(nb);
                        ++added;
                        candidates.push_back(route);
                    }
                    if ((int)candidates.size() >= MAX_CANDIDATES) break;
                }
            }

            // Add some random multi-village sequences (length 2..K_NEAREST)
            for (int t = 0; t < min(V, 80) && (int)candidates.size() < MAX_CANDIDATES; ++t) {
                int seed = vv[t];
                if (M_rem[seed] == 0 && O_rem[seed] == 0) continue;
                vector<int> route = { seed };
                int len = 1 + (int)(unif01(rng) * (K_NEAREST-1));
                int cursor = seed;
                for (int l = 1; l < len; ++l) {
                    // pick a random near neighbor among top 6
                    int nb_idx = (int)(unif01(rng) * min((int)v_nearest[cursor].size(), 6));
                    int nb = v_nearest[cursor][nb_idx].second;
                    if (find(route.begin(), route.end(), nb) == route.end()) route.push_back(nb);
                    cursor = nb;
                }
                candidates.push_back(route);
            }

            // Ensure candidates not huge
            if ((int)candidates.size() > MAX_CANDIDATES) candidates.resize(MAX_CANDIDATES);

            // Evaluate candidates across helicopters (use randomized heli order)
            for (int id = 0; id < H; ++id) {
                int h = heli_order[id];
                const Helicopter &hel = problem.helicopters[h];
                double heli_total_rem = heli_remaining_total[h];
                if (heli_total_rem <= 1e-9) continue;

                int city_idx = max(0, hel.home_city_id - 1);
                const Point &home_city = cities[city_idx];
                double dcap = hel.distance_capacity;
                double wcap = hel.weight_capacity;
                double trip_fixed_cost = hel.fixed_cost;
                double trip_alpha = hel.alpha;

                // randomize candidate evaluation order slightly
                // shuffle per-iteration to avoid deterministic ties
                shuffle(candidates.begin(), candidates.end(), rng);

                for (const auto &route : candidates) {
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
                                                    pkg_o.weight, pkg_o.value);
                    if (xp + xd + xo == 0) continue;

                    double trip_cost = trip_fixed_cost + trip_alpha * trip_dist;
                    double gain = trip_value_items - trip_cost;

                    // add tiny random tie-breaker so restarts explore different options
                    double noise = (unif01(rng) - 0.5) * 1e-6 * fabs(gain);
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
            } // helicopters

            if (!progress || best_h == -1) break;

            // commit best trip found in this iteration for this restart
            vector<Drop> drops = distribute_on_route(best_route, M_rem, O_rem,
                                                    best_xp, best_xd, best_xo, village_ids_1based);

            Trip trip;
            trip.dry_food_pickup = best_xd;
            trip.perishable_food_pickup = best_xp;
            trip.other_supplies_pickup = best_xo;
            trip.drops = drops;

            sol[best_h].trips.push_back(trip);
            heli_remaining_total[best_h] -= best_trip_dist;
            // update objective: delivered value minus cost
            current_obj += (best_trip_value - best_trip_cost);
            // continue loop
        } // end greedy loop for one restart

        // after building solution for this restart, check if it's the best
        if (current_obj > best_overall_obj + 1e-12) {
            best_overall_obj = current_obj;
            best_overall_solution = sol;
        }

        // optional: early exit if objective is good enough (not implemented)
        // check time limit for next restart
        if (time_limit_ms.count() > 0) {
            auto elapsed = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - wall_start);
            if (elapsed > time_limit_ms) break;
        }
    } // restarts

    // if never found any trip, return empty plans with ids set
    if (best_overall_obj == -numeric_limits<double>::infinity()) {
        Solution empty_sol;
        empty_sol.resize(H);
        for (int h = 0; h < H; ++h) {
            empty_sol[h].helicopter_id = problem.helicopters[h].id;
        }
        return empty_sol;
    }

    // return best overall solution found across restarts
    // Note: we used sol to store only pickups & drops per trip; route order is not stored in Trip.
    return best_overall_solution;
}
