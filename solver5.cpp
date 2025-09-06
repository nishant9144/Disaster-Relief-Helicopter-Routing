#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <tuple>
#include <numeric>
#include <random>
#include <chrono>

#include "structures.h"
using namespace std;

/*
  Assumptions:
   - problem.packages[0] = dry (d)
   - problem.packages[1] = perishable (p)
   - problem.packages[2] = other (o)
   - Helicopter.home_city_id is 1-based index into problem.cities (as in assignment statement)
   - Village.id is 1-based (we map to 0-based indices internally)
*/

namespace relief_impl {

static constexpr double EPS_DBL = 1e-9;

// Utility: euclidean distance between two Points (you already have inline distance; re-use if in scope)
static double euclid(const Point &a, const Point &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx*dx + dy*dy);
}

// Compute route distance starting/ending at a city, visiting villages in order.
// city_idx is 0-based index into problem.cities, route contains 0-based village indices.
static double route_distance_from_city(const Point &city,
                                       const vector<Point> &vcoords,
                                       const vector<int> &route) {
    if (route.empty()) return 0.0;
    double total = 0.0;
    // city -> first
    total += euclid(city, vcoords[route[0]]);
    // between villages
    for (size_t i = 1; i < route.size(); ++i) {
        total += euclid(vcoords[route[i-1]], vcoords[route[i]]);
    }
    // last -> city
    total += euclid(vcoords[route.back()], city);
    return total;
}

// Exact 3-type packing for doubles using enumeration of two variables and computing the third.
// Returns tuple (x_p, x_d, x_o, value). x_p = perishable, x_d = dry, x_o = other.
// Inputs:
//   wcap  - weight capacity of helicopter (double)
//   M_R   - total meal demand on the route (integer count: p+d <= M_R)
//   O_R   - total other demand on the route (integer count)
//   w_d,w_p,w_o - weights of types (double) (dry, perishable, other)
//   v_d,v_p,v_o - values of types (double)
static tuple<int,int,int,double> solve_packing_3types_double(
    double wcap,
    int M_R, int O_R,
    double w_d, double v_d,
    double w_p, double v_p,
    double w_o, double v_o)
{
    // upper bounds by demand and by weight individually
    int max_p = (w_p <= 0.0) ? M_R : min(M_R, (int)floor((wcap + EPS_DBL) / w_p));
    int max_d = (w_d <= 0.0) ? M_R : min(M_R, (int)floor((wcap + EPS_DBL) / w_d));
    int max_o = (w_o <= 0.0) ? O_R : min(O_R, (int)floor((wcap + EPS_DBL) / w_o));

    // If any bound is huge, cap enumeration to reasonable loops to avoid heavy runtime.
    // If enumeration caps trigger, we fall back to greedy ratio-based solution below.
    const int ENUMERATE_LOOP_LIMIT = 200000; // product of two loops threshold

    // Choose two types to enumerate: pick pair with smallest max bounds product
    struct PairChoice { int a, b, prod; }; // indices: 0:p,1:d,2:o
    // compute products
    PairChoice choices[3];
    // enumerate pairs: (p,d), (p,o), (d,o)
    choices[0] = {0,1, max(1, max_p) * max(1, max_d)};
    choices[1] = {0,2, max(1, max_p) * max(1, max_o)};
    choices[2] = {1,2, max(1, max_d) * max(1, max_o)};
    // sort ascending by prod
    sort(begin(choices), end(choices), [](const PairChoice &A, const PairChoice &B){
        return A.prod < B.prod;
    });

    double best_value = -numeric_limits<double>::infinity();
    int best_p = 0, best_d = 0, best_o = 0;

    auto evaluate_tuple = [&](int xp, int xd, int xo) {
        double value = xp * v_p + xd * v_d + xo * v_o;
        if (value > best_value + 1e-12) {
            best_value = value;
            best_p = xp; best_d = xd; best_o = xo;
        }
    };

    // Helper greedy fallback (ratio-based) when enumeration too large
    auto greedy_by_ratio = [&]() {
        // types: vector of (ratio, id)
        struct Type { double ratio; int id; double w; double v; int demand; };
        vector<Type> types;
        // id mapping: 0->p, 1->d, 2->o
        types.push_back({ (v_p / (w_p+EPS_DBL)), 0, w_p, v_p, M_R }); // perishable
        types.push_back({ (v_d / (w_d+EPS_DBL)), 1, w_d, v_d, M_R }); // dry
        types.push_back({ (v_o / (w_o+EPS_DBL)), 2, w_o, v_o, O_R }); // other
        sort(types.begin(), types.end(), [](const Type &A, const Type &B){
            return A.ratio > B.ratio;
        });
        int xp=0, xd=0, xo=0;
        double remw = wcap;
        for (auto &t: types) {
            if (t.w <= 0.0) continue;
            int can_take = (int)floor((remw + EPS_DBL) / t.w);
            int take = min(can_take, t.demand);
            if (t.id == 0) xp = take;
            else if (t.id == 1) xd = take;
            else xo = take;
            remw -= take * t.w;
        }
        evaluate_tuple(xp, xd, xo);
    };

    // If smallest product too large, do greedy fallback.
    if ((long long)choices[0].prod > ENUMERATE_LOOP_LIMIT) {
        greedy_by_ratio();
        return make_tuple(best_p, best_d, best_o, best_value);
    }

    // enumerations: iterate over the two chosen types in increasing order and compute third
    for (int choice_idx = 0; choice_idx < 3; ++choice_idx) {
        int A = choices[choice_idx].a;
        int B = choices[choice_idx].b;
        // Map ids: 0->p,1->d,2->o
        int bounds[3] = { max_p, max_d, max_o };
        double weights[3] = { w_p, w_d, w_o };
        double values[3] = { v_p, v_d, v_o };
        int demandMeals = M_R; // for meals (p + d <= M_R)
        int demandOthers = O_R;
        // iterate A,B
        // ensure loops are not too big
        long long prod = 1LL * max(1, bounds[A]) * max(1, bounds[B]);
        if (prod > ENUMERATE_LOOP_LIMIT) continue; // skip this pair, try next

        for (int a_cnt = 0; a_cnt <= bounds[A]; ++a_cnt) {
            double w_after_a = wcap - a_cnt * weights[A];
            if (w_after_a + EPS_DBL < 0.0) break;
            for (int b_cnt = 0; b_cnt <= bounds[B]; ++b_cnt) {
                double weight_used = a_cnt * weights[A] + b_cnt * weights[B];
                if (weight_used > wcap + EPS_DBL) break;
                double rem_weight = wcap - weight_used;
                // compute max for the remaining type C
                int C = 3 - (A + B); // because indices sum 0+1+2 = 3
                int max_by_weight = (weights[C] <= 0.0) ? (C == 2 ? demandOthers : demandMeals) :
                    (int)floor((rem_weight + EPS_DBL) / weights[C]);

                // compute max by demand depending on type
                int max_by_demand = 0;
                if (C == 2) { // other supplies
                    max_by_demand = demandOthers;
                } else {
                    // meals: we need to ensure p + d <= M_R
                    int already_meals = 0;
                    if (A == 0) already_meals += a_cnt; // A is perishable
                    if (A == 1) { /* a_cnt is dry */ }
                    if (B == 0) already_meals += b_cnt;
                    if (B == 1) { /* b_cnt is dry */ }
                    // compute remaining meal capacity
                    max_by_demand = max(0, demandMeals - already_meals);
                }
                int c_cnt = min(max_by_weight, max_by_demand);
                if (c_cnt < 0) c_cnt = 0;

                // Now map back to xp, xd, xo based on which index is which
                int xp = 0, xd = 0, xo = 0;
                for (int idx = 0; idx < 3; ++idx) {
                    int cnt = 0;
                    if (idx == A) cnt = a_cnt;
                    else if (idx == B) cnt = b_cnt;
                    else cnt = c_cnt;
                    if (idx == 0) xp = cnt;
                    else if (idx == 1) xd = cnt;
                    else xo = cnt;
                }
                // validity: meals constraint
                if (xp + xd > M_R) continue;
                evaluate_tuple(xp, xd, xo);
            }
        }
        // if we already found solution via this pair, it's fine; but we'll loop other pairs to be safe.
    }

    // as a fallback, if best_value still -inf, do greedy:
    if (!isfinite(best_value)) {
        greedy_by_ratio();
    }

    return make_tuple(best_p, best_d, best_o, best_value);
}


// Distribute chosen counts (xp, xd, xo) along route in order:
static vector<Drop> distribute_on_route(const vector<int> &route,
                                        vector<int> &M_rem_by_v, // in-place modified, indexed by 0-based village idx
                                        vector<int> &O_rem_by_v,
                                        int xp, int xd, int xo,
                                        const vector<int> &village_ids_1based) // village original ids for Drop.village_id
{
    // We'll deliver per-village: perishable first (xp), then dry (xd), then other supplies (xo).
    vector<Drop> drops;
    drops.reserve(route.size());
    int perishable_left = xp;
    int dry_left = xd;
    int other_left = xo;
    for (size_t i = 0; i < route.size(); ++i) {
        int vidx = route[i]; // 0-based
        int haveMeals = M_rem_by_v[vidx];
        int haveOther = O_rem_by_v[vidx];
        int give_p = 0, give_d = 0, give_o = 0;
        // give perishable up to village meal need and perishable_left
        if (perishable_left > 0 && haveMeals > 0) {
            give_p = min(perishable_left, haveMeals);
            perishable_left -= give_p;
            haveMeals -= give_p;
        }
        // give dry up to remaining meal need and dry_left
        if (dry_left > 0 && haveMeals > 0) {
            give_d = min(dry_left, haveMeals);
            dry_left -= give_d;
            haveMeals -= give_d;
        }
        // give other supplies up to O_rem and other_left
        if (other_left > 0 && haveOther > 0) {
            give_o = min(other_left, haveOther);
            other_left -= give_o;
            haveOther -= give_o;
        }
        // update remaining
        M_rem_by_v[vidx] = haveMeals;
        O_rem_by_v[vidx] = haveOther;
        Drop drop;
        drop.village_id = village_ids_1based[vidx];
        drop.dry_food = give_d;
        drop.perishable_food = give_p;
        drop.other_supplies = give_o;
        drops.push_back(drop);
    }
    // If there's leftover (due to rounding / no demand in route), we ignore leftovers (they remain unused).
    return drops;
}


// Simple 2-opt on a given route (in-place). Returns true if improvement made.
// Uses coordinates to compute distance starting/ending at city.
static bool try_2opt_improve_route(const Point &city, const vector<Point> &vcoords,
                                   vector<int> &route, double &best_route_dist) {
    bool improved = false;
    int n = (int)route.size();
    if (n <= 2) return false;
    // compute current distance (we receive it as best_route_dist)
    for (int i = 0; i < n - 1; ++i) {
        for (int k = i + 1; k < n; ++k) {
            // perform 2-opt swap between i..k
            // compute new distance delta quickly by recomputing whole route distance for simplicity
            vector<int> candidate = route;
            reverse(candidate.begin() + i, candidate.begin() + k + 1);
            double cand_dist = route_distance_from_city(city, vcoords, candidate);
            if (cand_dist + 1e-9 < best_route_dist) {
                route.swap(candidate); // assign
                best_route_dist = cand_dist;
                return true;
            }
        }
    }
    return improved;
}

} // namespace relief_impl


Solution solve(const ProblemData& problem) {
    using namespace relief_impl;

    // Prepare data
    int C = (int)problem.cities.size();
    int V = (int)problem.villages.size();
    int H = (int)problem.helicopters.size();

    // village coordinates vector (0-based)
    vector<Point> vcoords;
    vcoords.reserve(V);
    vector<int> village_ids_1based;
    village_ids_1based.reserve(V);
    for (int i = 0; i < V; ++i) {
        vcoords.push_back(problem.villages[i].coords);
        village_ids_1based.push_back(problem.villages[i].id);
    }
    // city coords
    vector<Point> cc = problem.cities; // 0-based

    // initial remaining demands per village
    vector<int> M_rem_by_v(V, 0); // meals (total p+d)
    vector<int> O_rem_by_v(V, 0); // other supplies
    for (int i = 0; i < V; ++i) {
        long long strands = problem.villages[i].population;
        long long meals = 9LL * strands;
        if (meals > (long long)numeric_limits<int>::max()) meals = numeric_limits<int>::max();
        M_rem_by_v[i] = (int)meals;
        long long others = strands;
        if (others > (long long)numeric_limits<int>::max()) others = numeric_limits<int>::max();
        O_rem_by_v[i] = (int)others;
    }

    // package weights/values (assume order dry, perishable, other)
    auto pkg_d = problem.packages[0];
    auto pkg_p = problem.packages[1];
    auto pkg_o = problem.packages[2];

    // Per helicopter remaining total distance (global DMax from problem.d_max)
    vector<double> heli_remaining_total(H);
    for (int h = 0; h < H; ++h) heli_remaining_total[h] = problem.d_max;

    // Prepare output structure
    Solution solution;
    solution.resize(H);
    for (int h = 0; h < H; ++h) {
        solution[h].helicopter_id = problem.helicopters[h].id;
        solution[h].trips.clear();
    }

    // Candidate generation parameter
    const int K_NEAREST = 6; // maximum villages per route we attempt
    const int MAX_CANDIDATES = max(500, V * 6);

    // Precompute village-to-village nearest lists (indices)
    vector<vector<pair<double,int>>> v_nearest(V);
    for (int i = 0; i < V; ++i) {
        v_nearest[i].reserve(V-1);
        for (int j = 0; j < V; ++j) if (i!=j) {
            double d = euclid(vcoords[i], vcoords[j]);
            v_nearest[i].push_back({d,j});
        }
        sort(v_nearest[i].begin(), v_nearest[i].end());
    }

    // Greedy main loop: pick the best profitable trip across all helicopters from a set of candidate routes
    bool progress = true;
    int global_iter = 0;
    const int MAX_GLOBAL_ITERS = 5000;

    while (progress && global_iter < MAX_GLOBAL_ITERS) {
        progress = false;
        ++global_iter;

        double best_gain = 1e-9; // require strictly positive
        int best_h = -1;
        vector<int> best_route;
        int best_xp=0, best_xd=0, best_xo=0;
        double best_trip_dist = 0.0;
        double best_trip_value = 0.0;
        // build candidate set cheaply: single-village and nearest-window routes
        vector<vector<int>> candidates;
        candidates.reserve(min(MAX_CANDIDATES, max(1,V*3)));
        // single village routes (only villages that still need something)
        for (int vi = 0; vi < V; ++vi) {
            if (M_rem_by_v[vi] > 0 || O_rem_by_v[vi] > 0) {
                candidates.push_back(vector<int>{vi});
            }
        }
        // nearest neighbor windows (for each village, extend by nearest neighbors up to K_NEAREST)
        for (int vi = 0; vi < V; ++vi) {
            if ((int)candidates.size() >= MAX_CANDIDATES) break;
            if (M_rem_by_v[vi] == 0 && O_rem_by_v[vi] == 0) continue;
            vector<int> route;
            route.push_back(vi);
            for (int k = 0; k < (int)min((int)v_nearest[vi].size(), K_NEAREST-1); ++k) {
                int nb = v_nearest[vi][k].second;
                if (M_rem_by_v[nb] == 0 && O_rem_by_v[nb] == 0) continue;
                route.push_back(nb);
                candidates.push_back(route);
                if ((int)candidates.size() >= MAX_CANDIDATES) break;
            }
        }
        // optionally generate some pair routes for top-demand villages
        // find top demand villages
        vector<int> idxs(V); iota(idxs.begin(), idxs.end(), 0);
        sort(idxs.begin(), idxs.end(), [&](int a, int b){
            long long da = (long long)M_rem_by_v[a] + (long long)O_rem_by_v[a];
            long long db = (long long)M_rem_by_v[b] + (long long)O_rem_by_v[b];
            return da > db;
        });
        for (int k = 0; k < min(V, 30); ++k) {
            int vi = idxs[k];
            if (M_rem_by_v[vi] == 0 && O_rem_by_v[vi] == 0) continue;
            // try combining with a couple of nearest neighbors
            for (int nb_iter = 0; nb_iter < min((int)v_nearest[vi].size(), 4); ++nb_iter) {
                int nb = v_nearest[vi][nb_iter].second;
                if (M_rem_by_v[nb] == 0 && O_rem_by_v[nb] == 0) continue;
                vector<int> r = { vi, nb };
                candidates.push_back(r);
            }
            if ((int)candidates.size() >= MAX_CANDIDATES) break;
        }

        // limit candidates size if too large
        if ((int)candidates.size() > MAX_CANDIDATES) {
            candidates.resize(MAX_CANDIDATES);
        }

        // For each helicopter, evaluate candidates
        for (int h = 0; h < H; ++h) {
            const Helicopter &hel = problem.helicopters[h];
            double heli_total_rem = heli_remaining_total[h];
            if (heli_total_rem <= 1e-9) continue; // no total distance left

            int home_city_id = hel.home_city_id; // probably 1-based
            int city_idx = max(0, home_city_id - 1);
            const Point &home_city = cc[city_idx];

            // per-trip constraints:
            double dcap = hel.distance_capacity;
            double wcap = hel.weight_capacity;
            double trip_fixed_cost = hel.fixed_cost;
            double trip_alpha = hel.alpha;

            for (const auto &route : candidates) {
                if (route.empty()) continue;
                // aggregated remaining demand on this route
                int M_R = 0, O_R = 0;
                for (int vidx : route) { M_R += M_rem_by_v[vidx]; O_R += O_rem_by_v[vidx]; }
                if (M_R == 0 && O_R == 0) continue;

                // compute route distance (start and end at home_city)
                double trip_dist = route_distance_from_city(home_city, vcoords, route);
                if (trip_dist > dcap + 1e-9) continue; // per-trip distance capacity
                if (trip_dist > heli_total_rem + 1e-9) continue; // overall helicopter remaining budget

                // solve packing for this route
                int xp, xd, xo; double trip_value_items;
                tie(xp, xd, xo, trip_value_items) =
                    solve_packing_3types_double(wcap, M_R, O_R,
                                                pkg_d.weight, pkg_d.value,
                                                pkg_p.weight, pkg_p.value,
                                                pkg_o.weight, pkg_o.value);
                if (xp + xd + xo == 0) continue; // nothing to carry

                double trip_cost = trip_fixed_cost + trip_alpha * trip_dist;
                double gain = trip_value_items - trip_cost;
                if (gain > best_gain + 1e-9) {
                    best_gain = gain;
                    best_h = h;
                    best_route = route;
                    best_xp = xp; best_xd = xd; best_xo = xo;
                    best_trip_dist = trip_dist;
                    best_trip_value = trip_value_items;
                    progress = true;
                }
            } // candidates loop
        } // helicopters loop

        if (!progress || best_h == -1) break;

        // Commit best trip:
        // distribute items on route, update remaining demands, append Trip to solution
        vector<int> route0 = best_route;
        vector<Drop> drops = distribute_on_route(route0, M_rem_by_v, O_rem_by_v,
                                                best_xp, best_xd, best_xo, village_ids_1based);

        Trip trip;
        trip.dry_food_pickup = best_xd;
        trip.perishable_food_pickup = best_xp;
        trip.other_supplies_pickup = best_xo;
        trip.drops = drops;

        solution[best_h].trips.push_back(trip);
        // decrement helicopter remaining total distance
        heli_remaining_total[best_h] -= best_trip_dist;

        // mark that we made progress; loop back to find next best trip
    } // main greedy loop

    // Local improvement: simple 2-opt per trip to reduce trip distance -> improves objective
    const int MAX_2OPT_PASSES = 20;
    for (int pass = 0; pass < MAX_2OPT_PASSES; ++pass) {
        bool any_improved = false;
        // For each helicopter and each trip, we only know drops and pickup counts, but not stored route.
        // Our Trip struct does not store explicit route order; for simplicity we skip heavy local search
        // that requires route re-assignment. A lightweight improvement: try to remove trips with zero delivered
        // value or very low marginal gain: (not implemented further here).
        // (If you want richer local search, store routes in Trip and recompute packaging when changing routes.)
        if (!any_improved) break;
    }

    // Return solution (each helicopter plan is already collected)
    return solution;
}
