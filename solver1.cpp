#include "structures.h"
#include <vector>
#include <random>
#include <chrono>
#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <cmath>
#include <iostream>

using namespace std;
using Clock = chrono::high_resolution_clock;

// ------------------ Utilities ------------------

static unordered_map<int,int> buildVillageIdIndex(const ProblemData &P) {
    unordered_map<int,int> m;
    for (int i = 0; i < (int)P.villages.size(); ++i) m[P.villages[i].id] = i;
    return m;
}

static double route_distance(const ProblemData &P, int home_city_id, const vector<int> &route_village_ids) {
    // uses distances from P.cities and P.villages via distance(...)
    if (route_village_ids.empty()) return 0.0;
    const Point &home = P.cities[home_city_id - 1];
    double d = 0.0;
    Point cur = home;
    for (int vid : route_village_ids) {
        // find village coords
        const Point &vp = P.villages[vid - 1].coords; // village.id assumed 1-based and order consistent
        d += distance(cur, vp);
        cur = vp;
    }
    d += distance(cur, home);
    return d;
}

// sanitize solution before return
static void sanitize_solution(Solution &sol) {
    for (auto &hp : sol) {
        for (auto &t : hp.trips) {
            if (!t.drops.empty()) {
                long long sd = 0, sp = 0, so = 0;
                for (const Drop &d : t.drops) {
                    sd += max(0, d.dry_food);
                    sp += max(0, d.perishable_food);
                    so += max(0, d.other_supplies);
                }
                t.dry_food_pickup = (int)sd;
                t.perishable_food_pickup = (int)sp;
                t.other_supplies_pickup = (int)so;
            } else {
                t.dry_food_pickup = max(0, t.dry_food_pickup);
                t.perishable_food_pickup = max(0, t.perishable_food_pickup);
                t.other_supplies_pickup = max(0, t.other_supplies_pickup);
            }
        }
    }
}

// compute trip route distance (per Trip uses `drops` order) with home city
static double trip_route_distance(const ProblemData &P, const Helicopter &H, const Trip &t, const unordered_map<int,int> &vidx) {
    if (t.drops.empty()) return 0.0;
    const Point &home = P.cities[H.home_city_id - 1];
    Point cur = home;
    double d = 0.0;
    for (const Drop &drop : t.drops) {
        auto it = vidx.find(drop.village_id);
        if (it == vidx.end()) continue;
        const Point &vp = P.villages[it->second].coords;
        d += distance(cur, vp);
        cur = vp;
    }
    d += distance(cur, home);
    return d;
}

// trip total weight and value (sum of drops if present or use trip-level pickups)
static double trip_total_weight(const ProblemData &P, const Trip &t) {
    double wd = P.packages[0].weight;
    double wp = P.packages[1].weight;
    double wo = P.packages[2].weight;
    long long sd=0, sp=0, so=0;
    for (const Drop &d : t.drops) { sd += d.dry_food; sp += d.perishable_food; so += d.other_supplies; }
    if (sd+sp+so > 0) return sd*wd + sp*wp + so*wo;
    long long td = max(0, t.dry_food_pickup), tp = max(0, t.perishable_food_pickup), to = max(0, t.other_supplies_pickup);
    return td*wd + tp*wp + to*wo;
}

static double trip_total_value(const ProblemData &P, const Trip &t) {
    double vd = P.packages[0].value;
    double vp = P.packages[1].value;
    double vo = P.packages[2].value;
    long long sd=0, sp=0, so=0;
    for (const Drop &d : t.drops) { sd += d.dry_food; sp += d.perishable_food; so += d.other_supplies; }
    if (sd+sp+so > 0) return sd*vd + sp*vp + so*vo;
    long long td = max(0, t.dry_food_pickup), tp = max(0, t.perishable_food_pickup), to = max(0, t.other_supplies_pickup);
    return td*vd + tp*vp + to*vo;
}

// evaluate full solution (returns Score where value = objective, trip_costs = total trip costs)
static Score evaluate_solution(const Solution &sol, const ProblemData &P, const unordered_map<int,int> &vidx) {
    double total_val = 0.0, total_cost = 0.0;
    for (const HelicopterPlan &hp : sol) {
        if (hp.helicopter_id < 1 || hp.helicopter_id > (int)P.helicopters.size()) continue;
        const Helicopter &H = P.helicopters[hp.helicopter_id - 1];
        for (const Trip &t : hp.trips) {
            total_val += trip_total_value(P, t);
            double dist = trip_route_distance(P, H, t, vidx);
            total_cost += H.fixed_cost + H.alpha * dist;
        }
    }
    double obj = total_val - total_cost;
    return Score(obj, total_cost);
}

// compute per-helicopter distance usage
static vector<double> compute_helicopter_distances(const Solution &sol, const ProblemData &P, const unordered_map<int,int> &vidx) {
    vector<double> used(P.helicopters.size(), 0.0);
    for (const HelicopterPlan &hp : sol) {
        const Helicopter &H = P.helicopters[hp.helicopter_id - 1];
        for (const Trip &t : hp.trips) used[hp.helicopter_id - 1] += trip_route_distance(P, H, t, vidx);
    }
    return used;
}

// ------------------ Clarke-Wright + greedy packing ------------------

// Build CW savings merges for a helicopter's assigned village indices (vill_indices are indices in P.villages vector: 0..V-1)
// returns a vector of candidate routes (each route is vector<int> of village ids (1-based) in order)
static vector<vector<int>> build_cw_routes_for_helicopter(const ProblemData &P, const Helicopter &H,
                                                          const vector<int> &vill_indices,
                                                          const unordered_map<int,int> &vidx,
                                                          const vector<long long> &meals_left,
                                                          const vector<long long> &others_left) {
    // Build initial map: route per village (only villages with some remaining demand)
    vector<vector<int>> routes;
    for (int vi : vill_indices) {
        if (meals_left[vi] <= 0 && others_left[vi] <= 0) continue;
        routes.push_back({ P.villages[vi].id }); // store village.id (1-based)
    }
    if (routes.empty()) return {};

    // savings list between all pairs
    struct Save { int a_idx, b_idx; double s; };
    vector<Save> saves;
    // function to get point by village id
    auto point_of = [&](int vid) -> const Point& {
        return P.villages[vid - 1].coords;
    };
    auto home = P.cities[H.home_city_id - 1];
    for (int i = 0; i < (int)routes.size(); ++i) {
        for (int j = i+1; j < (int)routes.size(); ++j) {
            const Point &pa = point_of(routes[i].front());
            const Point &pb = point_of(routes[j].front());
            double sij = distance(home, pa) + distance(home, pb) - distance(pa, pb);
            saves.push_back({i, j, sij});
        }
    }
    sort(saves.begin(), saves.end(), [](const Save &a, const Save &b){ return a.s > b.s; });

    // Helper: route-length (distance) and approximate weight if we were to deliver some to route.
    auto route_distance_from_ids = [&](const vector<int> &route)->double{
        Point cur = home;
        double d = 0.0;
        for (int vid : route) {
            const Point &vp = point_of(vid);
            d += distance(cur, vp);
            cur = vp;
        }
        d += distance(cur, home);
        return d;
    };

    // Merge attempts (classic CW: merge only if the villages are at ends of respective routes)
    // We'll maintain a flag merged[] and a mapping route index -> current route; for simplicity re-check end-matches each time.
    // This is a heuristic: we only merge if the merged route distance <= H.distance_capacity (per-trip) AND merging doesn't create route with no deliveries (trivial).
    // We don't compute exact weight here because packing occurs later; but we can early prune merges that create route distance > distance_capacity * 1.0
    vector<vector<int>> curr_routes = routes;
    for (const Save &sv : saves) {
        // find routes that contain original route indices sv.a_idx and sv.b_idx (they may have been merged)
        // We'll find route indices by searching for any route that contains the first village of the original route (slow but small).
        int ra = -1, rb = -1;
        for (int r = 0; r < (int)curr_routes.size(); ++r) {
            if (!curr_routes[r].empty() && curr_routes[r].front() == routes[sv.a_idx].front()) ra = r;
            if (!curr_routes[r].empty() && curr_routes[r].front() == routes[sv.b_idx].front()) rb = r;
        }
        if (ra == -1 || rb == -1 || ra == rb) continue;
        // attempt to merge if ends are compatible (we allow merging tail->head)
        // Simplify: always try (routeA + routeB) and (routeB + routeA) and accept if route_distance <= H.distance_capacity
        vector<int> cand1 = curr_routes[ra];
        cand1.insert(cand1.end(), curr_routes[rb].begin(), curr_routes[rb].end());
        double d1 = route_distance_from_ids(cand1);
        if (d1 <= H.distance_capacity + 1e-9) {
            // perform merge: remove rb then ra, then push cand1
            if (ra > rb) { curr_routes.erase(curr_routes.begin() + ra); curr_routes.erase(curr_routes.begin() + rb); }
            else { curr_routes.erase(curr_routes.begin() + rb); curr_routes.erase(curr_routes.begin() + ra); }
            curr_routes.push_back(move(cand1));
            continue;
        }
        vector<int> cand2 = curr_routes[rb];
        cand2.insert(cand2.end(), curr_routes[ra].begin(), curr_routes[ra].end());
        double d2 = route_distance_from_ids(cand2);
        if (d2 <= H.distance_capacity + 1e-9) {
            if (ra > rb) { curr_routes.erase(curr_routes.begin() + ra); curr_routes.erase(curr_routes.begin() + rb); }
            else { curr_routes.erase(curr_routes.begin() + rb); curr_routes.erase(curr_routes.begin() + ra); }
            curr_routes.push_back(move(cand2));
            continue;
        }
        // otherwise skip this saving
    }

    // Finally return the set of candidate routes
    return curr_routes;
}

// Given a route (sequence of village ids 1-based), pack payload greedily by value/weight across (village, item_type).
// meals_left_total: vector over village indices (0..V-1) containing remaining meals (both types combined)
// others_left: remaining others per village
// Returns Trip (drops filled) and total_value; but caller will compute trip cost and decide accept.
static pair<Trip,double> pack_route_greedy(const ProblemData &P, const Helicopter &H, const vector<int> &route_village_ids,
                                           vector<long long> &meals_left_total, vector<long long> &others_left) {
    // make local mutable copies for allocation
    Trip trip;
    trip.dry_food_pickup = 0;
    trip.perishable_food_pickup = 0;
    trip.other_supplies_pickup = 0;

    int V = (int)P.villages.size();
    // order villages in route by their position to create drops in that order
    vector<int> route_vids = route_village_ids; // each is village.id (1-based)

    // global value/weight ranking among item types:
    // We will consider two "meal types": perishable (index 1) and dry (index 0) — but we only have a single "meals_left_total" count.
    // We will allocate perishable first if its v/w is higher, otherwise dry first; then others.
    struct ItemType { int type; double vw; double weight; double value; }; // type: 0=dry meal, 1=perishable meal, 2=other
    ItemType it0{0, P.packages[0].value / P.packages[0].weight, P.packages[0].weight, P.packages[0].value};
    ItemType it1{1, P.packages[1].value / P.packages[1].weight, P.packages[1].weight, P.packages[1].value};
    ItemType it2{2, P.packages[2].value / P.packages[2].weight, P.packages[2].weight, P.packages[2].value};
    vector<ItemType> types = { it0, it1, it2 };
    sort(types.begin(), types.end(), [](const ItemType &a, const ItemType &b){ return a.vw > b.vw; });

    double remaining_weight = H.weight_capacity;
    double total_value = 0.0;

    // For each type in best v/w order, iterate route villages and allocate as much as possible while respecting per-village demand
    // Note: meals are shared between perishable & dry; for perishable allocation we limit by meals_left_total[vi], then for dry we use remaining meals_left_total after perishable.
    // Build drops map: village_id -> Drop
    unordered_map<int, Drop> dropmap;
    for (int vid : route_vids) {
        Drop d; d.village_id = vid; d.dry_food = 0; d.perishable_food = 0; d.other_supplies = 0;
        dropmap[vid] = d;
    }

    for (const ItemType &it : types) {
        if (remaining_weight < it.weight - 1e-9) continue; // cannot fit any unit of this type
        for (int vid : route_vids) {
            int vindex = vid - 1;
            // compute max count available at this village for this item type
            long long available = 0;
            if (it.type == 2) available = others_left[vindex];
            else {
                // for meal types, available is meals_left_total (shared)
                available = meals_left_total[vindex];
            }
            if (available <= 0) continue;
            long long can_by_weight = (long long)floor((remaining_weight + 1e-9) / it.weight);
            if (can_by_weight <= 0) continue;
            long long take = min(available, can_by_weight);
            if (take <= 0) continue;
            // assign
            if (it.type == 2) {
                dropmap[vid].other_supplies += (int)take;
                others_left[vindex] -= take;
            } else if (it.type == 1) {
                // perishable
                dropmap[vid].perishable_food += (int)take;
                meals_left_total[vindex] -= take;
            } else {
                // dry
                dropmap[vid].dry_food += (int)take;
                meals_left_total[vindex] -= take;
            }
            remaining_weight -= take * it.weight;
            total_value += take * it.value;
            if (remaining_weight < 1e-9) break;
        }
        if (remaining_weight < 1e-9) break;
    }

    // convert dropmap to trip.drops in route order, only keep villages with something delivered
    for (int vid : route_vids) {
        Drop d = dropmap[vid];
        if (d.dry_food + d.perishable_food + d.other_supplies > 0) {
            trip.drops.push_back(d);
        }
    }
    // set pickup fields to sums
    long long sd=0, sp=0, so=0;
    for (const Drop &d : trip.drops) { sd += d.dry_food; sp += d.perishable_food; so += d.other_supplies; }
    trip.dry_food_pickup = (int)sd;
    trip.perishable_food_pickup = (int)sp;
    trip.other_supplies_pickup = (int)so;
    return {trip, total_value};
}

// Build full plan for a helicopter given its assigned villages and remaining demands
// This function attempts to make profitable trips iteratively until no improvement or distance budget exhausted
static HelicopterPlan build_plan_for_helicopter(const ProblemData &P, const Helicopter &H,
                                                const vector<int> &assigned_vill_indices, // indices 0..V-1
                                                vector<long long> &meals_left_total, vector<long long> &others_left,
                                                double &heli_remaining_D, const unordered_map<int,int> &vidx) {
    HelicopterPlan plan;
    plan.helicopter_id = H.id;

    // repeat: build candidate routes with Clarke-Wright, evaluate packed value for each, pick best net gain trip and execute
    while (true) {
        // build candidate routes using CW (capacity-aware w.r.t. distance)
        auto candidate_routes = build_cw_routes_for_helicopter(P, H, assigned_vill_indices, vidx, meals_left_total, others_left);
        // also include singleton routes for any village with remaining demand
        for (int vi : assigned_vill_indices) {
            if (meals_left_total[vi] <= 0 && others_left[vi] <= 0) continue;
            int vid = P.villages[vi].id;
            bool present = false;
            for (auto &r: candidate_routes) { if (!r.empty() && r.front() == vid) { present = true; break; } }
            if (!present) candidate_routes.push_back({vid});
        }
        if (candidate_routes.empty()) break;

        // evaluate each route: pack greedily and compute trip net gain
        struct Cand { vector<int> route; Trip trip; double net_gain; double dist; double val; };
        vector<Cand> cands;
        for (auto &route : candidate_routes) {
            // compute route distance
            double rdist = route_distance(P, H.home_city_id, route);
            if (rdist > H.distance_capacity + 1e-9) continue;
            if (rdist > heli_remaining_D + 1e-9) continue; // cumulative budget
            // Make copies of demands for packing attempt
            vector<long long> meals_copy = meals_left_total, others_copy = others_left;
            auto packed = pack_route_greedy(P, H, route, meals_copy, others_copy);
            Trip t = packed.first;
            double val = packed.second;
            // if trip delivers nothing skip
            if (t.drops.empty()) continue;
            // compute trip cost
            double cost = H.fixed_cost + H.alpha * rdist;
            double net = val - cost;
            if (net <= 1e-9) continue; // only beneficial trips
            cands.push_back({route, t, net, rdist, val});
        }

        if (cands.empty()) break;
        // pick best net_gain candidate
        auto it = max_element(cands.begin(), cands.end(), [](const Cand &a, const Cand &b){ return a.net_gain < b.net_gain; });
        // execute trip: subtract delivered amounts from global arrays and update heli_remaining_D and append trip
        Trip chosen = it->trip;
        // apply deliveries
        for (const Drop &d : chosen.drops) {
            int idx = vidx.at(d.village_id);
            meals_left_total[idx] -= (d.dry_food + d.perishable_food);
            others_left[idx] -= d.other_supplies;
            if (meals_left_total[idx] < 0) meals_left_total[idx] = 0;
            if (others_left[idx] < 0) others_left[idx] = 0;
        }
        plan.trips.push_back(chosen);
        heli_remaining_D -= it->dist;
        if (heli_remaining_D < 0) heli_remaining_D = 0;
        // continue to find more trips for this helicopter
    }

    // ensure pickup fields match drops
    // (handled by sanitize later optionally)
    return plan;
}

// Build entire Solution given partition (village_to_heli mapping)
// village_to_heli size = V, value = helicopter index in 0..Hcount-1
static Solution decode_partition_to_solution(const ProblemData &P, const vector<int> &village_to_heli) {
    int V = (int)P.villages.size();
    int Hcount = (int)P.helicopters.size();
    auto vidx = buildVillageIdIndex(P);

    // initialize remaining demands per village
    vector<long long> meals_left(V), others_left(V);
    for (int i = 0; i < V; ++i) {
        meals_left[i] = 9LL * P.villages[i].population;
        others_left[i] = 1LL * P.villages[i].population;
    }

    // group villages by helicopter
    vector<vector<int>> assigned(Hcount);
    for (int i = 0; i < V; ++i) {
        int h = village_to_heli[i];
        if (h < 0 || h >= Hcount) h = 0;
        assigned[h].push_back(i);
    }

    Solution sol;
    sol.reserve(Hcount);
    // per-helicopter remaining D budget
    for (int h = 0; h < Hcount; ++h) {
        HelicopterPlan hp;
        hp.helicopter_id = P.helicopters[h].id;
        sol.push_back(move(hp));
    }

    // For each helicopter, build plan
    for (int h = 0; h < Hcount; ++h) {
        Helicopter &H = const_cast<Helicopter&>(P.helicopters[h]);
        double heli_remaining_D = P.d_max;
        HelicopterPlan plan = build_plan_for_helicopter(P, H, assigned[h], meals_left, others_left, heli_remaining_D, vidx);
        sol[h] = move(plan);
    }

    sanitize_solution(sol);
    return sol;
}

// ------------------ Partition-level Simulated Annealing ------------------

Solution solve_partition_sa(const ProblemData &P) {
    int V = (int)P.villages.size();
    int Hcount = (int)P.helicopters.size();
    auto vidx = buildVillageIdIndex(P);
    random_device rd; mt19937 rng(rd());

    // initial partition: assign each village to closest helicopter (by home city)
    vector<int> village_to_heli(V, 0);
    for (int i = 0; i < V; ++i) {
        double bestd = 1e300; int besth = 0;
        for (int h = 0; h < Hcount; ++h) {
            double d = distance(P.villages[i].coords, P.cities[P.helicopters[h].home_city_id - 1]);
            if (d < bestd) { bestd = d; besth = h; }
        }
        village_to_heli[i] = besth;
    }

    // decode initial solution
    Solution best_sol = decode_partition_to_solution(P, village_to_heli);
    Score best_score = evaluate_solution(best_sol, P, vidx);
    Solution cur_sol = best_sol;
    vector<int> cur_part = village_to_heli;
    Score cur_score = best_score;

    // SA parameters
    double T0 = max(1e-3, fabs(cur_score.value)) + 100.0;
    double T = T0;
    double Tmin = 1e-4;
    double cooling = 0.995;

    // time budget
    double time_limit_seconds = 5.0;
    if (P.time_limit_minutes > 0) time_limit_seconds = max(1.0, P.time_limit_minutes * 60.0 - 0.2);
    auto start = Clock::now();

    int iter = 0;
    int stagn = 0;
    const int stagn_restart = 500; // iterations before random restart
    const int max_no_improve_restarts = 5;
    int restarts = 0;

    while (true) {
        ++iter;
        auto now = Clock::now();
        double elapsed = chrono::duration_cast<chrono::duration<double>>(now - start).count();
        if (elapsed >= time_limit_seconds) break;
        if (T < Tmin) break;

        // propose a neighbor: either move a village to different heli or swap two villages
        vector<int> new_part = cur_part;
        double oproll = uniform_real_distribution<double>(0.0,1.0)(rng);
        if (oproll < 0.6) {
            // move single village
            int v = uniform_int_distribution<int>(0, V-1)(rng);
            int curh = new_part[v];
            int newh = uniform_int_distribution<int>(0, Hcount-2)(rng);
            if (newh >= curh) ++newh;
            new_part[v] = newh;
        } else {
            // swap two villages between helis
            int a = uniform_int_distribution<int>(0, V-1)(rng);
            int b = uniform_int_distribution<int>(0, V-2)(rng);
            if (b >= a) ++b;
            swap(new_part[a], new_part[b]);
        }

        // decode neighbor and compute score
        Solution cand_sol = decode_partition_to_solution(P, new_part);
        Score cand_score = evaluate_solution(cand_sol, P, vidx);
        double delta = cand_score.value - cur_score.value;

        bool accept = false;
        if (delta >= 0) accept = true;
        else {
            double prob = exp(delta / max(1e-8, T));
            double r = uniform_real_distribution<double>(0.0, 1.0)(rng);
            if (r < prob) accept = true;
        }

        if (accept) {
            cur_part = move(new_part);
            cur_sol = move(cand_sol);
            cur_score = cand_score;
            stagn = 0;
            if (cur_score.value > best_score.value) {
                best_score = cur_score;
                best_sol = cur_sol;
                restarts = 0;
            }
        } else {
            ++stagn;
        }
        // cooling
        if (iter % 10 == 0) T *= cooling;

        if (stagn > stagn_restart) {
            // random restart: reassign a few villages randomly to escape
            if (++restarts > max_no_improve_restarts) break;
            for (int k = 0; k < max(1, V/10); ++k) {
                int v = uniform_int_distribution<int>(0, V-1)(rng);
                int h = uniform_int_distribution<int>(0, Hcount-1)(rng);
                cur_part[v] = h;
            }
            cur_sol = decode_partition_to_solution(P, cur_part);
            cur_score = evaluate_solution(cur_sol, P, vidx);
            T = T0; // reheat
            stagn = 0;
        }
    }

    sanitize_solution(best_sol);
    return best_sol;
}

// ------------------ Public solve() ------------------

Solution solve(const ProblemData &problem) {
    return solve_partition_sa(problem);
}
