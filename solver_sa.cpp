// solver_sa.cpp
// Simulated annealing solver for the helicopter routing assignment.
// Matches the provided structures in structures.h
// --- FIXED: ensure trip pickup counts are never negative (sanitize merged trips) ---

#include "structures.h"

#include <chrono>
#include <random>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <unordered_map>

using namespace std;
using Clock = std::chrono::high_resolution_clock;

// ------------------- Helpers -------------------

static unordered_map<int,int> buildVillageIdIndex(const ProblemData &P){
    unordered_map<int,int> m;
    for(int i=0;i<(int)P.villages.size();++i) m[P.villages[i].id]=i;
    return m;
}

// compute euclidean distance using provided distance(...) helper
static double euclid_dist_points(const Point &a, const Point &b){
    return distance(a,b);
}

// compute route distance for a trip for a given helicopter (home city matters)
static double trip_route_distance(const ProblemData &P, const Helicopter &H, const Trip &t, const unordered_map<int,int> &vidx){
    if(t.drops.empty()) return 0.0;
    const Point &home = P.cities[H.home_city_id - 1];
    double d = 0.0;
    Point cur = home;
    for(const Drop &drop : t.drops){
        auto it = vidx.find(drop.village_id);
        if(it==vidx.end()) continue;
        const Point &vp = P.villages[it->second].coords;
        d += euclid_dist_points(cur, vp);
        cur = vp;
    }
    d += euclid_dist_points(cur, home);
    return d;
}

// compute weight (double) of a trip (sums drops if available; otherwise uses trip-level pickup fields)
static double trip_total_weight(const ProblemData &P, const Trip &t){
    double w_d = P.packages[0].weight;
    double w_p = P.packages[1].weight;
    double w_o = P.packages[2].weight;

    long long sum_d = 0, sum_p = 0, sum_o = 0;
    for(const Drop &drop : t.drops){ sum_d += drop.dry_food; sum_p += drop.perishable_food; sum_o += drop.other_supplies; }
    if(sum_d + sum_p + sum_o > 0){
        return sum_d * w_d + sum_p * w_p + sum_o * w_o;
    }
    // fallback to trip-level fields if drops empty or zero
    long long td = max(0, t.dry_food_pickup);
    long long tp = max(0, t.perishable_food_pickup);
    long long to = max(0, t.other_supplies_pickup);
    return td * w_d + tp * w_p + to * w_o;
}

static double trip_total_value(const ProblemData &P, const Trip &t){
    double v_d = P.packages[0].value;
    double v_p = P.packages[1].value;
    double v_o = P.packages[2].value;
    long long sum_d = 0, sum_p = 0, sum_o = 0;
    for(const Drop &drop : t.drops){ sum_d += drop.dry_food; sum_p += drop.perishable_food; sum_o += drop.other_supplies; }
    if(sum_d + sum_p + sum_o > 0)
        return sum_d*v_d + sum_p*v_p + sum_o*v_o;
    // fallback to trip-level counts
    return max(0, t.dry_food_pickup) * v_d + max(0, t.perishable_food_pickup) * v_p + max(0, t.other_supplies_pickup) * v_o;
}

// compute objective score and total trip-costs for the whole solution
static Score evaluate_solution(const Solution &sol, const ProblemData &P, const unordered_map<int,int> &vidx){
    double value = 0.0;
    double costs = 0.0;
    for(const HelicopterPlan &hp : sol){
        // helicopter_id assumed 1-based and present in P.helicopters
        const Helicopter &H = P.helicopters[hp.helicopter_id - 1];
        for(const Trip &t : hp.trips){
            value += trip_total_value(P, t);
            double dist = trip_route_distance(P, H, t, vidx);
            costs += (H.fixed_cost + H.alpha * dist);
        }
    }
    double obj = value - costs;
    return Score(obj, costs);
}

// helper: compute per-helicopter cumulative distance used
static vector<double> compute_helicopter_distances(const Solution &sol, const ProblemData &P, const unordered_map<int,int> &vidx){
    vector<double> dist(P.helicopters.size(), 0.0);
    for(const HelicopterPlan &hp : sol){
        const Helicopter &H = P.helicopters[hp.helicopter_id - 1];
        for(const Trip &t : hp.trips){
            dist[hp.helicopter_id - 1] += trip_route_distance(P, H, t, vidx);
        }
    }
    return dist;
}

// Validate a trip for a given helicopter and current remaining per-helicopter cumulative distance budget
static bool trip_feasible_for_heli(const ProblemData &P, const Helicopter &H, const Trip &t, double heli_used_distance, const unordered_map<int,int> &vidx){
    double route = trip_route_distance(P, H, t, vidx);
    if(route > H.distance_capacity + 1e-9) return false; // per-trip cap
    double wt = trip_total_weight(P, t);
    if(wt > H.weight_capacity + 1e-9) return false;
    return (heli_used_distance + route <= P.d_max + 1e-9);
}

// Create a simple single-village trip packing for (heli, village) -- used by initial solution and addTrip
static Trip create_single_village_trip(const ProblemData &P, const Helicopter &H, int village_idx){
    Trip trip;
    trip.dry_food_pickup = 0;
    trip.perishable_food_pickup = 0;
    trip.other_supplies_pickup = 0;
    Drop d;
    d.village_id = P.villages[village_idx].id;
    d.dry_food = 0; d.perishable_food = 0; d.other_supplies = 0;

    double cap_weight = H.weight_capacity;
    double w_d = P.packages[0].weight; double v_d = P.packages[0].value;
    double w_p = P.packages[1].weight; double v_p = P.packages[1].value;
    double w_o = P.packages[2].weight; double v_o = P.packages[2].value;

    // prefer higher value/weight meal type
    double r_d = v_d / (w_d + 1e-12);
    double r_p = v_p / (w_p + 1e-12);
    long long meals_needed = 9LL * P.villages[village_idx].population; // total needed (caller should manage remaining)

    if(r_p > r_d){
        long long max_p = (long long)floor(cap_weight / w_p + 1e-9);
        long long p = min(max_p, meals_needed);
        d.perishable_food = (int)p; cap_weight -= p*w_p;
        if(d.perishable_food < meals_needed){
            long long max_d = (long long)floor(cap_weight / w_d + 1e-9);
            long long dd = min(max_d, meals_needed - d.perishable_food);
            d.dry_food = (int)dd; cap_weight -= dd*w_d;
        }
    } else {
        long long max_d = (long long)floor(cap_weight / w_d + 1e-9);
        long long dd = min(max_d, meals_needed);
        d.dry_food = (int)dd; cap_weight -= dd*w_d;
        if(d.dry_food < meals_needed){
            long long max_p = (long long)floor(cap_weight / w_p + 1e-9);
            long long p = min(max_p, meals_needed - d.dry_food);
            d.perishable_food = (int)p; cap_weight -= p*w_p;
        }
    }
    if(cap_weight >= w_o){
        long long max_o = (long long)floor(cap_weight / w_o + 1e-9);
        long long o = min(max_o, (long long)P.villages[village_idx].population);
        d.other_supplies = (int)o; cap_weight -= o*w_o;
    }

    trip.drops.push_back(d);
    trip.dry_food_pickup = d.dry_food;
    trip.perishable_food_pickup = d.perishable_food;
    trip.other_supplies_pickup = d.other_supplies;
    return trip;
}

// sanitize: ensure trip-level pickup fields are non-negative and consistent with drops
static void sanitize_solution(Solution &sol){
    for(auto &hp : sol){
        for(auto &t : hp.trips){
            if(!t.drops.empty()){
                long long sum_d = 0, sum_p = 0, sum_o = 0;
                for(const Drop &d : t.drops){ sum_d += max(0, d.dry_food); sum_p += max(0, d.perishable_food); sum_o += max(0, d.other_supplies); }
                t.dry_food_pickup = (int)sum_d;
                t.perishable_food_pickup = (int)sum_p;
                t.other_supplies_pickup = (int)sum_o;
            } else {
                // clamp to >=0
                t.dry_food_pickup = max(0, t.dry_food_pickup);
                t.perishable_food_pickup = max(0, t.perishable_food_pickup);
                t.other_supplies_pickup = max(0, t.other_supplies_pickup);
            }
        }
    }
}

// ------------------- Initial greedy solution -------------------
static Solution greedy_initial_solution(const ProblemData &P, const unordered_map<int,int> &vidx){
    int V = (int)P.villages.size();
    vector<long long> meals_left(V), others_left(V);
    for(int i=0;i<V;++i){ meals_left[i] = 9LL * P.villages[i].population; others_left[i] = 1LL * P.villages[i].population; }

    Solution sol;
    sol.reserve(P.helicopters.size());
    // create one HelicopterPlan per helicopter in same order as P.helicopters
    for(const Helicopter &H : P.helicopters){
        HelicopterPlan hp; hp.helicopter_id = H.id;
        double remaining_D = P.d_max;
        while(true){
            struct Cand{int vid; Trip trip; double gain; double dist;};
            vector<Cand> cand;
            for(int vi=0; vi<V; ++vi){
                if(meals_left[vi]<=0 && others_left[vi]<=0) continue;
                Trip t = create_single_village_trip(P, H, vi);
                if(t.drops.empty()) continue;
                // reduce delivered by remaining demand
                Drop &d = t.drops[0];
                long long can_meals = min((long long)(d.dry_food + d.perishable_food), meals_left[vi]);
                if(can_meals < (d.dry_food + d.perishable_food)){
                    long long want_meals = can_meals;
                    long long p = min((long long)d.perishable_food, want_meals);
                    long long dd = max(0LL, want_meals - p);
                    d.perishable_food = (int)p; d.dry_food = (int)dd;
                }
                long long can_oth = min((long long)d.other_supplies, others_left[vi]);
                d.other_supplies = (int)can_oth;
                // recompute trip totals
                double val = trip_total_value(P, t);
                double dist = trip_route_distance(P, H, t, vidx);
                if(dist > H.distance_capacity || dist > remaining_D) continue;
                double cost = H.fixed_cost + H.alpha * dist;
                double gain = val - cost;
                if(gain > 1e-9) cand.push_back({vi, t, gain, dist});
            }
            if(cand.empty()) break;
            auto best = max_element(cand.begin(), cand.end(), [](const Cand&a,const Cand&b){return a.gain<b.gain;});
            hp.trips.push_back(best->trip);
            int vi = best->vid;
            Drop &d = hp.trips.back().drops[0];
            meals_left[vi] -= (d.dry_food + d.perishable_food);
            others_left[vi] -= d.other_supplies;
            remaining_D -= best->dist;
            if(remaining_D <= 0) break;
        }
        sol.push_back(hp);
    }
    sanitize_solution(sol);
    return sol;
}

// ------------------- Neighborhood moves -------------------

// Move a random trip from one helicopter to another (if feasible)
static bool move_trip_between_helicopters(Solution &sol, const ProblemData &P, vector<double> &heli_used, const unordered_map<int,int> &vidx, mt19937 &rng){
    int Hcount = (int)sol.size();
    vector<int> nonempty;
    for(int i=0;i<Hcount;++i) if(!sol[i].trips.empty()) nonempty.push_back(i);
    if(nonempty.empty()) return false;
    uniform_int_distribution<int> pick_heli(0,(int)nonempty.size()-1);
    int from_idx = nonempty[pick_heli(rng)];
    int to_idx = uniform_int_distribution<int>(0,Hcount-1)(rng);
    if(to_idx==from_idx) to_idx = (to_idx+1)%Hcount;
    int tcount = (int)sol[from_idx].trips.size();
    int ti = uniform_int_distribution<int>(0,tcount-1)(rng);
    Trip trip = sol[from_idx].trips[ti];
    const Helicopter &toH = P.helicopters[sol[to_idx].helicopter_id - 1];
    const Helicopter &fromH = P.helicopters[sol[from_idx].helicopter_id - 1];
    double used_to = heli_used[to_idx];
    if(!trip_feasible_for_heli(P, toH, trip, used_to, vidx)) return false;
    sol[to_idx].trips.push_back(trip);
    double dist_to = trip_route_distance(P, toH, trip, vidx);
    double dist_from = trip_route_distance(P, fromH, trip, vidx);
    heli_used[to_idx] += dist_to;
    heli_used[from_idx] -= dist_from;
    sol[from_idx].trips.erase(sol[from_idx].trips.begin() + ti);
    sanitize_solution(sol);
    return true;
}

// Swap two random trips between two helicopters (if feasible)
static bool swap_trips_between_helicopters(Solution &sol, const ProblemData &P, vector<double> &heli_used, const unordered_map<int,int> &vidx, mt19937 &rng){
    int Hcount = (int)sol.size();
    if(Hcount < 2) return false;
    int a = uniform_int_distribution<int>(0,Hcount-1)(rng);
    int b = uniform_int_distribution<int>(0,Hcount-2)(rng);
    if(b >= a) b += 1;
    if(sol[a].trips.empty() || sol[b].trips.empty()) return false;
    int ai = uniform_int_distribution<int>(0,(int)sol[a].trips.size()-1)(rng);
    int bi = uniform_int_distribution<int>(0,(int)sol[b].trips.size()-1)(rng);
    Trip ta = sol[a].trips[ai];
    Trip tb = sol[b].trips[bi];
    const Helicopter &Ha = P.helicopters[sol[a].helicopter_id - 1];
    const Helicopter &Hb = P.helicopters[sol[b].helicopter_id - 1];
    double used_a = heli_used[a];
    double used_b = heli_used[b];
    double dist_ta_in_b = trip_route_distance(P, Hb, ta, vidx);
    double dist_tb_in_a = trip_route_distance(P, Ha, tb, vidx);
    double wt_ta = trip_total_weight(P, ta);
    double wt_tb = trip_total_weight(P, tb);
    if(dist_ta_in_b > Hb.distance_capacity + 1e-9) return false;
    if(dist_tb_in_a > Ha.distance_capacity + 1e-9) return false;
    if(wt_ta > Hb.weight_capacity + 1e-9 || wt_tb > Ha.weight_capacity + 1e-9) return false;
    if(used_b - trip_route_distance(P, Hb, tb, vidx) + dist_ta_in_b > P.d_max + 1e-9) return false;
    if(used_a - trip_route_distance(P, Ha, ta, vidx) + dist_tb_in_a > P.d_max + 1e-9) return false;
    heli_used[a] = used_a - trip_route_distance(P, Ha, ta, vidx) + dist_tb_in_a;
    heli_used[b] = used_b - trip_route_distance(P, Hb, tb, vidx) + dist_ta_in_b;
    sol[a].trips[ai] = tb;
    sol[b].trips[bi] = ta;
    sanitize_solution(sol);
    return true;
}

// Merge two trips in same helicopter if feasible
static bool merge_two_trips_in_heli(Solution &sol, const ProblemData &P, vector<double> &heli_used, const unordered_map<int,int> &vidx, mt19937 &rng){
    int Hcount = (int)sol.size();
    int h = uniform_int_distribution<int>(0,Hcount-1)(rng);
    if(sol[h].trips.size() < 2) return false;
    int n = sol[h].trips.size();
    int i = uniform_int_distribution<int>(0,n-1)(rng);
    int j = uniform_int_distribution<int>(0,n-2)(rng);
    if(j >= i) j += 1;
    Trip A = sol[h].trips[i];
    Trip B = sol[h].trips[j];
    Trip merged;
    // merge drops
    merged.drops = A.drops;
    merged.drops.insert(merged.drops.end(), B.drops.begin(), B.drops.end());
    // compute trip-level pickups as sum of drops (no sentinel -1)
    long long sum_d = 0, sum_p = 0, sum_o = 0;
    for(const Drop &d : merged.drops){ sum_d += max(0, d.dry_food); sum_p += max(0, d.perishable_food); sum_o += max(0, d.other_supplies); }
    merged.dry_food_pickup = (int)sum_d;
    merged.perishable_food_pickup = (int)sum_p;
    merged.other_supplies_pickup = (int)sum_o;

    const Helicopter &H = P.helicopters[sol[h].helicopter_id - 1];
    double route = trip_route_distance(P, H, merged, vidx);
    double wt = trip_total_weight(P, merged);
    double new_total = heli_used[h] - trip_route_distance(P, H, A, vidx) - trip_route_distance(P, H, B, vidx) + route;
    if(route <= H.distance_capacity + 1e-9 && wt <= H.weight_capacity + 1e-9 && new_total <= P.d_max + 1e-9){
        if(i>j){ sol[h].trips.erase(sol[h].trips.begin()+i); sol[h].trips.erase(sol[h].trips.begin()+j); }
        else { sol[h].trips.erase(sol[h].trips.begin()+j); sol[h].trips.erase(sol[h].trips.begin()+i); }
        sol[h].trips.push_back(merged);
        heli_used[h] = new_total;
        sanitize_solution(sol);
        return true;
    }
    return false;
}

// Split a trip with multiple drops into single-drop trips
static bool split_trip_in_heli(Solution &sol, const ProblemData &P, vector<double> &heli_used, const unordered_map<int,int> &vidx, mt19937 &rng){
    int Hcount = (int)sol.size();
    int h = uniform_int_distribution<int>(0,Hcount-1)(rng);
    if(sol[h].trips.empty()) return false;
    int ti = uniform_int_distribution<int>(0,(int)sol[h].trips.size()-1)(rng);
    Trip t = sol[h].trips[ti];
    if(t.drops.size() < 2) return false;
    vector<Trip> news;
    for(const Drop &d : t.drops){
        Trip nt;
        nt.dry_food_pickup = d.dry_food;
        nt.perishable_food_pickup = d.perishable_food;
        nt.other_supplies_pickup = d.other_supplies;
        nt.drops.push_back(d);
        news.push_back(nt);
    }
    const Helicopter &H = P.helicopters[sol[h].helicopter_id - 1];
    double olddist = trip_route_distance(P,H,t,vidx);
    double newsum = 0.0;
    for(const Trip &nt: news){
        double r = trip_route_distance(P,H,nt,vidx);
        if(r > H.distance_capacity + 1e-9) return false;
        if(trip_total_weight(P, nt) > H.weight_capacity + 1e-9) return false;
        newsum += r;
    }
    if(heli_used[h] - olddist + newsum > P.d_max + 1e-9) return false;
    sol[h].trips.erase(sol[h].trips.begin() + ti);
    for(Trip &nt: news) sol[h].trips.push_back(nt);
    heli_used[h] = heli_used[h] - olddist + newsum;
    sanitize_solution(sol);
    return true;
}

// Remove a random trip
static bool remove_random_trip(Solution &sol, const ProblemData &P, vector<double> &heli_used, const unordered_map<int,int> &vidx, mt19937 &rng){
    int Hcount = (int)sol.size();
    vector<int> nonempty;
    for(int i=0;i<Hcount;++i) if(!sol[i].trips.empty()) nonempty.push_back(i);
    if(nonempty.empty()) return false;
    int h = nonempty[uniform_int_distribution<int>(0,(int)nonempty.size()-1)(rng)];
    int ti = uniform_int_distribution<int>(0,(int)sol[h].trips.size()-1)(rng);
    const Helicopter &H = P.helicopters[sol[h].helicopter_id - 1];
    double dist = trip_route_distance(P,H,sol[h].trips[ti], vidx);
    heli_used[h] -= dist;
    sol[h].trips.erase(sol[h].trips.begin()+ti);
    sanitize_solution(sol);
    return true;
}

// Add a greedy new single-village trip based on current demands
static bool add_greedy_trip_from_demands(Solution &sol, const ProblemData &P, vector<double> &heli_used, vector<long long> &meals_left, vector<long long> &others_left, const unordered_map<int,int> &vidx, mt19937 &rng){
    int V = (int)P.villages.size();
    int Hcount = (int)sol.size();
    vector<int> vids;
    for(int i=0;i<V;++i) if(meals_left[i]>0 || others_left[i]>0) vids.push_back(i);
    if(vids.empty()) return false;
    vector<int> heli_idx(Hcount); iota(heli_idx.begin(), heli_idx.end(), 0);
    shuffle(heli_idx.begin(), heli_idx.end(), rng);
    shuffle(vids.begin(), vids.end(), rng);
    for(int h : heli_idx){
        const Helicopter &H = P.helicopters[sol[h].helicopter_id - 1];
        for(int vi : vids){
            Trip t = create_single_village_trip(P, H, vi);
            if(t.drops.empty()) continue;
            Drop &d = t.drops[0];
            long long can_meals = min((long long)(d.dry_food + d.perishable_food), meals_left[vi]);
            if(can_meals < (d.dry_food + d.perishable_food)){
                long long want_meals = can_meals;
                long long p = min((long long)d.perishable_food, want_meals);
                long long dd = max(0LL, want_meals - p);
                d.perishable_food = (int)p; d.dry_food = (int)dd;
            }
            d.other_supplies = (int)min((long long)d.other_supplies, others_left[vi]);
            // update trip-level pickups to match drops
            t.dry_food_pickup = d.dry_food;
            t.perishable_food_pickup = d.perishable_food;
            t.other_supplies_pickup = d.other_supplies;

            double dist = trip_route_distance(P,H,t,vidx);
            if(dist > H.distance_capacity || heli_used[h] + dist > P.d_max) continue;
            double gain = trip_total_value(P,t) - (H.fixed_cost + H.alpha*dist);
            if(gain <= 1e-9) continue;
            sol[h].trips.push_back(t);
            heli_used[h] += dist;
            meals_left[vi] -= (d.dry_food + d.perishable_food);
            others_left[vi] -= d.other_supplies;
            sanitize_solution(sol);
            return true;
        }
    }
    return false;
}

// rebuild demands from current solution
static void rebuild_demands_from_solution(const Solution &sol, const ProblemData &P, vector<long long> &meals_left, vector<long long> &others_left){
    int V = (int)P.villages.size();
    meals_left.assign(V,0);
    others_left.assign(V,0);
    for(int i=0;i<V;++i){ meals_left[i] = 9LL * P.villages[i].population; others_left[i] = 1LL * P.villages[i].population; }
    unordered_map<int,int> vidx;
    for(int i=0;i<V;++i) vidx[P.villages[i].id]=i;
    for(const HelicopterPlan &hp : sol){
        for(const Trip &t : hp.trips){
            for(const Drop &d : t.drops){
                int idx = vidx[d.village_id];
                meals_left[idx] -= (d.dry_food + d.perishable_food);
                others_left[idx] -= d.other_supplies;
            }
        }
    }
    for(int i=0;i<V;++i){ meals_left[i] = max(0LL, meals_left[i]); others_left[i] = max(0LL, others_left[i]); }
}

// ------------------- Simulated Annealing main -------------------

static Solution simulated_annealing(const ProblemData &P){
    auto vidx = buildVillageIdIndex(P);
    // initial solution: greedy
    Solution best = greedy_initial_solution(P, vidx);
    Solution cur = best;
    Score best_score = evaluate_solution(best, P, vidx);
    Score cur_score = best_score;

    vector<double> cur_heli_used = compute_helicopter_distances(cur, P, vidx);
    vector<double> best_heli_used = cur_heli_used;

    vector<long long> meals_left, others_left;
    rebuild_demands_from_solution(cur, P, meals_left, others_left);

    random_device rd; mt19937 rng(rd());

    // SA parameters
    double T0 = max(1e-3, fabs(cur_score.value)) + 100.0;
    double Tmin = 1e-4;
    double alpha = 0.995;
    double T = T0;

    double time_limit_seconds = 5.0;
    if(P.time_limit_minutes > 0) time_limit_seconds = max(1.0, P.time_limit_minutes * 60.0 - 0.2);

    auto start = Clock::now();
    int iter = 0;
    int stagnation = 0;

    while(true){
        ++iter;
        auto now = Clock::now();
        double elapsed = chrono::duration_cast<chrono::duration<double>>(now - start).count();
        if(elapsed >= time_limit_seconds) break;
        if(T < Tmin) break;

        int op = uniform_int_distribution<int>(0,5)(rng);
        Solution candidate = cur;
        vector<double> cand_heli_used = cur_heli_used;
        vector<long long> cand_meals = meals_left, cand_others = others_left;
        bool moved = false;
        switch(op){
            case 0: moved = move_trip_between_helicopters(candidate, P, cand_heli_used, vidx, rng); break;
            case 1: moved = swap_trips_between_helicopters(candidate, P, cand_heli_used, vidx, rng); break;
            case 2: moved = merge_two_trips_in_heli(candidate, P, cand_heli_used, vidx, rng); break;
            case 3: moved = split_trip_in_heli(candidate, P, cand_heli_used, vidx, rng); break;
            case 4: moved = remove_random_trip(candidate, P, cand_heli_used, vidx, rng); break;
            case 5:
                rebuild_demands_from_solution(candidate, P, cand_meals, cand_others);
                moved = add_greedy_trip_from_demands(candidate, P, cand_heli_used, cand_meals, cand_others, vidx, rng);
                break;
        }
        if(!moved){
            T *= alpha;
            continue;
        }

        Score cand_score = evaluate_solution(candidate, P, vidx);
        double delta = cand_score.value - cur_score.value;
        bool accept = false;
        if(delta >= 0) accept = true;
        else {
            double prob = exp(delta / T);
            double roll = uniform_real_distribution<double>(0.0, 1.0)(rng);
            if(roll < prob) accept = true;
        }

        if(accept){
            cur = move(candidate);
            cur_score = cand_score;
            cur_heli_used = move(cand_heli_used);
            meals_left = move(cand_meals); others_left = move(cand_others);
            stagnation = 0;
            if(cur_score.value > best_score.value){
                best = cur; best_score = cur_score; best_heli_used = cur_heli_used;
            }
        } else {
            ++stagnation;
        }

        if(iter % 50 == 0) T *= alpha;
        if(stagnation > 500){
            remove_random_trip(cur, P, cur_heli_used, vidx, rng);
            rebuild_demands_from_solution(cur, P, meals_left, others_left);
            cur_score = evaluate_solution(cur, P, vidx);
            stagnation = 0;
        }
    }

    // ensure no negative pickup values before returning
    sanitize_solution(best);

    cerr << "SA finished: iterations=" << iter << " best_obj=" << best_score.value << "\n";
    return best;
}

// ------------------- Public solve() -------------------

Solution solve(const ProblemData &problem){
    return simulated_annealing(problem);
}
