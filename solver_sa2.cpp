#include "structures.h"

#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <random>
#include <chrono>
#include <unordered_map>
#include <cmath>

using namespace std;
using Clock = std::chrono::high_resolution_clock;

Solution generate_random_solution(const ProblemData &P, const unordered_map<int, int> &vidx,
                                  vector<long long> &meals_left, vector<long long> &others_left);

static unordered_map<int, int> buildVillageIdIndex(const ProblemData &P)
{
    unordered_map<int, int> m;
    for (int i = 0; i < (int)P.villages.size(); ++i)
        m[P.villages[i].id] = i;
    return m;
}

static double euclid_dist_points(const Point &a, const Point &b)
{
    return distance(a, b);
}

static double trip_route_distance(const ProblemData &P, const Helicopter &H, const Trip &t, const unordered_map<int, int> &vidx)
{
    if (t.drops.empty())
        return 0.0;
    const Point &home = P.cities[H.home_city_id - 1];
    double d = 0.0;
    Point cur = home;
    for (const Drop &drop : t.drops)
    {
        auto it = vidx.find(drop.village_id);
        if (it == vidx.end())
            continue;
        const Point &vp = P.villages[it->second].coords;
        d += euclid_dist_points(cur, vp);
        cur = vp;
    }
    d += euclid_dist_points(cur, home);
    return d;
}

static double trip_total_weight(const ProblemData &P, const Trip &t)
{
    long long sum_d = 0, sum_p = 0, sum_o = 0;
    for (const Drop &drop : t.drops)
    {
        sum_d += drop.dry_food;
        sum_p += drop.perishable_food;
        sum_o += drop.other_supplies;
    }
    return sum_d * P.packages[0].weight + sum_p * P.packages[1].weight + sum_o * P.packages[2].weight;
}

static double trip_total_value(const ProblemData &P, const Trip &t, const unordered_map<int, int> &vidx, const vector<long long> &meals_left, const vector<long long> &others_left)
{
    double value = 0.0;
    for (const Drop &drop : t.drops)
    {
        int village_idx = vidx.at(drop.village_id);
        long long food_needed = meals_left[village_idx];
        long long other_needed = others_left[village_idx];

        long long p_delivered = min((long long)drop.perishable_food, food_needed);
        long long d_delivered = min((long long)drop.dry_food, food_needed - p_delivered);
        long long o_delivered = min((long long)drop.other_supplies, other_needed);

        value += p_delivered * P.packages[1].value;
        value += d_delivered * P.packages[0].value;
        value += o_delivered * P.packages[2].value;
    }
    return value;
}

static double evaluate_solution_value(const Solution &sol, const ProblemData &P, const unordered_map<int, int> &vidx)
{
    double value = 0.0;
    double costs = 0.0;

    // Calculate total potential value and costs
    for (const auto &hp : sol)
    {
        const Helicopter &H = P.helicopters[hp.helicopter_id - 1];
        for (const auto &t : hp.trips)
        {
            costs += (H.fixed_cost + H.alpha * trip_route_distance(P, H, t, vidx));
        }
    }

    // Calculate value gained considering demand caps
    int V = P.villages.size();
    vector<long long> meals_left(V), others_left(V);
    for (int i = 0; i < V; ++i)
    {
        meals_left[i] = 9LL * P.villages[i].population;
        others_left[i] = 1LL * P.villages[i].population;
    }

    for (const auto &hp : sol)
    {
        for (const auto &t : hp.trips)
        {
            value += trip_total_value(P, t, vidx, meals_left, others_left);
            // Update demands after calculating value for this trip
            for (const auto &drop : t.drops)
            {
                int village_idx = vidx.at(drop.village_id);
                long long food_delivered = drop.perishable_food + drop.dry_food;
                meals_left[village_idx] -= min(food_delivered, meals_left[village_idx]);
                others_left[village_idx] -= min((long long)drop.other_supplies, others_left[village_idx]);
            }
        }
    }

    return value - costs;
}

static vector<double> compute_helicopter_distances(const Solution &sol, const ProblemData &P, const unordered_map<int, int> &vidx)
{
    vector<double> dist(P.helicopters.size(), 0.0);
    for (const auto &hp : sol)
    {
        const Helicopter &H = P.helicopters[hp.helicopter_id - 1];
        for (const auto &t : hp.trips)
        {
            dist[hp.helicopter_id - 1] += trip_route_distance(P, H, t, vidx);
        }
    }
    return dist;
}

static void sanitize_solution(Solution &sol)
{
    for (auto &hp : sol)
    {
        for (auto &t : hp.trips)
        {
            long long sum_d = 0, sum_p = 0, sum_o = 0;
            for (const auto &d : t.drops)
            {
                sum_d += max(0, d.dry_food);
                sum_p += max(0, d.perishable_food);
                sum_o += max(0, d.other_supplies);
            }
            t.dry_food_pickup = (int)sum_d;
            t.perishable_food_pickup = (int)sum_p;
            t.other_supplies_pickup = (int)sum_o;
        }
    }
}

// --- INCREMENTAL DEMAND TRACKING ---
static void subtract_demands_from_trip(const Trip &t, vector<long long> &meals, vector<long long> &others, const unordered_map<int, int> &vidx)
{
    for (const auto &drop : t.drops)
    {
        int village_idx = vidx.at(drop.village_id);
        meals[village_idx] -= (drop.dry_food + drop.perishable_food);
        others[village_idx] -= drop.other_supplies;
    }
}

static void add_demands_to_trip(const Trip &t, vector<long long> &meals, vector<long long> &others, const unordered_map<int, int> &vidx)
{
    for (const auto &drop : t.drops)
    {
        int village_idx = vidx.at(drop.village_id);
        meals[village_idx] += (drop.dry_food + drop.perishable_food);
        others[village_idx] += drop.other_supplies;
    }
}

// --- INITIAL SOLUTION ---
static Solution greedy_initial_solution(const ProblemData &P, const unordered_map<int, int> &vidx, vector<long long> &meals_left, vector<long long> &others_left)
{
    int V = P.villages.size();
    meals_left.assign(V, 0);
    others_left.assign(V, 0);
    for (int i = 0; i < V; ++i)
    {
        meals_left[i] = 9LL * P.villages[i].population;
        others_left[i] = 1LL * P.villages[i].population;
    }

    Solution sol;
    for (const auto &H : P.helicopters)
        sol.push_back({H.id, {}});

    while (true)
    {
        struct Candidate
        {
            int heli_idx;
            int village_idx;
            Trip trip;
            double gain;
            double dist;
        };
        vector<Candidate> candidates;

        for (int h_idx = 0; h_idx < P.helicopters.size(); ++h_idx)
        {
            const auto &H = P.helicopters[h_idx];
            double heli_dist_used = 0; // for initial build, can assume 0
            for (const auto &t : sol[h_idx].trips)
                heli_dist_used += trip_route_distance(P, H, t, vidx);

            for (int vi = 0; vi < V; ++vi)
            {
                if (meals_left[vi] <= 0 && others_left[vi] <= 0)
                    continue;

                // Create a trip packed for this village
                Trip t;
                double cap = H.weight_capacity;
                long long p = min((long long)floor(cap / P.packages[1].weight), meals_left[vi]);
                cap -= p * P.packages[1].weight;
                long long d = min((long long)floor(cap / P.packages[0].weight), meals_left[vi] - p);
                cap -= d * P.packages[0].weight;
                long long o = min((long long)floor(cap / P.packages[2].weight), others_left[vi]);

                if (p + d + o == 0)
                    continue;
                t.drops.push_back({P.villages[vi].id, (int)d, (int)p, (int)o});
                Solution tmp_sol = {{H.id, {t}}};
                sanitize_solution(tmp_sol); // sanitize single trip
                t = tmp_sol[0].trips[0];

                double dist = trip_route_distance(P, H, t, vidx);
                if (dist > H.distance_capacity || heli_dist_used + dist > P.d_max)
                    continue;

                double value = trip_total_value(P, t, vidx, meals_left, others_left);
                double cost = H.fixed_cost + H.alpha * dist;
                double gain = value - cost;

                if (gain > 1e-9)
                    candidates.push_back({h_idx, vi, t, gain, dist});
            }
        }
        if (candidates.empty())
            break;

        auto best_cand = max_element(candidates.begin(), candidates.end(), [](const auto &a, const auto &b)
                                     { return a.gain < b.gain; });
        sol[best_cand->heli_idx].trips.push_back(best_cand->trip);
        subtract_demands_from_trip(best_cand->trip, meals_left, others_left, vidx);
    }
    sanitize_solution(sol);
    return sol;
}

// a. (New) Optimize a multi-drop trip using 2-opt
static bool optimize_route_2opt(Solution &sol, const ProblemData &P, vector<double> &heli_used, const unordered_map<int, int> &vidx, mt19937 &rng)
{
    int h_idx = uniform_int_distribution<int>(0, sol.size() - 1)(rng);
    if (sol[h_idx].trips.empty())
        return false;
    int t_idx = uniform_int_distribution<int>(0, sol[h_idx].trips.size() - 1)(rng);

    Trip &trip = sol[h_idx].trips[t_idx];
    if (trip.drops.size() <= 2)
        return false;

    const Helicopter &H = P.helicopters[h_idx];
    double initial_dist = trip_route_distance(P, H, trip, vidx);
    bool improvement_found = true;
    int max_iters = 1000; // safety break

    while (improvement_found && max_iters-- > 0)
    {
        improvement_found = false;
        for (size_t i = 0; i < trip.drops.size() - 1; ++i)
        {
            for (size_t j = i + 2; j < trip.drops.size(); ++j)
            {
                Trip new_trip = trip;
                reverse(new_trip.drops.begin() + i + 1, new_trip.drops.begin() + j + 1);

                double new_dist = trip_route_distance(P, H, new_trip, vidx);
                if (new_dist < trip_route_distance(P, H, trip, vidx))
                {
                    trip = new_trip;
                    improvement_found = true;
                    goto next_iter; // break out of both loops
                }
            }
        }
    next_iter:;
    }

    double final_dist = trip_route_distance(P, H, trip, vidx);
    if (final_dist < initial_dist)
    {
        heli_used[h_idx] -= (initial_dist - final_dist);
        return true;
    }
    return false;
}

// b. (New) Move a single village drop from one trip to another
static bool reallocate_village(Solution &sol, const ProblemData &P, vector<double> &heli_used, const unordered_map<int, int> &vidx, mt19937 &rng)
{
    // Select a non-empty source trip
    vector<tuple<int, int, int>> sources; // h_idx, t_idx, drop_idx
    for (int i = 0; i < sol.size(); ++i)
        for (int j = 0; j < sol[i].trips.size(); ++j)
            for (int k = 0; k < sol[i].trips[j].drops.size(); ++k)
                sources.emplace_back(i, j, k);

    if (sources.empty())
        return false;

    const auto &src = sources[uniform_int_distribution<int>(0, sources.size() - 1)(rng)];
    int h1_idx = get<0>(src);
    int t1_idx = get<1>(src);
    int drop_idx = get<2>(src);

    // Select destination (can be a new trip)
    int h2_idx = uniform_int_distribution<int>(0, sol.size() - 1)(rng);
    int t2_idx = -1; // -1 means create a new trip
    if (!sol[h2_idx].trips.empty())
    {
        t2_idx = uniform_int_distribution<int>(-1, sol[h2_idx].trips.size() - 1)(rng);
    }

    // If moving within the same helicopter to an existing trip, ensure trips are different
    if (h1_idx == h2_idx && t1_idx == t2_idx && t2_idx != -1)
        return false;

    Drop drop_to_move = sol[h1_idx].trips[t1_idx].drops[drop_idx];

    // Create candidate trips based on the move
    Trip cand_t1 = sol[h1_idx].trips[t1_idx];
    cand_t1.drops.erase(cand_t1.drops.begin() + drop_idx);

    Trip cand_t2;
    if (t2_idx != -1)
        cand_t2 = sol[h2_idx].trips[t2_idx];
    cand_t2.drops.push_back(drop_to_move);

    // Feasibility checks
    const Helicopter &H1 = P.helicopters[h1_idx];
    const Helicopter &H2 = P.helicopters[h2_idx];

    double old_t1_dist = trip_route_distance(P, H1, sol[h1_idx].trips[t1_idx], vidx);
    double new_t1_dist = trip_route_distance(P, H1, cand_t1, vidx);

    double old_t2_dist = (t2_idx == -1) ? 0.0 : trip_route_distance(P, H2, sol[h2_idx].trips[t2_idx], vidx);
    double new_t2_dist = trip_route_distance(P, H2, cand_t2, vidx);

    if (new_t1_dist > H1.distance_capacity + 1e-9 || trip_total_weight(P, cand_t1) > H1.weight_capacity + 1e-9)
        return false;
    if (new_t2_dist > H2.distance_capacity + 1e-9 || trip_total_weight(P, cand_t2) > H2.weight_capacity + 1e-9)
        return false;

    // Check total helicopter distance budgets
    if (h1_idx == h2_idx)
    {
        if (heli_used[h1_idx] - old_t1_dist - old_t2_dist + new_t1_dist + new_t2_dist > P.d_max + 1e-9)
            return false;
    }
    else
    {
        if (heli_used[h1_idx] - old_t1_dist + new_t1_dist > P.d_max + 1e-9)
            return false;
        if (heli_used[h2_idx] - old_t2_dist + new_t2_dist > P.d_max + 1e-9)
            return false;
    }

    // --- All checks passed, apply the move safely ---

    // Update helicopter distances first
    heli_used[h1_idx] += (new_t1_dist - old_t1_dist);
    heli_used[h2_idx] += (new_t2_dist - old_t2_dist);

    // **FIXED LOGIC**: Apply changes carefully to avoid index invalidation.
    if (h1_idx == h2_idx && t2_idx != -1)
    {
        // If modifying the same helicopter, be careful with indices
        sol[h1_idx].trips[t1_idx] = cand_t1;
        sol[h2_idx].trips[t2_idx] = cand_t2;
        // The erase must be last. We find the now-empty trip and remove it.
        // This is safe because we are done using indices for this helicopter.
        sol[h1_idx].trips.erase(
            remove_if(sol[h1_idx].trips.begin(), sol[h1_idx].trips.end(), [](const Trip &t)
                      { return t.drops.empty(); }),
            sol[h1_idx].trips.end());
    }
    else
    {
        // If helicopters are different or making a new trip, it's safe.
        sol[h1_idx].trips[t1_idx] = cand_t1;
        if (sol[h1_idx].trips[t1_idx].drops.empty())
        {
            sol[h1_idx].trips.erase(sol[h1_idx].trips.begin() + t1_idx);
        }

        if (t2_idx == -1)
        {
            sol[h2_idx].trips.push_back(cand_t2);
        }
        else
        {
            sol[h2_idx].trips[t2_idx] = cand_t2;
        }
    }

    sanitize_solution(sol);
    return true;
}

// c. (Improved) Smarter add trip operator
static bool add_greedy_trip(Solution &sol, const ProblemData &P, vector<double> &heli_used, vector<long long> &meals, vector<long long> &others, const unordered_map<int, int> &vidx)
{
    struct Candidate
    {
        int heli_idx;
        Trip trip;
        double gain;
        double dist;
    };
    vector<Candidate> candidates;

    for (int h_idx = 0; h_idx < sol.size(); ++h_idx)
    {
        const Helicopter &H = P.helicopters[h_idx];
        for (int vi = 0; vi < P.villages.size(); ++vi)
        {
            if (meals[vi] <= 0 && others[vi] <= 0)
                continue;

            // Create a trip packed for this village's remaining demand
            Trip t;
            double cap = H.weight_capacity;
            long long p = min((long long)floor(cap / P.packages[1].weight), meals[vi]);
            cap -= p * P.packages[1].weight;
            long long d = min((long long)floor(cap / P.packages[0].weight), meals[vi] - p);
            cap -= d * P.packages[0].weight;
            long long o = min((long long)floor(cap / P.packages[2].weight), others[vi]);

            if (p + d + o == 0)
                continue;
            t.drops.push_back({P.villages[vi].id, (int)d, (int)p, (int)o});

            double dist = trip_route_distance(P, H, t, vidx);
            if (dist > H.distance_capacity + 1e-9 || heli_used[h_idx] + dist > P.d_max + 1e-9)
                continue;

            double value = trip_total_value(P, t, vidx, meals, others);
            double cost = H.fixed_cost + H.alpha * dist;
            double gain = value - cost;

            if (gain > 1e-9)
                candidates.push_back({h_idx, t, gain, dist});
        }
    }
    if (candidates.empty())
        return false;

    auto best = max_element(candidates.begin(), candidates.end(), [](const auto &a, const auto &b)
                            { return a.gain < b.gain; });
    sol[best->heli_idx].trips.push_back(best->trip);
    heli_used[best->heli_idx] += best->dist;
    subtract_demands_from_trip(best->trip, meals, others, vidx);
    sanitize_solution(sol);
    return true;
}

// d. (Existing) Remove a random trip
static bool remove_random_trip(Solution &sol, const ProblemData &P, vector<double> &heli_used, vector<long long> &meals, vector<long long> &others, const unordered_map<int, int> &vidx, mt19937 &rng)
{
    vector<pair<int, int>> sources;
    for (int i = 0; i < sol.size(); ++i)
        for (int j = 0; j < sol[i].trips.size(); ++j)
            sources.push_back({i, j});
    if (sources.empty())
        return false;

    pair<int, int> src = sources[uniform_int_distribution<int>(0, sources.size() - 1)(rng)];
    int h_idx = src.first;
    int t_idx = src.second;

    const Trip &trip_to_remove = sol[h_idx].trips[t_idx];
    heli_used[h_idx] -= trip_route_distance(P, P.helicopters[h_idx], trip_to_remove, vidx);
    add_demands_to_trip(trip_to_remove, meals, others, vidx);
    sol[h_idx].trips.erase(sol[h_idx].trips.begin() + t_idx);
    return true;
}

Solution solve(const ProblemData &P)
{
    auto vidx = buildVillageIdIndex(P);

    vector<long long> meals_left, others_left;
    Solution best = greedy_initial_solution(P, vidx, meals_left, others_left);
    Solution cur = best;

    double best_score = evaluate_solution_value(best, P, vidx);
    double cur_score = best_score;

    vector<double> cur_heli_used = compute_helicopter_distances(cur, P, vidx);
    vector<double> best_heli_used = cur_heli_used;
    vector<long long> best_meals_left = meals_left;
    vector<long long> best_others_left = others_left;

    random_device rd;
    mt19937 rng(rd());

    double T0 = max(100.0, abs(cur_score) * 0.1);
    double T = T0;
    double T_min = 1e-3;
    double alpha = 0.999;

    double time_limit_seconds = max(1.0, P.time_limit_minutes * 60.0 - 5);
    auto start_time = Clock::now();
    int iter = 0;
    int stagnation = 0;

    while (true)
    {
        ++iter;
        auto now = Clock::now();
        if (chrono::duration_cast<chrono::duration<double>>(now - start_time).count() >= time_limit_seconds)
            break;
        if (T < T_min)
            break;

        Solution cand_sol = cur;
        vector<double> cand_heli_used = cur_heli_used;
        vector<long long> cand_meals = meals_left;
        vector<long long> cand_others = others_left;

        int op = uniform_int_distribution<int>(0, 3)(rng);
        bool moved = false;
        switch (op)
        {
        case 0:
            moved = add_greedy_trip(cand_sol, P, cand_heli_used, cand_meals, cand_others, vidx);
            break;
        case 1:
            moved = remove_random_trip(cand_sol, P, cand_heli_used, cand_meals, cand_others, vidx, rng);
            break;
        case 2:
            moved = optimize_route_2opt(cand_sol, P, cand_heli_used, vidx, rng);
            break;
        case 3:
            moved = reallocate_village(cand_sol, P, cand_heli_used, vidx, rng);
            break;
        }

        if (!moved)
        {
            T *= alpha; // Cool down if no move was made
            continue;
        }

        double cand_score = evaluate_solution_value(cand_sol, P, vidx);
        double delta = cand_score - cur_score;

        if (delta > 0 || (T > 0 && uniform_real_distribution<double>(0.0, 1.0)(rng) < exp(delta / T)))
        {
            cur = cand_sol;
            cur_score = cand_score;
            cur_heli_used = cand_heli_used;
            meals_left = cand_meals;
            others_left = cand_others;

            if (cur_score > best_score)
            {
                best_score = cur_score;
                best = cur;
                best_heli_used = cur_heli_used;
                best_meals_left = meals_left;
                best_others_left = others_left;
                stagnation = 0;
            }
            else
            {
                stagnation++;
            }
        }
        else
        {
            stagnation++;
        }

        if (iter % 100 == 0)
            T *= alpha;

        // If stuck, jump back to the best solution to explore from there
        if (stagnation > 500)
        {
            // Replace your current stagnation handler with this:
            if (stagnation > 500)
            {
                // cout << "Performing random restart at iteration " << iter << endl;

                // Generate completely new random solution
                cur = generate_random_solution(P, vidx, meals_left, others_left);
                cur_score = evaluate_solution_value(cur, P, vidx);
                cur_heli_used = compute_helicopter_distances(cur, P, vidx);

                // Reset temperature to initial value for proper exploration
                T = T0;
                stagnation = 0;

                // Keep the best solution found so far
                if (cur_score > best_score)
                {
                    best = cur;
                    best_score = cur_score;
                    best_heli_used = cur_heli_used;
                    best_meals_left = meals_left;
                    best_others_left = others_left;
                }
            }
        }
    }

    sanitize_solution(best);
    cerr << "SA finished: iterations=" << iter << " best_obj=" << best_score << "\n";
    return best;
}

// Add this function to generate a random solution (not greedy)
Solution generate_random_solution(const ProblemData &P, const unordered_map<int, int> &vidx,
                                  vector<long long> &meals_left, vector<long long> &others_left)
{
    Solution sol;
    for (const auto &H : P.helicopters)
        sol.push_back({H.id, {}});

    // Reset demand tracking
    int V = P.villages.size();
    meals_left.assign(V, 0);
    others_left.assign(V, 0);
    for (int i = 0; i < V; ++i)
    {
        meals_left[i] = 9LL * P.villages[i].population;
        others_left[i] = 1LL * P.villages[i].population;
    }

    // Create a shuffled list of villages
    vector<int> village_indices(V);
    iota(village_indices.begin(), village_indices.end(), 0);
    random_device rd;
    mt19937 g(rd());
    shuffle(village_indices.begin(), village_indices.end(), g);

    // Randomly assign villages to helicopters and create trips
    for (int vi : village_indices)
    {
        // Skip with some probability to avoid overpacking
        if (uniform_real_distribution<double>(0, 1)(g) < 0.5)
            continue;

        // Pick random helicopter
        int h_idx = uniform_int_distribution<int>(0, P.helicopters.size() - 1)(g);
        const auto &H = P.helicopters[h_idx];

        // Create a random trip for this village
        Trip t;
        double cap = H.weight_capacity * uniform_real_distribution<double>(0.5, 0.9)(g); // Use 50-90% of capacity

        long long p = uniform_int_distribution<long long>(0, min((long long)floor(cap / P.packages[1].weight), meals_left[vi]))(g);
        cap -= p * P.packages[1].weight;

        long long d = uniform_int_distribution<long long>(0, min((long long)floor(cap / P.packages[0].weight), meals_left[vi] - p))(g);
        cap -= d * P.packages[0].weight;

        long long o = uniform_int_distribution<long long>(0, min((long long)floor(cap / P.packages[2].weight), others_left[vi]))(g);

        if (p + d + o > 0)
        {
            t.drops.push_back({P.villages[vi].id, (int)d, (int)p, (int)o});
            sol[h_idx].trips.push_back(t);
            meals_left[vi] -= (p + d);
            others_left[vi] -= o;
        }
    }

    sanitize_solution(sol);
    return sol;
}