#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <random>
#include <chrono>
#include "structures.h" // Assuming this contains the structures you provided

using namespace std;

// --- FORWARD DECLARATIONS of Helper Functions ---

// Calculates the total value and generates the final Solution object from a village assignment
double evaluate_state(const vector<vector<int>>& village_assignments, const ProblemData& problem, Solution& out_solution);

// The core inner optimization: creates an efficient plan for one helicopter and its assigned villages
HelicopterPlan optimize_helicopter_plan(const Helicopter& helicopter, const vector<int>& assigned_village_ids, const ProblemData& problem);

// --- MAIN SOLVE FUNCTION ---

/**
 * @brief Main function to solve the helicopter routing problem.
 * * Implements a local search algorithm (Hill Climbing with Random Restarts) based on your design.
 * - State: An assignment of villages to helicopters.
 * - Neighbor: A state created by moving one village from one helicopter to another.
 * - Inner Loop: For any given state, an efficient plan for each helicopter is calculated using a greedy routing and packing heuristic.
 */
Solution solve(const ProblemData& problem) {
    // Start the timer to respect the time limit
    auto start_time = chrono::high_resolution_clock::now();
    long long time_limit_seconds = (long long)(problem.time_limit_minutes * 60);

    // Setup for random number generation, essential for local search
    random_device rd;
    mt19937 gen(rd());

    // --- INITIAL STATE GENERATION ---
    // Create an initial random assignment of villages to helicopters.
    vector<vector<int>> current_assignments(problem.helicopters.size());
    for (int i = 0; i < problem.villages.size(); ++i) {
        uniform_int_distribution<> distrib(0, problem.helicopters.size() - 1);
        int helicopter_idx = distrib(gen);
        current_assignments[helicopter_idx].push_back(problem.villages[i].id);
    }

    // --- INITIAL EVALUATION ---
    // Evaluate this first random state to get a baseline solution
    Solution best_solution;
    double best_value = evaluate_state(current_assignments, problem, best_solution);
    vector<vector<int>> best_assignments = current_assignments;

    int iterations_without_improvement = 0;
    const int restart_threshold = 500; // Number of non-improving steps before we do a random restart

    // --- LOCAL SEARCH LOOP ---
    // The loop continues until the allocated time is nearly up.
    while (true) {
        auto current_time = chrono::high_resolution_clock::now();
        auto elapsed_seconds = chrono::duration_cast<chrono::seconds>(current_time - start_time).count();
        if (elapsed_seconds >= time_limit_seconds - 2) { // Leave a 2s buffer
            break;
        }

        // --- RANDOM RESTART ---
        // If we are stuck in a local optimum, jump to a new random state.
        if (iterations_without_improvement > restart_threshold) {
             for(auto& vec : current_assignments) vec.clear();
             for (int i = 0; i < problem.villages.size(); ++i) {
                uniform_int_distribution<> distrib(0, problem.helicopters.size() - 1);
                current_assignments[distrib(gen)].push_back(problem.villages[i].id);
            }
            iterations_without_improvement = 0;
            continue; // Re-evaluate this new random state in the next loop iteration
        }

        // --- GENERATE NEIGHBOR STATE ---
        // As you suggested: move a random village to a random (different) helicopter.
        vector<vector<int>> neighbor_assignments = current_assignments;
        
        uniform_int_distribution<> h_dist(0, problem.helicopters.size() - 1);
        int h_from_idx = h_dist(gen);
        while (neighbor_assignments[h_from_idx].empty()) { // Ensure we pick a helicopter that has villages
             if (problem.villages.empty()) break;
             h_from_idx = h_dist(gen);
        }
        if (neighbor_assignments[h_from_idx].empty()) continue; // Skip if no villages to move

        int h_to_idx = h_dist(gen);
        while (h_to_idx == h_from_idx) { // Ensure we move to a different helicopter
            h_to_idx = h_dist(gen);
        }

        uniform_int_distribution<> v_dist(0, neighbor_assignments[h_from_idx].size() - 1);
        int village_vec_idx = v_dist(gen);

        // Perform the move
        int village_id_to_move = neighbor_assignments[h_from_idx][village_vec_idx];
        neighbor_assignments[h_to_idx].push_back(village_id_to_move);
        neighbor_assignments[h_from_idx].erase(neighbor_assignments[h_from_idx].begin() + village_vec_idx);

        // --- EVALUATE NEIGHBOR ---
        Solution neighbor_solution;
        double neighbor_value = evaluate_state(neighbor_assignments, problem, neighbor_solution);

        // --- ACCEPTANCE CRITERION (HILL CLIMBING) ---
        if (neighbor_value > best_value) {
            best_value = neighbor_value;
            best_solution = neighbor_solution;
            best_assignments = neighbor_assignments;
            current_assignments = neighbor_assignments; // Move to the better state
            iterations_without_improvement = 0;
        } else {
            iterations_without_improvement++;
        }
    }

    return best_solution;
}


// --- HELPER FUNCTION IMPLEMENTATIONS ---

/**
 * @brief Evaluates a state (village assignment), returning its total value.
 * It calls the inner optimization for each helicopter and sums the results.
 * It also populates the 'out_solution' with the detailed plan.
 */
double evaluate_state(const vector<vector<int>>& village_assignments, const ProblemData& problem, Solution& out_solution) {
    out_solution.clear();
    out_solution.resize(problem.helicopters.size());
    double total_value = 0.0;

    for (int i = 0; i < problem.helicopters.size(); ++i) {
        HelicopterPlan plan = optimize_helicopter_plan(problem.helicopters[i], village_assignments[i], problem);
        
        double plan_value = 0;
        double total_trip_cost = 0;
        double value_from_packages = 0;

        const Point& home_city_coords = problem.cities[problem.helicopters[i].home_city_id - 1];

        for(const auto& trip : plan.trips) {
            Point current_loc = home_city_coords;
            double trip_distance = 0;
            for(const auto& drop : trip.drops) {
                const Point& village_coords = problem.villages[drop.village_id - 1].coords;
                trip_distance += distance(current_loc, village_coords);
                current_loc = village_coords;
                value_from_packages += drop.dry_food * problem.packages[0].value;
                value_from_packages += drop.perishable_food * problem.packages[1].value;
                value_from_packages += drop.other_supplies * problem.packages[2].value;
            }
            trip_distance += distance(current_loc, home_city_coords);
            total_trip_cost += problem.helicopters[i].fixed_cost + (problem.helicopters[i].alpha * trip_distance);
        }
        
        plan_value = value_from_packages - total_trip_cost;
        total_value += plan_value;
        out_solution[i] = plan;
    }
    return total_value;
}

/**
 * @brief The "inner loop" optimization for a single helicopter.
 * Creates a series of profitable trips to serve the assigned villages.
 * Uses a greedy Nearest Neighbor heuristic for routing and a greedy
 * value/weight ratio heuristic for packing.
 */
HelicopterPlan optimize_helicopter_plan(const Helicopter& h, const vector<int>& assigned_village_ids, const ProblemData& problem) {
    HelicopterPlan plan;
    plan.helicopter_id = h.id;
    if (assigned_village_ids.empty()) {
        return plan;
    }

    // Map village_id -> remaining demand [dry, perishable, other]
    unordered_map<int, vector<int>> demand;
    for (int vid : assigned_village_ids) {
        int pop = problem.villages[vid - 1].population;
        int food_demand = 9 * pop;
        int other_demand = 1 * pop;
        demand[vid] = {food_demand, food_demand, other_demand}; // dry/perishable share a total food demand
    }

    double total_distance_traveled = 0;
    const Point& home_coords = problem.cities[h.home_city_id - 1];

    // Create trips until no more profitable trips can be made or demand is met
    while (total_distance_traveled < problem.d_max) {
        bool demand_exists = false;
        for(const auto& pair : demand) {
            if (pair.second[0] > 0 || pair.second[2] > 0) {
                demand_exists = true;
                break;
            }
        }
        if (!demand_exists) break;

        // --- 1. Greedy Route Building (Nearest Neighbor style) ---
        Trip current_trip;
        vector<int> route_vids;
        unordered_map<int, bool> visited_in_trip;
        double current_dist = 0;
        Point current_pos = home_coords;
        
        // Build a route for one trip
        while(true) {
            int best_next_vid = -1;
            double min_dist = 1e9;

            // Find the closest, unvisited village with demand
            for (int vid : assigned_village_ids) {
                if (visited_in_trip.count(vid)) continue;
                if (demand[vid][0] <= 0 && demand[vid][2] <= 0) continue;

                double d = distance(current_pos, problem.villages[vid-1].coords);
                if (d < min_dist) {
                    // Check if adding this village and returning home is feasible
                    if (current_dist + d + distance(problem.villages[vid-1].coords, home_coords) <= h.distance_capacity) {
                        min_dist = d;
                        best_next_vid = vid;
                    }
                }
            }
            
            if (best_next_vid != -1) {
                route_vids.push_back(best_next_vid);
                visited_in_trip[best_next_vid] = true;
                current_dist += min_dist;
                current_pos = problem.villages[best_next_vid - 1].coords;
            } else {
                break; // No more villages can be added to this trip
            }
        }
        
        if (route_vids.empty()) break; // Cannot form any more valid trips
        current_dist += distance(current_pos, home_coords);

        // --- 2. Greedy Packing (Knapsack style) ---
        // Pack items based on best value/weight ratio
        double current_weight = 0;
        
        // Items are: {value/weight, weight, value, type_idx (0=d,1=p,2=o), village_id}
        vector<tuple<double, double, double, int, int>> items_to_pack;
        
        for (int vid : route_vids) {
            // Perishable food (highest value/weight ratio)
            items_to_pack.emplace_back((double)problem.packages[1].value / problem.packages[1].weight, problem.packages[1].weight, problem.packages[1].value, 1, vid);
            // Dry food
            items_to_pack.emplace_back((double)problem.packages[0].value / problem.packages[0].weight, problem.packages[0].weight, problem.packages[0].value, 0, vid);
            // Other supplies
            items_to_pack.emplace_back((double)problem.packages[2].value / problem.packages[2].weight, problem.packages[2].weight, problem.packages[2].value, 2, vid);
        }
        sort(items_to_pack.rbegin(), items_to_pack.rend()); // Sort descending by value/weight ratio

        unordered_map<int, vector<int>> drop_counts; // vid -> {d, p, o}
        for(int vid : route_vids) drop_counts[vid] = {0,0,0};
        
        vector<int> pickup_counts = {0,0,0};

        for (const auto& item : items_to_pack) {
            double item_weight = get<1>(item);
            int type_idx = get<3>(item);
            int vid = get<4>(item);

            int demand_to_fill = 0;
            if(type_idx == 0 || type_idx == 1) { // Dry or Perishable Food
                demand_to_fill = demand[vid][0]; // Total food demand
            } else { // Other supplies
                demand_to_fill = demand[vid][2];
            }
            
            int num_to_add = 0;
            if (item_weight > 0) {
                 num_to_add = min(demand_to_fill, (int)((h.weight_capacity - current_weight) / item_weight));
            }


            if (num_to_add > 0) {
                if (type_idx == 0) { // Dry food
                    drop_counts[vid][0] += num_to_add;
                    pickup_counts[0] += num_to_add;
                    demand[vid][0] -= num_to_add; // Reduce total food demand
                } else if (type_idx == 1) { // Perishable food
                    drop_counts[vid][1] += num_to_add;
                    pickup_counts[1] += num_to_add;
                    demand[vid][0] -= num_to_add; // Reduce total food demand
                } else { // Other supplies
                    drop_counts[vid][2] += num_to_add;
                    pickup_counts[2] += num_to_add;
                    demand[vid][2] -= num_to_add;
                }
                current_weight += num_to_add * item_weight;
            }
        }

        // --- 3. Finalize and Store the Trip ---
        double trip_cost = h.fixed_cost + h.alpha * current_dist;
        double trip_value = 0;
        
        current_trip.dry_food_pickup = pickup_counts[0];
        current_trip.perishable_food_pickup = pickup_counts[1];
        current_trip.other_supplies_pickup = pickup_counts[2];

        for (int vid : route_vids) {
            if (drop_counts[vid][0] > 0 || drop_counts[vid][1] > 0 || drop_counts[vid][2] > 0) {
                current_trip.drops.push_back({vid, drop_counts[vid][0], drop_counts[vid][1], drop_counts[vid][2]});
                trip_value += drop_counts[vid][0] * problem.packages[0].value;
                trip_value += drop_counts[vid][1] * problem.packages[1].value;
                trip_value += drop_counts[vid][2] * problem.packages[2].value;
            }
        }

        // Only add the trip if it's profitable and actually delivers something
        if (trip_value > trip_cost && !current_trip.drops.empty()) {
            plan.trips.push_back(current_trip);
            total_distance_traveled += current_dist;
        } else {
            // If the most promising trip isn't profitable, none will be.
            break; 
        }
    }

    return plan;
}