#include "solver.h"
#include <iostream>
#include <chrono>
#include <vector>
#include <numeric>
#include <algorithm>
#include <random>
#include <map>

using namespace std;

// --- FORWARD DECLARATIONS OF HELPER FUNCTIONS ---
double route_distance(const ProblemData &P, int home_city_id, const vector<int> &route_village_ids);

// Plans all trips for a single helicopter to serve a specific set of villages.
HelicopterPlan plan_trips_for_helicopter(int helicopter_id, const vector<int>& assigned_village_ids, const ProblemData& problem);

// Generates an initial partitioning of villages to helicopters (greedy assignment).
vector<vector<int>> generate_initial_partition(const ProblemData& problem);

// Evaluates the total score for a given partition of villages.
Score evaluate_partition(const vector<vector<int>>& partition, const ProblemData& problem, Solution& out_solution);


/**
 * @brief The main function to implement your search/optimization algorithm.
 * This implementation uses a simple Hill Climbing local search.
 */
Solution solve(const ProblemData& problem) {
    cout << "Starting solver..." << endl;

    // Set up a random number generator for selecting neighbors
    std::random_device rd;
    std::mt19937 gen(rd());

    // Define a time limit for the search
    auto start_time = chrono::high_resolution_clock::now();
    auto time_limit = chrono::duration<double>(problem.time_limit_minutes * 60.0 - 5.0); // 5s buffer

    // 1. Generate the initial solution
    vector<vector<int>> current_partition = generate_initial_partition(problem);
    Solution best_solution;
    Score best_score = evaluate_partition(current_partition, problem, best_solution);

    cout << "Initial score: " << best_score.value << endl;

    // 2. Start the local search (Hill Climbing)
    while (chrono::high_resolution_clock::now() - start_time < time_limit) {
        if (problem.villages.empty()) break;

        // --- Generate a neighbor state ---
        vector<vector<int>> next_partition = current_partition;

        // Randomly select a village to move
        uniform_int_distribution<> village_dist(0, problem.villages.size() - 1);
        int village_to_move_idx = village_dist(gen);
        int village_id_to_move = problem.villages[village_to_move_idx].id;

        // Find which helicopter currently has this village
        int from_heli_idx = -1;
        for (int i = 0; i < next_partition.size(); ++i) {
            if (find(next_partition[i].begin(), next_partition[i].end(), village_id_to_move) != next_partition[i].end()) {
                from_heli_idx = i;
                break;
            }
        }
        if (from_heli_idx == -1) continue; // Should not happen

        // Randomly select a different helicopter to move it to
        uniform_int_distribution<> heli_dist(0, problem.helicopters.size() - 2);
        int to_heli_idx = heli_dist(gen);
        if (to_heli_idx >= from_heli_idx) to_heli_idx++; // Ensure it's a different helicopter

        // Perform the move
        next_partition[from_heli_idx].erase(remove(next_partition[from_heli_idx].begin(), next_partition[from_heli_idx].end(), village_id_to_move), next_partition[from_heli_idx].end());
        next_partition[to_heli_idx].push_back(village_id_to_move);

        // --- Evaluate the neighbor ---
        Solution potential_solution;
        Score next_score = evaluate_partition(next_partition, problem, potential_solution);

        // --- Decide whether to accept the move ---
        if (next_score.value > best_score.value) {
            best_score = next_score;
            best_solution = potential_solution;
            current_partition = next_partition;
            cout << "Found a better solution with score: " << best_score.value << endl;
        }
        // NOTE: To implement random restart, you would add a counter. If no improvement
        // is found after N iterations, you'd regenerate a new random initial partition.
        // For simulated annealing, you would accept worse moves with a certain probability.
    }

    cout << "Solver finished. Best score found: " << best_score.value << endl;
    return best_solution;
}


// --- IMPLEMENTATIONS OF HELPER FUNCTIONS ---

/**
 * @brief Assigns each village to the helicopter with the closest home city.
 */
vector<vector<int>> generate_initial_partition(const ProblemData& problem) {
    vector<vector<int>> partition(problem.helicopters.size());
    if (problem.helicopters.empty()) return partition;

    for (const auto& village : problem.villages) {
        double min_dist = -1.0;
        int best_heli_idx = -1;

        for (int i = 0; i < problem.helicopters.size(); ++i) {
            const auto& helicopter = problem.helicopters[i];
            const Point& city_coords = problem.cities[helicopter.home_city_id - 1];
            double d = distance(village.coords, city_coords);
            if (best_heli_idx == -1 || d < min_dist) {
                min_dist = d;
                best_heli_idx = i;
            }
        }
        if (best_heli_idx != -1) {
            partition[best_heli_idx].push_back(village.id);
        }
    }
    return partition;
}

/**
 * @brief Calculates the total score for a given partition by planning and
 * evaluating trips for each helicopter.
 */
Score evaluate_partition(const vector<vector<int>>& partition, const ProblemData& problem, Solution& out_solution) {
    out_solution.clear();
    out_solution.reserve(problem.helicopters.size());

    for (int i = 0; i < problem.helicopters.size(); ++i) {
        out_solution.push_back(plan_trips_for_helicopter(problem.helicopters[i].id, partition[i], problem));
    }
    return evaluate(out_solution, problem);
}


/**
 * @brief A greedy heuristic to plan all trips for one helicopter given its assigned villages.
 * TODO: This is a key function to improve. A more sophisticated algorithm (like
 * your Knapsack + Clarke-Wright idea) would yield better results.
 */
HelicopterPlan plan_trips_for_helicopter(int helicopter_id, const vector<int>& assigned_village_ids, const ProblemData& problem) {
    HelicopterPlan plan;
    plan.helicopter_id = helicopter_id;

    if (assigned_village_ids.empty()) {
        return plan;
    }

    // Find the specific helicopter's data
    const Helicopter* heli = nullptr;
    for(const auto& h : problem.helicopters){
        if(h.id == helicopter_id) {
            heli = &h;
            break;
        }
    }
    if(!heli) return plan; // Should not happen

    const Point& home_city = problem.cities[heli->home_city_id - 1];

    // Calculate total demand for all assigned villages
    map<int, tuple<int, int, int>> remaining_demand; // village_id -> {dry, perishable, other}
    for (int vid : assigned_village_ids) {
        const Village& village = problem.villages[vid - 1];
        int food_demand = village.population * 9;
        int other_demand = village.population * 1;
        // Prioritize perishable food for its higher value
        remaining_demand[vid] = {0, food_demand, other_demand};
    }
    
    double total_distance_traveled = 0;

    // Greedily create trips until all demand is met or constraints are hit
    bool demand_exists = true;
    while(demand_exists) {
        demand_exists = false;
        for(const auto& pair : remaining_demand) {
            if (get<0>(pair.second) > 0 || get<1>(pair.second) > 0 || get<2>(pair.second) > 0) {
                demand_exists = true;
                break;
            }
        }
        if (!demand_exists) break;

        // --- Start a new trip ---
        Trip current_trip = {0, 0, 0, {}};
        double current_weight = 0;
        vector<int> route_village_ids; // The sequence of villages to visit in this trip
        map<int, Drop> drops_this_trip; // village_id -> Drop

        // --- Greedily add villages/packages to this trip ---
        // This is a simple greedy packing, not a formal knapsack.
        // It prioritizes perishable food due to higher value.
        for (int vid : assigned_village_ids) {
            auto& demand = remaining_demand[vid];
            int p_needed = get<1>(demand);
            int o_needed = get<2>(demand);

            // Perishable food
            int p_to_take = min(p_needed, (int)((heli->weight_capacity - current_weight) / problem.packages[1].weight));
            if (p_to_take > 0) {
                drops_this_trip[vid].perishable_food += p_to_take;
                current_weight += p_to_take * problem.packages[1].weight;
                get<1>(demand) -= p_to_take;
            }

            // Other supplies
            int o_to_take = min(o_needed, (int)((heli->weight_capacity - current_weight) / problem.packages[2].weight));
             if (o_to_take > 0) {
                drops_this_trip[vid].other_supplies += o_to_take;
                current_weight += o_to_take * problem.packages[2].weight;
                get<2>(demand) -= o_to_take;
            }

            // Dry food (if there is still capacity and food demand)
            int d_needed = p_needed - p_to_take; // Use dry food to cover remaining food needs
            int d_to_take = min(d_needed, (int)((heli->weight_capacity - current_weight) / problem.packages[0].weight));
            if (d_to_take > 0) {
                drops_this_trip[vid].dry_food += d_to_take;
                current_weight += d_to_take * problem.packages[0].weight;
                // No need to update demand, as it was a substitute for perishable
            }
        }

        // --- Plan the route for this trip (simple nearest neighbor) ---
        // TODO: Replace with a better routing algorithm like Clarke-Wright.
        if (drops_this_trip.empty()) continue;

        Point current_pos = home_city;
        while(!drops_this_trip.empty()) {
            double min_d = -1.0;
            int next_vid = -1;
            for(const auto& pair : drops_this_trip) {
                double d = distance(current_pos, problem.villages[pair.first - 1].coords);
                if (next_vid == -1 || d < min_d) {
                    min_d = d;
                    next_vid = pair.first;
                }
            }
            
            route_village_ids.push_back(next_vid);
            current_pos = problem.villages[next_vid - 1].coords;
            
            Drop final_drop = drops_this_trip[next_vid];
            final_drop.village_id = next_vid;
            current_trip.drops.push_back(final_drop);

            current_trip.dry_food_pickup += final_drop.dry_food;
            current_trip.perishable_food_pickup += final_drop.perishable_food;
            current_trip.other_supplies_pickup += final_drop.other_supplies;

            drops_this_trip.erase(next_vid);
        }

        // Finalize trip and check constraints
        double trip_dist = route_distance(problem, heli->home_city_id, route_village_ids);
        if (trip_dist <= heli->distance_capacity && total_distance_traveled + trip_dist <= problem.d_max) {
             plan.trips.push_back(current_trip);
             total_distance_traveled += trip_dist;
        } else {
            // This trip is invalid, revert the demand changes (this is a simplification)
            // A better implementation would check constraints *while* building the trip.
            // For now, we just discard this invalid trip and the remaining demand won't be served.
            break; 
        }
    }

    return plan;
}

double route_distance(const ProblemData &P, int home_city_id, const vector<int> &route_village_ids) {
    // Check for invalid home_city_id to prevent errors.
    if (home_city_id <= 0 || home_city_id > P.cities.size()) {
        return 0.0;
    }

    double total_distance = 0.0;
    const Point& home_city_coords = P.cities[home_city_id - 1];
    Point current_location = home_city_coords;

    // Calculate the distance for each leg of the trip to the villages.
    for (int village_id : route_village_ids) {
        // Check for invalid village_id.
        if (village_id <= 0 || village_id > P.villages.size()) {
            continue; // Skip invalid IDs.
        }
        const Point& next_village_coords = P.villages[village_id - 1].coords;
        total_distance += distance(current_location, next_village_coords);
        current_location = next_village_coords; // Update the current location to the village just visited.
    }

    // Add the distance for the final leg: from the last village back to the home city.
    // If the route was empty, this calculates distance from home to home (which is 0).
    total_distance += distance(current_location, home_city_coords);

    return total_distance;
}

Score evaluate(const Solution &sol, const ProblemData &problem) {
    double total_value_gained = 0.0;
    double total_trip_costs = 0.0;

    // Map to track supplies delivered to each village to handle the value cap.
    // village_id -> {total_food_delivered, total_other_delivered}
    std::map<int, std::pair<int, int>> delivered_count;

    for (const auto& plan : sol) {
        // Find the helicopter data for the current plan.
        const Helicopter* heli = nullptr;
        for (const auto& h : problem.helicopters) {
            if (h.id == plan.helicopter_id) {
                heli = &h;
                break;
            }
        }
        if (!heli) continue; // Skip if helicopter not found.

        for (const auto& trip : plan.trips) {
            // 1. Calculate the cost of this trip.
            std::vector<int> route_village_ids;
            for (const auto& drop : trip.drops) {
                route_village_ids.push_back(drop.village_id);
            }
            double dist = route_distance(problem, heli->home_city_id, route_village_ids);
            total_trip_costs += heli->fixed_cost + heli->alpha * dist;

            // 2. Calculate the value gained from drops in this trip.
            for (const auto& drop : trip.drops) {
                const Village& village = problem.villages[drop.village_id - 1];
                int food_cap = village.population * 9; 
                int other_cap = village.population * 1; 

                // Ensure the village is initialized in our tracking map.
                if (delivered_count.find(village.id) == delivered_count.end()) {
                    delivered_count[village.id] = {0, 0};
                }

                auto& delivered = delivered_count[village.id];
                int& food_delivered = delivered.first;
                int& other_delivered = delivered.second;

                // Value from perishable food (higher value, so counted first)[cite: 17].
                int p_for_value = std::min(drop.perishable_food, std::max(0, food_cap - food_delivered));
                total_value_gained += p_for_value * problem.packages[1].value;
                food_delivered += p_for_value;
                
                // Value from dry food.
                int d_for_value = std::min(drop.dry_food, std::max(0, food_cap - food_delivered));
                total_value_gained += d_for_value * problem.packages[0].value;
                food_delivered += d_for_value;
                
                // Value from other supplies.
                int o_for_value = std::min(drop.other_supplies, std::max(0, other_cap - other_delivered));
                total_value_gained += o_for_value * problem.packages[2].value;
                other_delivered += o_for_value;
            }
        }
    }

    // The final score's value is the net result.
    return Score(total_value_gained - total_trip_costs, total_trip_costs);
}