#include "solver.h"
#include <iostream>
#include <chrono>
#include <vector>
#include <numeric>
#include <algorithm>
#include <random>
#include <map>

#include "structures.h"

using namespace std;

// --- FORWARD DECLARATIONS ---
double evaluate(const Solution& solution, const ProblemData& data);
Solution get_neighbor(Solution current_solution, const ProblemData& data);
Solution generate_initial_solution(const ProblemData& data);

/**
 * @brief Main Simulated Annealing function.
 */
Solution simulated_annealing(const ProblemData& data) {
    auto start_time = chrono::high_resolution_clock::now();

    // 1. Set SA parameters (these often require tuning for best performance)
    double temperature = 10000.0;
    double cooling_rate = 0.9995;
    double absolute_min_temp = 0.1;

    // 2. Generate an initial empty solution
    Solution current_solution = generate_initial_solution(data);
    Solution best_solution = current_solution;

    double best_value = evaluate(best_solution, data);
    double current_value = best_value;
    
    srand(time(NULL));

    // 3. Main SA loop
    while (temperature > absolute_min_temp) {
        // Check if time is up
        auto current_time = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsed = current_time - start_time;
        if (elapsed.count() > data.time_limit_minutes - 3.0) { // Exit 2 seconds before limit
            break;
        }

        Solution neighbor_solution = get_neighbor(current_solution, data);
        double neighbor_value = evaluate(neighbor_solution, data);
        double delta_value = neighbor_value - current_value;

        if (delta_value > 0 || (exp(delta_value / temperature) > ((double)rand() / RAND_MAX))) {
            current_solution = neighbor_solution;
            current_value = neighbor_value;
        }

        if (current_value > best_value) {
            best_solution = current_solution;
            best_value = current_value;
        }
        
        temperature *= cooling_rate;
    }
    return best_solution;
}


/**
 * @brief Evaluates a solution, returning (Total Value - Total Cost - Penalties).
 */
double evaluate(const Solution& solution, const ProblemData& data) {
    double total_package_value = 0;
    double total_trip_cost = 0;
    double penalty = 0;
    const double PENALTY_FACTOR = 1e9; // A very large number for violations

    vector<double> food_delivered_to_village(data.villages.size() + 1, 0);

    for (const auto& plan : solution) {
        const Helicopter& heli = data.helicopters[plan.helicopter_id - 1];
        const Point& home_city_coords = data.cities[heli.home_city_id - 1];
        double total_heli_dist = 0;

        for (const auto& trip : plan.trips) {
            double trip_dist = 0;
            double trip_weight = 0;
            int total_drops_d = 0, total_drops_p = 0, total_drops_o = 0;

            Point last_pos = home_city_coords;
            for (const auto& drop : trip.drops) {
                const Village& village = data.villages[drop.village_id - 1];
                trip_dist += distance(last_pos, village.coords);
                last_pos = village.coords;

                // Accumulate value and weight from this drop
                total_package_value += drop.dry_food * data.packages[0].value;
                total_package_value += drop.perishable_food * data.packages[1].value;
                total_package_value += drop.other_supplies * data.packages[2].value;

                food_delivered_to_village[village.id] += drop.dry_food + drop.perishable_food;
                
                total_drops_d += drop.dry_food;
                total_drops_p += drop.perishable_food;
                total_drops_o += drop.other_supplies;
            }
            trip_dist += distance(last_pos, home_city_coords); // Return to home city [cite: 20]
            total_heli_dist += trip_dist;

            // Calculate trip cost [cite: 22]
            if (!trip.drops.empty()) {
                total_trip_cost += heli.fixed_cost + heli.alpha * trip_dist;
            }

            // Check trip constraints [cite: 19]
            if (trip_dist > heli.distance_capacity) penalty += PENALTY_FACTOR;

            trip_weight = trip.dry_food_pickup * data.packages[0].weight +
                          trip.perishable_food_pickup * data.packages[1].weight +
                          trip.other_supplies_pickup * data.packages[2].weight;
            if (trip_weight > heli.weight_capacity) penalty += PENALTY_FACTOR;

            // Check if pickups match drops
            if (trip.dry_food_pickup < total_drops_d) penalty += PENALTY_FACTOR;
            if (trip.perishable_food_pickup < total_drops_p) penalty += PENALTY_FACTOR;
            if (trip.other_supplies_pickup < total_drops_o) penalty += PENALTY_FACTOR;
        }

        // Check total helicopter distance constraint [cite: 21]
        if (total_heli_dist > data.d_max) penalty += PENALTY_FACTOR;
    }

    // Check food limits and remove value for excess food [cite: 15, 77]
    for (const auto& village : data.villages) {
        double max_food_needed = 9.0 * village.population;
        if (food_delivered_to_village[village.id] > max_food_needed) {
            // This is an approximation; assumes value is proportional to quantity
            double excess_food = food_delivered_to_village[village.id] - max_food_needed;
            double avg_food_value = (data.packages[0].value + data.packages[1].value) / 2.0;
            total_package_value -= excess_food * avg_food_value;
        }
    }

    return total_package_value - total_trip_cost - penalty;
}

/**
 * @brief Generates a neighbor by making a small random change.
 */
Solution get_neighbor(Solution current_solution, const ProblemData& data) {
    if (data.helicopters.empty()) return current_solution;

    Solution neighbor = current_solution;
    int move_type = rand() % 100;

    // Move priorities: 60% modify, 20% reorder, 15% relocate, 5% add trip
    if (move_type < 60) { // Modify Drop Quantities
        int h_idx = rand() % neighbor.size();
        if (neighbor[h_idx].trips.empty()) return current_solution;
        int t_idx = rand() % neighbor[h_idx].trips.size();
        if (neighbor[h_idx].trips[t_idx].drops.empty()) return current_solution;
        int d_idx = rand() % neighbor[h_idx].trips[t_idx].drops.size();
        
        int change = (rand() % 20) - 10; // Change by a small amount
        int pkg_type = rand() % 3;
        if (pkg_type == 0) {
            neighbor[h_idx].trips[t_idx].drops[d_idx].dry_food = max(0, neighbor[h_idx].trips[t_idx].drops[d_idx].dry_food + change);
        } else if (pkg_type == 1) {
            neighbor[h_idx].trips[t_idx].drops[d_idx].perishable_food = max(0, neighbor[h_idx].trips[t_idx].drops[d_idx].perishable_food + change);
        } else {
            neighbor[h_idx].trips[t_idx].drops[d_idx].other_supplies = max(0, neighbor[h_idx].trips[t_idx].drops[d_idx].other_supplies + change);
        }

        // Recalculate total pickups for the trip
        int total_d = 0, total_p = 0, total_o = 0;
        for(const auto& drop : neighbor[h_idx].trips[t_idx].drops) {
            total_d += drop.dry_food;
            total_p += drop.perishable_food;
            total_o += drop.other_supplies;
        }
        neighbor[h_idx].trips[t_idx].dry_food_pickup = total_d;
        neighbor[h_idx].trips[t_idx].perishable_food_pickup = total_p;
        neighbor[h_idx].trips[t_idx].other_supplies_pickup = total_o;

    } else if (move_type < 80) { // Reorder Villages in a Trip
        int h_idx = rand() % neighbor.size();
        if (neighbor[h_idx].trips.empty()) return current_solution;
        int t_idx = rand() % neighbor[h_idx].trips.size();
        if (neighbor[h_idx].trips[t_idx].drops.size() < 2) return current_solution;
        
        int idx1 = rand() % neighbor[h_idx].trips[t_idx].drops.size();
        int idx2 = rand() % neighbor[h_idx].trips[t_idx].drops.size();
        swap(neighbor[h_idx].trips[t_idx].drops[idx1], neighbor[h_idx].trips[t_idx].drops[idx2]);

    } else if (move_type < 95) { // Relocate a Village Drop (or add a new drop)
        if (data.villages.empty()) return current_solution;
        
        int h_dest_idx = rand() % neighbor.size();
        if (neighbor[h_dest_idx].trips.empty()) { // If dest has no trips, add one
             neighbor[h_dest_idx].trips.push_back(Trip());
        }
        int t_dest_idx = rand() % neighbor[h_dest_idx].trips.size();
        
        int v_id_to_add = (rand() % data.villages.size()) + 1;
        neighbor[h_dest_idx].trips[t_dest_idx].drops.push_back({v_id_to_add, 0, 0, 0});
        
    } else { // Add a Trip
        int h_idx = rand() % neighbor.size();
        neighbor[h_idx].trips.push_back(Trip());
    }
    return neighbor;
}

/**
 * @brief Creates an empty solution structure.
 */
Solution generate_initial_solution(const ProblemData& data) {
    Solution initial_sol;
    for(const auto& h : data.helicopters) {
        initial_sol.push_back({h.id, {}});
    }
    return initial_sol;
}

Solution solve(const ProblemData& problem) {
    return simulated_annealing(problem);
}