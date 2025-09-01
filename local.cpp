#include "local.h"
#include <iostream>
#include <chrono>
#include <vector>
#include <numeric>
#include <algorithm>
#include <random>
#include <map>
#include <cmath>
#include <limits>
#include <set>

using namespace std;
using namespace std::chrono;

// Helper structure to track village demand
struct VillageDemand {
    int dry_needed;
    int perishable_needed;
    int other_needed;
    int dry_delivered;
    int perishable_delivered;
    int other_delivered;
    
    VillageDemand(int population) {
        int food_needed = population * 9;
        dry_needed = food_needed;
        perishable_needed = food_needed;
        other_needed = population;
        dry_delivered = 0;
        perishable_delivered = 0;
        other_delivered = 0;
    }
    
    int getRemainingFoodDemand() const {
        return max(0, min(dry_needed, perishable_needed) - (dry_delivered + perishable_delivered));
    }
    
    int getRemainingOtherDemand() const {
        return max(0, other_needed - other_delivered);
    }
};

// Calculate trip distance
double calculateTripDistance(const ProblemData& problem, const Helicopter& heli, const Trip& trip) {
    if (trip.drops.empty()) return 0;
    
    double total_dist = 0;
    Point current = problem.cities[heli.home_city_id - 1];
    
    for (const auto& drop : trip.drops) {
        Point village_pos = problem.villages[drop.village_id - 1].coords;
        total_dist += distance(current, village_pos);
        current = village_pos;
    }
    
    total_dist += distance(current, problem.cities[heli.home_city_id - 1]);
    return total_dist;
}

// Calculate trip weight
double calculateTripWeight(const ProblemData& problem, const Trip& trip) {
    return trip.dry_food_pickup * problem.packages[0].weight +
           trip.perishable_food_pickup * problem.packages[1].weight +
           trip.other_supplies_pickup * problem.packages[2].weight;
}

// Check if a trip is valid
bool isValidTrip(const ProblemData& problem, const Helicopter& heli, const Trip& trip) {
    // Check weight capacity
    double weight = calculateTripWeight(problem, trip);
    if (weight > heli.weight_capacity) return false;
    
    // Check distance capacity
    double dist = calculateTripDistance(problem, heli, trip);
    if (dist > heli.distance_capacity) return false;
    
    // Check that drops don't exceed pickups
    int total_dry = 0, total_perish = 0, total_other = 0;
    for (const auto& drop : trip.drops) {
        total_dry += drop.dry_food;
        total_perish += drop.perishable_food;
        total_other += drop.other_supplies;
    }
    
    return total_dry <= trip.dry_food_pickup &&
           total_perish <= trip.perishable_food_pickup &&
           total_other <= trip.other_supplies_pickup;
}

// Calculate solution value
double calculateSolutionValue(const ProblemData& problem, const Solution& solution) {
    vector<VillageDemand> demands;
    for (const auto& village : problem.villages) {
        demands.emplace_back(village.population);
    }
    
    double total_value = 0;
    double total_cost = 0;
    
    for (const auto& heli_plan : solution) {
        const Helicopter& heli = problem.helicopters[heli_plan.helicopter_id - 1];
        
        for (const auto& trip : heli_plan.trips) {
            if (trip.drops.empty()) continue;
            
            double trip_dist = calculateTripDistance(problem, heli, trip);
            total_cost += heli.fixed_cost + heli.alpha * trip_dist;
            
            for (const auto& drop : trip.drops) {
                VillageDemand& demand = demands[drop.village_id - 1];
                
                // Calculate value for food (preferring perishable)
                int food_capacity = demand.getRemainingFoodDemand();
                int perish_used = min(drop.perishable_food, food_capacity);
                int dry_used = min(drop.dry_food, max(0, food_capacity - perish_used));
                
                total_value += perish_used * problem.packages[1].value;
                total_value += dry_used * problem.packages[0].value;
                
                // Calculate value for other supplies
                int other_used = min(drop.other_supplies, demand.getRemainingOtherDemand());
                total_value += other_used * problem.packages[2].value;
                
                // Update delivered amounts
                demand.dry_delivered += dry_used;
                demand.perishable_delivered += perish_used;
                demand.other_delivered += other_used;
            }
        }
    }
    
    return total_value - total_cost;
}

// Generate initial greedy solution
Solution generateInitialSolution(const ProblemData& problem) {
    Solution solution;
    vector<VillageDemand> demands;
    
    for (const auto& village : problem.villages) {
        demands.emplace_back(village.population);
    }
    
    for (const auto& heli : problem.helicopters) {
        HelicopterPlan plan;
        plan.helicopter_id = heli.id;
        double total_distance = 0;
        
        // Create trips for this helicopter
        while (total_distance < problem.d_max) {
            Trip trip;
            vector<int> village_order;
            
            // Find nearest unserved villages
            for (int v = 0; v < problem.villages.size(); v++) {
                if (demands[v].getRemainingFoodDemand() > 0 || 
                    demands[v].getRemainingOtherDemand() > 0) {
                    village_order.push_back(v);
                }
            }
            
            if (village_order.empty()) break;
            
            // Sort by distance from home city
            Point home = problem.cities[heli.home_city_id - 1];
            sort(village_order.begin(), village_order.end(), [&](int a, int b) {
                return distance(home, problem.villages[a].coords) < 
                       distance(home, problem.villages[b].coords);
            });
            
            // Build a trip
            double trip_weight = 0;
            double trip_dist = 0;
            Point current = home;
            
            for (int v_idx : village_order) {
                const Village& v = problem.villages[v_idx];
                double dist_to_village = distance(current, v.coords);
                double dist_to_home = distance(v.coords, home);
                
                if (trip_dist + dist_to_village + dist_to_home > heli.distance_capacity) continue;
                if (total_distance + trip_dist + dist_to_village + dist_to_home > problem.d_max) break;
                
                VillageDemand& demand = demands[v_idx];
                
                // Calculate how much we can deliver
                int food_needed = demand.getRemainingFoodDemand();
                int other_needed = demand.getRemainingOtherDemand();
                
                if (food_needed == 0 && other_needed == 0) continue;
                
                // Prefer perishable food
                double remaining_weight = heli.weight_capacity - trip_weight;
                int perish_to_deliver = min((int)(remaining_weight / problem.packages[1].weight), food_needed);
                remaining_weight -= perish_to_deliver * problem.packages[1].weight;
                
                int dry_to_deliver = min((int)(remaining_weight / problem.packages[0].weight), 
                                        max(0, food_needed - perish_to_deliver));
                remaining_weight -= dry_to_deliver * problem.packages[0].weight;
                
                int other_to_deliver = min((int)(remaining_weight / problem.packages[2].weight), other_needed);
                
                if (perish_to_deliver + dry_to_deliver + other_to_deliver > 0) {
                    Drop drop;
                    drop.village_id = v.id;
                    drop.dry_food = dry_to_deliver;
                    drop.perishable_food = perish_to_deliver;
                    drop.other_supplies = other_to_deliver;
                    
                    trip.drops.push_back(drop);
                    trip.dry_food_pickup += dry_to_deliver;
                    trip.perishable_food_pickup += perish_to_deliver;
                    trip.other_supplies_pickup += other_to_deliver;
                    
                    demand.dry_delivered += dry_to_deliver;
                    demand.perishable_delivered += perish_to_deliver;
                    demand.other_delivered += other_to_deliver;
                    
                    trip_weight += dry_to_deliver * problem.packages[0].weight +
                                  perish_to_deliver * problem.packages[1].weight +
                                  other_to_deliver * problem.packages[2].weight;
                    
                    trip_dist += dist_to_village;
                    current = v.coords;
                    
                    if (trip_weight >= heli.weight_capacity * 0.9) break;
                }
            }
            
            if (!trip.drops.empty()) {
                trip_dist += distance(current, home);
                total_distance += trip_dist;
                plan.trips.push_back(trip);
                
                if (total_distance >= problem.d_max * 0.95) break;
            } else {
                break;
            }
        }
        
        solution.push_back(plan);
    }
    
    return solution;
}

// Neighbor generation for local search
Solution generateNeighbor(const ProblemData& problem, const Solution& current, mt19937& rng) {
    Solution neighbor = current;
    uniform_int_distribution<> op_dist(0, 4);
    int operation = op_dist(rng);
    
    // Find helicopters with trips
    vector<int> active_helis;
    for (int i = 0; i < neighbor.size(); i++) {
        if (!neighbor[i].trips.empty()) {
            active_helis.push_back(i);
        }
    }
    
    if (active_helis.empty()) return neighbor;
    
    uniform_int_distribution<> heli_dist(0, active_helis.size() - 1);
    
    switch (operation) {
        case 0: { // Swap villages between trips
            if (active_helis.size() < 1) break;
            int h = active_helis[heli_dist(rng)];
            auto& trips = neighbor[h].trips;
            if (trips.size() >= 2) {
                uniform_int_distribution<> trip_dist(0, trips.size() - 1);
                int t1 = trip_dist(rng), t2 = trip_dist(rng);
                if (t1 != t2 && !trips[t1].drops.empty() && !trips[t2].drops.empty()) {
                    uniform_int_distribution<> drop_dist1(0, trips[t1].drops.size() - 1);
                    uniform_int_distribution<> drop_dist2(0, trips[t2].drops.size() - 1);
                    swap(trips[t1].drops[drop_dist1(rng)], trips[t2].drops[drop_dist2(rng)]);
                    
                    // Recalculate pickups
                    for (auto& trip : trips) {
                        trip.dry_food_pickup = 0;
                        trip.perishable_food_pickup = 0;
                        trip.other_supplies_pickup = 0;
                        for (const auto& drop : trip.drops) {
                            trip.dry_food_pickup += drop.dry_food;
                            trip.perishable_food_pickup += drop.perishable_food;
                            trip.other_supplies_pickup += drop.other_supplies;
                        }
                    }
                }
            }
            break;
        }
        case 1: { // Modify drop quantities
            int h = active_helis[heli_dist(rng)];
            auto& trips = neighbor[h].trips;
            if (!trips.empty()) {
                uniform_int_distribution<> trip_dist(0, trips.size() - 1);
                int t = trip_dist(rng);
                if (!trips[t].drops.empty()) {
                    uniform_int_distribution<> drop_dist(0, trips[t].drops.size() - 1);
                    auto& drop = trips[t].drops[drop_dist(rng)];
                    
                    // Randomly adjust quantities
                    uniform_int_distribution<> adjust_dist(-100, 100);
                    drop.dry_food = max(0, drop.dry_food + adjust_dist(rng));
                    drop.perishable_food = max(0, drop.perishable_food + adjust_dist(rng));
                    drop.other_supplies = max(0, drop.other_supplies + adjust_dist(rng));
                    
                    // Update pickups
                    trips[t].dry_food_pickup = 0;
                    trips[t].perishable_food_pickup = 0;
                    trips[t].other_supplies_pickup = 0;
                    for (const auto& d : trips[t].drops) {
                        trips[t].dry_food_pickup += d.dry_food;
                        trips[t].perishable_food_pickup += d.perishable_food;
                        trips[t].other_supplies_pickup += d.other_supplies;
                    }
                }
            }
            break;
        }
        case 2: { // Remove a trip
            int h = active_helis[heli_dist(rng)];
            auto& trips = neighbor[h].trips;
            if (trips.size() > 1) {
                uniform_int_distribution<> trip_dist(0, trips.size() - 1);
                trips.erase(trips.begin() + trip_dist(rng));
            }
            break;
        }
        case 3: { // Reorder villages in a trip
            int h = active_helis[heli_dist(rng)];
            auto& trips = neighbor[h].trips;
            if (!trips.empty()) {
                uniform_int_distribution<> trip_dist(0, trips.size() - 1);
                auto& drops = trips[trip_dist(rng)].drops;
                if (drops.size() > 1) {
                    shuffle(drops.begin(), drops.end(), rng);
                }
            }
            break;
        }
        case 4: { // Transfer trip between helicopters
            if (neighbor.size() >= 2 && active_helis.size() >= 1) {
                int h1 = active_helis[heli_dist(rng)];
                uniform_int_distribution<> h2_dist(0, neighbor.size() - 1);
                int h2 = h2_dist(rng);
                if (h1 != h2 && !neighbor[h1].trips.empty()) {
                    uniform_int_distribution<> trip_dist(0, neighbor[h1].trips.size() - 1);
                    int t = trip_dist(rng);
                    neighbor[h2].trips.push_back(neighbor[h1].trips[t]);
                    neighbor[h1].trips.erase(neighbor[h1].trips.begin() + t);
                }
            }
            break;
        }
    }
    
    return neighbor;
}

// Validate and repair solution
void validateAndRepair(const ProblemData& problem, Solution& solution) {
    for (auto& heli_plan : solution) {
        const Helicopter& heli = problem.helicopters[heli_plan.helicopter_id - 1];
        double total_distance = 0;
        
        auto it = heli_plan.trips.begin();
        while (it != heli_plan.trips.end()) {
            // Check and fix trip validity
            double weight = calculateTripWeight(problem, *it);
            double dist = calculateTripDistance(problem, heli, *it);
            
            if (weight > heli.weight_capacity || dist > heli.distance_capacity || 
                total_distance + dist > problem.d_max || !isValidTrip(problem, heli, *it)) {
                it = heli_plan.trips.erase(it);
            } else {
                total_distance += dist;
                ++it;
            }
        }
    }
}

Solution solve(const ProblemData& problem) {
    cout << "Starting solver with Simulated Annealing..." << endl;
    
    auto start_time = steady_clock::now();
    double time_limit = problem.time_limit_minutes * 60.0 * 0.95; // Use 95% of available time
    
    // Initialize random generator
    random_device rd;
    mt19937 rng(rd());
    
    // Generate initial solution
    Solution current_solution = generateInitialSolution(problem);
    Solution best_solution = current_solution;
    double best_value = calculateSolutionValue(problem, best_solution);
    
    cout << "Initial solution value: " << best_value << endl;
    
    // Simulated Annealing parameters
    double temperature = 10000.0;
    double cooling_rate = 0.9995;
    double min_temperature = 0.1;
    
    int iterations = 0;
    int improvements = 0;
    
    while (temperature > min_temperature) {
        auto current_time = steady_clock::now();
        double elapsed = duration_cast<duration<double>>(current_time - start_time).count();
        
        if (elapsed >= time_limit) {
            cout << "Time limit reached." << endl;
            break;
        }
        
        // Generate neighbor
        Solution neighbor = generateNeighbor(problem, current_solution, rng);
        validateAndRepair(problem, neighbor);
        
        double neighbor_value = calculateSolutionValue(problem, neighbor);
        double delta = neighbor_value - calculateSolutionValue(problem, current_solution);
        
        // Accept or reject neighbor
        if (delta > 0 || exp(delta / temperature) > uniform_real_distribution<>(0, 1)(rng)) {
            current_solution = neighbor;
            
            if (neighbor_value > best_value) {
                best_solution = neighbor;
                best_value = neighbor_value;
                improvements++;
                cout << "Improvement found! New best: " << best_value 
                     << " (iteration " << iterations << ")" << endl;
            }
        }
        
        // Cool down
        temperature *= cooling_rate;
        iterations++;
        
        // Periodic restart with best solution
        if (iterations % 10000 == 0) {
            current_solution = best_solution;
            temperature = 1000.0; // Reheat for exploration
            cout << "Restart at iteration " << iterations << ", best value: " << best_value << endl;
        }
    }
    
    cout << "Solver finished. Total iterations: " << iterations 
         << ", improvements: " << improvements 
         << ", final best value: " << best_value << endl;
    
    return best_solution;
}