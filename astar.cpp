#include "astar.h"
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
#include <queue>
#include <unordered_map>
#include <memory>

using namespace std;
using namespace std::chrono;

// ============================================================================
// APPROACH 1: A* SEARCH WITH ADMISSIBLE HEURISTIC
// ============================================================================

struct State {
    vector<vector<Trip>> helicopter_plans;
    vector<double> helicopter_distances;
    vector<int> village_food_delivered;
    vector<int> village_other_delivered;
    double g_cost; // Actual cost from start
    double h_cost; // Heuristic cost to goal
    double f_cost; // g + h
    
    State(int num_helis, int num_villages) {
        helicopter_plans.resize(num_helis);
        helicopter_distances.resize(num_helis, 0.0);
        village_food_delivered.resize(num_villages, 0);
        village_other_delivered.resize(num_villages, 0);
        g_cost = 0;
        h_cost = 0;
        f_cost = 0;
    }
    
    bool operator<(const State& other) const {
        return f_cost > other.f_cost; // For min-heap
    }
};

class AStarSolver {
private:
    const ProblemData& problem;
    
    double calculateStateValue(const State& state) {
        double total_value = 0;
        
        for (int v = 0; v < problem.villages.size(); v++) {
            int food_need = problem.villages[v].population * 9;
            int other_need = problem.villages[v].population;
            
            int food_satisfied = min(state.village_food_delivered[v], food_need);
            int other_satisfied = min(state.village_other_delivered[v], other_need);
            
            // Assume best case: all food is perishable
            total_value += food_satisfied * problem.packages[1].value;
            total_value += other_satisfied * problem.packages[2].value;
        }
        
        double total_cost = 0;
        for (int h = 0; h < problem.helicopters.size(); h++) {
            for (const auto& trip : state.helicopter_plans[h]) {
                if (!trip.drops.empty()) {
                    double dist = calculateTripDistance(problem, problem.helicopters[h], trip);
                    total_cost += problem.helicopters[h].fixed_cost + 
                                 problem.helicopters[h].alpha * dist;
                }
            }
        }
        
        return total_value - total_cost;
    }
    
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
    
    double calculateHeuristic(const State& state) {
        // Admissible heuristic: optimistic estimate of remaining value
        double remaining_value = 0;
        
        for (int v = 0; v < problem.villages.size(); v++) {
            int food_need = problem.villages[v].population * 9;
            int other_need = problem.villages[v].population;
            
            int food_remaining = max(0, food_need - state.village_food_delivered[v]);
            int other_remaining = max(0, other_need - state.village_other_delivered[v]);
            
            // Optimistic: all remaining food is perishable, no delivery cost
            remaining_value += food_remaining * problem.packages[1].value;
            remaining_value += other_remaining * problem.packages[2].value;
        }
        
        // Optimistic cost estimate (minimum possible)
        double min_cost = 0;
        for (int h = 0; h < problem.helicopters.size(); h++) {
            double remaining_dist = problem.d_max - state.helicopter_distances[h];
            if (remaining_dist > 0) {
                // At least one more trip if we have distance left
                min_cost += problem.helicopters[h].fixed_cost * 0.5; // Optimistic
            }
        }
        
        return remaining_value - min_cost;
    }
    
    vector<State> generateSuccessors(const State& current) {
        vector<State> successors;
        
        // Try adding a new trip for each helicopter
        for (int h = 0; h < problem.helicopters.size(); h++) {
            const Helicopter& heli = problem.helicopters[h];
            
            if (current.helicopter_distances[h] >= problem.d_max * 0.95) continue;
            
            // Try different village combinations (limit for performance)
            vector<int> unserved_villages;
            for (int v = 0; v < problem.villages.size(); v++) {
                int food_need = problem.villages[v].population * 9;
                int other_need = problem.villages[v].population;
                
                if (current.village_food_delivered[v] < food_need ||
                    current.village_other_delivered[v] < other_need) {
                    unserved_villages.push_back(v);
                }
            }
            
            if (unserved_villages.empty()) continue;
            
            // Generate a few promising trips
            for (int i = 0; i < min(3, (int)unserved_villages.size()); i++) {
                State next = current;
                Trip new_trip;
                
                int village_idx = unserved_villages[i];
                const Village& village = problem.villages[village_idx];
                
                // Calculate delivery amounts
                int food_need = village.population * 9 - current.village_food_delivered[village_idx];
                int other_need = village.population - current.village_other_delivered[village_idx];
                
                food_need = max(0, food_need);
                other_need = max(0, other_need);
                
                // Create a simple single-village trip
                Drop drop;
                drop.village_id = village.id;
                
                // Prefer perishable food
                double weight_limit = heli.weight_capacity;
                drop.perishable_food = min((int)(weight_limit / problem.packages[1].weight), food_need);
                weight_limit -= drop.perishable_food * problem.packages[1].weight;
                
                drop.dry_food = min((int)(weight_limit / problem.packages[0].weight), 
                                   max(0, food_need - drop.perishable_food));
                weight_limit -= drop.dry_food * problem.packages[0].weight;
                
                drop.other_supplies = min((int)(weight_limit / problem.packages[2].weight), other_need);
                
                if (drop.dry_food + drop.perishable_food + drop.other_supplies > 0) {
                    new_trip.drops.push_back(drop);
                    new_trip.dry_food_pickup = drop.dry_food;
                    new_trip.perishable_food_pickup = drop.perishable_food;
                    new_trip.other_supplies_pickup = drop.other_supplies;
                    
                    double trip_dist = calculateTripDistance(problem, heli, new_trip);
                    
                    if (trip_dist <= heli.distance_capacity &&
                        next.helicopter_distances[h] + trip_dist <= problem.d_max) {
                        
                        next.helicopter_plans[h].push_back(new_trip);
                        next.helicopter_distances[h] += trip_dist;
                        next.village_food_delivered[village_idx] += drop.dry_food + drop.perishable_food;
                        next.village_other_delivered[village_idx] += drop.other_supplies;
                        
                        // Update costs
                        double trip_cost = heli.fixed_cost + heli.alpha * trip_dist;
                        double trip_value = drop.perishable_food * problem.packages[1].value +
                                          drop.dry_food * problem.packages[0].value +
                                          drop.other_supplies * problem.packages[2].value;
                        
                        next.g_cost = current.g_cost + (trip_value - trip_cost);
                        next.h_cost = calculateHeuristic(next);
                        next.f_cost = next.g_cost + next.h_cost;
                        
                        successors.push_back(next);
                    }
                }
            }
        }
        
        return successors;
    }
    
public:
    AStarSolver(const ProblemData& p) : problem(p) {}
    
    Solution solve(double time_limit_seconds) {
        auto start_time = steady_clock::now();
        
        priority_queue<State> open_set;
        unordered_map<string, double> closed_set; // Simple state hash
        
        State initial(problem.helicopters.size(), problem.villages.size());
        initial.h_cost = calculateHeuristic(initial);
        initial.f_cost = initial.h_cost;
        open_set.push(initial);
        
        State best_state = initial;
        double best_value = -1e9;
        int expansions = 0;
        
        while (!open_set.empty()) {
            auto current_time = steady_clock::now();
            double elapsed = duration_cast<duration<double>>(current_time - start_time).count();
            
            if (elapsed >= time_limit_seconds) {
                cout << "A* time limit reached after " << expansions << " expansions" << endl;
                break;
            }
            
            State current = open_set.top();
            open_set.pop();
            expansions++;
            
            double current_value = calculateStateValue(current);
            if (current_value > best_value) {
                best_value = current_value;
                best_state = current;
                cout << "A* found better solution: " << best_value << " (expansion " << expansions << ")" << endl;
            }
            
            // Generate and evaluate successors
            vector<State> successors = generateSuccessors(current);
            for (const auto& next : successors) {
                open_set.push(next);
            }
            
            // Limit search space
            if (expansions > 10000) {
                cout << "A* reached expansion limit" << endl;
                break;
            }
        }
        
        // Convert best state to solution
        Solution solution;
        for (int h = 0; h < problem.helicopters.size(); h++) {
            HelicopterPlan plan;
            plan.helicopter_id = h + 1;
            plan.trips = best_state.helicopter_plans[h];
            solution.push_back(plan);
        }
        
        return solution;
    }
};

// Main solve function using A* Search
Solution solve(const ProblemData& problem) {
    cout << "Starting A* Search solver..." << endl;
    
    double time_limit = problem.time_limit_minutes * 60.0 * 0.95;
    
    AStarSolver solver(problem);
    Solution solution = solver.solve(time_limit);
    
    cout << "A* Search completed" << endl;
    return solution;
}