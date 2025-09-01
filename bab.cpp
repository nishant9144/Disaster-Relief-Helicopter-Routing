#include "bab.h"
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
#include <stack>

using namespace std;
using namespace std::chrono;

// ============================================================================
// APPROACH 2: BRANCH AND BOUND WITH DEPTH-FIRST SEARCH
// ============================================================================

struct BBNode {
    vector<vector<Trip>> helicopter_plans;
    vector<double> helicopter_distances;
    vector<int> village_food_delivered;
    vector<int> village_other_delivered;
    double current_value;
    double upper_bound;
    int depth;
    
    BBNode(int num_helis, int num_villages) {
        helicopter_plans.resize(num_helis);
        helicopter_distances.resize(num_helis, 0.0);
        village_food_delivered.resize(num_villages, 0);
        village_other_delivered.resize(num_villages, 0);
        current_value = 0;
        upper_bound = 0;
        depth = 0;
    }
};

class BranchAndBoundSolver {
private:
    const ProblemData& problem;
    Solution best_solution;
    double best_value;
    int nodes_explored;
    
    double calculateNodeValue(const BBNode& node) {
        double total_value = 0;
        
        // Calculate delivered value
        for (int v = 0; v < problem.villages.size(); v++) {
            int food_need = problem.villages[v].population * 9;
            int other_need = problem.villages[v].population;
            
            int food_satisfied = min(node.village_food_delivered[v], food_need);
            int other_satisfied = min(node.village_other_delivered[v], other_need);
            
            // Assuming perishable food for simplicity (upper bound)
            total_value += food_satisfied * problem.packages[1].value;
            total_value += other_satisfied * problem.packages[2].value;
        }
        
        // Calculate costs
        double total_cost = 0;
        for (int h = 0; h < problem.helicopters.size(); h++) {
            for (const auto& trip : node.helicopter_plans[h]) {
                if (!trip.drops.empty()) {
                    double dist = calculateTripDistance(problem.helicopters[h], trip);
                    total_cost += problem.helicopters[h].fixed_cost + 
                                 problem.helicopters[h].alpha * dist;
                }
            }
        }
        
        return total_value - total_cost;
    }
    
    double calculateUpperBound(const BBNode& node) {
        // Optimistic bound: assume we can deliver everything remaining with minimal cost
        double bound = node.current_value;
        
        for (int v = 0; v < problem.villages.size(); v++) {
            int food_need = problem.villages[v].population * 9;
            int other_need = problem.villages[v].population;
            
            int food_remaining = max(0, food_need - node.village_food_delivered[v]);
            int other_remaining = max(0, other_need - node.village_other_delivered[v]);
            
            // Optimistic: all food is perishable, minimal delivery cost
            bound += food_remaining * problem.packages[1].value;
            bound += other_remaining * problem.packages[2].value;
        }
        
        // Subtract minimal estimated cost for remaining deliveries
        double min_remaining_cost = 0;
        for (int h = 0; h < problem.helicopters.size(); h++) {
            double remaining_dist = problem.d_max - node.helicopter_distances[h];
            if (remaining_dist > 20) { // If significant distance remains
                // Estimate at least one more trip
                min_remaining_cost += problem.helicopters[h].fixed_cost * 0.3;
            }
        }
        
        return bound - min_remaining_cost;
    }
    
    double calculateTripDistance(const Helicopter& heli, const Trip& trip) {
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
    
    vector<BBNode> generateChildren(const BBNode& parent) {
        vector<BBNode> children;
        
        // Try adding trips for each helicopter
        for (int h = 0; h < problem.helicopters.size(); h++) {
            const Helicopter& heli = problem.helicopters[h];
            
            if (parent.helicopter_distances[h] >= problem.d_max * 0.98) continue;
            
            // Find villages that need supplies
            vector<pair<int, double>> village_priorities;
            Point home = problem.cities[heli.home_city_id - 1];
            
            for (int v = 0; v < problem.villages.size(); v++) {
                int food_need = problem.villages[v].population * 9;
                int other_need = problem.villages[v].population;
                
                int food_remaining = max(0, food_need - parent.village_food_delivered[v]);
                int other_remaining = max(0, other_need - parent.village_other_delivered[v]);
                
                if (food_remaining > 0 || other_remaining > 0) {
                    double priority = (food_remaining * problem.packages[1].value + 
                                     other_remaining * problem.packages[2].value) /
                                    (distance(home, problem.villages[v].coords) + 1);
                    village_priorities.push_back({v, priority});
                }
            }
            
            // Sort by priority
            sort(village_priorities.begin(), village_priorities.end(),
                 [](const auto& a, const auto& b) { return a.second > b.second; });
            
            // Generate children for top villages
            int max_children = min(3, (int)village_priorities.size());
            for (int i = 0; i < max_children; i++) {
                int v_idx = village_priorities[i].first;
                const Village& village = problem.villages[v_idx];
                
                BBNode child = parent;
                child.depth = parent.depth + 1;
                
                // Create a trip to this village
                Trip new_trip;
                Drop drop;
                drop.village_id = village.id;
                
                int food_need = village.population * 9 - parent.village_food_delivered[v_idx];
                int other_need = village.population - parent.village_other_delivered[v_idx];
                
                food_need = max(0, food_need);
                other_need = max(0, other_need);
                
                // Calculate optimal package mix
                double weight_limit = heli.weight_capacity;
                
                // Prioritize perishable food
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
                    
                    double trip_dist = calculateTripDistance(heli, new_trip);
                    
                    if (trip_dist <= heli.distance_capacity &&
                        child.helicopter_distances[h] + trip_dist <= problem.d_max) {
                        
                        child.helicopter_plans[h].push_back(new_trip);
                        child.helicopter_distances[h] += trip_dist;
                        child.village_food_delivered[v_idx] += drop.dry_food + drop.perishable_food;
                        child.village_other_delivered[v_idx] += drop.other_supplies;
                        
                        child.current_value = calculateNodeValue(child);
                        child.upper_bound = calculateUpperBound(child);
                        
                        children.push_back(child);
                    }
                }
            }
        }
        
        // Also consider stopping (no more trips)
        if (parent.depth > 0) {
            BBNode stop_node = parent;
            stop_node.depth = parent.depth + 1;
            stop_node.upper_bound = stop_node.current_value; // No more improvement possible
            children.push_back(stop_node);
        }
        
        return children;
    }
    
    void updateBestSolution(const BBNode& node) {
        if (node.current_value > best_value) {
            best_value = node.current_value;
            best_solution.clear();
            
            for (int h = 0; h < problem.helicopters.size(); h++) {
                HelicopterPlan plan;
                plan.helicopter_id = h + 1;
                plan.trips = node.helicopter_plans[h];
                best_solution.push_back(plan);
            }
            
            cout << "Branch & Bound found better solution: " << best_value 
                 << " (node " << nodes_explored << ")" << endl;
        }
    }
    
public:
    BranchAndBoundSolver(const ProblemData& p) : problem(p), best_value(-1e9), nodes_explored(0) {}
    
    Solution solve(double time_limit_seconds) {
        auto start_time = steady_clock::now();
        
        stack<BBNode> dfs_stack;
        
        // Initialize root node
        BBNode root(problem.helicopters.size(), problem.villages.size());
        root.upper_bound = calculateUpperBound(root);
        dfs_stack.push(root);
        
        while (!dfs_stack.empty()) {
            auto current_time = steady_clock::now();
            double elapsed = duration_cast<duration<double>>(current_time - start_time).count();
            
            if (elapsed >= time_limit_seconds) {
                cout << "Branch & Bound time limit reached after exploring " 
                     << nodes_explored << " nodes" << endl;
                break;
            }
            
            BBNode current = dfs_stack.top();
            dfs_stack.pop();
            nodes_explored++;
            
            // Pruning: skip if upper bound is worse than current best
            if (current.upper_bound <= best_value) {
                continue;
            }
            
            // Update best solution if current is better
            updateBestSolution(current);
            
            // Generate children and add to stack
            vector<BBNode> children = generateChildren(current);
            
            // Sort children by upper bound (best first)
            sort(children.begin(), children.end(),
                 [](const BBNode& a, const BBNode& b) { return a.upper_bound > b.upper_bound; });
            
            // Add children to stack (reverse order for DFS with best-first)
            for (auto it = children.rbegin(); it != children.rend(); ++it) {
                if (it->upper_bound > best_value) {
                    dfs_stack.push(*it);
                }
            }
            
            // Depth limit to prevent too deep exploration
            if (current.depth > 50) {
                continue;
            }
        }
        
        cout << "Branch & Bound explored " << nodes_explored << " nodes" << endl;
        cout << "Final best value: " << best_value << endl;
        
        return best_solution;
    }
};

// Main solve function using Branch and Bound
Solution solve(const ProblemData& problem) {
    cout << "Starting Branch and Bound solver..." << endl;
    
    double time_limit = problem.time_limit_minutes * 60.0 * 0.95;
    
    BranchAndBoundSolver solver(problem);
    Solution solution = solver.solve(time_limit);
    
    cout << "Branch and Bound completed" << endl;
    return solution;
}