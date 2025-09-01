#include "solver.h"
#include <iostream>
#include <chrono>

#include <vector>
#include <numeric>
#include <algorithm>
#include <random>
#include <map>

using namespace std;

/**
 * @brief The main function to implement your search/optimization algorithm.
 * * This is a placeholder implementation. It creates a simple, likely invalid,
 * * plan to demonstrate how to build the Solution object. 
 * TODO: REPLACE THIS ENTIRE FUNCTION WITH YOUR ALGORITHM.
 */

Solution solve(const ProblemData& problem) {
    cout << "Starting solver..." << endl;

    Solution solution;

    // --- START OF PLACEHOLDER LOGIC ---
    


    // --- END OF PLACEHOLDER LOGIC ---

    cout << "Solver finished." << endl;
    return solution;
}


double route_distance(const ProblemData &P, int home_city_id, const vector<int> &route_village_ids) {
    Point cur = P.cities[home_city_id-1];
    double total = 0.0;
    for (int vid : route_village_ids) {
        // find village coords
        Point vpos = P.villages[vid-1].coords; // villages assumed 1-based id
        total += distance(cur, vpos);
        cur = vpos;
    }
    // return to home
    total += distance(cur, P.cities[home_city_id-1]);
    return total;
}

Score evaluate(const Solution &sol, const ProblemData &problem)
{
    using T = tuple<int, int, int>;
    unordered_map<int, T> delivered;

    double total_trip_costs = 0.0;
    double total_value = 0.0;

    for (const auto &hp : sol)
    {
        const Helicopter *hptr = nullptr;
        for (const auto &h : problem.helicopters)
        {
            if (h.id == hp.helicopter_id)
            {
                hptr = &h;
                break;
            }
        }

        double heli_used = 0.0;

        if (!hptr)
            continue;

        for (const auto &trip : hp.trips) {
            vector<int> route;
            for (const auto& d: trip.drops) {
                route.push_back(d.village_id);
                auto &entry = delivered[d.village_id];
                get<0>(entry) += d.dry_food;
                get<1>(entry) += d.perishable_food;
                get<2>(entry) += d.other_supplies;
            }
            double td = route_distance(problem, hptr->home_city_id, route);
            heli_used += td;
            total_trip_costs += (hptr->fixed_cost + hptr->alpha * td);
        }
    }

    for (const auto &v: problem.villages) {
        int dry = 0, wet = 0, others = 0;
        if (delivered.count(v.id)) {
            dry = get<0>(delivered[v.id]), wet = get<1>(delivered[v.id]), others = get<2>(delivered[v.id]);
        }
        int cap_meals = v.population * 9;
        int cap_others = v.population * 1;

        int used_wet = min(wet, cap_meals);
        int used_dry = min(cap_meals - used_wet, dry);
        int used_others = min(others, cap_others);

        total_value += used_dry * problem.packages[0].value 
                    + used_wet * problem.packages[1].value
                    + used_others * problem.packages[2].value;
    }
    return Score(total_value, total_trip_costs);
}

