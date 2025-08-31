#include <fstream>
#include <iostream>
#include <bits/stdc++.h>

using namespace std;

#include "SportsLayout.h"

SportsLayout::SportsLayout(string inputfilename)
{
    readInInputFile(inputfilename);
    mapping = new int[z];
}

bool SportsLayout::check_output_format()
{

    vector<bool> visited(l, false);
    for (int i = 0; i < z; i++)
    {
        if ((mapping[i] >= 1 && mapping[i] <= l))
        {
            if (!visited[mapping[i] - 1])
                visited[mapping[i] - 1] = true;
            else
            {
                cout << "Repeated locations, check format\n";
                return false;
            }
        }
        else
        {
            cout << "Invalid location, check format\n";
            return false;
        }
    }

    return true;
}

// void SportsLayout::readOutputFile(string output_filename)
// {
//         fstream ipfile;
//         ipfile.open(output_filename, ios::in);
//         if (!ipfile) {
//             cout << "No such file\n";
//             exit( 0 );
//         }
//         else {

//             vector<int> ip;

//             while (1) {
//                 int t;
//                 ipfile >> t;
//                 ip.push_back(t);
//                 if (ipfile.eof())
//                     break;

//             }

//         if(ip.size()!=z)
//         {
//             cout<<"number of values not equal to number of zones, check output format\n";
//             exit(0);
//         }
//         for(int i=0;i<z;i++)
//         mapping[i]=ip[i];
//     ipfile.close();

//     if(!check_output_format())
//         exit(0);
//     cout<<"Read output file, format OK"<<endl;

//         }

// }

long long SportsLayout::cost_fn()
{

    long long cost = 0;

    for (int i = 0; i < z; i++)
    {
        for (int j = 0; j < z; j++)
        {
            cost += (long long)N[i][j] * (long long)T[mapping[i] - 1][mapping[j] - 1];
        }
    }

    return cost;
}

void SportsLayout::readInInputFile(string inputfilename)
{
    fstream ipfile;
    ipfile.open(inputfilename, ios::in);
    if (!ipfile)
    {
        cout << "No such file\n";
        exit(0);
    }
    else
    {

        ipfile >> time;
        ipfile >> z;
        ipfile >> l;

        if (z > l)
        {
            cout << "Number of zones more than locations, check format of input file\n";
            exit(0);
        }

        int **tempT;
        int **tempN;

        tempT = new int *[l];
        for (int i = 0; i < l; ++i)
            tempT[i] = new int[l];

        tempN = new int *[z];
        for (int i = 0; i < z; ++i)
            tempN[i] = new int[z];

        for (int i = 0; i < z; i++)
        {
            for (int j = 0; j < z; j++)
                ipfile >> tempN[i][j];
        }

        for (int i = 0; i < l; i++)
        {
            for (int j = 0; j < l; j++)
                ipfile >> tempT[i][j];
        }

        ipfile.close();

        T = tempT;
        N = tempN;
    }
}

void SportsLayout::write_to_file(string outputfilename)
{

    // Open the file for writing
    ofstream outputFile(outputfilename);

    // Check if the file is opened successfully
    if (!outputFile.is_open())
    {
        cerr << "Failed to open the file for writing." << std::endl;
        exit(0);
    }

    for (int i = 0; i < z; i++) {
        outputFile << mapping[i];
        if (i < z - 1)
            outputFile << " ";
    }

    // Close the file
    outputFile.close();

    cout << "Allocation written to the file successfully." << endl;
}

void SportsLayout::compute_allocation()
{
    // 1. SETUP: Timer and Random Number Generation
    auto start_time = chrono::high_resolution_clock::now();
    long long time_limit_seconds = time * 60 * 0.95; // Use 95% of time for safety

    // Modern C++ random number generation
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> distrib(0, z - 1);

    // 2. INITIAL SOLUTION: Create a random permutation
    vector<int> available_locations(l);
    for (int i = 0; i < l; ++i) {
        available_locations[i] = i + 1;
    }
    shuffle(available_locations.begin(), available_locations.end(), gen);

    for (int i = 0; i < z; ++i) {
        mapping[i] = available_locations[i];
    }
    
    long long current_cost = cost_fn();
    cout << "Initial random cost: " << current_cost << endl;

    // 3. LOCAL SEARCH LOOP (Hill Climbing)
    while (true) {
        // Check for timeout
        auto current_time = chrono::high_resolution_clock::now();
        auto elapsed_seconds = chrono::duration_cast<chrono::seconds>(current_time - start_time).count();
        if (elapsed_seconds >= time_limit_seconds) {
            break;
        }

        // Generate a neighbor by swapping two random zone assignments
        int zone1_idx = distrib(gen);
        int zone2_idx = distrib(gen);
        while (zone1_idx == zone2_idx) { // Ensure they are different
            zone2_idx = distrib(gen);
        }

        // Try the swap
        swap(mapping[zone1_idx], mapping[zone2_idx]);
        
        long long neighbor_cost = cost_fn();

        // Decision: If neighbor is better, keep it. Otherwise, revert the swap.
        if (neighbor_cost < current_cost) {
            current_cost = neighbor_cost; // Improvement found
        } else {
            swap(mapping[zone1_idx], mapping[zone2_idx]); // Revert to previous state
        }
    }
}

// void SportsLayout::compute_allocation() {
//     using namespace std::chrono;
//     // start timer
//     steady_clock::time_point t_start = steady_clock::now();
//     int time_limit_seconds = max(1, time); // if time is 0 or negative, we still allow at least 1 second

//     // --- 1) Build an initial feasible mapping: assign zones 0..z-1 to locations 1..z
//     // (simple start; guarantees distinct locations)
//     for (int i = 0; i < z; ++i) {
//         mapping[i] = i + 1; // locations are 1-based in this code
//     }

//     long long best_cost = cost_fn();
//     bool improved = true;

//     while (improved) {
//         improved = false;

//         // Quick time check
//         if (duration_cast<seconds>(steady_clock::now() - t_start).count() > time_limit_seconds)
//             break;

//         long long best_local_cost = best_cost;
//         int best_i = -1;
//         int best_j = -1;
//         int best_new_loc_for_i = -1;
//         bool best_is_swap = false;
//         bool time_exceeded = false;

//         // --- 2) Try all pairwise swaps (i < j): swap mapping[i] <-> mapping[j]
//         for (int i = 0; i < z && !time_exceeded; ++i) {
//             for (int j = i + 1; j < z && !time_exceeded; ++j) {
//                 std::swap(mapping[i], mapping[j]);
//                 long long c = cost_fn();
//                 if (c < best_local_cost) {
//                     best_local_cost = c;
//                     best_i = i;
//                     best_j = j;
//                     best_is_swap = true;
//                 }
//                 // undo swap
//                 std::swap(mapping[i], mapping[j]);

//                 // time check inside loops
//                 if (duration_cast<seconds>(steady_clock::now() - t_start).count() > time_limit_seconds)
//                     time_exceeded = true;
//             }
//         }

//         // --- 3) Try relocating a zone to an unused location (move to a location not currently assigned)
//         // Build used-location map
//         std::vector<bool> used(l, false);
//         for (int i = 0; i < z; ++i) used[mapping[i] - 1] = true;

//         for (int i = 0; i < z && !time_exceeded; ++i) {
//             int old_loc = mapping[i];
//             for (int loc = 0; loc < l && !time_exceeded; ++loc) {
//                 if (used[loc]) continue; // skip currently used locations (we only consider moves to unused locs)
//                 mapping[i] = loc + 1; // try assigning zone i to (loc+1)
//                 long long c = cost_fn();
//                 if (c < best_local_cost) {
//                     best_local_cost = c;
//                     best_i = i;
//                     best_j = -1;
//                     best_new_loc_for_i = loc + 1;
//                     best_is_swap = false;
//                 }
//                 // restore
//                 mapping[i] = old_loc;

//                 if (duration_cast<seconds>(steady_clock::now() - t_start).count() > time_limit_seconds)
//                     time_exceeded = true;
//             }
//         }

//         // If we found an improving move, apply it and continue
//         if (best_local_cost < best_cost) {
//             if (best_is_swap && best_i >= 0 && best_j >= 0) {
//                 std::swap(mapping[best_i], mapping[best_j]);
//             } else if (!best_is_swap && best_i >= 0 && best_new_loc_for_i != -1) {
//                 // update used map (not strictly necessary here, but consistent)
//                 mapping[best_i] = best_new_loc_for_i;
//             }
//             best_cost = best_local_cost;
//             improved = true;
//         }
//     } // end while
// }

// --- NEW FUNCTION TO ADD ---
long long SportsLayout::evaluate_sample_output(string sample_output_filename) {
    fstream ipfile;
    ipfile.open(sample_output_filename, ios::in);
    if (!ipfile) {
        cout << "Error: Could not open sample output file: " << sample_output_filename << endl;
        return -1;
    }

    int* sample_mapping = new int[z];
    for (int i = 0; i < z; ++i) {
        ipfile >> sample_mapping[i];
    }
    ipfile.close();

    int* original_mapping = this->mapping;
    this->mapping = sample_mapping;

    long long sample_cost = cost_fn();

    this->mapping = original_mapping;

    delete[] sample_mapping; 

    return sample_cost;
}

// void SportsLayout::compute_allocation()
// {
//     // Use the time read from the input as time limit in seconds.
//     using clock = std::chrono::steady_clock;
//     auto start_time = clock::now();
//     double time_limit = (double) time; // seconds

//     // Helper: compute full cost from an explicit mapping vector (1-based mapping as stored).
//     auto full_cost_vec = [&](const vector<int> &mapvec) -> long long {
//         long long c = 0;
//         for (int i = 0; i < z; ++i) {
//             int li = mapvec[i] - 1;
//             for (int j = 0; j < z; ++j) {
//                 int lj = mapvec[j] - 1;
//                 c += (long long)N[i][j] * (long long)T[li][lj];
//             }
//         }
//         return c;
//     };

//     // Initialize mapping vector (1-based) if not already
//     vector<int> cur_map(z, 1);
//     // Keep track of whether a location is used
//     vector<int> best_map_vec(z, 1);

//     // Precompute zone weights to order zones: row+col sums (importance)
//     vector<long long> zone_weight(z,0);
//     for (int i = 0; i < z; ++i) {
//         long long s = 0;
//         for (int j = 0; j < z; ++j) {
//             s += (long long)N[i][j] + (long long)N[j][i];
//         }
//         zone_weight[i] = s;
//     }

//     // GREEDY INITIALIZER
//     auto greedy_init = [&]() -> vector<int> {
//         vector<int> mapvec(z, -1); // 0-based loc indices for now
//         vector<bool> usedLoc(l, false);

//         // order zones by descending weight
//         vector<int> order(z);
//         iota(order.begin(), order.end(), 0);
//         sort(order.begin(), order.end(), [&](int a, int b){
//             return zone_weight[a] > zone_weight[b];
//         });

//         // assign first zone to the location that minimizes weighted distance sum (rough heuristic)
//         for (int idx = 0; idx < z; ++idx) {
//             int i = order[idx];
//             long long bestInc = LLONG_MAX;
//             int bestLoc = -1;
//             for (int loc = 0; loc < l; ++loc) {
//                 if (usedLoc[loc]) continue;
//                 long long inc = 0;
//                 // compute contributions with already assigned zones
//                 for (int jj = 0; jj < z; ++jj) {
//                     if (mapvec[jj] != -1) {
//                         int locj = mapvec[jj];
//                         inc += (long long)N[i][jj] * (long long)T[loc][locj];
//                         inc += (long long)N[jj][i] * (long long)T[locj][loc];
//                     }
//                 }
//                 // also add self-contribution
//                 inc += (long long)N[i][i] * (long long)T[loc][loc];
//                 if (inc < bestInc) {
//                     bestInc = inc;
//                     bestLoc = loc;
//                 }
//             }
//             // If no bestLoc found (shouldn't happen), pick the first unused
//             if (bestLoc == -1) {
//                 for (int loc = 0; loc < l; ++loc) if (!usedLoc[loc]) { bestLoc = loc; break; }
//             }
//             mapvec[i] = bestLoc;
//             usedLoc[bestLoc] = true;
//         }

//         // Convert to 1-based mapping required by class
//         vector<int> out(z);
//         for (int i = 0; i < z; ++i) out[i] = mapvec[i] + 1;
//         return out;
//     };

//     // Delta cost for swapping two zones a and b in current mapping (cur_map is 1-based)
//     auto delta_swap = [&](const vector<int> &mapvec, int a, int b) -> long long {
//         // mapvec: 1-based mapping
//         int locA = mapvec[a] - 1;
//         int locB = mapvec[b] - 1;
//         if (locA == locB) return 0;
//         long long delta = 0;
//         for (int k = 0; k < z; ++k) {
//             if (k == a || k == b) continue;
//             int locK = mapvec[k] - 1;
//             // contribution change for pairs (a,k) and (k,a)
//             delta += (long long)N[a][k] * ( (long long)T[locB][locK] - (long long)T[locA][locK] );
//             delta += (long long)N[k][a] * ( (long long)T[locK][locB] - (long long)T[locK][locA] );
//             // contribution change for pairs (b,k) and (k,b)
//             delta += (long long)N[b][k] * ( (long long)T[locA][locK] - (long long)T[locB][locK] );
//             delta += (long long)N[k][b] * ( (long long)T[locK][locA] - (long long)T[locK][locB] );
//         }
//         // pair (a,b) both directions
//         delta += (long long)N[a][b] * ( (long long)T[locB][locA] - (long long)T[locA][locB] );
//         delta += (long long)N[b][a] * ( (long long)T[locA][locB] - (long long)T[locB][locA] );
//         // self terms
//         delta += (long long)N[a][a] * ( (long long)T[locB][locB] - (long long)T[locA][locA] );
//         delta += (long long)N[b][b] * ( (long long)T[locA][locA] - (long long)T[locB][locB] );

//         return delta;
//     };

//     // SIMULATED ANNEALING (time-limited)
//     auto simulated_annealing = [&](vector<int> init_map) -> vector<int> {
//         vector<int> cur = init_map;
//         vector<int> best = init_map;
//         long long curCost = full_cost_vec(cur);
//         long long bestCost = curCost;

//         // SA parameters (tunable)
//         double T0 = 1e3;                       // initial temperature
//         double Tmin = 1e-3;
//         double alpha = 0.995;                  // cooling rate
//         double T = T0;

//         std::mt19937 rng((unsigned) std::chrono::high_resolution_clock::now().time_since_epoch().count());
//         uniform_int_distribution<int> zoneDist(0, z-1);
//         uniform_real_distribution<double> uni01(0.0, 1.0);

//         // Iterative improvement until time limit
//         auto now = clock::now();
//         while (std::chrono::duration<double>(now - start_time).count() < time_limit) {
//             // for each temperature, do some number of attempts
//             for (int iter = 0; iter < 30; ++iter) {
//                 int a = zoneDist(rng);
//                 int b = zoneDist(rng);
//                 if (z > 1) {
//                     while (b == a) b = zoneDist(rng);
//                 }
//                 long long d = delta_swap(cur, a, b);
//                 if (d < 0) {
//                     // accept
//                     swap(cur[a], cur[b]);
//                     curCost += d;
//                     if (curCost < bestCost) {
//                         bestCost = curCost;
//                         best = cur;
//                     }
//                 } else {
//                     double prob = exp(- (double)d / max(1e-9, T));
//                     if (uni01(rng) < prob) {
//                         swap(cur[a], cur[b]);
//                         curCost += d;
//                     }
//                 }
//                 // time check
//                 now = clock::now();
//                 if (std::chrono::duration<double>(now - start_time).count() >= time_limit) break;
//             }
//             T *= alpha;
//             if (T < Tmin) T = T0; // occasional reheating to escape local minima
//             now = clock::now();
//         }

//         return best;
//     };

//     // DFS Branch & Bound exact solver (for small z)
//     vector<int> bnb_best_map(z, 1);
//     long long bnb_best_cost = LLONG_MAX;
//     if (z <= 10) {
//         // prepare assignment order (largest-weight first)
//         vector<int> order(z);
//         iota(order.begin(), order.end(), 0);
//         sort(order.begin(), order.end(), [&](int a, int b){
//             return zone_weight[a] > zone_weight[b];
//         });

//         vector<int> assign_loc(z, -1); // assigned location index (0-based) or -1
//         vector<bool> usedLoc(l, false);

//         // partial cost accumulates all contributions between already-assigned zones
//         function<void(int, long long)> dfs_assign = [&](int depth, long long partialCost) {
//             // time prune
//             auto now = clock::now();
//             if (std::chrono::duration<double>(now - start_time).count() >= time_limit) return;

//             if (depth == z) {
//                 // complete assignment -> compare
//                 if (partialCost < bnb_best_cost) {
//                     bnb_best_cost = partialCost;
//                     for (int i = 0; i < z; ++i) bnb_best_map[i] = assign_loc[i] + 1;
//                 }
//                 return;
//             }

//             // simple pruning: if partialCost >= best found so far, prune
//             if (partialCost >= bnb_best_cost) return;

//             int zone = order[depth];
//             // try all free locations
//             for (int loc = 0; loc < l; ++loc) {
//                 if (usedLoc[loc]) continue;
//                 // compute incremental cost when assigning 'zone' -> loc with previously assigned zones
//                 long long inc = 0;
//                 // self term
//                 inc += (long long)N[zone][zone] * (long long)T[loc][loc];
//                 for (int pd = 0; pd < depth; ++pd) {
//                     int other_zone = order[pd];
//                     int loc_other = assign_loc[other_zone];
//                     // contributions both directions
//                     inc += (long long)N[zone][other_zone] * (long long)T[loc][loc_other];
//                     inc += (long long)N[other_zone][zone] * (long long)T[loc_other][loc];
//                 }
//                 long long newPartial = partialCost + inc;
//                 if (newPartial >= bnb_best_cost) continue; // prune
//                 // assign and recurse
//                 assign_loc[zone] = loc;
//                 usedLoc[loc] = true;
//                 dfs_assign(depth + 1, newPartial);
//                 // undo
//                 usedLoc[loc] = false;
//                 assign_loc[zone] = -1;

//                 // time check
//                 now = clock::now();
//                 if (std::chrono::duration<double>(now - start_time).count() >= time_limit) return;
//             }
//         };

//         // run BnB (with time limit)
//         dfs_assign(0, 0);
//     }

//     // Build greedy initial map
//     vector<int> greedy_map = greedy_init();
//     long long greedy_cost = full_cost_vec(greedy_map);

//     // Choose which solution to start from:
//     vector<int> start_map = greedy_map;
//     long long start_cost = greedy_cost;

//     if (z <= 10 && bnb_best_cost < LLONG_MAX) {
//         // If BnB found a solution (and likely exact), choose it as best
//         if (bnb_best_cost <= start_cost) {
//             start_map = bnb_best_map;
//             start_cost = bnb_best_cost;
//         }
//     }

//     // If z is small, also attempt SA for a short time to improve if possible.
//     // For larger instances, SA does most of the heavy lifting.
//     vector<int> sa_result = start_map;
//     {
//         // Reserve some time for finalization: keep 0.05*limit or min 0.5s
//         double reserved = max(0.5, time_limit * 0.05);
//         // shorten global time_limit a bit so SA doesn't run into the reserved window
//         double old_time = (double) time;
//         double allowed = max(0.1, max(0.5, time_limit - reserved));
//         // We temporarily adjust the class time for SA by modifying local start_time check:
//         // call simulated_annealing (it checks start_time and class time) — safe because we use start_time.
//         sa_result = simulated_annealing(start_map);
//     }

//     long long sa_cost = full_cost_vec(sa_result);

//     // Decide best among greedy, SA and BnB results
//     vector<pair<long long, vector<int>>> candidates;
//     candidates.push_back({greedy_cost, greedy_map});
//     candidates.push_back({sa_cost, sa_result});
//     if (z <= 10 && bnb_best_cost < LLONG_MAX) {
//         candidates.push_back({bnb_best_cost, bnb_best_map});
//     }

//     sort(candidates.begin(), candidates.end(), [](auto &a, auto &b){ return a.first < b.first; });

//     // pick best candidate
//     best_map_vec = candidates.front().second;
//     long long best_cost = candidates.front().first;

//     // copy back to class member mapping (mapping is int* and expects 1-based indices)
//     for (int i = 0; i < z; ++i) {
//         mapping[i] = best_map_vec[i];
//     }

//     // Print evaluation summary
//     cout << "Computed allocation. Best cost = " << best_cost << ". (Greedy=" << greedy_cost << ", SA=" << sa_cost;
//     if (z <= 10 && bnb_best_cost < LLONG_MAX) cout << ", BnB=" << bnb_best_cost;
//     cout << ")" << endl;
// }
