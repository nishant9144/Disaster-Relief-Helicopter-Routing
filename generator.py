#!/usr/bin/env python3
"""
Instance generator for SportsLayout assignment.

Writes an input file with the format your C++ code expects:
time
z
l
N (z x z matrix, row-major)
T (l x l matrix, row-major)

Additionally the script stores zone and location coordinates in a small JSON
sidecar file <out>.meta.json to allow visualization later.

Usage:
    python generator.py --z 8 --l 12 --time 5 --out inst1.txt --seed 42
"""

import argparse
import json
import math
import random
from pathlib import Path

import numpy as np

def euclid(a,b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

def generate_instance(z, l, time_limit, seed=None,
                      area=100.0, base_flow=50.0, decay=30.0,
                      noise_level=0.2):
    """
    Create:
      - l location coords uniformly in square [0, area]
      - z zone coords chosen close to some locations (so instances look realistic)
      - T: symmetric travel time = round(distance)
      - N: gravity-like flows between zones -> larger if zones are close
    """
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    # locations uniformly
    locs = [(random.random()*area, random.random()*area) for _ in range(l)]

    # zones: pick a subset of locations and jitter them, or random
    zones = []
    for i in range(z):
        base = locs[random.randrange(l)]
        jitter = (random.uniform(-area*0.05, area*0.05),
                  random.uniform(-area*0.05, area*0.05))
        zones.append((base[0]+jitter[0], base[1]+jitter[1]))

    # T: symmetric matrix, integer travel times
    T = [[0]*l for _ in range(l)]
    for i in range(l):
        for j in range(l):
            if i == j:
                T[i][j] = 0
            else:
                d = euclid(locs[i], locs[j])
                # scale and round to give integer travel times
                T[i][j] = int(round(d))

    # N: z x z flows (non-negative ints), gravity-like: base_flow * exp(-dist/decay)
    N = [[0]*z for _ in range(z)]
    for i in range(z):
        for j in range(z):
            if i == j:
                N[i][j] = 0
            else:
                dij = euclid(zones[i], zones[j])
                mean_flow = base_flow * math.exp(-dij/decay)
                # add asymmetry and noise
                v = np.random.poisson(max(0.1, mean_flow))
                # add small symmetric component
                noise = int(round(noise_level * base_flow * random.random()))
                N[i][j] = int(max(0, v + noise))
    return {
        "time": int(time_limit),
        "z": int(z),
        "l": int(l),
        "N": N,
        "T": T,
        "zones": zones,
        "locations": locs
    }

def write_instance(inst, out_path):
    out_path = Path(out_path)
    with out_path.open("w") as f:
        f.write(f"{inst['time']}\n")
        f.write(f"{inst['z']}\n")
        f.write(f"{inst['l']}\n")
        # N matrix (z x z)
        for i in range(inst['z']):
            row = inst["N"][i]
            f.write(" ".join(str(int(x)) for x in row) + "\n")
        # T matrix (l x l)
        for i in range(inst['l']):
            row = inst["T"][i]
            f.write(" ".join(str(int(x)) for x in row) + "\n")
    # also write metadata for visualization (coords)
    meta_path = out_path.with_suffix(out_path.suffix + ".meta.json")
    with meta_path.open("w") as fm:
        json.dump({"zones": inst["zones"], "locations": inst["locations"]}, fm, indent=2)

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--z", type=int, required=True)
    p.add_argument("--l", type=int, required=True)
    p.add_argument("--time", type=int, default=5, help="solver time limit (seconds)")
    p.add_argument("--out", required=True, help="output instance filename")
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--area", type=float, default=100.0, help="square area")
    p.add_argument("--base_flow", type=float, default=50.0)
    p.add_argument("--decay", type=float, default=30.0)
    args = p.parse_args()

    if args.l < args.z:
        raise SystemExit("Number of locations l must be >= number of zones z")

    inst = generate_instance(args.z, args.l, args.time, seed=args.seed,
                             area=args.area, base_flow=args.base_flow, decay=args.decay)
    write_instance(inst, args.out)
    print(f"Wrote instance to {args.out} and metadata to {args.out}.meta.json")

if __name__ == "__main__":
    main()
