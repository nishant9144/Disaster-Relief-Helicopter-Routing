#!/usr/bin/env python3
"""
Visualize an instance and an assignment (mapping).

Usage:
    python visualize.py --inst inst1.txt --mapping mapping.txt --out viz.png
If mapping is omitted, we draw only zones/locations and flows.
"""
import argparse
import json
from pathlib import Path
import math

import matplotlib.pyplot as plt
import numpy as np

def read_instance(path):
    p = Path(path)
    meta = p.with_suffix(p.suffix + ".meta.json")
    meta_data = None
    if meta.exists():
        meta_data = json.load(meta.open())

    with p.open() as f:
        time = int(f.readline().strip())
        z = int(f.readline().strip())
        l = int(f.readline().strip())
        N = []
        for _ in range(z):
            row = list(map(int, f.readline().strip().split()))
            N.append(row)
        T = []
        for _ in range(l):
            row = list(map(int, f.readline().strip().split()))
            T.append(row)
    return {"time":time, "z":z, "l":l, "N":N, "T":T, "meta":meta_data}

def read_mapping(path, z):
    with open(path) as f:
        parts = f.read().strip().split()
        vals = [int(x) for x in parts]
    if len(vals) != z:
        raise ValueError("mapping length != z")
    return vals

def compute_cost(N, T, mapping):
    z = len(N)
    cost = 0
    for i in range(z):
        for j in range(z):
            cost += N[i][j] * T[mapping[i]-1][mapping[j]-1]
    return cost

def plot_instance(inst, mapping=None, out=None, show=True, max_flow_thickness=6.0):
    meta = inst.get("meta", {})
    zones = meta.get("zones")
    locs = meta.get("locations")

    # if metadata missing, place them on grid
    if zones is None:
        z = inst["z"]
        zones = [(i, i) for i in range(z)]
    if locs is None:
        l = inst["l"]
        locs = [(i, i+2) for i in range(l)]

    zones = np.array(zones)
    locs = np.array(locs)
    z = inst["z"]
    l = inst["l"]
    N = inst["N"]

    fig, ax = plt.subplots(figsize=(10,8))
    # plot locations (squares)
    ax.scatter(locs[:,0], locs[:,1], marker="s", s=160, label="locations")
    for idx,(x,y) in enumerate(locs):
        ax.text(x, y+1.5, f"L{idx+1}", ha="center", fontsize=9, fontweight="bold")

    # plot zones (circles)
    ax.scatter(zones[:,0], zones[:,1], marker="o", s=120, label="zones", color="tab:orange")
    for idx,(x,y) in enumerate(zones):
        ax.text(x, y-1.5, f"Z{idx+1}", ha="center", fontsize=9)

    # draw flows between zones (thin grey lines)
    all_flows = []
    for i in range(z):
        for j in range(z):
            if inst["N"][i][j] > 0:
                all_flows.append(inst["N"][i][j])
    if all_flows:
        max_flow = max(all_flows)
    else:
        max_flow = 1

    for i in range(z):
        for j in range(z):
            fval = inst["N"][i][j]
            if fval <= 0:
                continue
            x1,y1 = zones[i]
            x2,y2 = zones[j]
            linewidth = max(0.3, (fval / max_flow) * max_flow_thickness)
            ax.plot([x1,x2],[y1,y2], color="gray", linewidth=linewidth, alpha=0.5)

    # draw mapping if provided
    if mapping is not None:
        # mapping: list of ints (1-based)
        colors = plt.cm.get_cmap("tab10", z)
        for i in range(z):
            loc_idx = mapping[i]-1
            zx, zy = zones[i]
            lx, ly = locs[loc_idx]
            ax.annotate("", xy=(lx,ly), xytext=(zx,zy),
                        arrowprops=dict(arrowstyle="->", lw=1.2, color=colors(i%10)))
            ax.text((zx+lx)/2, (zy+ly)/2, f"{i+1}->{mapping[i]}", fontsize=7, color=colors(i%10))

        cost = compute_cost(inst["N"], inst["T"], mapping)
        ax.set_title(f"Instance visualization (cost = {cost})")
    else:
        ax.set_title("Instance visualization (no mapping provided)")

    ax.set_aspect("equal", "box")
    ax.legend()
    ax.grid(alpha=0.3)
    if out:
        plt.savefig(out, dpi=200)
        print("Saved visualization to", out)
    if show:
        plt.show()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--inst", required=True)
    parser.add_argument("--mapping", required=False)
    parser.add_argument("--out", required=False, help="output image file (png)")
    args = parser.parse_args()

    inst = read_instance(args.inst)
    mapping = None
    if args.mapping:
        mapping = read_mapping(args.mapping, inst["z"])

    plot_instance(inst, mapping=mapping, out=args.out)

if __name__ == "__main__":
    main()
