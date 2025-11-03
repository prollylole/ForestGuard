#!/usr/bin/env python3
"""
poisson_trees.py — Poisson-disk tree placement with rounded-position uniqueness.

Key guarantees:
- No two points are closer than --min-sep (Bridson Poisson-disk).
- No two PRINTED positions are identical when rounded to --print-decimals.

Usage:
  python3 scripts/poisson_trees.py --n-good 25 --n-bad 3 --min-sep 3.0 \
    --xmin -9 --xmax 9 --ymin -9 --ymax 9 \
    --spawn-x 0 --spawn-y 0 --excl-radius 1.5 \
    --seed 424242 --print-decimals 3 --format sdf > /tmp/trees.sdf.inc
"""
import argparse, math, random, sys

def bridson_poisson(n_target, xmin, xmax, ymin, ymax, r, rng, k=25,
                    spawn_xy=None, excl_radius=0.0):
    """2D Poisson-disk sampling (Bridson). Returns up to n_target points."""
    w, h = xmax - xmin, ymax - ymin
    if w <= 0 or h <= 0:
        raise ValueError("Invalid bounds")
    cell = r / math.sqrt(2)
    cols = int(math.ceil(w / cell))
    rows = int(math.ceil(h / cell))
    grid = [[-1]*cols for _ in range(rows)]
    samples, active = [], []

    def in_bounds(x, y):
        return xmin <= x <= xmax and ymin <= y <= ymax

    def excluded(x, y):
        if not spawn_xy or excl_radius <= 0:
            return False
        sx, sy = spawn_xy
        return (x - sx)**2 + (y - sy)**2 <= excl_radius*excl_radius

    def too_close(x, y):
        gx = int((x - xmin) / cell)
        gy = int((y - ymin) / cell)
        for j in range(max(0, gy-2), min(rows, gy+3)):
            for i in range(max(0, gx-2), min(cols, gx+3)):
                idx = grid[j][i]
                if idx != -1:
                    sx, sy = samples[idx]
                    if (x - sx)**2 + (y - sy)**2 < r*r:
                        return True
        return False

    # Seed
    for _ in range(2000):
        x0 = rng.uniform(xmin, xmax)
        y0 = rng.uniform(ymin, ymax)
        if excluded(x0, y0) or too_close(x0, y0):
            continue
        samples.append((x0, y0))
        gx0 = int((x0 - xmin) / cell); gy0 = int((y0 - ymin) / cell)
        grid[gy0][gx0] = 0
        active.append(0)
        break

    # Grow
    while active and len(samples) < n_target:
        idx = rng.choice(active)
        sx, sy = samples[idx]
        found = False
        for _ in range(k):
            rho = rng.uniform(r, 2*r)
            ang = rng.uniform(0, 2*math.pi)
            x = sx + rho*math.cos(ang)
            y = sy + rho*math.sin(ang)
            if not in_bounds(x, y): continue
            if excluded(x, y):      continue
            if too_close(x, y):     continue
            samples.append((x, y))
            gx = int((x - xmin) / cell); gy = int((y - ymin) / cell)
            grid[gy][gx] = len(samples)-1
            active.append(len(samples)-1)
            found = True
            break
        if not found:
            active.remove(idx)
    return samples

def top_up_unique(n_target, pts, xmin, xmax, ymin, ymax, r, rng,
                  spawn_xy, excl_radius, print_decimals, max_tries=50000):
    """
    Ensure we have exactly n_target *unique* printed positions.
    Uses rejection sampling to top up, respecting Poisson min distance AND
    unique keys after rounding to `print_decimals`.
    """
    # Existing rounded keys
    def key(x,y): return (round(x, print_decimals), round(y, print_decimals))
    keys = set(key(x,y) for (x,y) in pts)
    out = list(pts)

    def valid(x, y):
        # Poisson min distance against current 'out'
        rr = r*r
        for sx, sy in out:
            if (x - sx)**2 + (y - sy)**2 < rr:
                return False
        # Spawn exclusion
        if spawn_xy and excl_radius > 0:
            sx, sy = spawn_xy
            if (x - sx)**2 + (y - sy)**2 <= excl_radius*excl_radius:
                return False
        # Bounds and unique rounded position
        if not (xmin <= x <= xmax and ymin <= y <= ymax):
            return False
        if key(x,y) in keys:
            return False
        return True

    tries = 0
    while len(out) < n_target and tries < max_tries:
        x = rng.uniform(xmin, xmax)
        y = rng.uniform(ymin, ymax)
        if valid(x, y):
            out.append((x, y))
            keys.add(key(x,y))
        tries += 1

    # If we still somehow fell short, relax very slightly by nudging with tiny jitter
    # (extremely rare; only happens if bounds are too tight vs min-sep/rounding).
    eps = 10**(-print_decimals) * 0.25
    while len(out) < n_target and tries < max_tries*2:
        x = rng.uniform(xmin, xmax)
        y = rng.uniform(ymin, ymax)
        # tiny nudge to avoid rounded collisions
        x += rng.uniform(-eps, eps)
        y += rng.uniform(-eps, eps)
        if valid(x, y):
            out.append((x, y))
            keys.add(key(x,y))
        tries += 1

    return out

def as_sdf_includes(pts, z=0.0, n_good=0, print_decimals=3):
    lines = []
    fmt = f"{{:.{print_decimals}f}}"
    for i, (x, y) in enumerate(pts):
        if i < n_good:
            name = f"tree1_{i}"; uri = "model://tree1"
        else:
            name = f"tree2_{i-n_good}"; uri = "model://tree2"
        yaw = 0.0
        xs, ys, zs, yaws = fmt.format(x), fmt.format(y), fmt.format(z), f"{yaw:.6f}"
        lines.append(
            f'    <include><uri>{uri}</uri><name>{name}</name>'
            f'<pose>{xs} {ys} {zs} 0 0 {yaws}</pose></include>'
        )
    return "\n".join(lines)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n-good", type=int, default=25)
    ap.add_argument("--n-bad",  type=int, default=3)
    ap.add_argument("--min-sep", type=float, default=3.0, help="minimum spacing (m)")
    ap.add_argument("--xmin", type=float, default=-9.0)
    ap.add_argument("--xmax", type=float, default= 9.0)
    ap.add_argument("--ymin", type=float, default=-9.0)
    ap.add_argument("--ymax", type=float, default= 9.0)
    ap.add_argument("--seed", type=int, default=424242)
    ap.add_argument("--spawn-x", type=float, default=0.0)
    ap.add_argument("--spawn-y", type=float, default=0.0)
    ap.add_argument("--excl-radius", type=float, default=1.5, help="keep trees away from spawn (m)")
    ap.add_argument("--z", type=float, default=0.0, help="tree Z in world coords")
    ap.add_argument("--format", choices=["sdf","csv"], default="sdf")
    ap.add_argument("--print-decimals", type=int, default=3, help="rounding shown in output")
    args = ap.parse_args()

    total = args.n_good + args.n_bad
    rng = random.Random(args.seed)

    # First pass via Bridson
    pts = bridson_poisson(
        n_target=total,
        xmin=args.xmin, xmax=args.xmax,
        ymin=args.ymin, ymax=args.ymax,
        r=args.min_sep,
        rng=rng,
        spawn_xy=(args.spawn_x, args.spawn_y),
        excl_radius=args.excl_radius
    )

    # Top up & enforce uniqueness after rounding
    pts = top_up_unique(
        n_target=total,
        pts=pts,
        xmin=args.xmin, xmax=args.xmax,
        ymin=args.ymin, ymax=args.ymax,
        r=args.min_sep,
        rng=rng,
        spawn_xy=(args.spawn_x, args.spawn_y),
        excl_radius=args.excl_radius,
        print_decimals=args.print_decimals
    )

    if len(pts) < total:
        sys.exit(f"Could not place {total} unique trees with the given bounds/min-sep/rounding. "
                 f"Placed {len(pts)}. Try reducing --min-sep or expanding bounds.")

    # Stable order (don’t reshuffle): first n_good printed as tree1_*, rest as tree2_*
    if args.format == "csv":
        fmt = f"{{:.{args.print_decimals}f}}"
        print("type,name,x,y,z,yaw")
        for i,(x,y) in enumerate(pts):
            if i < args.n_good:
                t,name="tree1",f"tree1_{i}"
            else:
                t,name="tree2",f"tree2_{i-args.n_good}"
            print(f"{t},{name},{fmt.format(x)},{fmt.format(y)},{fmt.format(args.z)},0.0")
    else:
        print("<!-- Paste these inside <world> after forest_env include -->")
        print(as_sdf_includes(pts, z=args.z, n_good=args.n_good, print_decimals=args.print_decimals))

if __name__ == "__main__":
    main()
