#!/usr/bin/env python3
# mesh_height_to_sdf.py — snap Poisson (x,y) to DAE terrain z and output SDF
# pip install trimesh shapely pyglet
import argparse, math, csv
import numpy as np
import trimesh

def load_terrain(mesh_path, scale=(17.5,17.5,17.5), pose=(0,0,0,0,0,0)):
    m = trimesh.load(mesh_path, force="mesh")
    m.apply_scale(scale)
    x, y, z, r, p, yy = pose  # only supporting translation + yaw=0 here; extend if needed
    m.apply_translation([x, y, z])
    return m

def z_at(mesh, x, y, z0=100.0):
    # cast ray down from above; return first hit z
    origins = np.array([[x, y, z0]])
    dirs    = np.array([[0, 0, -1]])
    loc, idx, dist = mesh.ray.intersects_location(origins, dirs, multiple_hits=False)
    if len(loc) == 0:
        return None
    return float(loc[0][2])

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mesh", required=True, help="path to Environment1.dae")
    ap.add_argument("--scale", default="15,15,15", help="sx,sy,sz from your model.sdf <scale>")
    ap.add_argument("--pose",  default="0,0,0,0,0,0", help="x,y,z,roll,pitch,yaw of terrain in world")
    ap.add_argument("--csv",   required=True, help="input CSV from poisson (type,name,x,y,...)")
    args = ap.parse_args()

    sx, sy, sz = [float(v) for v in args.scale.split(",")]
    px, py, pz, rr, pp, yy = [float(v) for v in args.pose.split(",")]
    mesh = load_terrain(args.mesh, scale=(sx, sy, sz), pose=(px, py, pz, rr, pp, yy))

    print("<!-- SDF includes snapped to mesh terrain -->")
    with open(args.csv) as f:
        r = csv.DictReader(f)
        for row in r:
            x = float(row["x"]); y = float(row["y"])
            z = z_at(mesh, x, y, z0=100.0)
            if z is None:
                z = 0.0  # fallback if ray misses (outside mesh); tweak if needed
            uri  = "model://tree1" if row["type"] == "tree1" else "model://tree2"
            name = row["name"]
            print(
                f'  <include><uri>{uri}</uri><name>{name}</name>'
                f'<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0.000000</pose></include>'
            )

if __name__ == "__main__":
    main()
