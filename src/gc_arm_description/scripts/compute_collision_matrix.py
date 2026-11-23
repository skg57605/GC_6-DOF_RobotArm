#!/usr/bin/env python3
"""SRDF disable_collisions 산출 (MoveIt Setup Assistant와 같은 방식).

관절 한계 내 무작위 자세를 샘플링해 링크 쌍마다 충돌 비율을 구한다.
  Adjacent: 관절로 직접 연결된 링크 → 제외
  Default : 95% 이상 항상 접촉 → 제외
  Never   : 샘플 중 한 번도 접촉하지 않음 → 제외
  나머지  : 실제로 충돌 가능 → 검사 유지

필요: pip install trimesh python-fcl numpy
  xacro src/gc_arm_description/urdf/gc_arm.urdf.xacro > /tmp/gc_arm.urdf
  python3 src/gc_arm_description/scripts/compute_collision_matrix.py /tmp/gc_arm.urdf --samples 20000
"""

import argparse
import itertools
import math
import os
import random
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def origin_tf(o):
    M = np.eye(4)
    if o is not None:
        M[:3, :3] = rpy_matrix(*[float(v) for v in (o.get("rpy") or "0 0 0").split()])
        M[:3, 3] = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
    return M


def rot_z(q):
    M = np.eye(4)
    M[:2, :2] = [[math.cos(q), -math.sin(q)], [math.sin(q), math.cos(q)]]
    return M


def load_geometry(c):
    g = c.find("geometry")[0]
    if g.tag == "mesh":
        path = g.get("filename").replace("package://gc_arm_description", PKG)
        m = trimesh.load(path)
        m.apply_scale(float((g.get("scale") or "1 1 1").split()[0]))
        return m
    if g.tag == "box":
        return trimesh.creation.box(extents=[float(v) for v in g.get("size").split()])
    if g.tag == "cylinder":
        return trimesh.creation.cylinder(radius=float(g.get("radius")), height=float(g.get("length")))
    if g.tag == "sphere":
        return trimesh.creation.icosphere(radius=float(g.get("radius")))
    raise ValueError(g.tag)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf")
    ap.add_argument("--samples", type=int, default=20000)
    args = ap.parse_args()

    root = ET.parse(args.urdf).getroot()
    joints = [j for j in root.findall("joint") if j.find("parent") is not None]
    movable = [j for j in joints if j.get("type") in ("revolute", "continuous", "prismatic")]
    limits = [(float(j.find("limit").get("lower")), float(j.find("limit").get("upper"))) for j in movable]
    adjacent = {frozenset((j.find("parent").get("link"), j.find("child").get("link"))) for j in joints}

    manager = trimesh.collision.CollisionManager()
    offsets = {}
    for link in root.findall("link"):
        c = link.find("collision")
        if c is not None:
            manager.add_object(link.get("name"), load_geometry(c))
            offsets[link.get("name")] = origin_tf(c.find("origin"))

    def frames(q):
        qmap = {j.get("name"): v for j, v in zip(movable, q)}
        F = {root.find("link").get("name"): np.eye(4)}
        for j in joints:
            F[j.find("child").get("link")] = (F[j.find("parent").get("link")] @ origin_tf(j.find("origin"))
                                              @ rot_z(qmap.get(j.get("name"), 0.0)))
        return F

    links = sorted(offsets)
    hits = {frozenset(p): 0 for p in itertools.combinations(links, 2)}
    random.seed(0)
    for _ in range(args.samples):
        F = frames([random.uniform(lo, hi) for lo, hi in limits])
        for name in links:
            manager.set_transform(name, F[name] @ offsets[name])
        _, names = manager.in_collision_internal(return_names=True)
        for a, b in names:
            hits[frozenset((a, b))] += 1

    print(f"<!-- {args.samples} samples -->")
    for pair in sorted(hits, key=lambda p: sorted(p)):
        a, b = sorted(pair)
        frac = hits[pair] / args.samples
        reason = "Adjacent" if pair in adjacent else "Default" if frac > 0.95 else "Never" if hits[pair] == 0 else None
        if reason:
            print(f'  <disable_collisions link1="{a}" link2="{b}" reason="{reason}"/>')
        else:
            print(f"  <!-- check: {a} - {b} ({frac * 100:.2f}%) -->")


if __name__ == "__main__":
    main()
