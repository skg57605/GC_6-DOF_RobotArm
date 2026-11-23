#!/usr/bin/env python3
"""CAD(STL) 메시를 URDF 링크 프레임에 배치하기 위한 origin 계산.

CAD 조립품은 하나의 공통 좌표계로 내보내져 있고, 특정 관절 자세(config/meshes.yaml의 cad.pose_deg)에
놓여 있다. 각 링크 L에 대해

    T_L←cad = (T_base←L(q_cad))⁻¹ · T_base←cad

를 계산하면, 메시를 링크 L에 붙였을 때 CAD 조립 상태와 동일한 위치가 된다.
T_base←L은 gc_arm.macro.xacro와 같은 DH 체인(joints.yaml geometry)으로 계산한다.

    python3 src/gc_arm_description/scripts/compute_mesh_origins.py            # meshes.yaml 갱신
    python3 src/gc_arm_description/scripts/compute_mesh_origins.py --collision  # + 충돌용 볼록 껍질 생성 (trimesh 필요)
"""

import argparse
import math
import os
import re
import sys

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

LINK_MESHES = {  # URDF 링크 → visual STL
    "base_link": "Base.stl",
    "link1": "Link1.stl",
    "link2": "Link2.stl",
    "link3": "Link3.stl",
    "link4": "Link4.stl",
    "link5": "Link5.stl",
    "link6": "Flange.stl",
}


# ---------------------------------------------------------------- 4x4 행렬
def mat(R, t):
    return [R[0] + [t[0]], R[1] + [t[1]], R[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return mat([[1, 0, 0], [0, c, -s], [0, s, c]], [0, 0, 0])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return mat([[c, 0, s], [0, 1, 0], [-s, 0, c]], [0, 0, 0])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return mat([[c, -s, 0], [s, c, 0], [0, 0, 1]], [0, 0, 0])


def trans(x, y, z):
    return mat([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [x, y, z])


def origin(xyz, rpy):
    return mul(trans(*xyz), mul(rz(rpy[2]), mul(ry(rpy[1]), rx(rpy[0]))))


def inv(T):
    R = [[T[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(R[i][k] * T[k][3] for k in range(3)) for i in range(3)]
    return mat(R, t)


def to_xyz_rpy(T):
    """URDF rpy (R = Rz(y)·Ry(p)·Rx(r))"""
    pitch = math.asin(max(-1.0, min(1.0, -T[2][0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(T[2][1], T[2][2])
        yaw = math.atan2(T[1][0], T[0][0])
    else:  # 짐벌락
        roll = 0.0
        yaw = math.atan2(-T[0][1], T[1][1])
    clean = lambda v: 0.0 if abs(v) < 1e-12 else v  # noqa: E731
    return [clean(T[i][3]) for i in range(3)], [clean(roll), clean(pitch), clean(yaw)]


# ---------------------------------------------------------------- DH 체인 (gc_arm.macro.xacro와 동일)
def joint_origins(g):
    p = math.pi
    return [
        origin([0, 0, 0], [0, 0, 0]),                 # joint1
        origin([g["r"], 0, g["h"]], [p / 2, 0, 0]),   # joint2
        origin([g["a2"], 0, 0], [0, 0, 0]),           # joint3
        origin([0, 0, 0], [p / 2, 0, p / 2]),         # joint4 (J3 theta +90 포함)
        origin([0, 0, g["d4"]], [-p / 2, 0, 0]),      # joint5
        origin([0, 0, 0], [p / 2, 0, 0]),             # joint6
    ]


def link_frames(g, q):
    frames = {"base_link": trans(0, 0, 0)}
    T = trans(0, 0, 0)
    for i, Jo in enumerate(joint_origins(g)):
        T = mul(T, mul(Jo, rz(q[i])))
        frames[f"link{i + 1}"] = T
    return frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--collision", action="store_true", help="충돌용 볼록 껍질 STL 생성 (trimesh 필요)")
    args = ap.parse_args()

    joints_cfg = yaml.safe_load(open(os.path.join(PKG, "config", "joints.yaml")))
    meshes_path = os.path.join(PKG, "config", "meshes.yaml")
    meshes_cfg = yaml.safe_load(open(meshes_path))
    g = joints_cfg["geometry"]
    cad = meshes_cfg["cad"]
    s = cad["scale"]

    h_cad = (cad["j2_axis_z_mm"] - cad["base_bottom_z_mm"]) * s
    if abs(h_cad - g["h"]) > 1e-6:
        print(f"⚠️  joints.yaml geometry.h={g['h']} 와 CAD에서 계산한 h={h_cad:.4f} 가 다릅니다", file=sys.stderr)

    # CAD 좌표(스케일 적용 후, m) → base_link
    jx, jy = cad["j1_axis_xy_mm"]
    T_base_cad = mul(rz(math.radians(cad["yaw_deg"])),
                     trans(-jx * s, -jy * s, -cad["base_bottom_z_mm"] * s))

    q_cad = [math.radians(v) for v in cad["pose_deg"]]
    frames = link_frames(g, q_cad)

    lines = ["links:"]
    for link, stl in LINK_MESHES.items():
        if not os.path.exists(os.path.join(PKG, "meshes", "visual", stl)):
            print(f"  {link}: {stl} 없음 → 단순 도형 사용")
            continue
        xyz, rpy = to_xyz_rpy(mul(inv(frames[link]), T_base_cad))
        has_col = os.path.exists(os.path.join(PKG, "meshes", "collision", stl)) or args.collision
        lines.append(f"  {link}:")
        lines.append(f"    visual: {stl}")
        lines.append(f"    collision: {stl if has_col else 'null'}")
        lines.append(f"    xyz: [{', '.join(f'{v:.6f}' for v in xyz)}]")
        lines.append(f"    rpy: [{', '.join(f'{v:.6f}' for v in rpy)}]")
        print(f"  {link:9s} ← {stl:11s} xyz={[round(v, 4) for v in xyz]} rpy(deg)={[round(math.degrees(v), 2) for v in rpy]}")

    text = open(meshes_path).read()
    text = re.sub(r"(?ms)^links:.*\Z", "\n".join(lines) + "\n", text)
    open(meshes_path, "w").write(text)
    print(f"갱신: {meshes_path}")

    if args.collision:
        import trimesh  # noqa: PLC0415
        for stl in LINK_MESHES.values():
            src = os.path.join(PKG, "meshes", "visual", stl)
            if os.path.exists(src):
                hull = trimesh.load(src).convex_hull
                hull.export(os.path.join(PKG, "meshes", "collision", stl))
                print(f"  collision/{stl}: 볼록 껍질 {len(hull.faces)} faces")


if __name__ == "__main__":
    main()
