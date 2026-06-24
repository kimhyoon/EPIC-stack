#!/usr/bin/env python3
"""FAST-LIO PCD 정리 스크립트.

Voxel 다운샘플 -> Statistical Outlier 제거 -> (선택) Radius Outlier 제거 순으로
지저분한 맵을 깔끔하게 만든다.

사용 예:
    python3 clean_pcd.py                          # scans.pcd -> scans_clean.pcd (기본값)
    python3 clean_pcd.py scans.pcd out.pcd
    python3 clean_pcd.py -v 0.1 --std 1.5         # 실외용: 더 굵게, 더 공격적으로
    python3 clean_pcd.py --no-radius --show       # radius 필터 끄고 결과 미리보기

의존성:
    pip install open3d
"""
import argparse
import os
import sys

import open3d as o3d


def clean(in_path, out_path, voxel, nb_neighbors, std_ratio,
          use_radius, radius_points, radius, show):
    if not os.path.exists(in_path):
        sys.exit(f"[에러] 입력 파일이 없습니다: {in_path}")

    print(f"[1/4] 불러오는 중: {in_path}")
    pcd = o3d.io.read_point_cloud(in_path)
    n0 = len(pcd.points)
    if n0 == 0:
        sys.exit("[에러] 점이 0개입니다. PCD 형식을 확인하세요.")
    print(f"      원본 점 개수: {n0:,}")

    # 1) Voxel 다운샘플 — 중복 누적 점 제거 + 밀도 균일화
    if voxel > 0:
        pcd = pcd.voxel_down_sample(voxel_size=voxel)
        print(f"[2/4] Voxel({voxel}m) 다운샘플 후: {len(pcd.points):,}")
    else:
        print("[2/4] Voxel 다운샘플 건너뜀 (-v 0)")

    # 2) Statistical Outlier Removal — 허공에 떠다니는 노이즈 제거
    pcd, _ = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    print(f"[3/4] SOR(nb={nb_neighbors}, std={std_ratio}) 후: {len(pcd.points):,}")

    # 3) Radius Outlier Removal — 고립된 점 제거 (선택)
    if use_radius:
        pcd, _ = pcd.remove_radius_outlier(
            nb_points=radius_points, radius=radius)
        print(f"[4/4] Radius(pts={radius_points}, r={radius}m) 후: {len(pcd.points):,}")
    else:
        print("[4/4] Radius 필터 건너뜀 (--no-radius)")

    n1 = len(pcd.points)
    print(f"\n결과: {n0:,} -> {n1:,} ({100 * (n0 - n1) / n0:.1f}% 제거)")

    o3d.io.write_point_cloud(out_path, pcd)
    print(f"저장 완료: {out_path}")

    if show:
        print("미리보기 창을 닫으면 종료됩니다...")
        o3d.visualization.draw_geometries([pcd])


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    p = argparse.ArgumentParser(description="FAST-LIO PCD 정리 도구")
    p.add_argument("input", nargs="?", default=os.path.join(here, "scans.pcd"),
                   help="입력 .pcd (기본: 이 폴더의 scans.pcd)")
    p.add_argument("output", nargs="?", default=os.path.join(here, "scans_clean.pcd"),
                   help="출력 .pcd (기본: scans_clean.pcd)")
    p.add_argument("-v", "--voxel", type=float, default=0.05,
                   help="voxel 크기(m). 실내 0.03~0.05, 실외 0.1~0.2. 0이면 건너뜀 (기본 0.05)")
    p.add_argument("--nb", type=int, default=20,
                   help="SOR 이웃 점 개수 (기본 20, 밀도 낮으면 줄이기)")
    p.add_argument("--std", type=float, default=2.0,
                   help="SOR std 비율. 작을수록 공격적 (기본 2.0)")
    p.add_argument("--no-radius", action="store_true",
                   help="Radius outlier 필터 끄기")
    p.add_argument("--radius-points", type=int, default=16,
                   help="Radius 필터 최소 이웃 점 (기본 16)")
    p.add_argument("--radius", type=float, default=0.1,
                   help="Radius 필터 반경(m) (기본 0.1)")
    p.add_argument("--show", action="store_true", help="처리 후 결과 미리보기")
    a = p.parse_args()

    clean(a.input, a.output, a.voxel, a.nb, a.std,
          not a.no_radius, a.radius_points, a.radius, a.show)


if __name__ == "__main__":
    main()
