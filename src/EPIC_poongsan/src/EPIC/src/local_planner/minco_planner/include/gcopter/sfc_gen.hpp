/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/

#ifndef SFC_GEN_HPP
#define SFC_GEN_HPP

// #include <ompl/base/DiscreteMotionValidator.h>
// #include <ompl/base/SpaceInformation.h>
// #include <ompl/base/objectives/PathLengthOptimizationObjective.h>
// #include <ompl/base/spaces/RealVectorStateSpace.h>
// #include <ompl/geometric/planners/rrt/InformedRRTstar.h>
// #include <ompl/util/Console.h>

#include <Eigen/Eigen>
#include <deque>
#include <memory>
#include <misc/visualizer.hpp>
#include <algorithm>
#include <unordered_set>

#include "firi.hpp"
#include "geo_utils.hpp"

namespace sfc_gen {

inline double sqDistToSegment(const Eigen::Vector3d &p, const Eigen::Vector3d &a,
                              const Eigen::Vector3d &b) {
  const Eigen::Vector3d ab = b - a;
  const double l2 = ab.squaredNorm();
  double t = l2 > 0.0 ? (p - a).dot(ab) / l2 : 0.0;
  t = std::min(1.0, std::max(0.0, t));
  return (p - (a + t * ab)).squaredNorm();
}

/**
 * Reduce the obstacle set for one FIRI piece, keeping the points that actually
 * constrain it: the ones nearest the seed segment [a, b].
 *
 * Replaces a stride decimation (`step = size / target`, keep every step-th
 * element in ARRAY order) that had no spatial criterion. FIRI only guarantees
 * that a polytope excludes the points it is handed, so dropping obstacle points
 * arbitrarily leaves the dropped ones INSIDE the corridor. It was also worse
 * than uniform for target < size < 2*target, where step == 1 kept the first
 * `target` points in whatever order the map query produced.
 *
 * The incoming cloud is already voxel-downsampled at
 * local_planning/corridor/obstacle_voxel_size (planner_manager.cpp), so it is a
 * uniform-density surface sample; nearest-to-segment is the meaningful way to
 * cut it further. The polytope is anchored on [a, b] and bounded by the box, so
 * the binding constraints are the near ones.
 */
inline void keepNearestToSegment(std::vector<Eigen::Vector3d> &points,
                                 const Eigen::Vector3d &a,
                                 const Eigen::Vector3d &b, size_t target_size) {
  if (points.size() <= target_size) {
    return;
  }
  std::vector<std::pair<double, Eigen::Vector3d>> keyed;
  keyed.reserve(points.size());
  for (const Eigen::Vector3d &p : points) {
    keyed.emplace_back(sqDistToSegment(p, a, b), p);
  }
  std::nth_element(keyed.begin(), keyed.begin() + target_size, keyed.end(),
                   [](const std::pair<double, Eigen::Vector3d> &x,
                      const std::pair<double, Eigen::Vector3d> &y) {
                     return x.first < y.first;
                   });
  points.clear();
  for (size_t i = 0; i < target_size; ++i) {
    points.push_back(keyed[i].second);
  }
}

/**
 * Optimized convex cover generation with performance improvements
 */
inline void convexCover(const std::unique_ptr<Visualizer> &vizer, const std::vector<Eigen::Vector3d> &path, const std::vector<Eigen::Vector3d> &points,
                        const Eigen::Vector3d &lowCorner, const Eigen::Vector3d &highCorner, const double &progress, const double &range, std::vector<Eigen::MatrixX4d> &hpolys,
                        const double eps, const double dilate_radius_,
                        const size_t max_obstacle_samples,
                        const double firi_obstacle_distance_limit,
                        const int firi_max_plane_count,
                        const int gap_violation_plane_threshold,
                        const Eigen::MatrixX4d &observed_boundary) {
  // hpolys.clear();
  const int n = path.size();
  Eigen::Matrix<double, 6, 4> box_bd =
      Eigen::Matrix<double, 6, 4>::Zero();
  box_bd(0, 0) = 1.0;
  box_bd(1, 0) = -1.0;
  box_bd(2, 1) = 1.0;
  box_bd(3, 1) = -1.0;
  box_bd(4, 2) = 1.0;
  box_bd(5, 2) = -1.0;

  Eigen::MatrixX4d hp, gap;
  Eigen::Vector3d a, b = path[0];
  
  // Pre-allocate vectors to avoid repeated allocations
  std::vector<Eigen::Vector3d> valid_pc;
  valid_pc.reserve(points.size());
  std::vector<Eigen::Vector3d> bs;
  bs.reserve(n);
  
  // Cache for point filtering to avoid redundant computations
  std::unordered_set<size_t> processed_points;
  
  for (int i = 1; i < n;) {
    a = b;
    // 路径太长了，沿着方向拓展最大距离progress
    if ((a - path[i]).norm() > progress) {
      b = (path[i] - a).normalized() * progress + a;
    } else {
      b = path[i];
      i++;
    }
    bs.emplace_back(b);

    box_bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
    box_bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
    box_bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
    box_bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
    box_bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
    box_bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));

    // Seed-path selection is integrated in the next phase. Until then, apply
    // only PR #1 faces that contain both segment endpoints; passing an outside
    // seed to FIRI makes the boundary problem invalid. The existing post-clip
    // remains responsible for incompatible segments.
    const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
    const Eigen::Vector4d bh(b(0), b(1), b(2), 1.0);
    std::vector<int> applicable_faces;
    applicable_faces.reserve(observed_boundary.rows());
    for (int row = 0; row < observed_boundary.rows(); ++row) {
      if ((observed_boundary.row(row) * ah)(0) <= eps &&
          (observed_boundary.row(row) * bh)(0) <= eps) {
        applicable_faces.push_back(row);
      }
    }

    Eigen::MatrixX4d exact_bd(6 + applicable_faces.size(), 4);
    exact_bd.topRows<6>() = box_bd;
    for (size_t row = 0; row < applicable_faces.size(); ++row)
      exact_bd.row(6 + row) = observed_boundary.row(applicable_faces[row]);

    // firi() uses an exact zero test for seed containment. Expand only the
    // observed faces by the geometric epsilon while solving so a seed lying on
    // a PR #1 face is not rejected by floating-point roundoff. The exact face
    // is restored before the polytope is accepted.
    Eigen::MatrixX4d firi_bd = exact_bd;
    for (int row = 6; row < firi_bd.rows(); ++row) {
      firi_bd(row, 3) -= eps * firi_bd.row(row).head<3>().norm();
    }

    Eigen::MatrixX4d segment_observed(applicable_faces.size(), 4);
    for (size_t row = 0; row < applicable_faces.size(); ++row)
      segment_observed.row(row) =
          observed_boundary.row(applicable_faces[row]);

    auto same_halfspace_plane = [&](const Eigen::Vector4d &lhs,
                                    const Eigen::Vector4d &rhs,
                                    const double tolerance) {
      const double lhs_norm = lhs.head<3>().norm();
      const double rhs_norm = rhs.head<3>().norm();
      if (!std::isfinite(lhs_norm) || !std::isfinite(rhs_norm) ||
          lhs_norm <= eps || rhs_norm <= eps)
        return false;
      const Eigen::Vector4d normalized_lhs = lhs / lhs_norm;
      const Eigen::Vector4d normalized_rhs = rhs / rhs_norm;
      return (normalized_lhs - normalized_rhs).cwiseAbs().maxCoeff() <=
             tolerance;
    };

    auto is_observed_plane = [&](const Eigen::Vector4d &plane) {
      for (int row = 0; row < segment_observed.rows(); ++row) {
        if (same_halfspace_plane(
                plane, segment_observed.row(row).transpose(), 2.0 * eps))
          return true;
      }
      return false;
    };

    // DilateRadiusSoft is an obstacle-clearance margin. PR #1 faces describe
    // an observation boundary, not a physical obstacle, so keep those planes
    // at their exact location while contracting the remaining FIRI planes.
    auto contract_obstacle_planes = [&](Eigen::MatrixX4d &poly) {
      for (int row = 0; row < poly.rows(); ++row) {
        if (!is_observed_plane(poly.row(row).transpose())) {
          poly(row, 3) +=
              dilate_radius_ * poly.row(row).head<3>().norm();
        }
      }
    };

    auto apply_observed_boundary = [&](Eigen::MatrixX4d &poly) {
      for (int row = 0; row < segment_observed.rows(); ++row) {
        const Eigen::Vector4d observed =
            segment_observed.row(row).transpose();
        bool already_present = false;
        for (int poly_row = 0; poly_row < poly.rows(); ++poly_row) {
          if (same_halfspace_plane(poly.row(poly_row).transpose(),
                                   observed, 0.25 * eps)) {
            already_present = true;
            break;
          }
        }
        if (already_present)
          continue;

        Eigen::MatrixX4d candidate(poly.rows() + 1, 4);
        candidate.topRows(poly.rows()) = poly;
        candidate.row(poly.rows()) = observed.transpose();
        Eigen::Vector3d interior;
        if (!geo_utils::findInterior(candidate, interior))
          return false;
        poly.swap(candidate);
      }
      return true;
    };

    // Optimized point filtering with early termination
    valid_pc.clear();
    const double max_dist_sq = range * range * 4.0; // Early termination threshold
    
    for (size_t idx = 0; idx < points.size(); ++idx) {
      const Eigen::Vector3d &p = points[idx];
      
      // Quick distance check before expensive matrix operations
      const double dist_sq = (p - a).squaredNorm();
      if (dist_sq > max_dist_sq && (p - b).squaredNorm() > max_dist_sq) {
        continue; // Skip points that are clearly too far
      }
      
      if ((exact_bd.leftCols<3>() * p + exact_bd.rightCols<1>()).maxCoeff() <
          0.0) {
        valid_pc.emplace_back(p);
      }
    }
    
    // Cap the obstacle set by keeping the points nearest the seed segment.
    keepNearestToSegment(valid_pc, a, b, max_obstacle_samples);
    
    // Skip firi computation if no valid points
    if (valid_pc.empty()) {
      if (segment_observed.rows() > 0) {
        // Keep the exact PR #1 boundary when no obstacle plane is available.
        hpolys.emplace_back(exact_bd);
      } else {
        // Preserve the legacy fallback when no observed face applies.
        Eigen::MatrixX4d simple_hp(6, 4);
        simple_hp << 1, 0, 0, -a(0) - range,
                      -1, 0, 0, a(0) - range,
                      0, 1, 0, -a(1) - range,
                      0, -1, 0, a(1) - range,
                      0, 0, 1, -a(2) - range,
                      0, 0, -1, a(2) - range;
        hpolys.emplace_back(simple_hp);
      }
      continue;
    }
    
    Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3, valid_pc.size());
    if (!firi::firi(firi_bd, pc, a, b, hp, 2, eps,
                    firi_obstacle_distance_limit, firi_max_plane_count) ||
        hp.rows() == 0) {
      continue;
    }
    Eigen::MatrixX4d hp_origin = hp;

    // 将凸包向里收缩，收缩大小为膨胀半径
    // if (i != 1)
      contract_obstacle_planes(hp);
    if (!apply_observed_boundary(hp))
      continue;
    double r = dilate_radius_;
    // 适当放宽条件，不能没有可行解

    // Reduced firi calls for performance - only call when necessary
    if (((hp * bh).array() > -eps).cast<int>().sum() > 0) {
      // Only generate additional polytopes if the current one is valid
      Eigen::MatrixX4d endpoint_hp;
      if (firi::firi(firi_bd, pc, a, a, endpoint_hp, 1, eps,
                     firi_obstacle_distance_limit, firi_max_plane_count) &&
          endpoint_hp.rows() > 0) {
        contract_obstacle_planes(endpoint_hp);
        if (apply_observed_boundary(endpoint_hp)) {
          hp = endpoint_hp;
          hpolys.emplace_back(hp);
        }
      }
      
      // Skip middle point generation for performance
      // Middle-point FIRI generation remains intentionally disabled.
      // hp.col(3) = hp.col(3).array() + dilate_radius_ * hp.leftCols(3).rowwise().norm().array();
      // hpolys.emplace_back(hp);
      
      if (firi::firi(firi_bd, pc, b, b, endpoint_hp, 1, eps,
                     firi_obstacle_distance_limit, firi_max_plane_count) &&
          endpoint_hp.rows() > 0) {
        contract_obstacle_planes(endpoint_hp);
        if (apply_observed_boundary(endpoint_hp)) {
          hp = endpoint_hp;
          hpolys.emplace_back(hp);
        }
      }
    }

    // Simplified gap generation logic
    if (hpolys.size() != 0) {
      if (gap_violation_plane_threshold <=
          ((hp * ah).array() > -eps).cast<int>().sum() +
              ((hpolys.back() * ah).array() > -eps).cast<int>().sum()) {
        Eigen::MatrixX4d candidate_gap;
        if (firi::firi(firi_bd, pc, a, a, candidate_gap, 1, eps,
                       firi_obstacle_distance_limit,
                       firi_max_plane_count) &&
            candidate_gap.rows() > 0 &&
            apply_observed_boundary(candidate_gap)) {
          gap = candidate_gap;
          hpolys.emplace_back(gap);
        }
      }
    }

    hpolys.emplace_back(hp);
  }
}

inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys,
                     const double overlap_tolerance) {
  std::vector<Eigen::MatrixX4d> htemp = hpolys;
  if (htemp.size() == 1) {
    Eigen::MatrixX4d headPoly = htemp.front();
    htemp.insert(htemp.begin(), headPoly);
  }
  hpolys.clear();

  int M = htemp.size();
  Eigen::MatrixX4d hPoly;
  bool overlap;
  std::deque<int> idices;
  idices.push_front(M - 1);
  for (int i = M - 1; i >= 0; i--) {
    for (int j = 0; j < i; j++) {
      if (j < i - 1) {
        overlap =
            geo_utils::overlap(htemp[i], htemp[j], overlap_tolerance);
      } else {
        overlap = true;
      }
      if (overlap) {
        idices.push_front(j);
        i = j + 1;
        break;
      }
    }
  }
  for (const auto &ele : idices) {
    hpolys.push_back(htemp[ele]);
  }
}

} // namespace sfc_gen

#endif
