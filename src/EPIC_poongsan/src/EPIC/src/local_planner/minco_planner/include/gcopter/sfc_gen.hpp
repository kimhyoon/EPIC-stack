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

/**
 * Downsample point cloud for faster processing
 * @param points Input point cloud
 * @param target_size Target number of points
 * @return Downsampled point cloud
 */
inline std::vector<Eigen::Vector3d> downsamplePoints(const std::vector<Eigen::Vector3d>& points, 
                                                     size_t target_size = 1000) {
    if (points.size() <= target_size) {
        return points;
    }
    
    std::vector<Eigen::Vector3d> downsampled;
    downsampled.reserve(target_size);
    
    // Simple uniform sampling
    const size_t step = points.size() / target_size;
    for (size_t i = 0; i < points.size() && downsampled.size() < target_size; i += step) {
        downsampled.push_back(points[i]);
    }
    
    return downsampled;
}

/**
 * Optimized convex cover generation with performance improvements
 */
inline void convexCover(const std::unique_ptr<Visualizer> &vizer, const std::vector<Eigen::Vector3d> &path, const std::vector<Eigen::Vector3d> &points,
                        const Eigen::Vector3d &lowCorner, const Eigen::Vector3d &highCorner, const double &progress, const double &range, std::vector<Eigen::MatrixX4d> &hpolys,
                        const double eps = 1.0e-6, const double dilate_radius_ = 0.1) {
  // hpolys.clear();
  const int n = path.size();
  Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
  bd(0, 0) = 1.0;
  bd(1, 0) = -1.0;
  bd(2, 1) = 1.0;
  bd(3, 1) = -1.0;
  bd(4, 2) = 1.0;
  bd(5, 2) = -1.0;

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

    bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
    bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
    bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
    bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
    bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
    bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));

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
      
      if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0) {
        valid_pc.emplace_back(p);
      }
    }
    
    // Downsample valid points for faster processing
    if (valid_pc.size() > 500) {
      valid_pc = downsamplePoints(valid_pc, 500);
    }
    
    // Skip firi computation if no valid points
    if (valid_pc.empty()) {
      // Create a simple bounding box as fallback
      Eigen::MatrixX4d simple_hp(6, 4);
      simple_hp << 1, 0, 0, -a(0) - range,
                    -1, 0, 0, a(0) - range,
                    0, 1, 0, -a(1) - range,
                    0, -1, 0, a(1) - range,
                    0, 0, 1, -a(2) - range,
                    0, 0, -1, a(2) - range;
      hpolys.emplace_back(simple_hp);
      continue;
    }
    
    Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3, valid_pc.size());
    firi::firi(bd, pc, a, b, hp); // 计算出包含a和b的凸包
    Eigen::MatrixX4d hp_origin = hp;

    // 将凸包向里收缩，收缩大小为膨胀半径
    // if (i != 1)
      hp.col(3) = hp.col(3).array() + dilate_radius_ * hp.leftCols(3).rowwise().norm().array();
    Eigen::Vector4d bh(b(0), b(1), b(2), 1.0);
    double r = dilate_radius_;
    // 适当放宽条件，不能没有可行解

    // Reduced firi calls for performance - only call when necessary
    if (((hp * bh).array() > -eps).cast<int>().sum() > 0) {
      // Only generate additional polytopes if the current one is valid
      firi::firi(bd, pc, a, a, hp, 1);
      hp.col(3) = hp.col(3).array() + dilate_radius_ * hp.leftCols(3).rowwise().norm().array();
      hpolys.emplace_back(hp);
      
      // Skip middle point generation for performance
      // firi::firi(bd, pc, (a + b) / 2.0, (a + b) / 2.0, hp, 1);
      // hp.col(3) = hp.col(3).array() + dilate_radius_ * hp.leftCols(3).rowwise().norm().array();
      // hpolys.emplace_back(hp);
      
      firi::firi(bd, pc, b, b, hp, 1);
      hp.col(3) = hp.col(3).array() + dilate_radius_ * hp.leftCols(3).rowwise().norm().array();
      hpolys.emplace_back(hp);
    }

    // Simplified gap generation logic
    if (hpolys.size() != 0) {
      const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
      if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() + ((hpolys.back() * ah).array() > -eps).cast<int>().sum()) {
        firi::firi(bd, pc, a, a, gap, 1);
        hpolys.emplace_back(gap);
      }
    }

    hpolys.emplace_back(hp);
  }
}

inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys) {
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
        overlap = geo_utils::overlap(htemp[i], htemp[j], 1e-2);
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
