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

#ifndef GCOPTER_HPP
#define GCOPTER_HPP

#include <Eigen/Eigen>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <gcopter/geo_utils.hpp>
#include <iostream>
#include <vector>

#include "gcopter/lbfgs.hpp"
#include "gcopter/minco.hpp"

namespace gcopter {

class GCOPTER_PolytopeSFC {
public:
  typedef Eigen::Matrix3Xd PolyhedronV;
  typedef Eigen::MatrixX4d PolyhedronH;
  typedef std::vector<PolyhedronV> PolyhedraV;
  typedef std::vector<PolyhedronH> PolyhedraH;

private:
  minco::MINCO_S3NU minco;
  minco::MINCO_S3NU yaw_minco;

  double rho;
  Eigen::Matrix3d headPVA;
  Eigen::Matrix3d tailPVA;
  Eigen::Vector2d initialStateWeights;
  Eigen::Vector3d terminalStateWeights;

  PolyhedraV vPolytopes;
  PolyhedraH hPolytopes;
  Eigen::Matrix3Xd shortPath;

  Eigen::VectorXi pieceIdx;
  Eigen::VectorXi vPolyIdx;
  Eigen::VectorXi hPolyIdx;

  int polyN;
  int pieceN;

  int spatialDim;
  int temporalDim;
  int spatialDim_yaw;
  int temporalDim_yaw;
  double dilate_radius_ = 0.2;
  double smoothEps;
  int integralRes;
  double maxVelocity;
  double maxAcceleration;
  Eigen::Vector3d runningPenaltyWeights;

  // yaw minco
  double yaw_rho_vis;
  double yaw_max_vel;
  double yaw_time_fwd;
  int integralResolution_yaw;
  double yaw_diff_eps;
  double vector_norm_eps;
  double min_segment_time;
  double linear_solve_pivot_eps;
  bool use_shorten_path = false;
  Eigen::Matrix3Xd guide_path;

  lbfgs::lbfgs_parameter_t lbfgs_params;

  Eigen::Matrix3Xd points;
  Eigen::VectorXd times;
  Eigen::Matrix3Xd gradByPoints;
  Eigen::VectorXd gradByTimes;
  Eigen::MatrixX3d partialGradByCoeffs;
  Eigen::VectorXd partialGradByTimes;

private:
  static inline void forwardT(const Eigen::VectorXd &tau, Eigen::VectorXd &T,
                              const double minSegmentTime) {
    const int sizeTau = tau.size();
    T.resize(sizeTau);
    for (int i = 0; i < sizeTau; i++) {
      const double positiveTime =
          tau(i) > 0.0 ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                       : 1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
      T(i) = minSegmentTime + positiveTime;
    }
    return;
  }

  template <typename EIGENVEC>
  static inline void backwardT(const Eigen::VectorXd &T, EIGENVEC &tau,
                               const double minSegmentTime) {
    const int sizeT = T.size();
    tau.resize(sizeT);
    for (int i = 0; i < sizeT; i++) {
      const double safeT =
          std::isfinite(T(i)) ? std::max(T(i), 2.0 * minSegmentTime)
                              : 2.0 * minSegmentTime;
      const double positiveTime = safeT - minSegmentTime;
      tau(i) = positiveTime > 1.0
                   ? (sqrt(2.0 * positiveTime - 1.0) - 1.0)
                   : (1.0 - sqrt(2.0 / positiveTime - 1.0));
    }

    return;
  }

  template <typename EIGENVEC>
  static inline void backwardGradT(const Eigen::VectorXd &tau, const Eigen::VectorXd &gradT,
                                   EIGENVEC &gradTau) {
    const int sizeTau = tau.size();
    gradTau.resize(sizeTau);
    double denSqrt;
    for (int i = 0; i < sizeTau; i++) {
      if (tau(i) > 0) {
        gradTau(i) = gradT(i) * (tau(i) + 1.0);
      } else {
        denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
        gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
      }
    }

    return;
  }

  static inline void forwardP(const Eigen::VectorXd &xi, const Eigen::VectorXi &vIdx,
                              const PolyhedraV &vPolys, Eigen::Matrix3Xd &P,
                              const double vectorNormEps) {
    const int sizeP = vIdx.size();
    P.resize(3, sizeP);
    Eigen::VectorXd q;
    for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k) {
      l = vIdx(i);
      k = vPolys[l].cols();
      const Eigen::VectorXd segment = xi.segment(j, k);
      const double segment_norm = std::max(segment.norm(), vectorNormEps);
      q = (segment / segment_norm).head(k - 1);
      P.col(i) = vPolys[l].rightCols(k - 1) * q.cwiseProduct(q) + vPolys[l].col(0);
    }
    return;
  }

  struct TinyNLSData {
    const Eigen::Matrix3Xd *overlap_poly;
    double vector_norm_eps;
  };

  static inline double costTinyNLS(void *ptr, const Eigen::VectorXd &xi,
                                   Eigen::VectorXd &gradXi) {
    if (!xi.allFinite()) {
      gradXi.setZero(xi.size());
      return std::numeric_limits<double>::infinity();
    }
    const int n = xi.size();
    const TinyNLSData &data = *static_cast<TinyNLSData *>(ptr);
    const Eigen::Matrix3Xd &ovPoly = *data.overlap_poly;

    const double sqrNormXi = xi.squaredNorm();
    const double invNormXi =
        1.0 / std::max(std::sqrt(std::max(0.0, sqrNormXi)),
                       data.vector_norm_eps);
    const Eigen::VectorXd unitXi = xi * invNormXi;
    const Eigen::VectorXd r = unitXi.head(n - 1);
    const Eigen::Vector3d delta =
    ovPoly.rightCols(n - 1) * r.cwiseProduct(r) + ovPoly.col(1) - ovPoly.col(0);

    double cost = delta.squaredNorm();
    gradXi.head(n - 1) =
    (ovPoly.rightCols(n - 1).transpose() * (2 * delta)).array() * r.array() * 2.0;
    gradXi(n - 1) = 0.0;
    gradXi = (gradXi - unitXi.dot(gradXi) * unitXi).eval() * invNormXi;

    const double sqrNormViolation = sqrNormXi - 1.0;
    if (sqrNormViolation > 0.0) {
      double c = sqrNormViolation * sqrNormViolation;
      const double dc = 3.0 * c;
      c *= sqrNormViolation;
      cost += c;
      gradXi += dc * 2.0 * xi;
    }

    return cost;
  }

  template <typename EIGENVEC>
  static inline void backwardP(const Eigen::Matrix3Xd &P, const Eigen::VectorXi &vIdx,
                               const PolyhedraV &vPolys, EIGENVEC &xi,
                               const double vectorNormEps) {
    const int sizeP = P.cols();

    double minSqrD;
    lbfgs::lbfgs_parameter_t tiny_nls_params;
    tiny_nls_params.past = 0;
    tiny_nls_params.delta = 1.0e-5;
    tiny_nls_params.g_epsilon = FLT_EPSILON;
    tiny_nls_params.max_iterations = 128;

    Eigen::Matrix3Xd ovPoly;
    for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k) {
      l = vIdx(i);
      k = vPolys[l].cols();

      ovPoly.resize(3, k + 1);
      ovPoly.col(0) = P.col(i);
      ovPoly.rightCols(k) = vPolys[l];
      Eigen::VectorXd x(k);
      x.setConstant(sqrt(1.0 / k));
      TinyNLSData data{&ovPoly, vectorNormEps};
      lbfgs::lbfgs_optimize(x, minSqrD, &GCOPTER_PolytopeSFC::costTinyNLS, nullptr, nullptr,
                            &data, tiny_nls_params);

      xi.segment(j, k) = x;
    }

    return;
  }

  template <typename EIGENVEC>
  static inline void backwardGradP(const Eigen::VectorXd &xi, const Eigen::VectorXi &vIdx,
                                   const PolyhedraV &vPolys, const Eigen::Matrix3Xd &gradP,
                                   EIGENVEC &gradXi,
                                   const double vectorNormEps) {
    const int sizeP = vIdx.size();
    gradXi.resize(xi.size());

    double normInv;
    Eigen::VectorXd q, gradQ, unitQ;
    for (int i = 0, j = 0, k, l; i < sizeP; i++, j += k) {
      l = vIdx(i);
      k = vPolys[l].cols();
      q = xi.segment(j, k);
      normInv = 1.0 / std::max(q.norm(), vectorNormEps);
      unitQ = q * normInv;
      gradQ.resize(k);
      gradQ.head(k - 1) = (vPolys[l].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                          unitQ.head(k - 1).array() * 2.0;
      gradQ(k - 1) = 0.0;
      gradXi.segment(j, k) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
    }

    return;
  }

  template <typename EIGENVEC>
  static inline void normRetrictionLayer(const Eigen::VectorXd &xi, const Eigen::VectorXi &vIdx,
                                         const PolyhedraV &vPolys, double &cost, EIGENVEC &gradXi) {
    const int sizeP = vIdx.size();
    gradXi.resize(xi.size());

    double sqrNormQ, sqrNormViolation, c, dc;
    Eigen::VectorXd q;
    for (int i = 0, j = 0, k; i < sizeP; i++, j += k) {
      k = vPolys[vIdx(i)].cols();

      q = xi.segment(j, k);
      sqrNormQ = q.squaredNorm();
      sqrNormViolation = sqrNormQ - 1.0;
      if (sqrNormViolation > 0.0) {
        c = sqrNormViolation * sqrNormViolation;
        dc = 3.0 * c;
        c *= sqrNormViolation;
        cost += c;
        gradXi.segment(j, k) += dc * 2.0 * q;
      }
    }

    return;
  }

  static inline bool smoothedL1(const double &x, const double &mu, double &f, double &df) {
    if (x < 0.0) {
      return false;
    } else if (x > mu) {
      f = x - 0.5 * mu;
      df = 1.0;
      return true;
    } else {
      const double xdmu = x / mu;
      const double sqrxdmu = xdmu * xdmu;
      const double mumxd2 = mu - 0.5 * x;
      f = mumxd2 * sqrxdmu * xdmu;
      df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
      return true;
    }
  }

  inline bool grad_cost_a(const Eigen::Vector3d &a, Eigen::Vector3d &grada, double &costa) {

    double apen = a.squaredNorm() - 2.0 * 2.0;
    if (apen > 0) {
      // grada = yaw_rho_acc * 6 * apen * apen * a;
      // costa = yaw_rho_acc * apen * apen * apen;
      // return true;
      double mu_ = 1.5e-1;
      double smooth_cost, smooth_grad;

      if (smoothedL1(apen, mu_, smooth_cost, smooth_grad)) {
        costa = 100 * smooth_cost;
        grada = 100 * smooth_grad * 2.0 * a; // 链式法则
        return true;
      }
    }
    return false;
  }

  inline bool grad_cost_visibility(const Eigen::Vector3d &p, const Eigen::Vector3d &guide_p,
                                   Eigen::Vector3d &gradp, double &costp) {

    // Eigen::Vector3d diff = p - guide_p;
    // double norm_diff = (p - guide_p).norm();
    // if (norm_diff <= 0) {
    //   return false;
    // }
    // double smooth_cost, smooth_grad;
    // if (smoothedL1(norm_diff, mu_, smooth_cost, smooth_grad)) {
    //   costp = yaw_rho_vis * smooth_cost;
    //   gradp = yaw_rho_vis * smooth_grad * (diff / norm_diff); // 链式法则
    //   // gradp = yaw_rho_vis * smooth_grad * 2 * diff; // 链式法则
    //   cout << "guide_p = " << guide_p.transpose() << endl;
    //   cout << "p = " << p.transpose() << endl;
    //   cout << "cost = " << costp << endl;
    //   cout << endl;
    //   return true;
    // }
    // return false;
    double diff = p.x() - guide_p.x();
    if (!std::isfinite(diff)) {
      costp = 0.0;
      gradp.setZero();
      return false;
    }
    const double abs_diff = std::abs(diff);

    double mu_ = 1.5e-1;
    double smooth_cost, smooth_grad;

    if (smoothedL1(abs_diff, mu_, smooth_cost, smooth_grad)) {
      costp = yaw_rho_vis * smooth_cost;
      gradp.setZero();
      const double safe_abs_diff = std::max(abs_diff, yaw_diff_eps);
      gradp.x() =
          yaw_rho_vis * smooth_grad * (diff / safe_abs_diff); // 链式法则
      return true;
    }

    return false;
  }

  static inline void attachPenaltyFunctional(
  const Eigen::VectorXd &T, const Eigen::MatrixX3d &coeffs, const Eigen::VectorXi &hIdx,
  const PolyhedraH &hPolys, const double &smoothFactor, const int &integralResolution,
  const double maxVelocity, const double maxAcceleration,
  const Eigen::Vector3d &penaltyWeights, double &cost,
  Eigen::VectorXd &gradT, Eigen::MatrixX3d &gradC) {
    const double velSqrMax = maxVelocity * maxVelocity;
    const double accSqrMax = maxAcceleration * maxAcceleration;
    const double weightPos = penaltyWeights(0);
    const double weightVel = penaltyWeights(1);
    const double weightAcc = penaltyWeights(2);

    Eigen::Vector3d pos, vel, acc, jer;
    Eigen::Vector3d gradPos, gradVel, gradAcc;

    double step, alpha;
    double s1, s2, s3, s4, s5;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;

    Eigen::Vector3d outerNormal;
    int K, L;
    double violaPos, violaVel, violaAcc;
    double violaPosPenaD, violaVelPenaD, violaAccPenaD;
    double violaPosPena, violaVelPena, violaAccPena;
    double node, pena;

    const int pieceNum = T.size();
    const double integralFrac = 1.0 / integralResolution;
    for (int i = 0; i < pieceNum; i++) {
      const Eigen::Matrix<double, 6, 3> &coeff =
          coeffs.block<6, 3>(i * 6, 0);

      step = T(i) * integralFrac;
      for (int j = 0; j <= integralResolution; j++) {
        s1 = j * step;
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3,
            5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2,
            20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        pos = coeff.transpose() * beta0;
        vel = coeff.transpose() * beta1;
        acc = coeff.transpose() * beta2;
        jer = coeff.transpose() * beta3;

        violaVel = vel.squaredNorm() - velSqrMax;
        violaAcc = acc.squaredNorm() - accSqrMax;
        gradPos.setZero();
        gradVel.setZero();
        gradAcc.setZero();
        pena = 0.0;

        L = hIdx(i);
        K = hPolys[L].rows();
        for (int k = 0; k < K; k++) {
          outerNormal = hPolys[L].block<1, 3>(k, 0);
          violaPos = outerNormal.dot(pos) + hPolys[L](k, 3);
          if (smoothedL1(violaPos, smoothFactor, violaPosPena,
                         violaPosPenaD)) // violaPos > 0表示超过pos限制
          {
            gradPos += weightPos * violaPosPenaD * outerNormal;
            pena += weightPos * violaPosPena;
          }
        }

        if (smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD)) {
          gradVel += weightVel * violaVelPenaD * 2.0 * vel;
          pena += weightVel * violaVelPena;
        }

        if (smoothedL1(violaAcc, smoothFactor, violaAccPena,
                       violaAccPenaD)) {
          gradAcc += weightAcc * violaAccPenaD * 2.0 * acc;
          pena += weightAcc * violaAccPena;
        }

        node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
        alpha = j * integralFrac;

        gradC.block<6, 3>(i * 6, 0) +=
            (beta0 * gradPos.transpose() + beta1 * gradVel.transpose() +
             beta2 * gradAcc.transpose()) *
            node * step;

        gradT(i) +=
            (gradPos.dot(vel) + gradVel.dot(acc) + gradAcc.dot(jer)) *
                    alpha * node * step +
                    node * integralFrac * pena;
        cost += node * step * pena;
      }
    }

    return;
  }

  inline void PenaltyFunctional_yaw(const Eigen::VectorXd &T, double &cost, Eigen::MatrixXd &gdC_,
                                    Eigen::VectorXd &gdT_) {
    cost = 0.0;
    gdC_.resize(6 * temporalDim_yaw, 3);
    gdC_.setZero();
    gdT_.resize(temporalDim_yaw);
    gdT_.setZero();

    Eigen::Vector3d pos, vel, acc, jer, sna;
    Eigen::Vector3d grad_tmp, grad_tmp_p, grad_tmp_v;
    double cost_tmp, cost_tmp_p, cost_tmp_v;

    double step, alpha;
    double s1, s2, s3, s4, s5;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc;
    double gradViolaPt, gradViolaVt, gradViolaAt;
    double omg = 1.0;

    const double integralFrac = 1.0 / integralResolution_yaw;
    for (int i = 0; i < temporalDim_yaw; i++) {
      const Eigen::Matrix<double, 6, 3> &c = yaw_minco.getCoeffs().block<6, 3>(i * 6, 0);
      step = T(i) * integralFrac;
      for (int j = 1; j < integralResolution_yaw; j++) {
        s1 = j * step;
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5;
        beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2,
        beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4;
        beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2,
        beta2(5) = 20.0 * s3;
        beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1,
        beta3(5) = 60.0 * s2;
        beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0,
        beta4(5) = 120.0 * s1;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        sna = c.transpose() * beta4;

        alpha = j * integralFrac;
        Eigen::Vector3d start_pt, end_pt;
        if (i == 0) {
          start_pt = headPVA.col(0);
        } else {
          start_pt = guide_path.col(i - 1);
        }
        if (i == temporalDim_yaw - 1) {
          end_pt = tailPVA.col(0);
        } else {
          end_pt = guide_path.col(i);
        }
        double start_ratio =
            1.0 - static_cast<double>(j) / integralResolution_yaw;
        Eigen::Vector3d guide_pos = start_ratio * start_pt + (1 - start_ratio) * end_pt;
        if (grad_cost_visibility(pos, guide_pos, grad_tmp, cost_tmp)) {
          double node = (j == 0 || j == integralResolution_yaw) ? 0.5 : 1.0;

          gradViolaPc = beta0 * grad_tmp.transpose();
          gradViolaPt = alpha * grad_tmp.dot(vel);
          gdC_.block<6, 3>(i * 6, 0) += node * step * gradViolaPc;
          gdT_(i) += node * (cost_tmp * integralFrac + step * gradViolaPt);
          cost += node * step * cost_tmp;
        }
        // if (grad_cost_a(acc, grad_tmp, cost_tmp)) {
        //   gradViolaAc = beta2 * grad_tmp.transpose();
        //   gradViolaAt = alpha * grad_tmp.dot(jer);
        //   gdC_.block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
        //   gdT_(i) += omg * (cost_tmp * integralFrac + step * gradViolaAt);
        //   cost += omg * step * cost_tmp;
        // }
      }
    }

    return;
  }

  static inline double costFunctional(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g) {
    if (!x.allFinite()) {
      g.setZero(x.size());
      return std::numeric_limits<double>::infinity();
    }
    GCOPTER_PolytopeSFC &obj = *(GCOPTER_PolytopeSFC *)ptr;
    const int dimTau = obj.temporalDim;
    const int dimXi = obj.spatialDim;
    const int boundaryOffset = dimTau + dimXi;
    const double weightT = obj.rho;
    Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
    Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
    Eigen::Map<const Eigen::Matrix<double, 3, 2>> optimizedHeadVA(
        x.data() + boundaryOffset);
    Eigen::Map<const Eigen::Matrix3d> optimizedTailPVA(
        x.data() + boundaryOffset + 6);
    Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
    Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);
    Eigen::Map<Eigen::Matrix<double, 3, 2>> gradHeadVA(
        g.data() + boundaryOffset);
    Eigen::Map<Eigen::Matrix3d> gradTailPVA(
        g.data() + boundaryOffset + 6);
    g.setZero();

    forwardT(tau, obj.times, obj.min_segment_time);
    forwardP(xi, obj.vPolyIdx, obj.vPolytopes, obj.points,
             obj.vector_norm_eps);
    Eigen::Matrix3d currentHeadPVA = obj.headPVA;
    currentHeadPVA.rightCols<2>() = optimizedHeadVA;
    const Eigen::Matrix3d currentTailPVA = optimizedTailPVA;
    if (!obj.times.allFinite() || !obj.points.allFinite() ||
        !currentHeadPVA.allFinite() || !currentTailPVA.allFinite() ||
        !obj.minco.setBoundaryConditions(currentHeadPVA, currentTailPVA)) {
      g.setZero();
      return std::numeric_limits<double>::infinity();
    }

    double cost = 0.0;
    if (!obj.minco.setParameters(obj.points, obj.times,
                                 obj.linear_solve_pivot_eps)) {
      g.setZero();
      return std::numeric_limits<double>::infinity();
    }
    obj.partialGradByCoeffs.setZero();
    obj.partialGradByTimes.setZero();

    attachPenaltyFunctional(
        obj.times, obj.minco.getCoeffs(), obj.hPolyIdx, obj.hPolytopes,
        obj.smoothEps, obj.integralRes, obj.maxVelocity,
        obj.maxAcceleration, obj.runningPenaltyWeights, cost,
        obj.partialGradByTimes, obj.partialGradByCoeffs);

    Eigen::Matrix3d gradByHeadPVA;
    Eigen::Matrix3d gradByTailPVAState;
    if (!obj.minco.propogateGrad(obj.partialGradByCoeffs,
                                 obj.partialGradByTimes, obj.gradByPoints,
                                 obj.gradByTimes, &gradByHeadPVA,
                                 &gradByTailPVAState)) {
      g.setZero();
      return std::numeric_limits<double>::infinity();
    }

    const Eigen::Vector3d initialVelocityError =
        currentHeadPVA.col(1) - obj.headPVA.col(1);
    const Eigen::Vector3d initialAccelerationError =
        currentHeadPVA.col(2) - obj.headPVA.col(2);
    cost += obj.initialStateWeights(0) *
                initialVelocityError.squaredNorm() +
            obj.initialStateWeights(1) *
                initialAccelerationError.squaredNorm();
    gradByHeadPVA.col(1) +=
        2.0 * obj.initialStateWeights(0) * initialVelocityError;
    gradByHeadPVA.col(2) +=
        2.0 * obj.initialStateWeights(1) * initialAccelerationError;

    for (int derivative = 0; derivative < 3; ++derivative) {
      const Eigen::Vector3d terminalError =
          currentTailPVA.col(derivative) -
          obj.tailPVA.col(derivative);
      cost += obj.terminalStateWeights(derivative) *
              terminalError.squaredNorm();
      gradByTailPVAState.col(derivative) +=
          2.0 * obj.terminalStateWeights(derivative) * terminalError;
    }

    // Shorter feasible trajectories are preferred. Velocity, acceleration and
    // corridor penalties oppose excessive time reduction; there is no
    // yaw-derived symmetric target duration.
    cost += weightT * obj.times.sum();
    obj.gradByTimes.array() += weightT;

    backwardGradT(tau, obj.gradByTimes, gradTau);
    backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, obj.gradByPoints, gradXi,
                  obj.vector_norm_eps);
    normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);
    gradHeadVA = gradByHeadPVA.rightCols<2>();
    gradTailPVA = gradByTailPVAState;

    return std::isfinite(cost) && g.allFinite()
               ? cost
               : std::numeric_limits<double>::infinity();
  }

  static inline double costFunctional_yaw(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g) {
    if (!x.allFinite()) {
      g.setZero(x.size());
      return std::numeric_limits<double>::infinity();
    }
    GCOPTER_PolytopeSFC &obj = *(GCOPTER_PolytopeSFC *)ptr;
    const int dimTau = obj.temporalDim_yaw;
    const int dimXi = obj.spatialDim_yaw / 3;
    Eigen::Map<const Eigen::MatrixXd> tau(x.data(), dimTau, 1);
    Eigen::Map<const Eigen::Matrix3Xd> xi(x.data() + dimTau, 3, dimXi);
    Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
    Eigen::Map<Eigen::Matrix3Xd> gradXi(g.data() + dimTau, 3, dimXi);
    gradTau.setZero();
    gradXi.setZero();

    // if(obj.use_shorten_path){
    //   cout<<"xi: "<<xi<<endl;
    //   cout<<"guide_path: "<<obj.guide_path<<endl;
    //   }

    Eigen::VectorXd times_yaw;
    forwardT(tau, times_yaw, obj.min_segment_time);
    if (!times_yaw.allFinite() || !xi.allFinite()) {
      g.setZero();
      return std::numeric_limits<double>::infinity();
    }
    // cout<<"times_yaw: "<<times_yaw<<endl;

    double cost = 0.0;
    Eigen::MatrixX3d gdC_;
    Eigen::VectorXd gdT_;
    if (!obj.yaw_minco.setParameters(xi, times_yaw,
                                     obj.linear_solve_pivot_eps)) {
      g.setZero();
      return std::numeric_limits<double>::infinity();
    }
    obj.yaw_minco.getEnergy(cost);
    obj.yaw_minco.getEnergyPartialGradByCoeffs(gdC_);
    obj.yaw_minco.getEnergyPartialGradByTimes(gdT_);

    double cost_constarin = 0.0;
    Eigen::MatrixXd gdC_constrain;
    Eigen::VectorXd gdT_constrain;
    obj.PenaltyFunctional_yaw(times_yaw, cost_constarin, gdC_constrain, gdT_constrain);
    cost += cost_constarin;
    Eigen::MatrixXd gdC_now = gdC_ + gdC_constrain;
    Eigen::VectorXd gdT_now = gdT_ + gdT_constrain;
    Eigen::Matrix3Xd gradP_temp;
    Eigen::VectorXd gradT_temp;
    if (!obj.yaw_minco.propogateGrad(gdC_now, gdT_now, gradP_temp,
                                     gradT_temp)) {
      g.setZero();
      return std::numeric_limits<double>::infinity();
    }

    // cost += obj.yaw_rho_t * times_yaw.sum();
    gradT_temp.setZero();

    obj.backwardGradT(tau, gradT_temp, gradTau);
    gradXi = gradP_temp;
    // cout<<"gradXi: "<<gradXi<<endl;
    return cost;
  }

  static inline double costDistance(void *ptr, const Eigen::VectorXd &xi, Eigen::VectorXd &gradXi) {
    if (!xi.allFinite()) {
      gradXi.setZero(xi.size());
      return std::numeric_limits<double>::infinity();
    }
    void **dataPtrs = (void **)ptr;
    const double &dEps = *((const double *)(dataPtrs[0]));
    const Eigen::Vector3d &ini = *((const Eigen::Vector3d *)(dataPtrs[1]));
    const Eigen::Vector3d &fin = *((const Eigen::Vector3d *)(dataPtrs[2]));
    const PolyhedraV &vPolys = *((PolyhedraV *)(dataPtrs[3]));
    const double &vectorNormEps = *((const double *)(dataPtrs[4]));

    double cost = 0.0;
    const int overlaps = vPolys.size() / 2;

    Eigen::Matrix3Xd gradP = Eigen::Matrix3Xd::Zero(3, overlaps);
    Eigen::Vector3d a, b, d;
    Eigen::VectorXd r;
    double smoothedDistance;
    for (int i = 0, j = 0, k = 0; i <= overlaps; i++, j += k) {
      a = i == 0 ? ini : b;
      if (i < overlaps) {
        k = vPolys[2 * i + 1].cols();
        Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
        r = (q / std::max(q.norm(), vectorNormEps)).head(k - 1);
        b = vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) + vPolys[2 * i + 1].col(0);
      } else {
        b = fin;
      }

      d = b - a;
      smoothedDistance = sqrt(d.squaredNorm() + dEps);
      cost += smoothedDistance;

      if (i < overlaps) {
        gradP.col(i) += d / smoothedDistance;
      }
      if (i > 0) {
        gradP.col(i - 1) -= d / smoothedDistance;
      }
    }

    Eigen::VectorXd unitQ;
    double sqrNormQ, invNormQ, sqrNormViolation, c, dc;
    for (int i = 0, j = 0, k; i < overlaps; i++, j += k) {
      k = vPolys[2 * i + 1].cols();
      Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
      Eigen::Map<Eigen::VectorXd> gradQ(gradXi.data() + j, k);
      sqrNormQ = q.squaredNorm();
      invNormQ =
          1.0 / std::max(std::sqrt(std::max(0.0, sqrNormQ)),
                         vectorNormEps);
      unitQ = q * invNormQ;
      gradQ.head(k - 1) = (vPolys[2 * i + 1].rightCols(k - 1).transpose() * gradP.col(i)).array() *
                          unitQ.head(k - 1).array() * 2.0;
      gradQ(k - 1) = 0.0;
      gradQ = (gradQ - unitQ * unitQ.dot(gradQ)).eval() * invNormQ;

      sqrNormViolation = sqrNormQ - 1.0;
      if (sqrNormViolation > 0.0) {
        c = sqrNormViolation * sqrNormViolation;
        dc = 3.0 * c;
        c *= sqrNormViolation;
        cost += c;
        gradQ += dc * 2.0 * q;
      }
    }

    return cost;
  }

  static inline bool getShortestPath(const Eigen::Vector3d &ini, const Eigen::Vector3d &fin,
                                     const PolyhedraV &vPolys, const double &smoothD,
                                     const double &vectorNormEps,
                                     Eigen::Matrix3Xd &path) {
    const int overlaps = vPolys.size() / 2;
    Eigen::VectorXi vSizes(overlaps);
    for (int i = 0; i < overlaps; i++) {
      vSizes(i) = vPolys[2 * i + 1].cols();
    }
    Eigen::VectorXd xi(vSizes.sum());
    for (int i = 0, j = 0; i < overlaps; i++) {
      xi.segment(j, vSizes(i)).setConstant(sqrt(1.0 / vSizes(i)));
      j += vSizes(i);
    }

    double minDistance;
    void *dataPtrs[5];
    dataPtrs[0] = (void *)(&smoothD);
    dataPtrs[1] = (void *)(&ini);
    dataPtrs[2] = (void *)(&fin);
    dataPtrs[3] = (void *)(&vPolys);
    dataPtrs[4] = (void *)(&vectorNormEps);
    lbfgs::lbfgs_parameter_t shortest_path_params;
    shortest_path_params.past = 3;
    shortest_path_params.delta = 1.0e-3;
    shortest_path_params.g_epsilon = 1.0e-5;

    const int result =
        lbfgs::lbfgs_optimize(xi, minDistance,
                              &GCOPTER_PolytopeSFC::costDistance, nullptr,
                              nullptr, dataPtrs, shortest_path_params);
    if (result < 0 || !std::isfinite(minDistance) || !xi.allFinite()) {
      return false;
    }

    path.resize(3, overlaps + 2);
    path.leftCols<1>() = ini;
    path.rightCols<1>() = fin;
    Eigen::VectorXd r;
    for (int i = 0, j = 0, k; i < overlaps; i++, j += k) {
      k = vPolys[2 * i + 1].cols();
      Eigen::Map<const Eigen::VectorXd> q(xi.data() + j, k);
      r = (q / std::max(q.norm(), vectorNormEps)).head(k - 1);
      path.col(i + 1) =
      vPolys[2 * i + 1].rightCols(k - 1) * r.cwiseProduct(r) + vPolys[2 * i + 1].col(0);
    }

    return path.allFinite();
  }

  static inline bool processCorridor(const PolyhedraH &hPs, PolyhedraV &vPs) {
    const int sizeCorridor = hPs.size() - 1;

    vPs.clear();
    vPs.reserve(2 * sizeCorridor + 1);

    int nv;
    PolyhedronH curIH;
    PolyhedronV curIV, curIOB;
    for (int i = 0; i < sizeCorridor; i++) {
      if (!geo_utils::enumerateVs(hPs[i], curIV)) {
        return false;
      }
      nv = curIV.cols();
      curIOB.resize(3, nv);
      curIOB.col(0) = curIV.col(0);
      curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
      vPs.push_back(curIOB);

      curIH.resize(hPs[i].rows() + hPs[i + 1].rows(), 4);
      curIH.topRows(hPs[i].rows()) = hPs[i];
      curIH.bottomRows(hPs[i + 1].rows()) = hPs[i + 1];
      if (!geo_utils::enumerateVs(curIH, curIV)) {
        return false;
      }
      nv = curIV.cols();
      curIOB.resize(3, nv);
      curIOB.col(0) = curIV.col(0);
      curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
      vPs.push_back(curIOB);
    }

    if (!geo_utils::enumerateVs(hPs.back(), curIV)) {
      return false;
    }
    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB.col(0) = curIV.col(0);
    curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    vPs.push_back(curIOB);

    return true;
  }

  static inline void setInitial(const Eigen::Matrix3Xd &path,
                                const double maxVelocity,
                                const double maxAcceleration,
                                const double minSegmentTime,
                                const Eigen::VectorXi &intervalNs,
                                Eigen::Matrix3Xd &innerPoints,
                                Eigen::VectorXd &timeAlloc) {
    const int sizeM = intervalNs.size();
    const int sizeN = intervalNs.sum();
    innerPoints.resize(3, sizeN - 1);
    timeAlloc.resize(sizeN);

    Eigen::Vector3d a, b, c;
    for (int i = 0, j = 0, k = 0, l; i < sizeM; i++) {
      l = intervalNs(i);
      a = path.col(i);
      b = path.col(i + 1);
      c = (b - a) / l;
      const double distance = c.norm();
      const double velocityTime = 1.875 * distance / maxVelocity;
      const double accelerationTime =
          std::sqrt(5.7735026919 * distance / maxAcceleration);
      timeAlloc.segment(j, l).setConstant(
          std::max(minSegmentTime,
                   std::max(velocityTime, accelerationTime)));
      j += l;
      for (int m = 0; m < l; m++) {
        if (i > 0 || m > 0) {
          innerPoints.col(k++) = a + c * m;
        }
      }
    }
  }

public:
  // magnitudeBounds = [v_max, a_max]^T
  // penaltyWeights = [corridor_weight, velocity_weight, acceleration_weight]^T
  // initialWeights = [velocity_reference_weight, acceleration_reference_weight]^T
  // terminalWeights = [position_weight, velocity_weight, acceleration_weight]^T
  inline bool setup(const double &timeWeight, const double &dilate_radius,
                    const Eigen::Matrix3d &initialPVA,
                    const Eigen::Matrix3d &terminalPVA,
                    const PolyhedraH &safeCorridor,
                    const double &lengthPerPiece, const double &smoothingFactor,
                    const int &integralResolution,
                    const Eigen::Vector2d &magnitudeBounds,
                    const Eigen::Vector3d &penaltyWeights,
                    const Eigen::Vector2d &initialWeights,
                    const Eigen::Vector3d &terminalWeights,
                    const double &vectorNormEps,
                    const double &minSegmentTime,
                    const double &linearSolvePivotEps) {
    if (integralResolution <= 0 || !std::isfinite(vectorNormEps) ||
        !std::isfinite(minSegmentTime) ||
        !std::isfinite(linearSolvePivotEps) || vectorNormEps <= 0.0 ||
        minSegmentTime <= 0.0 || linearSolvePivotEps <= 0.0) {
      return false;
    }
    if (!std::isfinite(timeWeight) || timeWeight < 0.0 ||
        !std::isfinite(dilate_radius) || dilate_radius < 0.0 ||
        (!std::isfinite(lengthPerPiece) &&
         !(std::isinf(lengthPerPiece) && lengthPerPiece > 0.0)) ||
        !std::isfinite(smoothingFactor) || smoothingFactor <= 0.0 ||
        !initialPVA.allFinite() || !terminalPVA.allFinite() ||
        !magnitudeBounds.allFinite() || !penaltyWeights.allFinite() ||
        !initialWeights.allFinite() || !terminalWeights.allFinite() ||
        (magnitudeBounds.array() <= 0.0).any() ||
        (penaltyWeights.array() < 0.0).any() ||
        (initialWeights.array() < 0.0).any() ||
        (terminalWeights.array() < 0.0).any() ||
        safeCorridor.empty()) {
      return false;
    }
    vector_norm_eps = vectorNormEps;
    min_segment_time = minSegmentTime;
    linear_solve_pivot_eps = linearSolvePivotEps;
    dilate_radius_ = dilate_radius; // 膨胀半径
    rho = timeWeight;
    headPVA = initialPVA;
    tailPVA = terminalPVA;

    hPolytopes = safeCorridor;
    for (size_t i = 0; i < hPolytopes.size(); i++) {
      const Eigen::ArrayXd norms = hPolytopes[i].leftCols<3>().rowwise().norm();
      if (!norms.isFinite().all() || (norms <= vector_norm_eps).any()) {
        return false;
      }
      hPolytopes[i].array().colwise() /= norms;
    } // hPolytope 的每个平面 法向量 归一化
    if (!processCorridor(hPolytopes, vPolytopes)) {
      return false;
    }
    for (const auto &polytope : vPolytopes) {
      if (polytope.cols() < 2 || !polytope.allFinite()) {
        return false;
      }
    }

    polyN = hPolytopes.size();
    smoothEps = smoothingFactor;
    integralRes = integralResolution;
    maxVelocity = magnitudeBounds(0);
    maxAcceleration = magnitudeBounds(1);
    runningPenaltyWeights = penaltyWeights;
    initialStateWeights = initialWeights;
    terminalStateWeights = terminalWeights;

    if (!getShortestPath(headPVA.col(0), tailPVA.col(0), vPolytopes,
                         smoothEps, vector_norm_eps, shortPath) ||
        !shortPath.allFinite()) {
      return false;
    }
    const Eigen::Matrix3Xd deltas = shortPath.rightCols(polyN) - shortPath.leftCols(polyN);
    pieceIdx = (deltas.colwise().norm() / lengthPerPiece).cast<int>().transpose();
    pieceIdx.array() += 1;
    pieceN = pieceIdx.sum();

    temporalDim = pieceN;
    spatialDim = 0;
    vPolyIdx.resize(pieceN - 1);
    hPolyIdx.resize(pieceN);
    for (int i = 0, j = 0, k; i < polyN; i++) {
      k = pieceIdx(i);
      for (int l = 0; l < k; l++, j++) {
        if (l < k - 1) {
          vPolyIdx(j) = 2 * i;
          spatialDim += vPolytopes[2 * i].cols();
        } else if (i < polyN - 1) {
          vPolyIdx(j) = 2 * i + 1;
          spatialDim += vPolytopes[2 * i + 1].cols();
        }
        hPolyIdx(j) = i;
      }
    }

    // MINCO supplies the differentiable quintic representation. Its internal
    // waypoints and segment times remain optimization variables.
    minco.setConditions(headPVA, tailPVA, pieceN);

    // Allocate temp variables
    points.resize(3, pieceN - 1);
    times.resize(pieceN);
    gradByPoints.resize(3, pieceN - 1);
    gradByTimes.resize(pieceN);
    partialGradByCoeffs.resize(6 * pieceN, 3);
    partialGradByTimes.resize(pieceN);

    return true;
  }

  inline bool setup_yaw(const double &rho_vis, const int &integralResolution,
                        const double &yawDiffEps,
                        const double &minSegmentTime,
                        const double &linearSolvePivotEps) {
    if (integralResolution <= 0 || !std::isfinite(yawDiffEps) ||
        !std::isfinite(minSegmentTime) ||
        !std::isfinite(linearSolvePivotEps) || yawDiffEps <= 0.0 ||
        minSegmentTime <= 0.0 || linearSolvePivotEps <= 0.0 ||
        !std::isfinite(rho_vis) || rho_vis < 0.0) {
      return false;
    }
    integralResolution_yaw = integralResolution;
    yaw_rho_vis = rho_vis;
    yaw_diff_eps = yawDiffEps;
    min_segment_time = minSegmentTime;
    linear_solve_pivot_eps = linearSolvePivotEps;
    return true;
  }

  inline double optimize(Trajectory<5> &traj, const double &relCostTol) {
    constexpr int boundaryDim = 15;
    Eigen::VectorXd x(temporalDim + spatialDim + boundaryDim);
    Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim);
    Eigen::Map<Eigen::VectorXd> xi(x.data() + temporalDim, spatialDim);
    Eigen::Map<Eigen::Matrix<double, 3, 2>> optimizedHeadVA(
        x.data() + temporalDim + spatialDim);
    Eigen::Map<Eigen::Matrix3d> optimizedTailPVA(
        x.data() + temporalDim + spatialDim + 6);

    setInitial(shortPath, maxVelocity, maxAcceleration, min_segment_time,
               pieceIdx, points, times);
    backwardT(times, tau, min_segment_time);
    backwardP(points, vPolyIdx, vPolytopes, xi, vector_norm_eps);
    optimizedHeadVA = headPVA.rightCols<2>();
    optimizedTailPVA = tailPVA;

    double minCostFunctional;
    lbfgs_params.mem_size = 256;
    lbfgs_params.past = 3;
    lbfgs_params.min_step = 1.0e-20;
    lbfgs_params.g_epsilon = FLT_EPSILON;
    lbfgs_params.delta = relCostTol;

    int ret = lbfgs::lbfgs_optimize(x, minCostFunctional, &GCOPTER_PolytopeSFC::costFunctional,
                                    nullptr, nullptr, this, lbfgs_params);
    // if (ret == lbfgs::LBFGSERR_MINIMUMSTEP) {
    //   ROS_ERROR("min-step!!!!");
    //   Eigen::Vector3d start = headPVA.col(0);
    //   Eigen::Vector3d end = tailPVA.col(0);
    //   if ((start - end).norm() > 0.5) {
    //     minCostFunctional = INFINITY;
    //     std::cout << "Optimization Failed: " << lbfgs::lbfgs_strerror(ret) << std::endl;
    //   } else {
    //     Eigen::Vector3d center = (start + end) / 2.0;
    //     points.resize(3, 1);
    //     points.col(0) = center;
    //     times.resize(2);
    //     minco.setParameters(points, times);
    //     minco.getTrajectory(traj);
    //   }

    // } else 
    if (ret >= 0 && std::isfinite(minCostFunctional) && x.allFinite()) {
      forwardT(tau, times, min_segment_time);
      forwardP(xi, vPolyIdx, vPolytopes, points, vector_norm_eps);
      Eigen::Matrix3d optimizedHeadPVA = headPVA;
      optimizedHeadPVA.rightCols<2>() = optimizedHeadVA;
      if (!minco.setBoundaryConditions(optimizedHeadPVA,
                                       optimizedTailPVA) ||
          !minco.setParameters(points, times, linear_solve_pivot_eps)) {
        return INFINITY;
      }
      minco.getTrajectory(traj);
    } else {
      // traj.clear();
      minCostFunctional = INFINITY;
      std::cout << "Optimization Failed: " << lbfgs::lbfgs_strerror(ret) << std::endl;
    }

    return minCostFunctional;
  }

  inline double optimize_yaw(const Eigen::Matrix3d &inityaw, const Eigen::Matrix3d &endyaw,
                             const int &pieceNum, const Eigen::Matrix3Xd &yaw_points,
                             const Eigen::VectorXd &ts, Trajectory<5> &traj) {
    use_shorten_path = false;
    // if(use_shorten_path){
    guide_path.setZero();
    guide_path = yaw_points;
    // cout<<"guide_path: "<<guide_path.cols()<<endl;
    // cout<<"pieceNum: "<<pieceNum<<endl;
    // }

    if (pieceNum <= 0 || ts.size() != pieceNum || !inityaw.allFinite() ||
        !endyaw.allFinite() || !yaw_points.allFinite()) {
      return INFINITY;
    }

    // The yaw penalty uses these as the first and last guide states.
    headPVA = inityaw;
    tailPVA = endyaw;

    Eigen::VectorXd x_yaw, init_tau, yaw_vec;
    yaw_vec = Eigen::Map<const Eigen::VectorXd>(yaw_points.data(), yaw_points.size());
    int yaw_pt_count = yaw_vec.size();
    temporalDim_yaw = pieceNum;
    spatialDim_yaw = yaw_pt_count;
    int opt_dim = pieceNum + yaw_pt_count;
    x_yaw.resize(opt_dim);
    Eigen::VectorXd safe_ts = ts.unaryExpr([this](double t) {
      return std::isfinite(t) ? std::max(t, min_segment_time)
                              : min_segment_time;
    });
    yaw_minco.setConditions(inityaw, endyaw, pieceNum);
    if (!yaw_minco.setParameters(guide_path, safe_ts,
                                 linear_solve_pivot_eps)) {
      return INFINITY;
    }
    backwardT(safe_ts, init_tau, min_segment_time);
    x_yaw.segment(0, pieceNum) = init_tau;
    x_yaw.segment(pieceNum, yaw_pt_count) = yaw_vec;

    double minCostFunctional;
    lbfgs_params.mem_size = 256;
    lbfgs_params.past = 3;
    lbfgs_params.min_step = 1.0e-20;
    lbfgs_params.g_epsilon = 1e-5;
    lbfgs_params.delta = 1e-5;

    // int ret =0;
    int ret =
    lbfgs::lbfgs_optimize(x_yaw, minCostFunctional, &GCOPTER_PolytopeSFC::costFunctional_yaw,
                          nullptr, nullptr, this, lbfgs_params);
    if (ret >= 0 && std::isfinite(minCostFunctional) &&
        x_yaw.allFinite()) {
      const Eigen::VectorXd optimized_tau =
          Eigen::Map<const Eigen::VectorXd>(x_yaw.data(), pieceNum);
      Eigen::VectorXd t;
      forwardT(optimized_tau, t, min_segment_time);
      Eigen::Map<const Eigen::Matrix3Xd> optimized_yaw_points(
          x_yaw.data() + pieceNum, 3, yaw_pt_count / 3);
      if (!yaw_minco.setParameters(optimized_yaw_points, t,
                                   linear_solve_pivot_eps)) {
        return INFINITY;
      }
      yaw_minco.getTrajectory(traj);
    } else {
      // traj.clear();
      minCostFunctional = INFINITY;
      std::cout << "Optimization Failed: " << lbfgs::lbfgs_strerror(ret) << std::endl;
    }

    return minCostFunctional;
  }
};

} // namespace gcopter

#endif
