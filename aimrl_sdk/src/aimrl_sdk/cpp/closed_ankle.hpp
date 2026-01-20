#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace aimrl_sdk {

struct ClosedAnkleParams {
  int motor1_direction{1};
  int motor2_direction{1};
  int pitch_direction{1};
  int roll_direction{1};

  double d{0.0315};      // m
  double l{0.063};       // m
  double h1{0.239};      // m
  double h2{0.145};      // m

  double actuator_pos_limit{1.0};  // rad
  double pitch_limit{1.0};         // rad
  double roll_limit{0.5};          // rad
};

namespace closed_ankle_detail {

struct Vec2 {
  double v0{0.0};
  double v1{0.0};
};

struct Mat2 {
  double a00{1.0}, a01{0.0}, a10{0.0}, a11{1.0};

  Vec2 mul(Vec2 x) const {
    return Vec2{a00 * x.v0 + a01 * x.v1, a10 * x.v0 + a11 * x.v1};
  }

  Mat2 transpose() const { return Mat2{a00, a10, a01, a11}; }

  Mat2 inverse() const {
    const double det = a00 * a11 - a01 * a10;
    const double eps = 1e-12;
    if (!std::isfinite(det) || std::abs(det) <= eps) {
      return Mat2{};  // identity fallback
    }
    const double inv_det = 1.0 / det;
    return Mat2{a11 * inv_det, -a01 * inv_det, -a10 * inv_det, a00 * inv_det};
  }
};

inline double clamp_abs(double x, double limit) {
  if (limit <= 0.0)
    return x;
  if (x > limit)
    return limit;
  if (x < -limit)
    return -limit;
  return x;
}

class LoopAnkleAnalyticalSolver {
 public:
  void SetLinkLength(double dd, double ll1, double hh1, double hh2) {
    d_ = dd;
    l1_ = ll1;
    h1_ = hh1;
    h2_ = hh2;
  }

  Vec2 IK(Vec2 pr) const {
    // pr = [pitch, roll]
    const double tx = pr.v1;
    const double ty = pr.v0;

    const double cx = std::cos(tx);
    const double sx = std::sin(tx);
    const double cy = std::cos(ty);
    const double sy = std::sin(ty);

    const double AL = -l1_ * l1_ * cy + l1_ * d_ * sx * sy;
    const double BL = -l1_ * l1_ * sy + l1_ * h1_ - l1_ * d_ * sx * cy;
    const double CL =
        -(l1_ * l1_ + d_ * d_ - d_ * d_ * cx - l1_ * h1_ * sy - d_ * h1_ * sx * cy);
    const double LenL = std::sqrt(AL * AL + BL * BL);

    const double AR = -l1_ * l1_ * cy - l1_ * d_ * sx * sy;
    const double BR = -l1_ * l1_ * sy + l1_ * h2_ + l1_ * d_ * sx * cy;
    const double CR =
        -(l1_ * l1_ + d_ * d_ - d_ * d_ * cx - l1_ * h2_ * sy + d_ * h2_ * sx * cy);
    const double LenR = std::sqrt(AR * AR + BR * BR);

    if (LenL <= std::abs(CL) || LenR <= std::abs(CR)) {
      return Vec2{};
    }

    const double tL_1 = std::asin(CL / LenL) - std::asin(AL / LenL);
    const double tR_1 = std::asin(CR / LenR) - std::asin(AR / LenR);
    return Vec2{tL_1, tR_1};
  }

  Vec2 FK(Vec2 mc) const {
    const double mL = mc.v0;
    const double mR = mc.v1;

    const double halfSumAngles = (mL / 2.0) + (mR / 2.0);
    const double sinHalfSum = std::sin(halfSumAngles);
    const double cosHalfSum = std::cos(halfSumAngles);

    const double numerator =
        l1_ * (h2_ - h1_) * sinHalfSum + l1_ * h1_ * std::sin(mL) - l1_ * h2_ * std::sin(mR);

    double denominator = d_ * (h1_ + h2_) * cosHalfSum;
    if (std::abs(denominator) <= 1e-3) {
      denominator = 1e-3;
    }

    double sinTx = numerator / denominator;
    if (sinTx < -1.0)
      sinTx = -1.0;
    if (sinTx > 1.0)
      sinTx = 1.0;

    // returns [pitch, roll]
    return Vec2{0.5 * (mL + mR), std::asin(sinTx)};
  }

  void UpdateJ1(Vec2 mc) {
    j1_.a00 = 0.5;
    j1_.a01 = 0.5;
    j1_.a10 = CalculateTxML_(mc);
    j1_.a11 = CalculateTxMR_(mc);
  }

  Vec2 DFK(Vec2 vel_m) const { return j1_.mul(vel_m); }

  Vec2 DIK(Vec2 vel_pr) const { return j1_.inverse().mul(vel_pr); }

  Vec2 FDyn(Vec2 tau_m) const {
    const auto j2t = j1_.inverse().transpose();
    return j2t.mul(tau_m);
  }

  Vec2 IDyn(Vec2 tau_pr) const {
    const auto jt = j1_.transpose();
    return jt.mul(tau_pr);
  }

 private:
  double CalculateTxML_(Vec2 mc) {
    double tML = mc.v0;
    double tMR = mc.v1;

    tML = clamp_abs(tML, 1.52);
    tMR = clamp_abs(tMR, 1.52);

    const double t5 = 1.0 / d_;
    const double t12 = 1.0 / (h1_ + h2_);
    double t14 = tML / 2.0 + tMR / 2.0;
    const double t15 = std::cos(t14);
    t14 = std::sin(t14);
    const double t17 = 1.0 / (t15 * t15);

    const double t19_tmp = l1_ * (h1_ - h2_);
    const double b_t19_tmp = l1_ * h1_;
    const double t19 = (l1_ * h2_ * std::sin(tMR) - b_t19_tmp * std::sin(tML)) + t19_tmp * t14;
    const double tXtML_tmp = t5 * t12;

    double result =
        -1.0 / std::sqrt(-(t5 * t5) * (t12 * t12) * t17 * (t19 * t19) + 1.0) *
        (tXtML_tmp * (t19_tmp * t15 / 2.0 - b_t19_tmp * std::cos(tML)) / t15 +
         tXtML_tmp * t14 * t17 * t19 / 2.0);
    if (!std::isfinite(result)) {
      result = pre_tx_ml_;
    } else {
      pre_tx_ml_ = result;
    }
    return result;
  }

  double CalculateTxMR_(Vec2 mc) {
    double tML = mc.v0;
    double tMR = mc.v1;

    tML = clamp_abs(tML, 1.52);
    tMR = clamp_abs(tMR, 1.52);

    const double t5 = 1.0 / d_;
    const double t12 = 1.0 / (h1_ + h2_);
    double t14 = tML / 2.0 + tMR / 2.0;
    const double t15 = std::cos(t14);
    t14 = std::sin(t14);
    const double t17 = 1.0 / (t15 * t15);

    const double t19_tmp = l1_ * (h1_ - h2_);
    const double b_t19_tmp = l1_ * h2_;
    const double t19 = (b_t19_tmp * std::sin(tMR) - l1_ * h1_ * std::sin(tML)) + t19_tmp * t14;
    const double tXtMR_tmp = t5 * t12;

    double result =
        -1.0 / std::sqrt(-(t5 * t5) * (t12 * t12) * t17 * (t19 * t19) + 1.0) *
        (tXtMR_tmp * (t19_tmp * t15 / 2.0 + b_t19_tmp * std::cos(tMR)) / t15 +
         tXtMR_tmp * t14 * t17 * t19 / 2.0);
    if (!std::isfinite(result)) {
      result = pre_tx_mr_;
    } else {
      pre_tx_mr_ = result;
    }
    return result;
  }

 private:
  Mat2 j1_{};
  double d_{0.0315};
  double l1_{0.062992};
  double h1_{0.145};
  double h2_{0.239};
  double pre_tx_ml_{0.0};
  double pre_tx_mr_{0.0};
};

inline Vec2 solver_mc_from_actual(int leg_index, Vec2 actual_mc, const ClosedAnkleParams &p) {
  const double m1 = static_cast<double>(p.motor1_direction);
  const double m2 = static_cast<double>(p.motor2_direction);
  if (leg_index == 0) {
    return Vec2{actual_mc.v1 * m1, actual_mc.v0 * m2};  // swap
  }
  return Vec2{actual_mc.v0 * m1, actual_mc.v1 * m2};
}

inline Vec2 actual_mc_from_solver(int leg_index, Vec2 solver_mc, const ClosedAnkleParams &p) {
  const double m1 = static_cast<double>(p.motor1_direction);
  const double m2 = static_cast<double>(p.motor2_direction);
  if (leg_index == 0) {
    return Vec2{solver_mc.v1 * m2, solver_mc.v0 * m1};  // swap back
  }
  return Vec2{solver_mc.v0 * m1, solver_mc.v1 * m2};
}

inline Vec2 apply_pr_direction(Vec2 pr, const ClosedAnkleParams &p) {
  pr.v0 *= static_cast<double>(p.pitch_direction);
  pr.v1 *= static_cast<double>(p.roll_direction);
  return pr;
}

inline Vec2 remove_pr_direction(Vec2 pr, const ClosedAnkleParams &p) {
  // direction is +/-1, inverse is itself
  return apply_pr_direction(pr, p);
}

inline void solver_link_lengths_for_leg(LoopAnkleAnalyticalSolver &solver, int leg_index,
                                       const ClosedAnkleParams &p) {
  if (leg_index == 0) {
    solver.SetLinkLength(p.d, p.l, p.h2, p.h1);
  } else {
    solver.SetLinkLength(p.d, p.l, p.h1, p.h2);
  }
}

}  // namespace closed_ankle_detail

}  // namespace aimrl_sdk

