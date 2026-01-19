#pragma once

#include "types.hpp"

namespace aimrl_sdk {

struct FrameLayout final {
  static constexpr int ArmPos0 = 0;
  static constexpr int ArmVel0 = ArmPos0 + kArmDof;
  static constexpr int ArmEff0 = ArmVel0 + kArmDof;

  static constexpr int LegPos0 = ArmEff0 + kArmDof;
  static constexpr int LegVel0 = LegPos0 + kLegDof;
  static constexpr int LegEff0 = LegVel0 + kLegDof;

  static constexpr int ImuQuat0 = LegEff0 + kLegDof;
  static constexpr int ImuGyro0 = ImuQuat0 + 4;
  static constexpr int ImuAcc0 = ImuGyro0 + 3;

  static constexpr int Dim = kFrameDim;
  static_assert(Dim == 88);
};

}  // namespace aimrl_sdk
