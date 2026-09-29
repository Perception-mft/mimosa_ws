// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// Eigen
#include <Eigen/Core>

namespace mimosa
{
enum class NavigationFrameConvention
{
  ENU,
  NED,
};

inline Eigen::Vector3d gravityDirection(const NavigationFrameConvention convention)
{
  return Eigen::Vector3d(
    0.0, 0.0, convention == NavigationFrameConvention::ENU ? -1.0 : 1.0);
}
}  // namespace mimosa
