/*
 * WacoWeightedCost.h
 *
 *  Created on: July 4, 2021
 *      Author: Farbod Farshidian
 */


#pragma once

#include <cmath>

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/ConstraintBase.h>

#include "smb_mpc/ObstaclesParameters.h"

namespace smb_path_following {

class SmbConstraints final : public ocs2::ConstraintBase {
 public:
  using scalar_t = ocs2::scalar_t;
  using vector_t = ocs2::vector_t;
  using matrix_t = ocs2::matrix_t;

  /** Constructor */
  SmbConstraints(ObstaclesParameters obstaclesParam) : obstaclesParam_(std::move(obstaclesParam)) {}

  ~SmbConstraints() override = default;
  SmbConstraints* clone() const override { return new SmbConstraints(*this); }

  vector_t inequalityConstraint(scalar_t t, const vector_t& x, const vector_t& u) override {
    vector_t inequalityConstraints(1);
    inequalityConstraints(0) = getDistance(t, x).second;
    return inequalityConstraints;
  }

  ocs2::VectorFunctionQuadraticApproximation inequalityConstraintQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                        const vector_t& u) override {
    ocs2::VectorFunctionQuadraticApproximation approx =
        ocs2::VectorFunctionQuadraticApproximation::Zero(1, x.size(), u.size());

    vector_t weight;
    std::tie(weight, approx.f(0)) = getDistance(t, x);

    for (size_t i = 0; i < obstaclesParam_.numberOfObstacles_; i++) {
      const size_t j = obstaclesParam_.numberOfParamsPerObstacle_ * i;
      approx.dfdx(0) += weight(i) * 2.0 * (x(0) - obstaclesParam_.vectorOfObstacles_(j + 2));
      approx.dfdx(1) += weight(i) * 2.0 * (x(1) - obstaclesParam_.vectorOfObstacles_(j + 3));
    }

    // ignore the second order derivatives since it makes the Hessian matrix indefinite
    // approx.dfdxx[0](0, 0) = 2.0;
    // approx.dfdxx[0](1, 1) = 2.0;

    return approx;
  }

 private:
  SmbConstraints(const SmbConstraints& other) = default;

  std::pair<vector_t, scalar_t> getDistance(scalar_t t, const vector_t& x) {
    vector_t distance(obstaclesParam_.numberOfObstacles_);
    for (size_t i = 0; i < obstaclesParam_.numberOfObstacles_; i++) {
      const int j = obstaclesParam_.numberOfParamsPerObstacle_ * i;
      distance(i) = std::pow((x(0) - obstaclesParam_.vectorOfObstacles_(j + 2)), 2) +
                    std::pow((x(1) - obstaclesParam_.vectorOfObstacles_(j + 3)), 2) -
                    std::pow((obstaclesParam_.vectorOfObstacles_(j) + smbRadius_), 2);
    }

    const vector_t minDistance = distance.minCoeff() * vector_t::Ones(obstaclesParam_.numberOfObstacles_);
    const vector_t distanceNormalized = -10.0 * (distance - minDistance);
    const vector_t expDistance = distanceNormalized.array().exp();
    const vector_t weight  = expDistance / expDistance.sum();
    const scalar_t softMinDistance = distance.dot(weight);

    //  int minElement;
    //  const scalar_t minDistance = distance.minCoeff(&minElement);
    //  return {minElement, minDistance};

    return {weight, softMinDistance};
  }

  const ocs2::scalar_t smbRadius_ = 0.3;
  ObstaclesParameters obstaclesParam_;
};

} // namespace smb_path_following
