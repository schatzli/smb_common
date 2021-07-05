/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include "smb_mpc/ObstaclesParameters.h"

namespace smb_path_following {

class SmbSynchronizedModule final : public ocs2::SolverSynchronizedModule {
public:
  using scalar_t = ocs2::scalar_t;
  using vector_t = ocs2::vector_t;
  using matrix_t = ocs2::matrix_t;

  SmbSynchronizedModule(ObstaclesParameters obstaclesParameters)
  : obstaclesParameters_(std::move(obstaclesParameters)) {}

  ~SmbSynchronizedModule() override = default;

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ocs2::CostDesiredTrajectories& costDesiredTrajectory) override {}

  void postSolverRun(const ocs2::PrimalSolution& primalSolution) override {}

  const ObstaclesParameters& getObstaclesParameters() const { return obstaclesParameters_; }

 private:
  ObstaclesParameters obstaclesParameters_;
};

}  // namespace smb_path_following
