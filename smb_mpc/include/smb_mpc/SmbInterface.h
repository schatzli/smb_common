//
// Created by johannes on 29.04.19.
//

#pragma once

// C++
#include <iostream>
#include <stdlib.h>
#include <string>

// OCS2
#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mpc/MPC_DDP.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

#include "smb_mpc/ObstaclesParameters.h"
#include "smb_mpc/SmbDefinitions.h"
#include "smb_mpc/SmbModelSettings.h"
#include "smb_mpc/SmbSynchronizedModule.h"

// ros
#include <ros/package.h>
#include <ros/ros.h>

namespace smb_path_following {

class SmbInterface : public ocs2::RobotInterface {
public:
  /** Constructor */
  SmbInterface();

  /** Destructor */
  ~SmbInterface() override = default;

  const ocs2::SystemDynamicsBase& getDynamics() const override { return *dynamicsPtr_; }
  const ocs2::CostFunctionBase& getCost() const override { return *costPtr_; }
  const ocs2::OperatingPoints& getOperatingPoints() const override { return *operatingPointPtr_; }
  const ocs2::ConstraintBase* getConstraintPtr() const override { return constraintPtr_.get(); }

  /** Gets the model settings. */
  const SmbModelSettings& modelSettings() const { return modelSettings_; }
  /** Gets the mpc settings. */
  const ocs2::mpc::Settings& mpcSettings() const { return mpcSettings_; }
  /** Gets the mpc settings. */
  const ocs2::ddp::Settings& ddpSettings() const { return ddpSettings_; }

  const SmbSynchronizedModule& getSmbSynchronizedModule() const { return *smbSynchronizedModulePtr_; }

  /** Gets a reference to the internal SLQ_MPC. */
  ocs2::MPC_DDP& getMPC() { return *mpcPtr_; }

private:
  SmbModelSettings modelSettings_;
  ocs2::mpc::Settings mpcSettings_;
  ocs2::ddp::Settings ddpSettings_;

  std::unique_ptr<ocs2::RolloutBase> ddpSmbRolloutPtr_;
  std::unique_ptr<ocs2::SystemDynamicsBase> dynamicsPtr_;
  std::unique_ptr<ocs2::CostFunctionBase> costPtr_;
  std::unique_ptr<ocs2::OperatingPoints> operatingPointPtr_;
  std::unique_ptr<ocs2::ConstraintBase> constraintPtr_;

  ObstaclesParameters obstaclesParam_;
  std::shared_ptr<SmbSynchronizedModule> smbSynchronizedModulePtr_;

  std::unique_ptr<ocs2::MPC_DDP> mpcPtr_;
};

} // namespace smb_path_following
