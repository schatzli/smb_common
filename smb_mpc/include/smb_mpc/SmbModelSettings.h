//
// Created by johannes on 29.04.19.
//

#pragma once

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <iostream>
#include <string>

#include <ocs2_core/misc/LoadData.h>

namespace smb_path_following {

struct SmbModelSettings {
  /** flag to generate dynamic files **/
  bool recompileLibraries_ = true;
  std::string systemName_ = "smb";

  bool activateObstacleAvoidance_ = false;

  virtual void loadSettings(const std::string& filename, bool verbose = true);
};

inline void SmbModelSettings::loadSettings(const std::string& filename, bool verbose /*= true*/) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  if (verbose) {
    std::cerr << "\n #### Robot Model Settings:";
    std::cerr << "\n #### ==================================================\n";
  }

  ocs2::loadData::loadPtreeValue(pt, recompileLibraries_, "model_settings.recompileLibraries", verbose);
  ocs2::loadData::loadPtreeValue(pt, systemName_, "model_settings.systemName", verbose);

  ocs2::loadData::loadPtreeValue(pt, activateObstacleAvoidance_, "model_settings.activateObstacleAvoidance", verbose);

  if (verbose) {
    std::cerr << " #### ==================================================\n" << std::endl;
  }
}

}  // namespace smb_path_following
