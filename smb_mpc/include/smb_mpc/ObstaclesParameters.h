/*
 * WacoWeightedCost.h
 *
 *  Created on: July 4, 2021
 *      Author: Farbod Farshidian
 */


#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>

namespace smb_path_following {

struct ObstaclesParameters {
  void display() {
    std::cerr << "Obstacles parameters: " << std::endl;
    std::cerr << "numberOfParamsPerObstacle:   " << numberOfParamsPerObstacle_ << std::endl;
    std::cerr << "numberOfObstacles:   " << numberOfObstacles_ << std::endl;
    std::cerr << "vectorOfObstacles:   \n" << vectorOfObstacles_ << std::endl;
  }

  /**
   * Loads the obstacles parameters.
   */
  void loadSettings(const std::string& filename, const std::string& fieldName, bool verbose = true) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);
    if (verbose) {
      std::cerr << "\n #### Obstacles Parameters:";
      std::cerr << "\n #### =========================================\n";
    }
    ocs2::loadData::loadPtreeValue(pt, numberOfParamsPerObstacle_, fieldName + ".numberOfParamsPerObstacle", verbose);
    ocs2::loadData::loadPtreeValue(pt, numberOfObstacles_, fieldName + ".numberOfObstacles", verbose);
    vectorOfObstacles_.resize(numberOfParamsPerObstacle_ * numberOfObstacles_);
    ocs2::loadData::loadEigenMatrix(filename, fieldName + ".vectorOfObstacles", vectorOfObstacles_);
  }

  // For safety, these parameters cannot be modified
  size_t numberOfParamsPerObstacle_ = 4;
  size_t numberOfObstacles_ = 1;
  ocs2::vector_t vectorOfObstacles_;
};

} // namespace smb_path_following
