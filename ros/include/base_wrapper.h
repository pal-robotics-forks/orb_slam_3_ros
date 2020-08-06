/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José
* M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University
* of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of
* the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License,
* or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <ros/ros.h>
#include "System.h"

class BaseWrapper
{
public:
  /**
   * @brief BaseWrapper c'tor
   * @param sensor
   * @param nh
   */
  BaseWrapper(const ORB_SLAM3::System::eSensor& sensor, ros::NodeHandle& nh);


  /**
   * @brief BaseWrapper d'tor
   */ ~BaseWrapper();

protected:
  // ros node handle
  ros::NodeHandle nh_;

  // orb slam system (to perform mapping and localization)
  std::shared_ptr<ORB_SLAM3::System> system_;
  std::string vocabulary_filename_;
  std::string settings_filename_;
};
