/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2025
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the execution file of FC-Vision.
 * License      :    GNU General Public License <http://www.gnu.org/licenses/>.
 * Project      :    FC-Vision is free software: you can redistribute it and/or
 *                   modify it under the terms of the GNU Lesser General Public
 *                   License as published by the Free Software Foundation,
 *                   either version 3 of the License, or (at your option) any
 *                   later version.
 *                   FC-Vision is distributed in the hope that it will be useful,
 *                   but WITHOUT ANY WARRANTY; without even the implied warranty
 *                   of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *                   See the GNU General Public License for more details.
 * Website      :    https://hkust-aerial-robotics.github.io/FC-Vision/
 *⭐⭐⭐*****************************************************************⭐⭐⭐*/

#include "fc_vision_manager/backward.hpp"
#include "fc_vision_manager/manager_node.h"
#include <csignal>
#include <ros/ros.h>
namespace backward
{
  backward::SignalHandling sh;
}

using namespace std;
using namespace fc_vision;
using fc_vision::ManagerNode;

namespace
{
volatile std::sig_atomic_t shutdown_requested = 0;

void handleSigint(int)
{
  shutdown_requested = 1;
}
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "fc_vision_manager", ros::init_options::NoSigintHandler);
  std::signal(SIGINT, handleSigint);
  std::signal(SIGTERM, handleSigint);
  ros::NodeHandle nh("~");

  // * Module Initialization
  ManagerNode manager_node;
  manager_node.init(nh);

  ros::Duration(1.0).sleep();
  ros::WallRate spin_rate(100.0);
  while (ros::ok() && !shutdown_requested)
  {
    ros::spinOnce();
    spin_rate.sleep();
  }
  manager_node.shutdown();
  ros::shutdown();

  return 0;
}
