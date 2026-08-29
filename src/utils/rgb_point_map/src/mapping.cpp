/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Apr. 2025
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file is the independent test script of the rgbd mapping in FC-Vision.
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

#include "rgb_point_map/rgb_point_map.h"
#include <ros/ros.h>

using namespace std;
using namespace fc_vision;

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "rgb_point_mapping_node");
  ros::NodeHandle nh("~");

  std::shared_ptr<RGBPointMap> rgb_map(new RGBPointMap);
  rgb_map->init(nh);
  // rgb_map->start(); // auto start mapping

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}