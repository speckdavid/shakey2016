//=================================================================================================
// Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================


#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <limits>

class LaserScanFilter
{
public:
  LaserScanFilter()
  {
    ros::NodeHandle nh;

    scan_sub_ = nh.subscribe("scan", 3, &LaserScanFilter::scanCallback, this);
    scan_filtered_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan_filtered",1,false);
    ros::NodeHandle pnh("~");
    pnh.param("use_rep_117", use_rep_117_, true);
    ROS_INFO("~use_rep_117: %d", use_rep_117_);
    XmlRpc::XmlRpcValue my_list;
    pnh.getParam("filter_index_list", my_list);
    if (my_list.getType() != XmlRpc::XmlRpcValue::TypeArray) ros::shutdown();

    for (int32_t i = 0; i < my_list.size(); ++i)
    {
      if (my_list[i].getType() != XmlRpc::XmlRpcValue::TypeArray) ros::shutdown();

      int min = my_list[i][0];
      int max = my_list[i][1];

      addFilterIndices(min, max);

      ROS_INFO("scan filter index interval %d : min: %d max: %d",i, min, max);
    }
  }

  void scanCallback(const sensor_msgs::LaserScan& scan)
  {
    this->pubFilteredScan(scan);
  }

  void pubFilteredScan(const sensor_msgs::LaserScan& scan)
  {
    filtered_scan_ = scan;

    size_t filter_indices_size = filter_indices_.size();

    for (size_t i = 0; i < filter_indices_size; ++i)
    {
        if(use_rep_117_)    // values are too close to the robot to be good
            filtered_scan_.ranges[filter_indices_[i]] = -std::numeric_limits<double>::infinity();
        else
            filtered_scan_.ranges[filter_indices_[i]] = scan.range_max + 1.0;
    }

    scan_filtered_pub_.publish(filtered_scan_);
  }

  void addFilterIndices(size_t min, size_t max)
  {
    for (size_t i = min; i < max; ++i){
      filter_indices_.push_back(i);
    }
  }

protected:
  ros::Subscriber scan_sub_;
  ros::Publisher scan_filtered_pub_;

  sensor_msgs::LaserScan filtered_scan_;

  std::vector<size_t> filter_indices_;

  bool use_rep_117_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_turtlebot_scan_filter");

  LaserScanFilter lsf;

  ros::spin();

  return 0;
}
