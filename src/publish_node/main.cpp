/**
 * @file main.cpp
 * @author LDRobot (marketing1@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD06 products 
 * sold by Shenzhen LDROBOT Co., LTD    
 * @version 0.1
 * @date 2021-10-28
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>

#include <iostream>
#include <string>

#include "ros_api.h"
#include "cmd_interface_linux.h"
#include "lipkg.h"

void  ToLaserscanMessagePublish(Points2D& src, LiPkg* commpkg, LaserScanSetting& setting, ros::Publisher& lidarpub);

int main(int argc, char **argv) {
  ros::init(argc, argv, "ldldiar_publisher");
  ros::NodeHandle nh;  // create a ROS Node
  ros::NodeHandle nh_private("~");
  std::string product_name;
	std::string topic_name;
	std::string port_name;
  LaserScanSetting setting;
	
  nh_private.getParam("product_name", product_name);
	nh_private.getParam("topic_name", topic_name);
	nh_private.getParam("port_name", port_name);
	nh_private.getParam("frame_id", setting.frame_id);
  nh_private.param("laser_scan_dir", setting.laser_scan_dir, bool(true));
  nh_private.param("enable_angle_crop_func", setting.enable_angle_crop_func, bool(false));
  nh_private.param("angle_crop_min", setting.angle_crop_min, double(0.0));
  nh_private.param("angle_crop_max", setting.angle_crop_max, double(0.0));
  
  ROS_INFO("[ldrobot] SDK Pack Version is v2.3.1");
  ROS_INFO("[ldrobot] <product_name>: %s,<topic_name>: %s,<port_name>: %s,<frame_id>: %s", 
    product_name.c_str(), topic_name.c_str(), port_name.c_str(), setting.frame_id.c_str());

  ROS_INFO("[ldrobot] <laser_scan_dir>: %s,<enable_angle_crop_func>: %s,<angle_crop_min>: %f,<angle_crop_max>: %f",
   (setting.laser_scan_dir?"Counterclockwise":"Clockwise"), (setting.enable_angle_crop_func?"true":"false"), setting.angle_crop_min, setting.angle_crop_max);

  LiPkg *lidar = new LiPkg(product_name);
  CmdInterfaceLinux cmd_port;

  if (port_name.empty()) {
    ROS_ERROR("[ldrobot] Can't find %s device", product_name.c_str());
    exit(EXIT_FAILURE);
  } else {
    ROS_INFO("[ldrobot] FOUND %s device", product_name.c_str());
    cmd_port.SetReadCallback(std::bind(&LiPkg::CommReadCallback, lidar, std::placeholders::_1, std::placeholders::_2));
  }

  if (cmd_port.Open(port_name)) {
    ROS_INFO("[ldrobot] open %s device %s is success!", product_name.c_str(), port_name.c_str());
  }else {
    ROS_ERROR("[ldrobot] open %s device %s is fail!", product_name.c_str(), port_name.c_str());
    exit(EXIT_FAILURE);
  }
  
  ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>(topic_name, 10);  // create a ROS topic
  
  ros::Rate r(10); //10hz
  while (ros::ok()) {
    if (lidar->IsFrameReady()) {
      lidar->ResetFrameReady();
      Points2D laserscandata = lidar->GetLaserScanData();
      ToLaserscanMessagePublish(laserscandata, lidar, setting, lidar_pub);
    }
    r.sleep();
  }
  return 0;
}

void  ToLaserscanMessagePublish(Points2D& src, LiPkg* commpkg, LaserScanSetting& setting, ros::Publisher& lidarpub) {
  float angle_min, angle_max, range_min, range_max, angle_increment;
  float scan_time;
  static ros::Time start_scan_time;
  static ros::Time end_scan_time;

  start_scan_time = ros::Time::now();
  scan_time = (start_scan_time - end_scan_time).toSec();

  // Adjust the parameters according to the demand
  angle_min = ANGLE_TO_RADIAN(src.front().angle);
  angle_max = ANGLE_TO_RADIAN(src.back().angle);
  range_min = 0.02;
  range_max = 12;
  float spin_speed = static_cast<float>(commpkg->GetSpeedOrigin());
  float scan_freq = static_cast<float>(commpkg->kPointFrequence);
  angle_increment = ANGLE_TO_RADIAN(spin_speed/scan_freq);
  // ROS_INFO_STREAM("[ldrobot] angle min: " << src.front().angle);
  // ROS_INFO_STREAM("[ldrobot] angle max: " << src.back().angle);
  // ROS_INFO_STREAM("[ldrobot] speed(hz): " << GetSpeed());

  // Calculate the number of scanning points
  if (commpkg->GetSpeedOrigin() > 0) {
    unsigned int beam_size = static_cast<unsigned int>(ceil((angle_max - angle_min) / angle_increment));
    // ROS_INFO_STREAM("[ldrobot] beam_size: " << beam_size);
    sensor_msgs::LaserScan output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = setting.frame_id;
    output.angle_min = angle_min;
    output.angle_max = angle_max;
    output.range_min = range_min;
    output.range_max = range_max;
    output.angle_increment = angle_increment;
    if (beam_size <= 1) {
      output.time_increment = 0;
    } else {
      output.time_increment = scan_time / (beam_size - 1);
    }
    output.scan_time = scan_time;
    // First fill all the data with Nan
    output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

    unsigned int last_index = 0;
    for (auto point : src) {
      float range = point.distance / 1000.f;  // distance unit transform to meters
      float intensity = point.intensity;      // laser receive intensity 
      float dir_angle;

      if ((point.distance == 0) && (point.intensity == 0)) { // filter is handled to  0, Nan will be assigned variable.
        range = std::numeric_limits<float>::quiet_NaN(); 
        intensity = std::numeric_limits<float>::quiet_NaN();
      }

      if (setting.laser_scan_dir) {
        dir_angle = static_cast<float>(360.f - point.angle); // Lidar rotation data flow changed from clockwise to counterclockwise
      } else {
        dir_angle = point.angle;
      }

      if (setting.enable_angle_crop_func) { // Angle crop setting, Mask data within the set angle range
        if ((dir_angle >= setting.angle_crop_min) && (dir_angle <= setting.angle_crop_max)) {
          range = std::numeric_limits<float>::quiet_NaN();
          intensity = std::numeric_limits<float>::quiet_NaN();
        }
      }

      float angle = ANGLE_TO_RADIAN(dir_angle); // Lidar angle unit form degree transform to radian
      unsigned int index = (unsigned int)((angle - output.angle_min) / output.angle_increment);
      if (index < beam_size) {
        // If the current content is Nan, it is assigned directly
        if (std::isnan(output.ranges[index])) {
          output.ranges[index] = range;
          unsigned int err = index - last_index;
          if (err == 2){
            output.ranges[index - 1] = range;
            output.intensities[index - 1] = intensity;
          }
        } else { // Otherwise, only when the distance is less than the current
                //   value, it can be re assigned
          if (range < output.ranges[index]) {
            output.ranges[index] = range;
          }
        }
        output.intensities[index] = intensity;
        last_index = index;
      }
    }
    lidarpub.publish(output);
    end_scan_time = start_scan_time;
    ROS_INFO("[ldrobot] pub lidar data");
  } 
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
