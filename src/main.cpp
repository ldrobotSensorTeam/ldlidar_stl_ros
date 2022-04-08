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

#include <ros/ros.h>
#include <stdio.h>

#include <iostream>
#include <string>

#include "cmd_interface_linux.h"
#include "lipkg.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ldldiar_publisher");
  ros::NodeHandle nh;  // create a ROS Node
  ros::NodeHandle n("~");
  std::string product_name;
	std::string topic_name;
	std::string port_name;
	std::string frame_id;
  bool laser_scan_dir = true;
  bool enable_angle_crop_func = false;
  double angle_crop_min = 0.0;
  double angle_crop_max = 0.0;

  n.getParam("product_name", product_name);
	n.getParam("topic_name", topic_name);
	n.getParam("port_name", port_name);
	n.getParam("frame_id", frame_id);
  n.getParam("laser_scan_dir", laser_scan_dir);
  n.getParam("enable_angle_crop_func", enable_angle_crop_func);
  n.getParam("angle_crop_min", angle_crop_min);
  n.getParam("angle_crop_max", angle_crop_max);
  
  ROS_INFO("[ldrobot] SDK Pack Version is v2.2.6");
  ROS_INFO("[ldrobot] <product_name>: %s,<topic_name>: %s,<port_name>: %s,<frame_id>: %s", 
    product_name.c_str(), topic_name.c_str(), port_name.c_str(), frame_id.c_str());

  ROS_INFO("[ldrobot] <laser_scan_dir>: %s,<enable_angle_crop_func>: %s,<angle_crop_min>: %f,<angle_crop_max>: %f",
   (laser_scan_dir?"Counterclockwise":"Clockwise"), (enable_angle_crop_func?"true":"false"), angle_crop_min, angle_crop_max);

  LiPkg *lidar = new LiPkg(frame_id, laser_scan_dir, enable_angle_crop_func, angle_crop_min, angle_crop_max);
  CmdInterfaceLinux cmd_port;

  if (port_name.empty()) {
    ROS_ERROR("[ldrobot] Can't find %s device", product_name.c_str());
    exit(EXIT_FAILURE);
  } else {
    ROS_INFO("[ldrobot] FOUND %s device", product_name.c_str());
    cmd_port.SetReadCallback([&lidar](const char *byte, size_t len) {
      if (lidar->Parse((uint8_t *)byte, len)) {
        lidar->AssemblePacket();
      }
    });
  }

  if (cmd_port.Open(port_name)) {
    ROS_INFO("[ldrobot] open %s device %s is success!", product_name.c_str(), port_name.c_str());
  }else {
    ROS_INFO("[ldrobot] open %s device %s is fail!", product_name.c_str(), port_name.c_str());
    exit(EXIT_FAILURE);
  }
    

  ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>(topic_name, 10);  // create a ROS topic
  
  ros::Rate r(10); //10hz

  while (ros::ok()) {
    if (lidar->IsFrameReady()) {
      lidar_pub.publish(lidar->GetLaserScan());  // Fixed Frame:  lidar_frame
      lidar->ResetFrameReady();
			ROS_INFO_STREAM("[ldrobot] current_speed(Hz): " << lidar->GetSpeed()); 
			ROS_INFO_STREAM("----^---^-----");
    }
    r.sleep();
  }
  return 0;
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
