/*
 * Copyright 2021 Daniele Morra, Scuola Superiore Sant'Anna, Pisa, Italy 
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>
#include <fstream>

#include "rotors_gazebo/Quaternion.h"
#include "rotors_gazebo/transform_datatypes.h"
#include "rotors_gazebo/parameters_ros.h"
#include <nav_msgs/Odometry.h>
#include "rotors_gazebo/Matrix3x3.h"
#include <ros/console.h>
#include <time.h>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <math.h>
#include "rotors_gazebo/spline_trajectory_generator.h"
#include <nlohmann/json.hpp>
#include "rotors_gazebo/tracking_spline.h"


#define FREQUENCY_NODE  200
#define START_SIMULATION_TIME 3   /* TIME GAZEBO NEEDS TO INITIALIZE THE ENVIRONMENT */

template<typename T> inline void GetRosParameterHovering(const ros::NodeHandle& nh,
                                                         const std::string& key,
                                                         const T& default_value,
                                                         T* value) {

  ROS_ASSERT(value != nullptr);
  bool have_parameter = nh.getParam(key, *value);
  if (!have_parameter) {
    ROS_WARN_STREAM("[rosparam]: could not find parameter " << nh.getNamespace()
                    << "/" << key << ", setting to default: " << default_value);
    *value = default_value;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "hovering_example_spline");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");
  ros::Publisher trajectory_pub =
      nh.advertise<mav_msgs::DroneState>(
          mav_msgs::default_topics::DRONE_STATE, 10);
	ros::Rate rate(FREQUENCY_NODE);
  ROS_INFO("Started hovering example with spline.");

	// Upload the trajectory file --> TO DO
	std::string path; 		// --> TO DO
	std::string config_file = "../../spline_generator/spline/" + path + ".json";
	std::ifstream iConfig(config_file);
	nlohmann::json json_path;
	iConfig >> json_path;

	Path traj {json_path["x"].get<std::vector<double>>(),
						 json_path["y"].get<std::vector<double>>(),
						 json_path["z"].get<std::vector<double>>(),
						 json_path["vx"].get<std::vector<double>>(),
						 json_path["vy"].get<std::vector<double>>(),
						 json_path["vz"].get<std::vector<double>>(),
						 json_path["ax"].get<std::vector<double>>(),
						 json_path["ay"].get<std::vector<double>>(),
						 json_path["az"].get<std::vector<double>>(),
						 json_path["roll"].get<std::vector<double>>(),
						 json_path["pitch"].get<std::vector<double>>(),
						 json_path["yaw"].get<std::vector<double>>(),
						 json_path["omegax"].get<std::vector<double>>(),
						 json_path["omegay"].get<std::vector<double>>(),
						 json_path["omegaz"].get<std::vector<double>>()};

	// Computing the minimum distance between waypoints --> TO DO
	double r_min;

	// Evaluate the Check condition for switching waypoints to publish
	double r_check = r_min/2;

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Trajectory message
  mav_msgs::DroneState trajectory_msg, trajectory_msg_pre;
  trajectory_msg.header.stamp = ros::Time::now();
  mav_msgs::EigenDroneState eigen_reference;

	// Stop publishing condition --> TO DO
  Eigen::Vector3f check_position_final;

  ROS_DEBUG("[Node] Position final x: %f, y: %f, z :%f", check_position_final.x(),
    check_position_final.y(), check_position_final.z());

  // Wait for 5 seconds to let the Gazebo GUI show up.
  if (ros::Time::now().toSec() < START_SIMULATION_TIME){
    ros::Duration(START_SIMULATION_TIME).sleep();
  }

  // Publish the trajectory values until the final values is reached
  while(ros::ok()){
    double current_time = ros::Time::now().toSec();

    // new message
    if(eigen_reference.position_W[0] <= check_position_final.x() &&
      eigen_reference.position_W[1] <= check_position_final.y() &&
      eigen_reference.position_W[2] <= check_position_final.z()){
      trajectory_pub.publish(trajectory_msg);
      trajectory_msg_pre = trajectory_msg;
    }

    ROS_DEBUG("Publishing waypoint from msg: [%f, %f, %f].", trajectory_msg.position.x, trajectory_msg.position.y, trajectory_msg.position.z);
    ROS_DEBUG("Publishing waypoint: [%f, %f, %f].", eigen_reference.position_W[0], eigen_reference.position_W[1], eigen_reference.position_W[2]);

    rate.sleep();

    // Hold the message until the simulation ends
    if(eigen_reference.position_W[0] > check_position_final.x() &&
      eigen_reference.position_W[1] > check_position_final.y() &&
      eigen_reference.position_W[2] > check_position_final.z())
      trajectory_pub.publish(trajectory_msg_pre);

  }

  return 0;
}
