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

#define TF_EULER_DEFAULT_ZYX
#define FREQUENCY_NODE  200
#define START_SIMULATION_TIME 3   /* TIME GAZEBO NEEDS TO INITIALIZE THE ENVIRONMENT */

// template<typename T> inline void GetRosParameterHovering(const ros::NodeHandle& nh,
//                                                          const std::string& key,
//                                                          const T& default_value,
//                                                          T* value) {

//   ROS_ASSERT(value != nullptr);
//   bool have_parameter = nh.getParam(key, *value);
//   if (!have_parameter) {
//     ROS_WARN_STREAM("[rosparam]: could not find parameter " << nh.getNamespace()
//                     << "/" << key << ", setting to default: " << default_value);
//     *value = default_value;
//   }
// }


namespace rotors_gazebo {
  TrackingSplineNode::TrackingSplineNode() {

    ROS_INFO_ONCE("Started TrackingSplineNode");

    ros::NodeHandle nh;

    // subscription init
    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
        &TrackingSplineNode::OdometryCallback, this);

    // publisher init
    trajectory_pub_ = nh.advertise<mav_msgs::DroneState>(mav_msgs::default_topics::DRONE_STATE, 1);

    // Initialization
    ros::Rate rate(FREQUENCY_NODE);
    // Upload the trajectory file --> TO DO
    std::string path = "circular_path"; 		// --> TO DO
    std::string config_file = "/home/euroracing/crazyflie_sil/src/CrazyS/rotors_gazebo/spline_generator/spline/" + path + ".json";
    std::ifstream iConfig(config_file);
    nlohmann::json json_path;
    iConfig >> json_path;

    traj_ = PathJson{json_path["x"].get<std::vector<double>>(),
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

    length_traj_ = traj_.x.size();

    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    unsigned int i = 0;
    // Trying to unpause Gazebo for 10 seconds.
    while (i < 10 && !unpaused) {
      ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      unpaused = ros::service::call("/gazebo/unpause_physics", srv);
      ++i;
    }
    if (!unpaused)
      ROS_FATAL("Could not wake up Gazebo.");
    else
      ROS_INFO("Unpaused the Gazebo simulation.");

  }

  TrackingSplineNode::~TrackingSplineNode(){}

  void TrackingSplineNode::CheckingWaypoint(){
    // calculate the Checking condition
    double dx_wayp, dy_wayp, dz_wayp, dx_state, dy_state, dz_state;
    dx_wayp = traj_.x[current_index_path_]-traj_.x[current_index_path_+1];
    dy_wayp = traj_.y[current_index_path_]-traj_.y[current_index_path_+1];
    dx_wayp = traj_.z[current_index_path_]-traj_.z[current_index_path_+1];
    dx_state = traj_.x[current_index_path_]-eigen_odom_.position_W.x();
    dy_state = traj_.y[current_index_path_]-eigen_odom_.position_W.y();
    dx_state = traj_.z[current_index_path_]-eigen_odom_.position_W.z();

    double dist_between_waypoints_ = distance(dx_wayp, dy_wayp, dz_wayp);
    double dist_between_drone_targetwp_ = distance(dx_state, dy_state, dz_state);

    double r_check = dist_between_waypoints_/2;
    // Update the current waypoint
    if(dist_between_drone_targetwp_ < r_check) {
      current_index_path_++;
    }
  }

  void TrackingSplineNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){
    mav_msgs::eigenOdometryFromMsg(*odometry_msg, &eigen_odom_);
  }

  void TrackingSplineNode::FromTrajWaypointToMsg(){
    trajectory_msg_.header.stamp = ros::Time::now();

    trajectory_msg_.position.x = traj_.x[current_index_path_];
    trajectory_msg_.position.y = traj_.y[current_index_path_];
    trajectory_msg_.position.z = traj_.z[current_index_path_];

    trajectory_msg_.linear_velocity.x = traj_.vx[current_index_path_];
    trajectory_msg_.linear_velocity.y = traj_.vy[current_index_path_];
    trajectory_msg_.linear_velocity.z = traj_.vz[current_index_path_];

    geometry_msgs::Quaternion q_msg;
    tf::Quaternion q_tf;
    q_tf.setRPY(traj_.roll[current_index_path_],
                traj_.pitch[current_index_path_],
                traj_.yaw[current_index_path_]);
    tf::quaternionTFToMsg(q_tf, q_msg);
    trajectory_msg_.orientation = q_msg;

    trajectory_msg_.linear_acceleration.x = traj_.ax[current_index_path_];
    trajectory_msg_.linear_acceleration.y = traj_.ay[current_index_path_];
    trajectory_msg_.linear_acceleration.z = traj_.az[current_index_path_];

    trajectory_msg_.angular_velocity.x = traj_.omegax[current_index_path_];
    trajectory_msg_.angular_velocity.y = traj_.omegay[current_index_path_];
    trajectory_msg_.angular_velocity.z = traj_.omegaz[current_index_path_];

    // What TO DO?
    // trajectory_msg_.angular_acceleration.x = 0;
    // trajectory_msg_.angular_acceleration.y = 0;
    // trajectory_msg_.angular_acceleration.z = 0;

  }

  void TrackingSplineNode::PublishWaypoint() {
    TrackingSplineNode::FromTrajWaypointToMsg();
    trajectory_pub_.publish(trajectory_msg_);
    int count = 0;
    if(current_index_path_ == length_traj_ && count==0) {
        ROS_DEBUG("[Node] Final position x: %f, y: %f, z :%f", traj_.x[current_index_path_],
          traj_.y[current_index_path_], traj_.z[current_index_path_]);
        count++; 
    }
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "tracking_spline_node");
  ros::NodeHandle nh2;
  rotors_gazebo::TrackingSplineNode tracking_spline_node;

  ros::Rate rate(FREQUENCY_NODE);

  while(ros::ok()){
    tracking_spline_node.PublishWaypoint();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}