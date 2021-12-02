#include <iostream>
#include <vector>
#include "rotors_control/common.h"
#include <ros/ros.h>
#include <ros/time.h>
#include <cmath>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>


struct PathJson {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    std::vector<double> vx;
    std::vector<double> vy;
    std::vector<double> vz;
    std::vector<double> ax; 
    std::vector<double> ay;
    std::vector<double> az;
    std::vector<double> roll;
    std::vector<double> pitch;
    std::vector<double> yaw;
    std::vector<double> omegax; 
    std::vector<double> omegay;
    std::vector<double> omegaz;
};

inline double distance(double dx,double dy,double dz) {
    return std::pow(std::pow(dx, 2)+std::pow(dy, 2)+std::pow(dz, 2), 0.5);
}


namespace rotors_gazebo {

    class TrackingSplineNode{
        public:
            TrackingSplineNode();
            ~TrackingSplineNode();
            void PublishWaypoint();


        private:
            // Subscriber
            ros::Subscriber odometry_sub_;

            // Publisher
            ros::Publisher trajectory_pub_;

            // Inline functions
            void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
            void CheckingWaypoint();
            void FromTrajWaypointToMsg();

            // Inline variables
            PathJson traj_;
            mav_msgs::EigenOdometry eigen_odom_;
            int current_index_path_ = 0;
            mav_msgs::DroneState trajectory_msg_;
            int length_traj_;
    };
}