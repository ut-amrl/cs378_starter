//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "f1tenth_course/AckermannCurvatureDriveMsg.h"
#include "f1tenth_course/Pose2Df.h"
#include "f1tenth_course/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using f1tenth_course::AckermannCurvatureDriveMsg;
using f1tenth_course::VisualizationMsg;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, const double dist, const double curv, ros::NodeHandle* n) :
    curv(curv),
    dist(dist),
    start_loc(0, 0),
    initialized(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
    nav_goal_loc_ = loc;
    nav_goal_angle_ = angle;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
    robot_loc_ = loc;
    robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
    if (!initialized && loc.x() != 0) {
        initialized = true;
        start_loc = loc;
    }
    robot_loc_ = loc;
    robot_angle_ = angle;
    robot_vel_ = vel;
    robot_omega_ = ang_vel;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
}

void Navigation::Run() {
    double t = 1.0/20.0;
    double v_0 = robot_vel_.x();
    double v_delta;

    if (!initialized)
        return;
    double x_delta = abs(robot_loc_.x() - start_loc.x());
    double y_delta = abs(robot_loc_.y() - start_loc.y());
    dist -= std::sqrt(std::pow(x_delta, 2) + std::pow(y_delta, 2));
    start_loc = robot_loc_;
    if (dist < 0.0)
        return;
    // accelerate for 1 step
    double a_max = 3;
    double a_min = -3;
    double v_f = v_0 + a_max*t;
    double x_1 = t*(v_0 + v_f)/2;
    double t_2 = v_f/-a_min;
    double x_2 = t_2*(v_f/2);
    if (x_1 + x_2 <= dist) {
        v_delta = a_max * t;
    } else {
        // cruise for 1 step
        x_1 = t*v_0;
        t_2 = v_0/-a_min;
        x_2 = t_2*(v_0/2);
        if (x_1 + x_2 <= dist) {
            v_delta = 0;
        } else {
            // decelerate for 1 step
            double a = (std::pow(v_0,2))/(2*dist);
            v_delta = -a * t;
        }
    }

    double target_v = v_0 + v_delta;
    double max_v = 1;
    target_v = std::min(target_v, max_v);
    target_v = std::max(target_v, 0.0);
    drive_msg_.velocity = target_v;
    drive_msg_.curvature = curv;
    drive_pub_.publish(drive_msg_);
    //std::cout << robot_loc_.x() << "\n";
    //std::cout << start_loc.x() << "\n";
  // Create Helper functions here
  // Milestone 1 will fill out part of this class.
  // Milestone 3 will complete the rest of navigation.
}

}  // namespace navigation
