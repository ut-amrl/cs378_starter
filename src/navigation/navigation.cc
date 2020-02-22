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
    point_cloud(),
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
    }
    robot_loc_ = loc;
    robot_angle_ = angle;
    robot_vel_ = vel;
    robot_omega_ = ang_vel;
}

double Euclid2D(const double x, const double y) {
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
    visualization::ClearVisualizationMsg(local_viz_msg_);
    point_cloud.clear();
    for (Vector2f point : cloud) {
        point_cloud.push_back(point);
        //Vector2f global_point = globalize_point(point);
        //visualization::DrawCross(global_point, .01, 0xFF0000, local_viz_msg_);
    }
    //viz_pub_.publish(local_viz_msg_);
}

// TODO: remove this when local visualization is fixed
Vector2f Navigation::globalize_point(const Vector2f& local_point) {
    float range = Euclid2D(local_point.x(), local_point.y());
    float angle_orig = atan2(local_point.y(), local_point.x());
    float angle = robot_angle_ + angle_orig;
    Vector2f global_point(range * cos(angle), range * sin(angle));
    global_point += robot_loc_;
    return global_point;
}

double CalcVDelta(const double v_0, const double t, const double d) {
    double v_delta;

    // accelerate for 1 step
    double a_max = 3;
    double a_min = -3;
    double v_f = v_0 + a_max*t;
    double x_1 = t*(v_0 + v_f)/2;
    double t_2 = v_f/-a_min;
    double x_2 = t_2*(v_f/2);
    if (x_1 + x_2 <= d) {
        v_delta = a_max * t;
    } else {
        // cruise for 1 step
        x_1 = t*v_0;
        t_2 = v_0/-a_min;
        x_2 = t_2*(v_0/2);
        if (x_1 + x_2 <= d) {
            v_delta = 0;
        } else {
            // decelerate for 1 step
            double a = (v_0 * v_0)/(2*d);
            v_delta = -a * t;
        }
    }
    return v_delta;
}

// Uses local message
void Navigation::draw_car(const Vector2f& local_point, uint32_t color) {
    Vector2f p1(local_point.x(), local_point.y() + w);
    Vector2f p2(local_point.x() + h, local_point.y() + w);
    Vector2f p3(local_point.x() + h, local_point.y() - w);
    Vector2f p4(local_point.x(), local_point.y() - w);
    visualization::DrawLine(globalize_point(p1), globalize_point(p2), color, local_viz_msg_);
    visualization::DrawLine(globalize_point(p2), globalize_point(p3), color, local_viz_msg_);
    visualization::DrawLine(globalize_point(p3), globalize_point(p4), color, local_viz_msg_);
    visualization::DrawLine(globalize_point(p4), globalize_point(p1), color, local_viz_msg_);
}

void Navigation::Run() {
    if (!initialized)
        return;

    // constants
    float curv_inc = .2;
    float dist = 5.0;
    // relative goal
    Vector2f goal(dist, 0.0);
    //visuals
    visualization::ClearVisualizationMsg(local_viz_msg_);
    draw_car(Vector2f(0,0), 0xFF0000);
    visualization::DrawCross(globalize_point(goal), .1, 0xFF0000, local_viz_msg_);

    // evaluate possible paths
    float best_curv = 0;
    float best_score = -9999.0;
    float best_fpl = 0;
    for (float curv = -1; curv <= 1; curv += curv_inc) {
        float fpl;
        float clearance = .2;
        float goal_dist;
        Vector2f dest;
        if (abs(curv) < .05) {
            fpl = 3;
            for (Vector2f point : point_cloud)
                if (abs(point.y()) <= w)
                    fpl = std::min(fpl, point.x() - h);
            for (Vector2f point : point_cloud)
                if (point.x() >= 0 && point.x() <= fpl + h)
                    clearance = std::min(clearance, abs(point.y()) + w);
            goal_dist = Euclid2D(abs(fpl - goal.x()), goal.y());
            dest = Vector2f(fpl, 0);
        } else {
            float r = 1/curv;

            Vector2f g(goal.x(), goal.y());
            // turning right, flip all points over x axis
            if (r < 0) {
                r = -r;
                for (Vector2f point : point_cloud) {
                    point.y() = -point.y();
                }
                g.y() = -g.y();
            }
            // Assumes goal is straight ahead dist meters
            fpl = r * atan2(dist, r);
            double r_1 = r - w;
            double r_2 = Euclid2D(r + w, h);
            double omega = atan2(h, r - w);

            // compute free path length
            for (Vector2f point : point_cloud) {
                if (point.x() < 0)
                    continue;
                double r_point = Euclid2D(point.x(), point.y() - r);
                
                if (r_point >= r_1 && r_point <= r_2) {
                    double theta = atan2(point.x(), r - point.y());
                    assert(theta >= 0);
                    // TODO: why is subtracting constant needed
                    //float curv_dist = r * (theta - omega) - 0.001; 
                    float curv_dist = r * (theta - omega); 
                    if (curv_dist < 0)
                        continue;
                    fpl = std::min(fpl, curv_dist);
                }
            }
            for (Vector2f point : point_cloud) {
                if (point.x() < 0)
                    continue;
                double r_point = Euclid2D(point.x(), point.y() - r);
                double theta = atan2(point.x(), r - point.y());
                float curv_dist = r * (theta - omega); 
                if (curv_dist <= fpl && curv_dist >= 0) {
                    float clear_curr = std::min(r_1 - r_point, r_point - r_2);
                    clearance = std::min(clearance, clear_curr);
                }
            }
            float rad = fpl / r;
            float dest_x = r * sin(rad);
            float dest_y = r - r*cos(rad);
            dest = Vector2f(dest_x, dest_y);
            goal_dist = Euclid2D(abs(dest_x - goal.x()), abs(dest_y - goal.y()));
        }
        float w1 = .1;
        float w2 = -.1;
        float score = fpl + w1*clearance + w2*goal_dist;
        std::cout << "curv " << curv << " fpl " << fpl << " clearance " << clearance << " goal_dist " << goal_dist << "\n";
        if (score > best_score) {
            best_score = score;
            best_curv = curv;
            best_fpl = fpl;
        }
        visualization::DrawPathOption(curv, fpl, clearance, local_viz_msg_);
        draw_car(dest, 0xFF00FF);
    }
    visualization::DrawPathOption(best_curv, best_fpl, 0, local_viz_msg_);
    viz_pub_.publish(local_viz_msg_);
    //if (best_fpl <= 0.01)
        return;
    // 1d TOC
    double t = 1.0/20.0;
    double v_0 = Euclid2D(robot_vel_.x(), robot_vel_.y());
    double v_delta;

    float d = best_fpl;
    v_delta = CalcVDelta(v_0, t, d);
    // account for latency
    // TODO: have other calculations account for latency as well
    double latency = .05;
    d = best_fpl - (v_0 +(v_delta)/2) * latency;
    v_delta = CalcVDelta(v_0, t, d);

    double target_v = v_0 + v_delta;
    double max_v = 1;
    target_v = std::min(target_v, max_v);
    target_v = std::max(target_v, 0.0);

    drive_msg_.velocity = target_v;
    drive_msg_.curvature = best_curv;
    drive_pub_.publish(drive_msg_);
  // Create Helper functions here
  // Milestone 1 will fill out part of this class.
  // Milestone 3 will complete the rest of navigation.
}

}  // namespace navigation
