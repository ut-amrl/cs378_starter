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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

// Fill in the body of these functions and create helpers as needed
// in order to implement localization using a particle filter.

// Milestone 2 will be implemented here.

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    particles_(10),
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    mean_loc(0, 0),
    mean_angle(0) {}


void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
    const Vector2f kLaserLoc(0.2, 0);
    const Vector2f laser_loc = loc + kLaserLoc; //TODO depends on where car is facing
    float laser_angle = angle + angle_min;
    float laser_angle_incr = (angle_max - angle_min) / num_ranges;
    //const float range_diff = range_max - range_min; 
    
    for (int i = 0; i < num_ranges; i++) {
        //TODO include range_min, range_diff
        float laser_x = laser_loc.x() + range_max * cos(laser_angle); 
        float laser_y = laser_loc.y() + range_max * sin(laser_angle);

        bool collides = false;
        Vector2f laser_line(laser_x, laser_y);
        Vector2f intersection;
        float closest_intersection_dist = range_max;
        for (geometry::line2f line : map_.lines) {
            if (line.Intersection(laser_loc, laser_line, &intersection)) {
                collides = true;
                float intersection_dist = std::sqrt(Sq(intersection.x() - laser_loc.x()) + Sq(intersection.y() - laser_loc.y()));
                if (intersection_dist < closest_intersection_dist) {
                    closest_intersection_dist = intersection_dist;
                    
                }

            }
        }

        if (collides) {
            laser_x = laser_loc.x() + closest_intersection_dist * cos(laser_angle); 
            laser_y = laser_loc.y() + closest_intersection_dist * sin(laser_angle);
            laser_line = Vector2f(laser_x, laser_y);
        }

        scan_ptr->push_back(laser_line);
        laser_angle += laser_angle_incr;
    }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
}

void ParticleFilter::Resample() {
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
    //std::cout << odom_angle << "\n";
    float k1 = 0.1;
    float k2 = 0.1;
    float k3 = 0.1;
    float k4 = 0.1;

    if (odom_initialized_) {

        float x_total = 0;
        float y_total = 0;
        float sin_theta_total = 0;
        float cos_theta_total = 0;

        Vector2f loc_delta = odom_loc - prev_odom_loc_;
        float r_delta = loc_delta.y()/sin(odom_angle);
        float delta_theta_hat = math_util::AngleDiff(odom_angle, prev_odom_angle_);
        for (Particle& particle : particles_) {
            float delta_x_hat = r_delta*cos(particle.angle);
            float delta_y_hat = r_delta*sin(particle.angle);
            float delta_x = rng_.Gaussian(delta_x_hat, k1*std::sqrt(Sq(delta_x_hat) + Sq(delta_y_hat)) + k2*abs(delta_theta_hat));
            float delta_y = rng_.Gaussian(delta_y_hat, k1*std::sqrt(Sq(delta_x_hat) + Sq(delta_y_hat)) + k2*abs(delta_theta_hat));
            float delta_theta = rng_.Gaussian(delta_theta_hat, k3*std::sqrt(Sq(delta_x_hat) + Sq(delta_y_hat)) + k4*abs(delta_theta_hat));
            // Check for collision
            bool collides = false;
            Vector2f intersection;
            Vector2f p1 = particle.loc;
            Vector2f p2 = particle.loc + Vector2f(delta_x,delta_y);
            for (geometry::line2f line : map_.lines) {
                if (line.Intersection(p1, p2, &intersection)) {
                    collides = true;
                    break;
                }
            }

            if (collides) {
                //particle.loc = intersection + ((p1 - intersection) * .2);
                
            }
            else
                particle.loc = p2;

            particle.angle += delta_theta;
            particle.weight = 1;

            x_total += particle.loc.x();
            y_total += particle.loc.y();
            sin_theta_total += sin(particle.angle);
            cos_theta_total += cos(particle.angle);

            //std::cout << particle.loc.x() << " " << particle.loc.y() << "\n";
        }

        //std::cout << x_total / particles_.size() << " " << y_total / particles_.size() << "\n";

        mean_loc = Vector2f(x_total / particles_.size(), y_total / particles_.size());
        mean_angle = atan2(sin_theta_total / particles_.size(), cos_theta_total / particles_.size());
    } else {
        odom_initialized_ = true;
    }
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
    float k = .25;
    float k2 = .1;
    for (Particle& particle : particles_) {
        float x = rng_.Gaussian(loc.x(), k);
        float y = rng_.Gaussian(loc.y(), k);
        particle.loc = Vector2f(x, y);
        particle.angle = rng_.Gaussian(angle, k2);
        particle.weight = 1;
    }
    // Is this needed on actual car?
    odom_initialized_ = false;
    map_.Load(map_file);
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc, float* angle) const {
    *loc = mean_loc;
    *angle = mean_angle;
}


}  // namespace particle_filter
