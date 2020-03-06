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
    odom_initialized_(false) {}

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
    float k1 = 0.1;
    float k2 = 0.1;
    float k3 = 0.1;
    float k4 = 0.1;
    if (odom_initialized_) {
        Vector2f loc_delta = odom_loc - prev_odom_loc_;
        float delta_theta_hat = math_util::AngleDiff(odom_angle, prev_odom_angle_);
        float delta_x_hat = loc_delta.x();
        float delta_y_hat = loc_delta.y();
        for (Particle& particle : particles_) {
            float delta_x = rng_.Gaussian(delta_x_hat, k1*std::sqrt(Sq(delta_x_hat) + Sq(delta_y_hat)) + k2*abs(delta_theta_hat));
            float delta_y = rng_.Gaussian(delta_y_hat, k1*std::sqrt(Sq(delta_x_hat) + Sq(delta_y_hat)) + k2*abs(delta_theta_hat));
            float delta_theta = rng_.Gaussian(delta_theta_hat, k3*std::sqrt(Sq(delta_x_hat) + Sq(delta_y_hat)) + k4*abs(delta_theta_hat));
            particle.loc += Vector2f(delta_x,delta_y);
            particle.angle += delta_theta;
            particle.weight = 1;
        }
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
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc, float* angle) const {
}


}  // namespace particle_filter
