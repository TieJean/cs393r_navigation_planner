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
\file    particle-filter.h
\brief   Particle Filter Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/math/line2d.h"
#include "shared/util/random.h"
#include "vector_map/vector_map.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "auto_tune.h"

#ifndef SRC_PARTICLE_FILTER_H_
#define SRC_PARTICLE_FILTER_H_

using namespace auto_tune;
using amrl_msgs::VisualizationMsg;

namespace particle_filter {

struct Particle {
  Eigen::Vector2f loc;
  float angle;
  double weight;
};

class ParticleFilter {
 public:
  // Default Constructor.
   ParticleFilter();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Predict particle motion based on odometry.
  void Predict(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Initialize the robot location.
  void Initialize(const std::string& map_file,
                  const Eigen::Vector2f& loc,
                  const float angle);

  // Return the list of particles.
  void GetParticles(std::vector<Particle>* particles) const;

  // Get robot's current location.
  void GetLocation(Eigen::Vector2f* loc, float* angle) const;

  // Update particle weight based on laser.
  void Update(const std::vector<float>& ranges,
              float range_min,
              float range_max,
              float angle_min,
              float angle_max,
              Particle* p);

  // Resample particles.
  void Resample();

  // For debugging: get predicted point cloud from current location.
  void GetPredictedPointCloud(const Eigen::Vector2f& loc,
                              const float angle,
                              int num_ranges,
                              float range_min,
                              float range_max,
                              float angle_min,
                              float angle_max,
                              std::vector<Eigen::Vector2f>* scan);

 private:
  // List of particles being tracked.
  AutoTune autoTune;
  std::vector<Particle> particles_;

  // Map of the environment.
  vector_map::VectorMap map_;

  // Random number generator.
  util_random::Random rng_;

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;
  const Eigen::Vector2f kLaserLoc = Eigen::Vector2f(0.2, 0);

  // constants
  const float HORIZON = 36.0;
  const float kEpsilon = 1e-4;

  float d_dist;
  float d_angle;
  bool updated;

  // auto-tuned params
  bool auto_tune = true;
  float CONFIG_SENSOR_STD_DEV = 0.1;
  float CONFIG_D_SHORT = 0.1 * 1.5;
  float CONFIG_D_LONG = 0.1 * 2.0;
  float CONFIG_MOTION_DIST_K1 = 0.1;
  float CONFIG_MOTION_DIST_K2 = 0.05;
  float CONFIG_MOTION_A_K1 = 0.1;
  float CONFIG_MOTION_A_K2 = 0.5;
  
  float calculateLogGaussian(float mean, float x, float stddev, float range_min, float range_max);
};
}  // namespace slam

#endif   // SRC_PARTICLE_FILTER_H_
