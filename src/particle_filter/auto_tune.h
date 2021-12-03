#include <vector>
#include <string>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "vector_map/vector_map.h"

#ifndef SRC_AUTO_TUNE_H_
#define SRC_AUTO_TUNE_H_

using namespace std;
using namespace vector_map;
using namespace Eigen;

namespace auto_tune {

struct Parameter {
  float sensor_std;
  float d_short;
  float d_long;
  float motion_dist_k1;
  float motion_dist_k2;
  float motion_a_k1;
  float motion_a_k2;

  Parameter()
    : sensor_std(0.1), d_short(0.05 * 1.5), d_long(0.05 * 2.0), 
      motion_dist_k1(0.3), motion_dist_k2(0.05), 
      motion_a_k1(0.1), motion_a_k2(1.0) {}
  Parameter(float sensor_std, float d_short, float d_long, float motion_dist_k1, float motion_dist_k2, float motion_a_k1, float motion_a_k2)
    : sensor_std(sensor_std), d_short(d_short), d_long(d_long), 
      motion_dist_k1(motion_dist_k1), motion_dist_k2(motion_dist_k2), 
      motion_a_k1(motion_a_k1), motion_a_k2(motion_a_k2) {}
};

struct Context {
  string name;
  Parameter param;
  Context() {}
  Context(string name, Parameter param) : 
    name(name), param(param) {}
};

class AutoTune {
public:
  AutoTune();
  // calls the three private methods and changes the parameters
  Parameter DetectContext(vector_map::VectorMap map,
                          const Vector2f& loc,
                          const float angle,
                          const vector<float>& ranges,
                          float range_min,
                          float range_max,
                          float angle_min,
                          float angle_max);
private:
  // vector_map::VectorMap map_;
  const string BEST_PARAMS_FILE = "config/best_params.txt";
  // holds the context name and best set of parameters we pre-computed
  vector<Context> contexts;  // TODO: change this to a map
  const Eigen::Vector2f kLaserLoc = Eigen::Vector2f(0.2, 0);
  const float HORIZON = 36.0;
  // cutoff to determine whether a point is an obstacle or not
  const float OBSTACLE_DIST_CUTOFF = 0.5;
  // ratio of obstacles over all observation point cloud
  const float OBSTACLE_CUTOFF = 0.1;
  const float OBSERVATION_CUTOFF = 0.25;
  const float DOWNSAMPLE_RATE = 20;

  // loads in the best params for each context from best_params.txt
  void LoadContexts_();
  // detects whether the current context is high or low observation noise
  Parameter DetectObservationContext_(const vector_map::VectorMap& map,
                                      const Vector2f& loc,
                                      const float angle,
                                      const vector<float>& ranges,
                                      float range_min,
                                      float range_max,
                                      float angle_min,
                                      float angle_max);
  // detects whether the current context is high or low motion noise
  void DetectMotionContext_();
  // detects whether the current context has many or few obstacles 
  Parameter DetectObstacleContext_(const vector_map::VectorMap& map,
                              const Vector2f& loc,
                              const float angle,
                              const vector<float>& ranges,
                              float range_min,
                              float range_max,
                              float angle_min,
                              float angle_max);
  // calculate the stadard deviation of a point cloud sengment distribution
  float getPointDistStddev_(const vector_map::VectorMap& map_,
                         const Vector2f& loc,
                         const float angle,
                         const vector<float>& ranges,
                         float range_min,
                         float range_max,
                         float angle_step,
                         size_t start, size_t end);
  bool IsObstacle_(const vector_map::VectorMap& map_,
                const Vector2f& loc,
                const float angle,
                const float angle_i,
                const float range_i,
                const float range_min,
                const float range_max);
};


} // namespace auto_tune

#endif // SRC_AUTO_TUNE_H_
