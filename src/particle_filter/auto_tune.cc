#include <fstream>
#include <iostream>
#include <bits/stdc++.h>
#include "eigen3/Eigen/Dense"

#include "auto_tune.h"

using namespace std;
using namespace Eigen;
using namespace geometry;

namespace auto_tune {

AutoTune::AutoTune() {
  LoadContexts_();
}

void AutoTune::LoadContexts_() {
  string line;
  string context_name;
  ifstream fp;
  fp.open(BEST_PARAMS_FILE);
  if (!fp.is_open()) {
    printf("error in opening file\n");
    exit(1);
  }
  while(getline(fp, line)) {
    stringstream tokens(line);
    tokens >> context_name;
    
    Parameter param;
    tokens >> param.sensor_std >> param.d_short >> param.d_long >> param.motion_dist_k1 >> param.motion_dist_k2 >> param.motion_a_k1 >> param.motion_a_k2;
  
    contexts.emplace_back(context_name, param);
  }
  fp.close();

  if (0) {
    for (auto& context : contexts) {
      cout << context.name << endl;
      cout << context.param.sensor_std << " ";
      cout << context.param.d_short << " ";
      cout << context.param.d_long << " ";
      cout << context.param.motion_dist_k1 << " ";
      cout << context.param.motion_dist_k2 << " ";
      cout << context.param.motion_a_k1 << " ";
      cout << context.param.motion_a_k2 << endl;
    }
  }
  
}

Parameter AutoTune::DetectContext(const vector_map::VectorMap& map,
                                  const Vector2f& loc,
                                  const float angle,
                                  const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // return DetectObservationContext_(map, loc, angle, ranges, range_min, range_max, angle_min, angle_max);
  // return DetectObstacleContext_(map, loc, angle, ranges, range_min, range_max, angle_min, angle_max);
  return DetectMotionContext_(map, loc, angle, ranges, range_min, range_max, angle_min, angle_max);
  // TODO: add obstacle and motion noises
  // seperate obstabcles vs walls
  // for all obstacles: DetectObstacleContext_()
  // for point clouds on walls: 
}

/*
VectorXf linearRegression(const VectorXf& x, const VectorXf& y, size_t length) {
  VectorXf ones(length);
  MatrixXf A(ones, x);
  // return A.colPivHouseholderQr().solve(y);
  return A.fullPivHouseholderQr().solve(y); // slow but stable
}
*/

/*
 * loc_ab and angle_ab is the position of frame A in frame B.
 * loc_pa and angle_pa is the position of the object p in frame A.
 * Calculates the position of p in frame B.
 */
void TransformAToB(const Vector2f& loc_ab, float angle_ab,
                   const Vector2f& loc_pa, float angle_pa,
                   Vector2f* loc_ptr, float* angle_ptr) {
  float& new_angle = *angle_ptr;
  Vector2f& new_loc = *loc_ptr;
  
  Rotation2Df r(angle_ab);
  new_loc = loc_ab + r * loc_pa;
  new_angle = angle_ab + angle_pa;
}

Parameter AutoTune::DetectObservationContext_(const vector_map::VectorMap& map,
                                              const Vector2f& loc,
                                              const float angle,
                                              const vector<float>& ranges,
                                              float range_min,
                                              float range_max,
                                              float angle_min,
                                              float angle_max) {  
  float angle_step = (angle_max - angle_min) / ranges.size();
  
  // segment points from scan
  size_t segments = (angle_max - angle_min) / (M_PI / 180 * 5);
  size_t segment_size = ranges.size() / segments;
  float sum_stddev = 0.0;
  segments = 0;
  for (size_t start = 0; start < ranges.size(); start += segment_size) {
    size_t end = start + segment_size > ranges.size() ? ranges.size() : start + segment_size;
    sum_stddev += getPointDistStddev_(map, loc, angle, ranges, range_min, range_max, angle_step, start, end);
    ++segments;
  }
  if (sum_stddev / segments < OBSERVATION_CUTOFF) {
    // LS_LM_LO
    return contexts[0].param;
  } else {
    // HS_LM_LO
    return contexts[5].param;
  }
}

// does linear regression on the segment and returns the standard deviation from the regression line
float AutoTune::getPointDistStddev_(const vector_map::VectorMap& map_,
                         const Vector2f& loc,
                         const float angle,
                         const vector<float>& ranges,
                         float range_min,
                         float range_max,
                         float angle_step,
                         size_t start, size_t end) {
  vector<Vector2f> points;
  for (size_t i = start; i < end; i += DOWNSAMPLE_RATE) {
    // if (IsObstacle_(map_, loc, angle, i * angle_step, ranges[i], range_min, range_max)) { continue; }
    // points
    
  }
  return 0.0; // TODO: fix me
}

// determines if a single point is an obstacle or not
float AutoTune::CalculateDistanceToWall_(const vector_map::VectorMap& map_,
                                         const Vector2f& loc,
                                         const float angle,
                                         const float angle_i,
                                         const float range_i,
                                         const float range_min,
                                         const float range_max) {
  // Get laser frame's position relative to the map frame.
  Rotation2Df r(angle);
  Vector2f kLaserLoc(0.2, 0);
  Vector2f mLaserLoc = loc + r * kLaserLoc;
  float mLaserAngle = angle;

  // Transform points location from laser frame to map frame
  Rotation2Df r_i(angle_i);
  Vector2f v_min = r_i * Vector2f(range_min, 0);
  Vector2f v_max = r_i * Vector2f(range_max, 0);
  
  Vector2f loc_min(0,0);
  float angle_min = 0.0;
  Vector2f loc_max(0,0);
  float angle_max = 0.0;
  TransformAToB(mLaserLoc, mLaserAngle, v_min, angle_i, &loc_min, &angle_min);
  TransformAToB(mLaserLoc, mLaserAngle, v_max, angle_i, &loc_max, &angle_max);

  line2f laser_line(loc_min.x(), loc_min.y(), loc_max.x(), loc_max.y());

  // Assumes the intersection point is at horizon if no intersection detected
  Vector2f v_horizon = r_i * Vector2f(HORIZON, 0);
  Vector2f intersection_point(0,0);
  float angle_default = 0.0;
  TransformAToB(mLaserLoc, mLaserAngle, v_horizon, angle_i, &intersection_point, &angle_default);
  
  // Get closest interesction
  Vector2f intersection_point_tmp (0.0, 0.0);
  line2f intersection_line;
  bool has_intersection = false;
  for (size_t j = 0; j < map_.lines.size(); ++j) {
    const line2f map_line = map_.lines[j];
    bool intersects = map_line.Intersection(laser_line, &intersection_point_tmp);
    if (!intersects) continue;
    else has_intersection = true;
    
    bool closer = false;
    if (abs(intersection_point_tmp.x() - mLaserLoc.x()) < abs(intersection_point.x() - mLaserLoc.x()) ) {
      closer = true;
    } else if (abs(intersection_point_tmp.y() - mLaserLoc.y()) < abs(intersection_point.y() - mLaserLoc.y())) {
      closer = true;
    }
    
    if(closer) {
      intersection_point.x() = intersection_point_tmp.x();
      intersection_point.y() = intersection_point_tmp.y();
      intersection_line = map_line;
    }
  }

  // get the location of the actual laser reading in the map frame
  Vector2f v = r_i * Vector2f(range_i, 0);
  Vector2f laser_loc(0,0);
  float angle_loc = 0.0;
  TransformAToB(mLaserLoc, mLaserAngle, v, angle_i, &laser_loc, &angle_loc);

  if (has_intersection) {
    // cout << "intersects" << endl;
    // 1) calculate distance to intersection line to determine if obstacle
    Vector2f projected_point = ProjectPointOntoLineSegment(laser_loc, intersection_line.p0, intersection_line.p1);
    return (projected_point - laser_loc).norm();
  }
  return -1; // never correlated with a wall
}

Parameter AutoTune::DetectMotionContext_(const vector_map::VectorMap& map,
                                           const Vector2f& loc,
                                           const float angle,
                                           const vector<float>& ranges,
                                           float range_min,
                                           float range_max,
                                           float angle_min,
                                           float angle_max) {
  float angle_step = (angle_max - angle_min) / ranges.size();
  
  float angle_i;
  float total_dist = 0;
  size_t cnt = 0;
  for (size_t i = 0; i < ranges.size(); i += DOWNSAMPLE_RATE) {
    angle_i = angle_min + i * angle_step;
    float dist = CalculateDistanceToWall_(map, loc, angle, angle_i, ranges[i], range_min, range_max);
    if (dist == -1) { continue; }
    ++cnt;
    total_dist += dist;
  }
  
  cout << "avg dist: " << total_dist / cnt << endl;
  if (total_dist / cnt < MOTION_CUTOFF) {
    // LS_LM_LO
    cout << "LS_LM_LO" << endl;
    return contexts[0].param;
  } else {
    // LS_HM_LO
    cout << "LS_HM_LO" << endl;
    return contexts[2].param;
  }
}

Parameter AutoTune::DetectObstacleContext_(const vector_map::VectorMap& map,
                                           const Vector2f& loc,
                                           const float angle,
                                           const vector<float>& ranges,
                                           float range_min,
                                           float range_max,
                                           float angle_min,
                                           float angle_max) {
  float angle_step = (angle_max - angle_min) / ranges.size();
  
  size_t num_obstacles = 0;
  float angle_i;
  for (size_t i = 0; i < ranges.size(); i += DOWNSAMPLE_RATE) {
    angle_i = angle_min + i * angle_step;
    float dist = CalculateDistanceToWall_(map, loc, angle, angle_i, ranges[i], range_min, range_max);
    if (dist != -1 && dist >= OBSTACLE_DIST_CUTOFF)
      num_obstacles++;
  }
  
  // cout << "obs: " << num_obstacles << " total: " << ranges.size() / DOWNSAMPLE_RATE << endl;
  // cout << (float) num_obstacles / (ranges.size() / DOWNSAMPLE_RATE) << endl;
  if ((float) num_obstacles / (ranges.size() / DOWNSAMPLE_RATE) < OBSTACLE_CUTOFF) {
    // LS_LM_LO
    // cout << "LS_LM_LO" << endl;
    return contexts[0].param;
  } else {
    // LS_LM_HO
    // cout << "LS_LM_HO" << endl;
    return contexts[1].param;
  }
}

} // namespace auto_tune



