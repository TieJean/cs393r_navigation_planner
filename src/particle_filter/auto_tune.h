#include <vector>
#include <string>


#ifndef SRC_AUTO_TUNE_H_
#define SRC_AUTO_TUNE_H_

using namespace std;

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
    : sensor_std(0.05), d_short(0.05 * 1.5), d_long(0.05 * 2.0), 
      motion_dist_k1(0.1), motion_dist_k2(0.25), 
      motion_a_k1(0.5), motion_a_k2(1.5) {}
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
  void DetectContext();

private:
  const string BEST_PARAMS_FILE = "config/best_params.txt";
  // holds the context name and best set of parameters we pre-computed
  vector<Context> contexts;
  
  // loads in the best params for each context from best_params.txt
  void LoadContexts_();
  // detects whether the current context is high or low observation noise
  void DetectObservationContext_();
  // detects whether the current context is high or low motion noise
  void DetectMotionContext_();
  // detects whether the current context has many or few obstacles
  void DetectObstacleContext_();
};


} // namespace auto_tune

#endif // SRC_AUTO_TUNE_H_
