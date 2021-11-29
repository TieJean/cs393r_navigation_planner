#include <fstream>
#include <iostream>
#include <bits/stdc++.h>

#include "auto_tune.h"

using namespace std;

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

  // for (auto& context : contexts) {
	// 	cout << context.name << endl;
  //   cout << context.param.sensor_std << " ";
  //   cout << context.param.d_short << " ";
  //   cout << context.param.d_long << " ";
  //   cout << context.param.motion_dist_k1 << " ";
  //   cout << context.param.motion_dist_k2 << " ";
  //   cout << context.param.motion_a_k1 << " ";
  //   cout << context.param.motion_a_k2 << endl;
	// }
}

void AutoTune::DetectContext() {
  // TODO
}



} // namespace auto_tune

int main() {
  cout << "Auto" << endl;
	auto_tune::AutoTune autoTune;
	return 0;
}


