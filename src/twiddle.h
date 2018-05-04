//
// This is a recursive implementation of the Twiddle Algorithm
//

#ifndef PID_TWIDDLE_H
#define PID_TWIDDLE_H

#include <vector>

class Twiddle {
public:

  int i;
  int j;
  int step;
  int totalSteps;
  double error;
  double best_error;

  std::vector<double> p;
  std::vector<double> dp;

  /*
  * Constructor
  */
  Twiddle(std::vector<double>& p, std::vector<double>& dp, int totalSteps);

  /*
  * Destructor.
  */
  virtual ~Twiddle();

  void Reset();

  void ExitEarly(double maxError);

  void UpdateOnTimeStep(double errorOnTimeStep);

  void Run();
};


#endif //PID_TWIDDLE_H
