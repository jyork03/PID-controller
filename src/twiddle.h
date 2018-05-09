//
// This is a recursive implementation of the Twiddle Algorithm
//

#ifndef PID_TWIDDLE_H
#define PID_TWIDDLE_H

#include <vector>
#include <uWS/uWS.h>
#include "PID.h"

class Twiddle {
public:

  int i;
  int j;
  int step;
  int totalSteps;
  int firstNIgnoredSteps;
  double error;
  double best_mse;
  double maxError;

  std::vector<double> dp;

  /*
  * Constructor
  */
  explicit Twiddle(int totalSteps);

  /*
  * Destructor.
  */
  virtual ~Twiddle() = default;

  void Reset();

  void ExitEarly();

  void UpdateOnTimeStep(double errorOnTimeStep);

  void UpdateCoefficients(std::vector<double> &p);

  void Run(double error_t, uWS::WebSocket<uWS::SERVER> &ws, PID &pid, bool outOfBounds, std::string msg);
};


#endif //PID_TWIDDLE_H
