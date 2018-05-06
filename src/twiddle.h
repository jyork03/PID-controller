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
  double error;
  double best_mse;

//  std::vector<double> p;
//  std::vector<double> dp;

  /*
  * Constructor
  */
  explicit Twiddle(int totalSteps);

  /*
  * Destructor.
  */
  virtual ~Twiddle() = default;

  void Reset();

  void ExitEarly(double maxError);

  void UpdateOnTimeStep(double errorOnTimeStep);

  void UpdateCoefficients(std::vector<double> &p, std::vector<double> &dp);

  void Run(double error_t, uWS::WebSocket<uWS::SERVER> &ws, PID &pid, bool outOfBounds);
};


#endif //PID_TWIDDLE_H
