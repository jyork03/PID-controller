#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(std::vector<double> p) {
  Kp = p[0];
  Ki = p[1];
  Kd = p[2];

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) {
  // use the old p_error as the previous cte value
  d_error = cte - p_error;

  // the cte
  p_error = cte;

  // sum of cte values
  i_error += cte;
}

double PID::TotalError() {
  return - Kp * p_error - Kd * d_error - Ki * i_error;
}
