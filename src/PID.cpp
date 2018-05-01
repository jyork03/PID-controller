#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->optimizing = true;
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

void PID::Twiddle() {
}

