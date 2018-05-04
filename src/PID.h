#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:

  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;


  /*
  * Twiddle Vars
  */

  // Twiddle Iteration
  int i;
  // Twiddle Error
  double t_error;
  // Best Twiddle Error
  double best_t_error;
  bool optimizing;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(std::vector<double> p);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

};

#endif /* PID_H */
