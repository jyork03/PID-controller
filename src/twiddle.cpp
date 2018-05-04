//
// This is a recursive implementation of the Twiddle Algorithm
//

#include "twiddle.h"
#include <iostream>

Twiddle::Twiddle(std::vector<double> &p, std::vector<double> &dp, int totalSteps) {
  this->p = p;
  this->dp = dp;
  this->totalSteps = totalSteps;

  i = 0;
  j = 0;
  step = 0;

  error = 0.0;
  best_error = 0.0;
}


Twiddle::~Twiddle() {}


void Twiddle::Reset() {
  i = 0;
  j = 0;
  step = 0;

  error = 0.0;
}


void Twiddle::ExitEarly(double maxError) {
  // Add the square of the error for each future time step until totalSteps
  error += (totalSteps - i) * (maxError * maxError);
  i = totalSteps;
}

void Twiddle::UpdateOnTimeStep(double errorOnTimeStep) {
  error += errorOnTimeStep * errorOnTimeStep;
  i++;
}


void Twiddle::Run() {
  // mean squared error
  error = error / totalSteps;

  // Kp: coeff == 0 , Ki: coeff == 1, Kd: coeff == 2
  int coeff = j % 3;


  if(best_error == 0.0) {
    // first time through
    best_error = error;

    p[coeff] += dp[coeff];
  } else {
    std::cout << "error: " << error << " best error: " << best_error << std::endl;

    if(step % 2 == 0) {
      // check if the previous increment improved performance
      if(error < best_error) {
        // there was improvement, update the best_error
        best_error = error;
        // expand the next step's search space
        dp[coeff] *= 1.1;
        // skip step 2, since we already found an improvement by incrementing
        step += 2;
        // move to next coefficient
        j++;
      } else {
        // no improvement, try a decrement
        p[coeff] -= 2 * dp[coeff];

        // go on to step 2 to check if decrementing resulted in improvements
        step += 1;
      }

    } else if(step % 2  == 1) {
      // if step one resulted in a change of direction (decrement, not improved, etc.)
      // ie. we should only enter step 2 if step 1 resulted in a decrement
      if(error < best_error) {
        // if the previous decrement was an improvement, update the best_error
        best_error = error;
        // expand the next step's search space
        dp[coeff] *= 1.1;
      } else {
        // neither increasing  or decreasing p[coeff] by dp[coeff] resulted in an improvement
        // try resetting to the original p[coeff] value and decreasing te next step's search
        // space because the optimal p[coeff] value must be somewhere between the two.

        p[coeff] += dp[coeff]; // reset to original p[coeff] value before incrementing or decrementing
        dp[coeff] *= 0.9; // next dp[coeff] will be 90% of the size
      }

      // move to next coefficient
      j++;

      // go ahead and increment p[next_coeff] by the dp[next_coeff]
      int next_coeff = j % 3;
      p[next_coeff] += dp[next_coeff];

      // go to step 1 for the next coefficient
      step++;
    }
  }

}

