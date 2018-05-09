//
// This is a recursive implementation of the Twiddle Algorithm
//

#include "twiddle.h"
#include "PID.h"
#include <iostream>
#include "json.hpp"

// for convenience
using json = nlohmann::json;

Twiddle::Twiddle(int totalSteps) {
  this->totalSteps = totalSteps;
  firstNIgnoredSteps = 200;
  dp = {0.00778083, 0.00129147, 0.707348};

  i = 0;
  j = 0;
  step = 0;

  error = 0.0;
  best_mse = 0.0;
}

void Twiddle::Reset() {
  i = 0;
  error = 0.0;
}


void Twiddle::ExitEarly() {
  // Add the square of the error for each future time step until totalSteps
  error += (totalSteps - i) * (maxError * maxError);
  i = totalSteps;
}

void Twiddle::UpdateOnTimeStep(double errorOnTimeStep) {
  if(i == firstNIgnoredSteps) {
    std::cout << "Starting to record error..." << std::endl;
  }
  if(i > firstNIgnoredSteps) {
    // don't start calculating error until after firstNIgnoredSteps
    error += errorOnTimeStep * errorOnTimeStep;
  }
  i++;
}


void Twiddle::UpdateCoefficients(std::vector<double> &p) {
  // mean squared error
  double mse = error / (totalSteps - firstNIgnoredSteps);

  // Kp: coeff == 0 , Ki: coeff == 1, Kd: coeff == 2
  int coeff = j % 3;


  if (best_mse == 0.0) {
    // first time through
    best_mse = mse;

    p[coeff] += dp[coeff];
  } else {
    std::cout << "mse: " << mse << " best error: " << best_mse << std::endl;

    if (step % 2 == 0) {
      // check if the previous increment improved performance
      if (mse < best_mse) {
        // there was improvement, update the best_mse
        best_mse = mse;
        // expand the next step's search space
        dp[coeff] *= 1.1;
        // skip step 2, since we already found an improvement by incrementing
        step += 2;
        // move to next coefficient
        j++;
        // go ahead and increment p[next_coeff] by the dp[next_coeff]
        int next_coeff = j % 3;
        p[next_coeff] += dp[next_coeff];

      } else {
        // no improvement, try a decrement
        p[coeff] -= 2 * dp[coeff];

        // go on to step 2 to check if decrementing resulted in improvements
        step++;
      }

    } else if (step % 2 == 1) {
      // if step one resulted in a change of direction (decrement, not improved, etc.)
      // ie. we should only enter step 2 if step 1 resulted in a decrement
      if (mse < best_mse) {
        // if the previous decrement was an improvement, update the best_mse
        best_mse = mse;
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

void Twiddle::Run(double error_t, uWS::WebSocket<uWS::SERVER> &ws, PID &pid, bool outOfBounds, std::string msg) {
  if(i < totalSteps) {
    if(outOfBounds) {
      ExitEarly();
    } else {
      UpdateOnTimeStep(error_t);
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    }
  }
  if(error == (maxError * maxError) * (totalSteps - firstNIgnoredSteps)) {
    // prevent erroneous second pass due to asynchronous nature of the socket connection
    // Extra frames from the simulation were being received before the server initiated the restart.

    // reset values
    std::vector<double> p = {pid.Kp, pid.Ki, pid.Kd};
    Reset();
    pid.Init(p);

    std::string reset_msg = "42[\"reset\",{}]";
    ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
    return;
  }
  if(i == totalSteps) {
    // check exactly equal to totalTimeSteps.  A simple else clause was getting fired twice

    std::vector<double> p = {pid.Kp, pid.Ki, pid.Kd};

    std::cout << "Using Coeffecients: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
    std::cout << "dp: " << dp[0] << ", " << dp[1] << ", " << dp[2] << std::endl;

    UpdateCoefficients(p);

    if(dp[0] + dp[1] + dp[2] > 0.00001) {
      // reset values
      Reset();
      pid.Init(p);

      std::string reset_msg = "42[\"reset\",{}]";
      ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

    } else {
      std::cout << "Converged Coeffecients: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
    }
  }
}