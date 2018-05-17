#include <uWS/uWS.h>
#include <iostream>
#include <vector>
#include "json.hpp"
#include "PID.h"
#include "twiddle.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // Initialize the pid variable.
  PID steerPid;
  PID throttlePid;

  std::vector<double> tp = {100.0, 0.003, 5.0};
  std::vector<double> p = {0.2205, 0.000005, 6.7081};

  int totalTimeSteps = 4000;
  double target_throttle = 0.5;

  // initialize the PID controllers
  steerPid.Init(p);
  throttlePid.Init(tp);

  // Should only update one at a time
  steerPid.optimizing = false;
  throttlePid.optimizing = false;

  // initialize twiddle with the totalTimeSteps
  Twiddle twiddle(totalTimeSteps);

  h.onMessage(
      [&steerPid, &throttlePid, &twiddle, &target_throttle]
          (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value, throttle;
          /*
          * Calcuate steering value here, remember the steering value is [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          // update the proportional, integral and derivative cross-track error
          steerPid.UpdateError(cte);

          // update the proportional, integral and derivative target throttle error
          double tte = speed - (target_throttle * 100.0); // target throttle error
          throttlePid.UpdateError(tte);

          // normalize to the range of accepted steering angles -25 to 25 (50 total)
          steer_value = steerPid.TotalError();
          // limit the steer_value from -1.0 to 1.0
          if(steer_value > 1.0) steer_value = 1.0;
          if(steer_value < -1.0) steer_value = -1.0;

          // normalize to the range of accepted
          throttle = throttlePid.TotalError() / 100.0;
          // limit the throttle from 0.0 to 1.0
          if(throttle > 1.0) throttle = 1.0;
          if(throttle < 0.0) throttle = 0.0;

          // DEBUG
//          std::cout << "Speed: " << speed << std::endl;
//          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
//          std::cout << "TTE: " << tte << " Throttle: " << throttle << std::endl;

          // Go ahead and create the message to send back to the sim
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          if(throttlePid.optimizing) {
            // if we're optimizing the throttle PID coefficients
            // consider it out of bounds if the absolute value of CTE is > 4.0
            bool outOfBounds = fabs(cte) > 4.0;
            // set a reasonable maximum error for the target throttle error
            twiddle.maxError = 100.0;
            // run the twiddle optimizer
            twiddle.Run(tte, ws, throttlePid, outOfBounds, msg);
          } else if(steerPid.optimizing) {
            // if we're optimizing the steering PID coefficients
            // consider it out of bounds if the absolute value of CTE is > 4.0
            bool outOfBounds = fabs(cte) > 4.0;
            // set a reasonable maximum error for the CTE. In this case, we're using the boundary value
            twiddle.maxError = 4.0;
            // run the twiddle optimizer
            twiddle.Run(cte, ws, steerPid, outOfBounds, msg);
          } else {
            // if we're just running the simulator with the preset coefficient values
//            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h, &steerPid, &throttlePid](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;

    // go ahead and reset the errors for whichever pid is not the one currently optimizing.
    if(steerPid.optimizing) {
      throttlePid.ResetErrors();
    }
    if(throttlePid.optimizing) {
      steerPid.ResetErrors();
    }
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
