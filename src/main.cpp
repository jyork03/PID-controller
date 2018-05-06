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

  PID steerPid;
  PID throttlePid;
  // TODO: Initialize the pid variable.
//  std::vector<double> p = {0.267467, 0.002, 9.33626};
  // throttle p
  std::vector<double> tp = {0.25, 0.002, 10.0};
//  std::vector<double> p = {0.157101, 0.00118952, 8.86528};
//  std::vector<double> p = {2.8541, 0.0015, 9.20589};
//  std::vector<double> p = {0.0569862, 0.00101864, 3.9014};
//  std::vector<double> p = {0.268432, 0.00018952, 9.41039};
  std::vector<double> p = {0.228018, 0.00732519, 7.54436};
//  3.30619, -0.000949124, 9.89487: error of 0.0535367
  std::vector<double> dp = {0.0115008, 0.000156183, 0.0348525};
//  std::vector<double> dp = {1.0, 0.01, 1.0};
//  std::vector<double> dp = {0.000271154, 3.01282e-06, 0.000328096};
  std::vector<double> tdp = {0.000271154, 3.01282e-06, 0.000328096};
//  std::vector<double> dp = {0.0261852, 3.20042e-05, 0.0214243};

//  int i = 0;
//  int k = 0;
//  int step = 0;
  int totalTimeSteps = 2000;
  double target_throttle = 0.5;

//  steerPid.best_t_error = 0.0;
//  throttlePid.best_t_error = 0.0;

  steerPid.Init(p);
  throttlePid.Init(tp);


  // Should probably only update one at a time
  steerPid.optimizing = true;
  throttlePid.optimizing = false;

  Twiddle twiddle(totalTimeSteps);

  h.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          // update the proportional, integral and derivative cross-track error
          steerPid.UpdateError(cte);

          // update the proportional, integral and derivative target throttle error
          double tte = speed - (target_throttle * 100.0); // target throttle error
          throttlePid.UpdateError(tte);

          // normalize to the range of accepted steering angles -25 to 25 (50 total)
          steer_value = steerPid.TotalError() / deg2rad(50.0);
          // limit the steer_value from -1.0 to 1.0
          if(steer_value > 1.0) steer_value = 1.0;
          if(steer_value < -1.0) steer_value = -1.0;

          // normalize to the range of accepted
//          throttle = throttlePid.TotalError() / 100.0;
          throttle = target_throttle;
          // limit the throttle from 0.0 to 1.0
          if(throttle > 1.0) throttle = 1.0;
          if(throttle < 0.0) throttle = 0.0;

          // DEBUG
//          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

//          if(throttlePid.optimizing) {
//
//          } else {
//            json msgJson;
//            msgJson["steering_angle"] = steer_value;
//            msgJson["throttle"] = throttle;
//            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//            std::cout << msg << std::endl;
//            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
//          }

          if(!steerPid.optimizing && !throttlePid.optimizing) {
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }

          if(throttlePid.optimizing) {
            bool outOfBounds = fabs(cte) > 4.0;
            twiddle.Run(tte, ws, throttlePid, outOfBounds);
          } else if(steerPid.optimizing) {
            bool outOfBounds = fabs(cte) > 4.0;
            twiddle.Run(cte, ws, throttlePid, outOfBounds);
          }

          if(steerPid.optimizing) {

            if(twiddle.i < twiddle.totalSteps) {


              if(fabs(cte) > 4.0) {
                // if the car goes off the track, add the square of the cross-track
                // error for each future timestep until totalTimeSteps
                twiddle.ExitEarly(4.0);

              } else {
                json msgJson;
                msgJson["steering_angle"] = steer_value;
                msgJson["throttle"] = throttle;
                auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                twiddle.UpdateOnTimeStep(cte);
              }

            }
            if(twiddle.error == 16.0 * twiddle.totalSteps) {
              // prevent erroneous second pass due to asynchronous nature of the socket connection
              // Extra frames from the simulation were being received before the

              // reset values
              twiddle.Reset();
              steerPid.Init(p);

              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              return;
            }
            if(twiddle.i == twiddle.totalSteps) {
              // check exactly equal to totalTimeSteps.  A simple else clause was getting fired twice
              std::cout << "Using Coeffecients: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
              std::cout << "dp: " << dp[0] << ", " << dp[1] << ", " << dp[2] << std::endl;

              twiddle.UpdateCoefficients(p, dp);

              if(dp[0] + dp[1] + dp[2] > 0.00001) {
                // reset values
                twiddle.Reset();
                steerPid.Init(p);

                std::string msg = "42[\"reset\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

              } else {
                std::cout << "Converged Coeffecients: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
              }
            }
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
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
