#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
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
  // TODO: Initialize the pid variable.
//  std::vector<double> p = {0.267467, 0.002, 9.33626};
  std::vector<double> p = {0.157101, 0.00118952, 8.86528};
//  std::vector<double> p = {2.8541, 0.0015, 9.20589};
//  std::vector<double> p = {0.0569862, 0.00101864, 3.9014};
//  std::vector<double> p = {0.0, 0.0, 0.0};
  // 0.41601, 0, 5.75949
//  3.30619, -0.000949124, 9.89487: error of 0.0535367
  std::vector<double> dp = {0.1, 0.001, 0.1};
//  std::vector<double> dp = {0.0261852, 3.20042e-05, 0.0214243};

  int i = 0;
  int k = 0;
  int step = 0;
  int totalTimeSteps = 10000;
  float throttle = 0.5;

  steerPid.best_t_error = 0.0; // pick up where we left off

  steerPid.Init(p[0], p[1], p[2]);


  steerPid.optimizing = true;

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
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          steerPid.UpdateError(cte);

          // the car in the sim can handle a steering angle between -1 and 1 radian.
//          steer_value = deg2rad(steerPid.TotalError());
//          steer_value = steerPid.TotalError();
          steer_value = steerPid.TotalError() / deg2rad(25.0);
          if(steer_value > 1.0) steer_value = 1.0;
          if(steer_value < -1.0) steer_value = -1.0;

          // DEBUG
//          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;


          if(steerPid.optimizing) {

            if(i < totalTimeSteps) {


              if(fabs(cte) > 4.0) {
                // if the car goes off the track, add the square of the cross-track
                // error for each future timestep until totalTimeSteps
                steerPid.t_error += (totalTimeSteps - i) * (4.0 * 4.0);

                i = totalTimeSteps; // just skip to finish

              } else {
                json msgJson;
                msgJson["steering_angle"] = steer_value;
                msgJson["throttle"] = throttle;
                auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                steerPid.t_error += (cte * cte);

                i++;
              }

            }
            if(steerPid.t_error == 16.0 * totalTimeSteps) {
              // prevent erroneous second pass due to asynchronous nature of the socket connection

              // reset values
              i = 0;
              steerPid.t_error = 0.0;
              steerPid.p_error = 0.0;
              steerPid.i_error = 0.0;
              steerPid.d_error = 0.0;

              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              return;
            }
            if(i == totalTimeSteps) {
              // check exactly equal to totalTimeSteps.  A simple else clause was getting fired twice
              std::cout << "cte: " << cte << std::endl;
              std::cout << "step: " << step % 2 << std::endl;
              std::cout << "Using Coeffecients: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
              std::cout << "dp: " << dp[0] << ", " << dp[1] << ", " << dp[2] << std::endl;

              steerPid.t_error = steerPid.t_error / i; // mean squared

              // Kp: coeff == 0 , Ki: coeff == 1, Kd: coeff == 2
              int coeff = k % 3;

              if(steerPid.best_t_error == 0.0) {
                // first time through
                steerPid.best_t_error = steerPid.t_error;

                p[coeff] += dp[coeff];
              } else {
                std::cout << "error: " << steerPid.t_error << " best error: " << steerPid.best_t_error << std::endl;

                if(step % 2 == 0) {
                  if(steerPid.t_error < steerPid.best_t_error) {
                    steerPid.best_t_error = steerPid.t_error;
                    dp[coeff] *= 1.1;
                  } else {
                    p[coeff] -= 2 * dp[coeff];
                  }
                } else if(step % 2  == 1) {
                  if(steerPid.t_error < steerPid.best_t_error) {
                    steerPid.best_t_error = steerPid.t_error;
                    dp[coeff] *= 1.1;
                  } else {
                    p[coeff] += dp[coeff];
                    dp[coeff] *= 0.9;
                  }

                  // go ahead and add the next coefficient
                  p[(k + 1) % 3] += dp[(k + 1) % 3];

                  k++;
                }

                step++;
              }


              if(dp[0] + dp[1] + dp[2] > 0.00001) {
                // reset values
                i = 0;
                steerPid.t_error = 0.0;
                steerPid.p_error = 0.0;
                steerPid.i_error = 0.0;
                steerPid.d_error = 0.0;

                steerPid.Init(p[0], p[1], p[2]);

                std::string msg = "42[\"reset\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

              } else {
                std::cout << "Converged Coeffecients: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
              }
            }
          } else {
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
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
