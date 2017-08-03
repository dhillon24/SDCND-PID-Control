#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <cmath>

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

  // Initialize the pid controllers and desired speed

  PID steering_pid;
  PID throttle_pid;

  steering_pid.Init(-0.1125, -0.0005, -1.00, 0.25, false);     // alternate (-0.1, -0.0005, -1.0, 0.25, false)
  throttle_pid.Init(0.07, 0.0015, 0.0, 0.1, false);            // alternate (0.1, 0.002, 0.0, 0.0, false)
  double desired_speed = 60.0;                            

  h.onMessage([&steering_pid,&throttle_pid,desired_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          
          // update steering and throttle controllers and get control inputs
          double steer_value = 0.0;
          double throttle_value = 0.3;

          steering_pid.UpdateError(cte); 
          steer_value = steering_pid.TotalError();

          // set adaptive desired speed to incorporate braking
          double speed_error = (1-0.5*tanh(fabs(cte)))*desired_speed - speed;
          throttle_pid.UpdateError(speed_error); 
          throttle_value = throttle_pid.TotalError();
          
          // DEBUG
          std::cout << "Count: " << steering_pid.count << std::endl;
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          std::cout << "Speed Error (Desired Speed = " << desired_speed << ") : "<< cte << " Throttle Value: " << throttle_value << std::endl;
          std::cout << "Steering Kp, Ki, Kd: " << steering_pid.Kp << ", "<<steering_pid.Ki << ", "<<steering_pid.Kd << std::endl;
          std::cout << "Throttle Kp, Ki, Kd: " << throttle_pid.Kp << ", "<<throttle_pid.Ki << ", "<<throttle_pid.Kd << std::endl; 

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
