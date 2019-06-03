#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "helpers.h"
#include "spline.h"
#include "json.hpp"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  int init = 0;

  vector<double> alphas; // Polynomial constants

  // lane indication

  int lane = 1;

  // reference velocity

  double refVel = 0; // in mph

  h.onMessage([&refVel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          for (int i=0; i<sensor_fusion.size();i++){
            std::cout << sensor_fusion[i] << std::endl;
          }

          double laneInD = 2+4*lane; //lane in relation to d axis
          double followDist = 35; // 10 meter following distance
          double distDiff = 0; // difference between ego vehicle and next vehicle in ego lane
          double velocity = 0; // velocity nearest ego lane car
          double distDiffT1 = 0;
          double distDiffT2;
          double vehID;
          double lowestSpeed; //lowest speed recorded for the ego lane vehicle, m/s
          double lowestSpeedPrev; // previous lowest speed recorded for the ego lane vehicle, m/s
          bool tooClose = false; // flag for whether the vehicle is too close

          // check on other cars in front and slow down to necessary speed

          for(int i=0; i<sensor_fusion.size(); i++){
            if ((sensor_fusion[i][6] < l aneInD + 0.5) && (sensor_fusion[i][6] > laneInD - 0.5)){
              double disNextVeh = (double)sensor_fusion[i][5];
              distDiff = disNextVeh - car_s;
              std::cout << "car is : "<< distDiff << " m away" << std::endl;
              if (distDiff <= followDist && disNextVeh > car_s){
                vehID = sensor_fusion[i][0];
                distDiffT1 = sensor_fusion[i][5];
                std::cout << "car ID : "<< vehID << std::endl;
                std::cout << "distDiffT1: "<< distDiffT1 << ", distDiffT2: " << distDiffT2 << std::endl;
                velocity = fabs(distDiffT1 - distDiffT2)/.020;
                if(velocity < lowestSpeedPrev){
                  lowestSpeed = velocity;
                }
                std::cout << "car is near and traveling with: "<< lowestSpeed << "m/s" << std::endl;
                std::cout << "Ego Velocity : "<< car_speed << std::endl;
                distDiffT2 = distDiffT1;
                lowestSpeedPrev = lowestSpeed;
                tooClose = true;
              }
            }
          }

          if(tooClose){
            refVel -= 0.224;
          } else if(refVel < 49.5){
            refVel += 0.224;
          }






          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // stay in lane

          int prevSize = previous_path_x.size();

          // widely spaced way points list

          vector<double> ptsx;
          vector<double> ptsy;

          // reference x,y, yaw states

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if(prevSize < 2){

            //std::cout << "START " << std::endl;

            // previous path size is small - just starting out
            //use to points that create a path tangent to the yaw of the cars

            double prevCarX = car_x - cos(car_yaw);  // go backwards in time to generate another point based on yaw
            double prevCarY = car_y - sin(car_yaw);

            ptsx.push_back(prevCarX);
            ptsx.push_back(car_x);

            ptsy.push_back(prevCarY);
            ptsy.push_back(car_y);

          } else {

            //std::cout << "ELSE " << std::endl;

            // path contains some points
            ref_x = previous_path_x[prevSize-1];
            ref_y = previous_path_y[prevSize-1];

            //std::cout << "ref_x " << ref_x <<  std::endl;

            double refXPrevious = previous_path_x[prevSize-2];
            double refYPrevious = previous_path_y[prevSize-2];

            //std::cout << "refXPrevious" << previous_path_x[prevSize-2] <<  std::endl;

            ref_yaw = atan2(ref_y-refYPrevious, ref_x-refXPrevious);

            ptsx.push_back(refXPrevious);
            ptsx.push_back(ref_x);

            ptsy.push_back(refYPrevious);
            ptsy.push_back(ref_y);

          }

          // add spaced waypoints within Frenet coordinates

          vector<double> wp1 = getXY(car_s+30,(laneInD),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> wp2 = getXY(car_s+60,(laneInD),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> wp3 = getXY(car_s+90,(laneInD),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          ptsx.push_back(wp1[0]);
          ptsx.push_back(wp2[0]);
          ptsx.push_back(wp3[0]);

          ptsy.push_back(wp1[1]);
          ptsy.push_back(wp2[1]);
          ptsy.push_back(wp3[1]);

          //std::cout << "Points in ptsx prior to transform into veh coords: " << std::endl;

          // for (int i=0; i<ptsx.size();i++){
          //   std::cout << ptsx[i] << std::endl;
          // }

          // shift to car reference set_points

          for (int i=0; i < ptsx.size(); i++){

            double shiftX = ptsx[i]-ref_x;
            double shiftY = ptsy[i]-ref_y;

            ptsx[i] = (shiftX*cos(0-ref_yaw)-shiftY*sin(0-ref_yaw));
            ptsy[i] = (shiftX*sin(0-ref_yaw)+shiftY*cos(0-ref_yaw));

          }

          //std::cout << "part 1 " << std::endl;

          // create TK_SPLINE_H
          tk::spline s;

          //std::cout << "part 1.2 " << std::endl;

          //set x,y spline set_points

          //std::cout << "Points in ptsx prior to spline func: " << std::endl;

          // for (int i=0; i<ptsx.size();i++){
          //   std::cout << ptsx[i] << std::endl;
          // }

          //std::cout << "Points within spline function:  " << std::endl;

          s.set_points(ptsx,ptsy);




          //std::cout << "part 2 " << std::endl;

          // define actual x,y points we will user for the Planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Add in previous path set_points
          for (int i=0; i<previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // break up points for the TK_SPLINE_H

          double targetX = 30.0;
          double targetY = s(targetX);
          double targetDist = sqrt((targetX)*(targetX)+(targetY)*(targetY));
          double xAddOn = 0;

          //std::cout << "part 3 " << std::endl;

          // fill out rest of path_planning

          //std::cout << "Generate All Points "<< std::endl;

          for (int i = 1; i <= 50-previous_path_x.size(); i++){
            double N = (targetDist/(.02*refVel/2.24));
            double xPoint = xAddOn+(targetX)/N;
            double yPoint = s(xPoint);

            xAddOn = xPoint;

            double xRef = xPoint;
            double yRef = yPoint;

            // rotate back to map coordinates from vehicle coordinates

            xPoint = (xRef*cos(ref_yaw)-yRef*sin(ref_yaw));
            yPoint = (xRef*sin(ref_yaw)+yRef*cos(ref_yaw));

            xPoint += ref_x;
            yPoint += ref_y;

            //std::cout << "xPoint: " << xPoint << std::endl;

            next_x_vals.push_back(xPoint);
            next_y_vals.push_back(yPoint);

          }

          //
          // vector<double> xyVect;
          //
          // double distInc = 0.3;
          // for (int i=0; i<50; ++i){
          //   car_s = car_s + i*0.5;
          //   car_d = car_d;
          //
          //   // std::cout << "car position: " << car_s << std::endl;
          //
          //   xyVect = getXY(car_s,car_d, map_waypoints_s, map_waypoints_x,map_waypoints_y);
          //
          //   next_x_vals.push_back(xyVect[0]);
          //   next_y_vals.push_back(xyVect[1]);
          // }


          //std::cout << "car speed: " << car_speed << std::endl;
          std::cout << "- - - - - - -- - - - - - - --  " << std::endl;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
