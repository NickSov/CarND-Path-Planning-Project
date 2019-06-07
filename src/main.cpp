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

  // Determine lane change need

  int tooCloseCtr = 0;
  int ctrVal = 200;

  bool consdResult = false;
  bool tryRight = false;

  h.onMessage([&consdResult,&tryRight, &ctrVal,&tooCloseCtr, &refVel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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

          // for (int i=0; i<sensor_fusion.size();i++){
          //   std::cout << sensor_fusion[i] << std::endl;
          // }

          double laneInD = 2+4*lane; //lane in relation to d axis
          double centerLane_d = 2+4*1; //lane in relation to d axis
          double leftLane_d = 2+4*0; //left lane in relation to d axis
          double rightLane_d = 2+4*2; //right lane in relation to d axis
          double followDist = 35; // 10 meter following distance
          double distDiff = 0; // difference between ego vehicle and next vehicle in ego lane
          double velFrVeh = 0; // front ego lane vehicle velocity, mph
          double distDiffT1 = 0;
          double distDiffT2;
          double laneChgSpaceMin = 40;
          double vehID;
          double prevLaneRq; // previous lane request
          bool tooClose = false; // flag for whether the vehicle is too close
          bool vehAhead = false; // veh ahead indicator, true if slowing down.

          prevLaneRq = lane;

          // check on other cars in front and slow down to necessary speed

          for(int i=0; i<sensor_fusion.size(); i++){
            if ((sensor_fusion[i][6] < laneInD + 1) && (sensor_fusion[i][6] > laneInD - 1)){
              double disNextVeh = (double)sensor_fusion[i][5];
              distDiff = disNextVeh - car_s;
              //std::cout << "car is : "<< distDiff << " m away" << std::endl;
              if (distDiff <= followDist && disNextVeh > car_s){
                // std::cout << "Distance difference : "<< distDiff << std::endl;
                vehID = sensor_fusion[i][0];
                velFrVeh = 2.24 * sqrt(pow((double)sensor_fusion[i][3],2) + pow((double)sensor_fusion[i][4],2));
                // std::cout << "Front vehicle speed (mph) : "<< velFrVeh << std::endl;
                tooClose = true;
                tooCloseCtr += 1;
              }
            }
          }

          // std::cout << "Front vehicle speed (mph) : "<< velFrVeh << std::endl;
          // std::cout << "tooCloseCtr : "<< tooCloseCtr << std::endl;
          // std::cout << "car_speed : "<< car_speed << std::endl;
          // std::cout << "refVel : "<< refVel << std::endl;

          if(tooClose && car_speed > velFrVeh){
            //std::cout << "Phase 1 " << std::endl;
            refVel -= 0.224; //-.224
            std::cout << "slowing down... " << std::endl;
            // decrease to speed of front vehicle
          } else if (refVel < 49.5){
            refVel += 0.200;
          }

          // Create a matrix out of incoming sensor_fusion JSON element

          vector<vector<double>> sensFusVect;
          vector<double> tempVect;
          int laneChangeOption;

          for(int i=0; i<sensor_fusion.size(); i++){
            for (int j=0; j<7; j++){
              tempVect.push_back((double)sensor_fusion[i][j]);
            }
            sensFusVect.push_back(tempVect);
            tempVect.clear();
          }

          // std::cout << sensFusVect[0][0] << sensFusVect[0][6] << std::endl;
          // std::cout << sensFusVect[6][0] << sensFusVect[6][6] << std::endl;

          // laneChangeOption 0 = check if lane change is possible to left lane
          // laneChangeOption 1 = check if lane change is possible to center lane
          // laneChangeOption 2 = check if lane change is possible to right lane

          // check for lane change

          if (tooClose == true){
            if(lane == 0){
              laneChangeOption = 1;
              consdResult = ConsiderLaneChange(car_s,laneChangeOption,laneChgSpaceMin,
                          leftLane_d, rightLane_d, centerLane_d, sensFusVect);
            } else if (lane == 2){
              laneChangeOption = 1;
              consdResult = ConsiderLaneChange(car_s,laneChangeOption,laneChgSpaceMin,
                          leftLane_d, rightLane_d, centerLane_d, sensFusVect);
            } else if (lane == 1){
              laneChangeOption = 0; // first checking if merging possible to left lane
              consdResult = ConsiderLaneChange(car_s,laneChangeOption,laneChgSpaceMin,
                          leftLane_d, rightLane_d, centerLane_d, sensFusVect);
              if(consdResult == false){
                laneChangeOption = 2; // second checking if merging possible to right lane
                consdResult = ConsiderLaneChange(car_s,laneChangeOption,laneChgSpaceMin,
                            leftLane_d, rightLane_d, centerLane_d, sensFusVect);
              }
            } else {
              consdResult = false;
            }
          } else {
            consdResult = false;
          }

          //std::cout << "tooCloseCtr: " << tooCloseCtr << std::endl;

          // Execute lane change

          if(lane == 0 && consdResult == true && tooCloseCtr > ctrVal){
            std::cout << "Change from left to center" << std::endl;
            lane = 1;
            tooCloseCtr = 0;
          } else if (lane == 2 && consdResult == true && tooCloseCtr > ctrVal){
            std::cout << "Change from right to center" << std::endl;
            lane = 1;
            tooCloseCtr = 0;
          } else if (lane == 1 && consdResult == true && tryRight == false && tooCloseCtr > ctrVal){
            std::cout << "Change from center to left" << std::endl;
            lane = 0;
            tooCloseCtr = 0;
          } else if (lane == 1 && consdResult == true && tryRight == true && tooCloseCtr > ctrVal)  {
            std::cout << "Change from center to right" << std::endl;
            lane = 2;
            tooCloseCtr = 0;
          } else {
            lane = prevLaneRq;
          }

          //
          // //std::cout << "Too close iteration : "<< tooCloseCtr << std::endl;
          //
          // ///// CHANGE LANES /////
          //
          //
          // // Left Lane Change Variables
          // int t1_CL1_TL0 = 0; // test 1, current lane one, target lane zero
          // int t2_CL1_TL0 = 0; // test 2, current lane one, target lane zero
          // int t1_CL2_TL1 = 0; // test 1, current lane two, target lane one
          // int t2_CL2_TL1 = 0; // test 2, current lane two, target lane one
          // // Right Lane Change Variables
          // int t1_CL0_TL1 = 0; // test 1, current lane zero, target lane one
          // int t2_CL0_TL1 = 0; // test 2, current lane zero, target lane one
          // int t1_CL1_TL2 = 0; // test 1, current lane one, target lane two
          // int t2_CL1_TL2 = 0; // test 2, current lane one, target lane two
          //
          // /// Change Lanes Left
          //
          // if(car_speed < 49 && tooClose == true){ // if staying behind slow car for too long, change lanes
          //   for (int i=0; i<sensor_fusion.size(); i++){
          //     if(laneInD == centerLaneInD){ // if starting in center lane
          //       if(((sensor_fusion[i][6] < leftLaneInD + 0.5)
          //       && (sensor_fusion[i][6] > leftLaneInD - 0.5))){
          //         t1_CL1_TL0 += 1;
          //         double distLftLaneVeh = (double)sensor_fusion[i][5];
          //         double lftLaneVehID = (double)sensor_fusion[i][0];
          //         double disDiffLftLaneVeh = fabs(distLftLaneVeh - car_s);
          //         double spaceLeftLane = 50;
          //         if(distLftLaneVeh > spaceLeftLane){
          //           t2_CL1_TL0 += 1;
          //         }
          //       }
          //     }
          //     if(laneInD == rightLaneInD){ // if starting in left lane
          //       if (((sensor_fusion[i][6] < centerLaneInD + 0.5)
          //       && (sensor_fusion[i][6] > centerLaneInD - 0.5))){
          //         t1_CL2_TL1 += 1;
          //         double distLftLaneVeh = (double)sensor_fusion[i][5];
          //         double lftLaneVehID = (double)sensor_fusion[i][0];
          //         double disDiffLftLaneVeh = fabs(distLftLaneVeh - car_s);
          //         double spaceLeftLane = 50;
          //         if(distLftLaneVeh > spaceLeftLane){
          //           t2_CL2_TL1 += 1;
          //         }
          //       }
          //     }
          //   }
          // }
          //
          // // execute change lanes
          // if (car_speed < 49 && tooClose == true){  //check to that all lanes are clear
          //   if(t1_CL1_TL0 == t2_CL1_TL0 && t2_CL1_TL0 > 0){
          //     std::cout << "changing from center lane to left lane" << std::endl;
          //     if(laneInD == centerLaneInD){
          //       lane = 0;
          //     }
          //   } else if (t1_CL2_TL1 == t2_CL2_TL1 && t2_CL2_TL1 > 0 ){
          //     std::cout << " changing from right lane to center lane" << std::endl;
          //     if(laneInD == rightLaneInD){
          //       lane = 1;
          //     }
          //   }
          // }
          //
          // /// Change lanes right
          //
          // if(car_speed < 49 && tooClose == true){ // if staying behind slow car for too long, change lanes
          //   for (int i=0; i<sensor_fusion.size(); i++){
          //     if(laneInD == centerLaneInD){ // if starting in center lane
          //       if(((sensor_fusion[i][6] < rightLaneInD + 0.5)
          //       && (sensor_fusion[i][6] > rightLaneInD - 0.5))){
          //         t1_CL1_TL2 += 1;
          //         double distRghtLaneVeh = (double)sensor_fusion[i][5];
          //         double rghtLaneVehID = (double)sensor_fusion[i][0];
          //         double disDiffRghtLaneVeh = fabs(distRghtLaneVeh - car_s);
          //         double spaceRightLane = 50;
          //         if(distRghtLaneVeh > spaceRightLane){
          //           t2_CL1_TL2 += 1;
          //         }
          //       }
          //     }
          //     if(laneInD == leftLaneInD){ // if starting in left lane
          //       if (((sensor_fusion[i][6] < centerLaneInD + 0.5)
          //       && (sensor_fusion[i][6] > centerLaneInD - 0.5))){
          //         t1_CL0_TL1 += 1;
          //         double distRghtLaneVeh = (double)sensor_fusion[i][5];
          //         double rghtLaneVehID = (double)sensor_fusion[i][0];
          //         double disDiffRghtLaneVeh = fabs(distRghtLaneVeh - car_s);
          //         double spaceRightLane = 50;
          //         if(distRghtLaneVeh > spaceRightLane){
          //           t2_CL0_TL1 += 1;
          //         }
          //       }
          //     }
          //   }
          // }
          //
          // //std::cout << "car speed: " << car_speed << std::endl;
          //
          // // execute change lanes
          // if (car_speed < 49 && tooClose == true){  //check to that all lanes are clear
          //   if(t1_CL1_TL2 == t2_CL1_TL2 && t2_CL1_TL2 > 0){
          //     std::cout << "changing from center lane to right lane" << std::endl;
          //     if(laneInD == centerLaneInD){
          //       lane = 2;
          //     }
          //   } else if (t1_CL0_TL1 == t2_CL0_TL1 && t2_CL0_TL1 > 0){
          //     std::cout << " changing from left lane to center lane" << std::endl;
          //     if(laneInD == centerLaneInD){
          //       lane = 1;
          //     }
          //   }
          // }

          // std::cout << "t1_CL1_TL0: "<< t1_CL1_TL0 << std::endl;
          // std::cout << "t2_CL1_TL0: "<< t2_CL1_TL0 << std::endl;
          // std::cout << "t1_CL2_TL1: "<< t1_CL2_TL1 << std::endl;
          // std::cout << "t2_CL2_TL1: "<< t2_CL2_TL1 << std::endl;
          //
          // std::cout << "t1_CL0_TL1: "<< t1_CL0_TL1 << std::endl;
          // std::cout << "t2_CL0_TL1: "<< t2_CL0_TL1 << std::endl;
          // std::cout << "t1_CL1_TL2: "<< t1_CL1_TL2 << std::endl;
          // std::cout << "t2_CL1_TL2: "<< t2_CL1_TL2 << std::endl;
          // std::cout << "laneInD: "<< laneInD << std::endl;

          //std::cout << "- - - - - - - -- - - - - - - -- - - - " << std::endl;


          // if(tooCloseCtr > 100){ // reset too close variable
          //   tooCloseCtr = 0;
          // }

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
          //std::cout << "- - - - - - -- - - - - - - --  " << std::endl;

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
