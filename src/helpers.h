#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include "Eigen/Dense"

// for convenience
using std::string;
using std::vector;
using Eigen::MatrixXi;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y)
{
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Change lane function

bool ConsiderLaneChange(double car_s, int laneChangeOption, double laneChgSpaceMin, double leftLane_d,
     double rightLane_d, double centerLane_d, vector<vector<double>> sensFusVect){

  // laneChangeOption 0 = left lane change
  // laneChangeOption 1 = center lane change
  // laneChangeOption 2 = right lane change

  bool changeLanes = false;
  double laneID;
  double distNearVeh;
  double distDiff;
  double numVehClearCt = 0;
  vector<vector<double>> vehInLanes;

  // left lane change if starting in center lane
  if(laneChangeOption == 0){
    laneID = leftLane_d; // from center lane, check left lane for merging potential
  } else if(laneChangeOption == 1){
    laneID = centerLane_d; //from left or righ lane, check center lane for merging potential
  } else if (laneChangeOption == 2){
    laneID = rightLane_d; //from center lane, check right lane for merging potential
  }

  //std::cout << "laneChangeOption: " << laneChangeOption << std::endl;

  //std::cout << " ---------------------------- " << std::endl;

  for (int i=0; i<sensFusVect.size(); i++){
      if(((sensFusVect[i][6] < laneID + 0.5)
      && (sensFusVect[i][6] > laneID - 0.5))){
        vehInLanes.push_back(sensFusVect[i]);
        //std::cout << "vehicle in lane: " << laneID << " | location: " << sensFusVect[i][6]<< std::endl;
        }
    }

  for (int i=0; i<vehInLanes.size(); i++){
    distNearVeh = (double)vehInLanes[i][5];
    distDiff = fabs(distNearVeh - car_s);
    //std::cout << "distDiff: " << distDiff << std::endl;

    if(distDiff > laneChgSpaceMin){
      numVehClearCt += 1;
    }
  }

  if (vehInLanes.size() == numVehClearCt){
    changeLanes = true;
    // include truth counter in order to verify that the lane change really is OK
    //std::cout << " -------------- TRUE -------------- " << std::endl;
  } else {
    //std::cout << " ---------------------------- " << std::endl;
  }

  return changeLanes;
}

// Jerk minimizing trajectory polynomial solver

vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculates the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */

   // Start and End variables

   double s_i = start[0];
   double s_i_dot = start[1];
   double s_i_doubleDot = start[2];

   double s_f = end[0];
   double s_f_dot = end[1];
   double s_f_doubleDot = end[2];

   // Coefficients @ t=0, alpha 0,1,and 2

   double alpha_i = start[0];
   double alpha_i_dot = start[1];
   double alpha_i_doubleDot = 0.5*start[2];

   // Define matrices

   MatrixXd A(3,3);

   A << pow(T,3.0), pow(T,4.0), pow(T,5.0),
        3*pow(T,2.0), 4*pow(T,3.0), 5*pow(T,4),
        6*T, 12*pow(T,2), 20*pow(T,3);

   VectorXd X(3);

   VectorXd B(3);
   B << s_f-(s_i+s_i_dot*T+0.5*s_i_doubleDot*pow(T,2)),
        s_f_dot-(s_i_dot+s_i_doubleDot*T),
        s_f_doubleDot-s_i_doubleDot;

   // Solve system of equations, AX=B -> find X.

   X = A.inverse()*B;

   // Coefficients @ t=final, alpha 3,4,and 5

   double alpha_f = X[0];
   double alpha_f_dot = X[1];
   double alpha_f_doubleDot = X[2];

   vector<double> alphas;

   alphas = {alpha_i,alpha_i_dot,alpha_i_doubleDot,alpha_f,alpha_f_dot,alpha_f_doubleDot};

   return alphas;
  }

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

#endif  // HELPERS_H
