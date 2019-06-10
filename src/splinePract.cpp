#include <iostream>
#include <fstream>
#include <vector>
#include "spline.h"

int main() {

  double car_s = 50;

  std::vector<double> S={car_s+0, car_s+20, car_s+40,car_s+60,car_s+80,car_s+100};
  std::vector<double> D={6,6,6,2,2,2};
  std::vector<double> S2={car_s+0, car_s+20, car_s+40,car_s+70,car_s+80,car_s+100};
  std::vector<double> D2={6,6,6,2,2,2};
  std::ofstream dataFileOut;
  double s3;

  dataFileOut.open("spline_curve.txt", std::ofstream::out | std::ofstream::trunc);;

  // more aggresive lane change w/ spline
  tk::spline s;
  s.set_points(S,D);    // currently it is required that X is already sorted

  // less aggressive lane change
  tk::spline s2;
  s2.set_points(S2,D2);    // currently it is required that X is already sorted

  for(int i=car_s+0; i<100+car_s;i=i+2){

  // Add in values for an instaneous lane change
    if (i <= car_s+40){
      s3 = 6;
    } else {
      s3 = 2;
    }
    dataFileOut << i << " " << s3 << " " << s(i) << " " << s2(i) <<"\n" ;
  }

  dataFileOut.close();

  return EXIT_SUCCESS;
}
