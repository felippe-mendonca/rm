#include <iostream>
#include "pioneer.hpp"

using namespace std;
using namespace arma;
using namespace robot::local; // simulated robot or using serial port
// using namespace robot::remote; // remote robot

int main() {
  Pioneer robot;     // robot will connect on localhost:8101 (MobileSim)
  // Pioneer robot2; // second robot will connect on localhost:8102 (MobileSim)
  
  // Pioneer robot1("192.168.1.142", 1110); // connect to first remote robot
  // Pioneer robot2("192.168.1.149", 1111); // connect to second remote robot

  mat speed;
  speed << 100.0 << endr << deg2rad(5.0);  // creates a 2x1 matrix
  robot.set_speed(speed);                  // expects a matrix with 2 elements {v,w} [mm/s,rad/s]
                                           // if necessary, the rad2deg function is also available
  mat initial_pose;
  initial_pose << 0.0 << endr << 0.0 << endr << deg2rad(0.0); // creates a 3x1 matrix
  robot.set_pose(initial_pose); // expects a matrix with 3 elements {x,y,psi} [mm,mm,rad]

  mat odometry;

  Loop loop;  // loop.wait() waits 100 ms (default value). Choose another with: Loop loop(200);
  do {
    mat pose = robot.get_pose();  // returns a 3x1 matrix {x,y,psi} [mm,mm,rad]
    pose.print("pose");

    odometry = join_horiz(odometry, pose);  // generetes a 3xN matrix with odometry

    loop.wait();                // waits until complete 100ms
  } while (loop.until_sec(3));  // until_sec(3) returns true while loop execution time is less than 3 seconds.
                                // until_ms is also available.

  odometry.save("odometry.mat", raw_ascii);  // to load on Matlab: load('odometry.mat','-ASCII');

  robot.disconnect();
  return 0;
}