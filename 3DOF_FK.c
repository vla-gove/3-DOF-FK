#include <stdio.h>
#include <math.h>

#define PI 3.14159265

// point in space representation
typedef struct {
  double x;
  double y;
  double z;
} Point;

// #DOF robot struct
typedef struct {
  double theta1; // joint 1 angle
  double theta2; // joint 2 angle
  double theta3; // joint 3 angle
  double l1;     // link 1 length
  double l2;     // link 2 length
  double l3;     // link 3 length
} Robot;

// forward kinematics functions which takes joint parameters as arguments
Point forwardKinematics(Robot robot) {

  // joint position computation

  Point p1 = {0, 0, 0};

  Point p2 = {robot.l1 * cos(robot.theta1), robot.l1 * sin(robot.theta1), 0};

  Point p3 = {p2.x + robot.l2 * cos(robot.theta1 + robot.theta2),
              p2.y + robot.l2 * sin(robot.theta1 + robot.theta2),
              0};

  // end effector position
  Point p4 = {p3.x + robot.l3 * cos(robot.theta1 + robot.theta2 + robot.theta3),
              p3.y + robot.l3 * sin(robot.theta1 + robot.theta2 + robot.theta3),
              0};

  // returning the position of an end effector
  return p4;
}

int main() {
  // defining a robot variable using Robot
  Robot robot = {PI/3, PI, PI/4, 1, 1, 1};

  // computing the forward kinematics for the given joint parameters of robot
  Point p = forwardKinematics(robot);

  // printing the end effector position for the given joint parameters
  printf("End-effector position: (%.2f, %.2f, %.2f)\n", p.x, p.y, p.z);

  return 0;
}
