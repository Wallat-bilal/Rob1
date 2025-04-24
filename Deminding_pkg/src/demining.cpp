// includes - reading of all files needed
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include "stdlib.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;  // type definition - everything from left saved as MoveBaseClient

double *findNextGoal(double A[5], float c) // initiation of function returning new coordinates - takes parameters float c and doublearray A
{ 
  // lines 14-20: initiating values from parameter array in new values named lenghtX, lenghtY, currentx, currentY, state, newX and newY
  c = c;
  float lengthX = A[0];
  float widthY = A[1];
  float currentX = A[2];
  float currentY = A[3];
  float state = A[4];
  float newX, newY;
  static double newPoint[5]; // initiation of list to be returned with pointer, therefor it must be static

  // line 23-47 goes through an if statement dependent on states. Each section changes the values of newX and newy based on the current position.
  if (state == 1)
  {
    newX = lengthX;
    newY = currentY;
    state++;
  }
  else if (state == 2)
  {
    newX = currentX;
    newY = currentY + c;
    state++;
  }
  else if (state == 3)
  {
    newX = currentX - lengthX;
    newY = currentY;
    state++;
  }
  else if (state == 4)
  {
    newX = currentX;
    newY = currentY + c;
    state = 1;
  }
  if (newY > widthY) // if statement checking if the newY value is outside the assigned area. if it is, the newY and newX values will be assigned at o and state will be assigned as 0.
  {
    newX = 0;
    newY = 0;
    state = 0;
  }

  // line 56-60- values in newPoint is updated with the new values determined in the if statements
  newPoint[0] = lengthX;
  newPoint[1] = widthY;
  newPoint[2] = newX;
  newPoint[3] = newY;
  newPoint[4] = state;

  return newPoint; // return statemnet, returning newPoint as pointer
}

int main(int argc, char **argv) // initation of main
{
  ros::init(argc, argv, "demining"); // initiation ROS

  MoveBaseClient ac("move_base", true); // using the previous typedef to initiate client from actionlib.h as "ac"

  while (!ac.waitForServer(ros::Duration(5.0))) // connecting ac to server with 5 second buffer
  {
    ROS_INFO("Waiting for the move_base action server to come up"); // printing statement to terminaæ
  }

  double A[5] = {}; // initiating array A with 5 elements

  ROS_INFO("Insert length in meters"); // print to teminal
  std::cin >> A[0]; // get input from user
  ROS_INFO("Insert width in meters"); // print to terminal 
  std::cin >> A[1]; // get input from user

  // lines 84-86 - initiating the last values in A
  A[2] = 0; 
  A[3] = 0;
  A[4] = 1;

  move_base_msgs::MoveBaseGoal goal; // creating intance of MoveBaseGoal named "goal"

  goal.target_pose.header.frame_id = "odom"; // initiating goal frame_id as "odom"
  goal.target_pose.header.stamp = ros::Time::now(); // initiation stamp as ros::Time::now()

  double *p; // initiating pointer p
  p = A; // changing p's values to equal A
  while (p[4] != 0) //while loop running while state in array p differs from 0
  {
    p = findNextGoal(p, 0.5); // call coordanate function and updating pointer p

    goal.target_pose.pose.position.x = p[2]; // assigning new target_x from pointer p 
    goal.target_pose.pose.position.y = p[3]; // assigning new target_y from pointer p
    goal.target_pose.pose.orientation.w = 1.0; // assign target orientation

    ac.sendGoal(goal); // send new target to server

    ac.waitForResult(); // wait for server to respond

    while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) // while loop running while goal is not reached
    {
    }
  }
  return 0;
}
