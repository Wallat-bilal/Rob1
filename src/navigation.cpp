// includes - reading of all files needed
#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <math.h>

class Nav // Navigavional class that defines the required search area variables. And defines a function for calculation of goal points within the search area.
{
private:
  double area_length, area_width, goal_x, goal_y, line_width, start_x, start_y; // Variable initialisation
  int state = 1;                            // The state of the turtle is initiated with a value of 1

public:
  Nav(double x, double y);                  // 2 arg constructor
  void calc_new_goal(double x, double y);   // Function 
  double get_x() { return goal_x; }         // Getter/accessor 
  double get_y() { return goal_y; }         // Getter/accessor
  double get_state() { return state; }      // Getter/accessor
};

Nav::Nav(double x, double y)                // The 2 arg constructor is set to ask the user for input to set the dimensions of the area, and distance between the lines. And to input the argmunets as start_x and y.
{
  start_x = x;
  start_y = y;
  std::cout << "Insert length in meters: ";
  std::cin >> area_length;
  std::cout << "Insert width in meters: ";
  std::cin >> area_width;
  std::cout << "Insert line width in the search area in metres: ";
  std::cin >> line_width;
}

void Nav::calc_new_goal(double x, double y) // Void function that updates the current location, the new goal point and checks if goal is within area size.
{
  double current_x = x;      // Current_x stores the x-location of the robot. This is updated each time the calc_new_goal is called.
  double current_y = y;      // Current_y stores the y-location of the robot. This is updated each time the calc_new_goal is called.
    switch (state){            // Switch function that updates the goal location depending on the state.
    case 1:
    {
      goal_x = start_x + area_length;
      goal_y = current_y;
      state++;
    }
    break;
    case 2:
    {
      goal_x = start_x + area_length;
      goal_y = current_y + line_width;
      state++;
    }
    break;
    case 3:
    {
      goal_x = start_x;
      goal_y = current_y;
      state++;
    }
    break;
    case 4:
    {
      goal_x = start_x;
      goal_y = current_y + line_width;
      state = 1;
    }
    default:
    {
    }
    break;
  }
  if (goal_y > area_width){  // If-statement that checks to see if the switch put the goal outside the area
    goal_x = start_x;
    goal_y = start_y;
    state = 0;
  }
}

class Turtle // Turtle class that defines the position- and velocity- variables for the turtle. Also defines functions for movement and postition updates.
{
  private:
  double distance_tolerance, lin_speed_multi, ang_vel_multi, t_pose_x, t_pose_y, t_pose_theta; // Variable initialisation. 

  public:
  ros::NodeHandle n; // Creates instance of a ROS nodehandler
  ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100); // Creates instance of a ROS publisher, that publishes velocity commands to the "turtle1".
  ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 10, &Turtle::pose_callback, this);       // Creates instance of a ROS subscriber, that subscribes to the "turtle1" position.
  Turtle();                                                                                       // No arg constructor                                            
  void move_to_goal(double goal_x, double goal_y);                                                  // Function
  double get_distance(double x1, double x2, double y1, double y2);                                 // Function
  void pose_callback(const turtlesim::Pose::ConstPtr& message);                                    // Function
  double get_t_pose_x() {return t_pose_x;}                                                           // Getter/accessor
  double get_t_pose_y() {return t_pose_y;}                                                           // Getter/accessor
};

Turtle::Turtle() // The no arg constructor is set to ask the user for inputs about the distance tolerance and velocity multiplyer.
  {
    std::cout << "Insert distance tolerance in meters: ";
    std::cin >> distance_tolerance;
    std::cout << "Insert line speed multiplyer: ";
    std::cin >> lin_speed_multi;
    std::cout << "Insert angular velocity multiplyer: ";
    std::cin >> ang_vel_multi;
  }

double Turtle::get_distance(double x1, double y1, double x2, double y2) // Function that calculates and returns the euclidian distance between two points.
{       
  return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

void Turtle::move_to_goal(double goal_x, double goal_y) // Function that calculates the velocity required to move to the goal point, who are taken as input parameters. In other words this is called a proportional controller or simply a P-controller.
{      
  geometry_msgs::Twist vel_msg;                                                         // Creates instance of geometry::Twist, this is used to store velocity information. 
  ros::Rate loop_rate(10);                                                              // The loop rate is set to 10 Hz. 
  while(get_distance(t_pose_x, t_pose_y, goal_x, goal_y)>distance_tolerance){                // While loop that publishes the vel_msg while the turtle is further away from the goal than the tolerated distance.
    // Linear velocities
    vel_msg.linear.x = lin_speed_multi*get_distance(t_pose_x, t_pose_y, goal_x, goal_y);     // Assigns velocity to linear x based on the euclidian distance formula.
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    // Angular velocities
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    double ang_diff = atan2(goal_y-t_pose_y, goal_x-t_pose_x)-t_pose_theta; // Function that calculates the angular difference between goal point and turtle position.
    if (ang_diff > M_PI){                // If statement that checks if the angular difference is greater than pi
      ang_diff = ang_diff - (2*M_PI);    // If condition is true we then subtract 2 pi from the difference
    }
    else if(ang_diff < (-M_PI)){         // If statement that checks if the angular difference is lower than negative pi
      ang_diff = ang_diff + (2*M_PI);    // If condition is true we then add 2 pi from the difference
    }
    vel_msg.angular.z = ang_vel_multi*ang_diff; // Assigns velocity to angular z based on the steering formula 
    
    velocity_publisher.publish(vel_msg); // The ROS pubisher publishes the vel_msg

    ros::spinOnce();                     // Spinner that runs through a single round of callback functions.
  }
    // Ending the movement of the Turtle
    vel_msg.linear.y = 0;
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    
    velocity_publisher.publish(vel_msg); // The ROS pubisher publishes the vel_msg
}
 void Turtle::pose_callback(const turtlesim::Pose::ConstPtr& message){ // Function that updates the turtle position when called.
   t_pose_x = message->x;
   t_pose_y = message->y;
   t_pose_theta = message->theta;
  }

int main(int argc, char **argv)          // Initation of main   
{
  ros::init(argc, argv, "navigation");   // Initiation ROS
  double start_x(1), start_y(1);           // Initiation of starting coordinates

  Turtle controller;                     // Creates instance "controller" of type "Turtle"

  ros::spinOnce();                       // A round of callback is performed, this required for the next function to know position of the turtle.
  controller.move_to_goal(start_x, start_y); // Movement to starting coordinates

  Nav nav(start_x, start_y);               // Creates instance "nav" of type "Nav". 
  while (nav.get_state() != 0)           // Main while loop. The condition is only true, when the turtle have been at all goal points within the box.
  {
    ros::spinOnce();                                                   // A round of callback is performed. This updates the turtle position.
    nav.calc_new_goal(controller.get_t_pose_x(), controller.get_t_pose_y()); // Calculates the new goal point within the box, based on turtles position that was just updated
    controller.move_to_goal(nav.get_x(), nav.get_y());                   // Moves the turtle to calculated goal.
  }
  return 0;
}
