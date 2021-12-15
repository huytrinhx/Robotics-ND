#include <ros/ros.h>
#include <sstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include <functional>
#include "add_markers/AddMarkers.h"
#include "geometry_msgs/Pose.h"



// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Define Listener class
class Listener {
  private:
    MoveBaseClient *_ac;
    ros::ServiceClient *_client;
    geometry_msgs::Pose _goal_pose;
    geometry_msgs::Pose _pickup;
    geometry_msgs::Pose _dropoff;
    float _tolerance;
    int _robot_state;

  public:
    //Define initialization and default values
    Listener(MoveBaseClient *ac, ros::ServiceClient *sc,
      geometry_msgs::Pose pickup_pose, geometry_msgs::Pose dropoff_pose)
      : _ac(ac), _client(sc), _robot_state(0), _tolerance(0.8),
        _pickup(pickup_pose), _dropoff(dropoff_pose) {};

    //Define move function
    void MoveTowardGoal() {
      move_base_msgs::MoveBaseGoal goal;
      // set up the frame parameters
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      // Define a position and orientation for the robot to reach
      goal.target_pose.pose = this->_goal_pose;

      // Send the goal position and orientation for the robot to reach
      ROS_INFO("Sending goal X: %.2f, Y: %.2f , W: %.2f",
      _goal_pose.position.x,
      _goal_pose.position.y,
      _goal_pose.orientation.w);
      _ac->sendGoal(goal);

      // Wait an infinite time for the results
      _ac->waitForResult();

      // Check if the robot reached its goal
      if(_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, the base has got to the goal position");
      } else {
        ROS_INFO("The base failed to move to the goal for some reason");
      }
    }

    //Define function that make a call to add_markers server
    void requestAddMarkers(const char *req) {
      add_markers::AddMarkers srv;
      srv.request.str_request = req;
      if (_client->call(srv)) {
        return;
      } else {
        ROS_ERROR("Failed to call add_markers service");
        return;
      }
    }

    //Define sequence of actions for home service bot
    void pickObjectsCallback(const nav_msgs::Odometry::ConstPtr &msg) {

      if (this->_robot_state != 4) {
      ROS_INFO("Current robot pose: X:  %.2f; Y: %.2f",msg->pose.pose.position.x, msg->pose.pose.position.y);
      ROS_INFO("Current goal pose: X:  %.2f; Y: %.2f",this->_goal_pose.position.x, this->_goal_pose.position.y);
      }

      switch (this->_robot_state) {
        case 0: //begin, set goal to pick up zone and moving toward that goal
          ROS_INFO("Heading to pick up zone");
          this->requestAddMarkers("default"); //tell the marker object to display the pickup marker
          _goal_pose = _pickup;
          MoveTowardGoal();
          this->_robot_state = 1;
          break;
        case 1: //arrive at pick up zone
          ROS_INFO("Arrive at pick up zone. Start picking up");
          this->requestAddMarkers("pickup"); //tell the marker object to hide the pickup marker
          if (fabs(msg->pose.pose.position.x - this->_goal_pose.position.x) < this->_tolerance &&
              fabs(msg->pose.pose.position.y - this->_goal_pose.position.y) < this->_tolerance) {
                this->_robot_state = 2;
                ros::Duration(5).sleep();
              }
          break;
        case 2: //on the way to drop off zone
          ROS_INFO("Heading to drop off zone");
          _goal_pose = _dropoff;
          this->requestAddMarkers("dropoff"); //tell the marker object to show the dropoff marker
          MoveTowardGoal(); 
          if (fabs(msg->pose.pose.position.x - this->_goal_pose.position.x) < this->_tolerance &&
              fabs(msg->pose.pose.position.y - this->_goal_pose.position.y) < this->_tolerance) {
                this->_robot_state = 3;
              }
          break;
        case 3: //arrive drop off zone and done
          ROS_INFO("Finish the job!!!");
          this->requestAddMarkers("default"); //tell the marker object to display the pickup marker
          this->_robot_state = 4;
          break;
        default:
          break;
      }

      return;

    }
};


int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //Communicate with add_marker note through client-server service architecture
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<add_markers::AddMarkers>("add_markers");

  //Define pickup_pose and dropoff_pose (hard-coded for now)
  //TODO: Finding a way to get it from add_markers config
  geometry_msgs::Pose pickup_pose, dropoff_pose;
  pickup_pose.position.x = -10.0;
  pickup_pose.position.y = 1.0;
  pickup_pose.orientation.w = 1.0;

  dropoff_pose.position.x = -10.0;
  dropoff_pose.position.y = -4.0;
  dropoff_pose.orientation.w = 1.0;

  // Send these info to listener object (which is defined above)
  Listener listener(&ac, &client, pickup_pose, dropoff_pose);

  //Subscribe to odom and with the call back function from listener class
  ros::Subscriber sub = n.subscribe("odom",1000, &Listener::pickObjectsCallback, &listener);

  //Handle ROS communicaion events
  ros::spin();

  return 0;
}


