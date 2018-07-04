/* Based very loosely on:
https://github.com/ros-planning/navigation/blob/melodic-devel/move_base/src/move_base.cpp
*/
#include <drive/drive.h>

#include <geometry_msgs/Twist.h>


namespace drive {
  
  Drive::Drive(tf::TransformListener& tf):
    tf_(tf), as_(NULL) {

    as_ = new ActionServer(ros::NodeHandle(), "drive", boost::bind(&Drive::run, this, _1), false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("action_time_tolerance", action_time_tolerance, 1.0);

    state_ = WAITING;

    velocity_publisher_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    agent_subscriber_ = nh.subscribe("actions", 1, agentCallback);

    done_ = false;

    as_->start();
  }

  Drive::~Drive() {
    if(as_ != NULL)
      delete as_;
  }

  /* Main logic - state machine that reads actions from RL agent.
     This is purposefully set up to be easily multithreaded later for
     performance gains so we aren't as bottlenecked in Python-land */
  void Drive::run(const drive::DriveGoalConstPtr& drive_goal) {
    ros::NodeHandle n;
    while(n.ok()) {
      
      if(as_->isPreemptRequested()) {
	as_->setAborted();
	return;
      }
      
      geometry_msgs::Twist cmd_vel;

      switch(state_) {

      case WAITING:
	ROS_DEBUG_NAMED("drive", "Waiting on an action...");
	ros::Rate r(planner_frequency_);
	r.sleep()
	  break;
      
      case PLANNING:
	{
	  cmd_vel = action_;
	  now = ros::Time::now().toSec();
	  timestamp = action_timestamp_;
	  done = took_action_;
	  if(!done && timestamp + action_time_tolerance >= now)
	    state_ = CONTROLLING;
	  else {
	    state_ = WAITING;
	    ROS_DEBUG_NAMED("drive", "Action rejected with done = %d timestamp = %d", done, timestamp);
	  }
	}
	ROS_DEBUG_NAMED("drive", "In planning state");
	break;

      case CONTROLLING:
	ROS_DEBUG_NAMED("drive", "In controlling state");
	{
	  if(done_) {
	    ROS_DEBUG_NAMED("drive", "Goal reached!");
	    resetState();
	    as_->setSucceeded();
	  }
	  velocity_publisher_.publish(cmd_vel);
	  ros::Rate r(controller_frequency_);
	  r.sleep();
	}
	break;

      case STOPPING:
	ROS_DEBUG_NAMED("drive", "In Stopping state");
	break;
      
      default:
	ROS_ERROR("Error in state machine. Reached bad state");
	resetState();
	return;
      }

    }

    as_->setAborted();
    return;
  }

  /* Listen for actions into a buffer of size one */
  void Drive::agentCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    action_timestamp_ = ros::Time::now().toSec();
    action_ = msg->data;
    took_action_ = false;
    state_ = WAITING;
    ROS_DEBUG_NAMED("drive", "Received agent action %s", msg->data.c_str());
  }

  /* Stop! */ 
  void Drive::emergencyStop() {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    velocity_publisher_.publish(cmd_vel);
  }

  void Drive::resetState() {
    state_ = WAITING;
    emergencyStop();
  }
	
