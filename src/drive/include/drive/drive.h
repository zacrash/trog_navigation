#ifndef NAV_DRIVE_ACTION_H
#define NAV_DRIVE_ACTION_H

#include <vector>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/simple_action_server.h>
#include <action/DriveAction.h>


namespace drive {

  typedef actionlib::SimpleActionServer<drive::DriveAction> ActionServer;
  
  enum DriveState {
    WAITING,
    PLANNING,
    CONTROLLING,
    STOPPING
  };

  class Drive {
  public:
    Drive(tf::TransformListener& tf);
    virtual ~Drive();
    bool run(geometry_msgs::PoseStamped& goal);
    
  private:
    /* Publish a zero velocity immediately */
    void emergencyStop();

    void resetState();

    ActionServer* as_;
    tf::TransformListener& tf_;
    tf::Stamped<tf::Pose> local_pose_;
    double planner_frequency_;
    double controller_frequency_;
    ros::Publisher velocity_publisher_;
    ros::Subscriber goal_subscriber_;
    ros::Subscriber agent_subscriber_;
    DriveState state_;
    bool done_;
    geometry_msgs::Twist action_;
    double action_timestamp_;
    bool took_action_;

  };
};
#endif
