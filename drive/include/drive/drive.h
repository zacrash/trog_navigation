#include <vector>

#include <ros.h>
#include <geometry_msgs/PoseStamped.h>


namespace drive {
  
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
