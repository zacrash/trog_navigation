#include <drive/drive.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "drive_node");
  tf::TransformListener tf(ros::Duration(10));

  drive::Drive drive(tf);

  ros::spin();
}
