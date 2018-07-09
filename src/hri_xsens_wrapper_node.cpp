#include "xsens_driver/hri_xsens_wrapper.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "xsens_wrapper_node");

  ros::NodeHandle nh;
  HriXSensWrapper xsens_wrapper(nh);

  ros::spin();

  return 0;
}
