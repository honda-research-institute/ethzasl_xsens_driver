#ifndef HRI_XSENS_WRAPPER_H
#define HRI_XSENS_WRAPPER_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include "geometry_msgs/TwistStamped.h"
#include "honda_msgs/GpsReading.h"
#include "memory.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

using namespace message_filters;

class HriXSensWrapper {
 public:
  struct Params {
    std::string gps_output_topic;
    std::string xsens_ns;
  };

  typedef sync_policies::ApproximateTime<
      geometry_msgs::TwistStamped, sensor_msgs::NavSatFix, sensor_msgs::Imu>
      MessageSyncPolicy;

 private:
  double last_fix_time_m;
  honda_msgs::GpsReading gps_reading_m;
  Params params_m;
  ros::NodeHandle nh_m;
  ros::Publisher pub_xsens_gps_m;
  std::shared_ptr<message_filters::Subscriber<geometry_msgs::TwistStamped>>
      sub_velocity_m;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::NavSatFix>>
      sub_fix_m;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::Imu>> sub_imu_m;
  std::shared_ptr<Synchronizer<MessageSyncPolicy>> message_synchronizer_m;

 public:
  HriXSensWrapper(ros::NodeHandle& nh);

 private:
  void initParams();
  void initPublishers();
  void initSubscribers();
  void callback(const geometry_msgs::TwistStampedConstPtr& velocity_msg,
                const sensor_msgs::NavSatFixConstPtr& fix_msg,
                const sensor_msgs::ImuConstPtr& imu_msg);
};

#endif  // HRI_XSENS_WRAPPER_H
