#include "xsens_driver/hri_xsens_wrapper.h"
#include <time.h>
#include <ctime>
#include "honda_libs/honda_lib.h"

HriXSensWrapper::HriXSensWrapper(ros::NodeHandle &nh) : nh_m(nh) {
  last_fix_time_m = 0;
  initParams();
  initPublishers();
  initSubscribers();
}

void HriXSensWrapper::initParams() {
  ros::param::param<std::string>("~gps_output_topic", params_m.gps_output_topic,
                                 "/xsens_gps_reading");
  ros::param::param<std::string>("~xsens_ns", params_m.xsens_ns, "/xsens");
}

void HriXSensWrapper::initPublishers() {
  pub_xsens_gps_m =
      nh_m.advertise<honda_msgs::GpsReading>(params_m.gps_output_topic, 1000);
}

void HriXSensWrapper::initSubscribers() {
  sub_velocity_m = std::make_shared<
      message_filters::Subscriber<geometry_msgs::TwistStamped>>(
      nh_m, params_m.xsens_ns + "/velocity", 1);

  sub_fix_m =
      std::make_shared<message_filters::Subscriber<sensor_msgs::NavSatFix>>(
          nh_m, params_m.xsens_ns + "/fix", 1);

  sub_imu_m = std::make_shared<message_filters::Subscriber<sensor_msgs::Imu>>(
      nh_m, params_m.xsens_ns + "/imu", 1);

  message_synchronizer_m = std::make_shared<Synchronizer<MessageSyncPolicy>>(
      MessageSyncPolicy(10), *sub_velocity_m, *sub_fix_m, *sub_imu_m);

  message_synchronizer_m->registerCallback(
      boost::bind(&HriXSensWrapper::callback, this, _1, _2, _3));
}

void HriXSensWrapper::callback(
    const geometry_msgs::TwistStampedConstPtr &velocity_msg,
    const sensor_msgs::NavSatFixConstPtr &fix_msg,
    const sensor_msgs::ImuConstPtr &imu_msg) {
  if (fix_msg->header.stamp.toSec() - last_fix_time_m > 0.01) {
    last_fix_time_m = fix_msg->header.stamp.toSec();

    had::toEulerianAngle(imu_msg->orientation, gps_reading_m.rpy.roll,
                         gps_reading_m.rpy.pitch, gps_reading_m.rpy.yaw);
    gps_reading_m.fix = *fix_msg;
    gps_reading_m.vel.linear = velocity_msg->twist.linear;
    gps_reading_m.vel.angular = imu_msg->angular_velocity;
    gps_reading_m.accel.linear = imu_msg->linear_acceleration;

    gps_reading_m.header.stamp = ros::Time::now();
    gps_reading_m.header.frame_id = "xsens";

    pub_xsens_gps_m.publish(gps_reading_m);
  }
}
