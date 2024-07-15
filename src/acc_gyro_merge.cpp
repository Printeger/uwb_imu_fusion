#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <deque>
#include <mutex>
#include <sensor_msgs/Imu.h>

using namespace std;

std::deque<sensor_msgs::Imu> gryo_buffer;
ros::Publisher pub_imu;

mutex vicon_mtx, uwb_mtx;

void SubscribeAndPublish()
{
}

void gyro_callback(const sensor_msgs::Imu &msg)
{
  unique_lock<mutex> lock(vicon_mtx);
  gryo_buffer.push_back(msg);
}

void acc_callback(const sensor_msgs::Imu &input)
{
  unique_lock<mutex> lock(vicon_mtx);

  sensor_msgs::Imu output, tmp_in;
  double min_duration = 1e18; // initialize with a large value
  double duration = 0.0;
  // Find the vicon pose with the closest timestamp
  sensor_msgs::Imu closest_pose;
  auto iter = gryo_buffer.begin();
  if (!gryo_buffer.empty())
  {
    closest_pose = gryo_buffer.begin();
    // min_duration = std::abs(input.header.stamp.toSec() - closest_pose->header.stamp.toSec());
    for (auto it = gryo_buffer.begin(); it != gryo_buffer.end(); ++it)
    {
      duration = std::abs(input.header.stamp.toSec() - it->header.stamp.toSec());
      if (duration < min_duration)
      {
        min_duration = duration;
        closest_pose = *it;
        iter = it;
      }
    }
    gryo_buffer.erase(iter);
  }
  else
  {
    ROS_WARN("vicon buffer is empty");
  }

  lock.unlock();

  output = input;
  output.angular_velocity.x = closest_pose.angular_velocity.x;
  output.angular_velocity.y = closest_pose.angular_velocity.y;
  output.angular_velocity.z = closest_pose.angular_velocity.z;
  pub_imu.publish(output);
}

int main(int argc, char **argv)
{
  ros::NodeHandle n_;
  ros::Subscriber sub_acc;
  ros::Subscriber sub_gyro;

  ros::init(argc, argv, "subscribe_and_publish");

  sub_acc = n_.subscribe("/camera/accel/sample", 1000, acc_callback);
  sub_gyro = n_.subscribe("/camera/gyro/sample", 1000, gyro_callback);
  pub_imu = n_.advertise<sensor_msgs::Imu>("/camera_imu", 1000);

  ros::spin();
  return 0;
}