#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "test_pkg/wheels_rpm.h"

#include <sstream>

#define T 5
#define N 42
#define R 0.07
#define L 0.2
#define W 0.169

class ComputeRPM {
public:

  ComputeRPM() {
    this->sub_reader = this->n.subscribe("cmd_vel", 1000, &ComputeRPM::computeRPM, this);
    this->pub_rpm = this->n.advertise<test_pkg::wheels_rpm>("wheels_rpm", 1000);
  }

  void computeRPM(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    float w_bz = msg->twist.angular.z;
    float v_bx = msg->twist.linear.x;
    float v_by = msg->twist.linear.y;

    this->w_fl = ((-L-W)*w_bz + v_bx - v_by)/R;
    this->w_fr = ((L+W)*w_bz + v_bx + v_by)/R;
    this->w_rl = ((L+W)*w_bz + v_bx - v_by)/R;
    this->w_rr = ((-L-W)*w_bz + v_bx + v_by)/R;

    test_pkg::wheels_rpm send;
    send.header.stamp = ros::Time::now();
    send.header.frame_id = "wheels rpm";

    send.rpm_fl = 60 * w_fl * T;
    send.rpm_fr = 60 * w_fr * T;
    send.rpm_rl = 60 * w_rl * T;
    send.rpm_rr = 60 * w_rr * T;

    this->pub_rpm.publish(send);
  }

  void run() {
    ros::spin();
  }

private:
  ros::NodeHandle n;
  ros::Subscriber sub_reader;
  ros::Publisher pub_rpm;
  float w_fl = 0;
  float w_fr = 0;
  float w_rl = 0;
  float w_rr = 0;

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "velocity_reader");

  ComputeRPM read_velocity;
  read_velocity.run();

  return 0;

}
