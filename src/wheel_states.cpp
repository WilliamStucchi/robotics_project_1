#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"

#include <sstream>

#define T 5
#define N 42
#define R 0.07
#define L 0.2
#define W 0.169

class WheelState {
public:

  WheelState() {
    // the parameter 'this' at the end is MANDATORY [ otherwise everything explodes :( ]
    this->sub_reader = this->n.subscribe("wheel_states", 1000, &WheelState::wheelStatePrint, this);
    this->pub = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
  }

  void wheelStatePrint(const sensor_msgs::JointState::ConstPtr &msg) {
    // To read data in msg i checked the documentation online http://wiki.ros.org/sensor_msgs
    // There you can find the definition of the JointState type of message.
    // It is divided in four categories: 'name', 'position', 'velocity' and 'effort'.
    // Here I decided to print only the position and velocity (for our project i think they are the relevant ones).
    // These categories are arrays so the content is accessed like down here.

    float w_fl = (((msg->velocity[0])/60)*2*3.14)/T;
    float w_fr = (((msg->velocity[1])/60)*2*3.14)/T;
    float w_rl = (((msg->velocity[2])/60)*2*3.14)/T;
    float w_rr = (((msg->velocity[3])/60)*2*3.14)/T;

    float lv_fl = w_fl*R;
    float lv_fr = w_fr*R;
    float lv_rl = w_rl*R;
    float lv_rr = w_rr*R;

    // from the book in the slides (page 541 reversed)
    float w_bz = (R * (w_rr - w_fl))/(2 * (L+W));
    float v_bx = 0.5 * R * (w_fr + w_fl);
    float v_by = (-W-L) * w_bz + v_bx - R * w_fl;

    ROS_INFO("Wheel_state:");
    ROS_INFO(" fl: {p:[%lf],v:[%lf]->[%lf]\nlv:[%lf]}", msg->position[0], msg->velocity[0], w_fl, lv_fl);
    ROS_INFO(" fr: {p:[%lf],v:[%lf]->[%lf]\nlv:[%lf]}", msg->position[1], msg->velocity[1], w_fr, lv_fr);
    ROS_INFO(" rl: {p:[%lf],v:[%lf]->[%lf]\nlv:[%lf]}", msg->position[2], msg->velocity[2], w_rl, lv_rl);
    ROS_INFO(" rr: {p:[%lf],v:[%lf]->[%lf]\nlv:[%lf]}", msg->position[3], msg->velocity[3], w_rr, lv_rr);
    ROS_INFO(" w_bz: [%lf]", w_bz);
    ROS_INFO(" v_bx: [%lf]", v_bx);
    ROS_INFO(" v_by: [%lf]", v_by);

    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.twist.linear.x = v_bx;
    cmd_vel.twist.linear.y = v_by;
    cmd_vel.twist.linear.z = 0;
    cmd_vel.twist.angular.x = 0;
    cmd_vel.twist.angular.y = 0;
    cmd_vel.twist.angular.z = w_bz;

    pub.publish(cmd_vel);
  }

  void run() {
    ros::spin();
  }

private:
  ros::NodeHandle n;
  ros::Subscriber sub_reader;
  ros::Publisher pub;

};

int main(int argc, char **argv) {
  // I don't know if it's relevant by i tend to use the name also in the 'type' category of the launch file
  ros::init(argc, argv, "wheel_state_reader");

  WheelState wheel_state;
  wheel_state.run();

  return 0;

}
