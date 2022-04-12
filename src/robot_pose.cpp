#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include <sstream>

class RobotPose {
public:

  RobotPose() {
    // the parameter 'this' at the end is MANDATORY [ otherwise everything explodes :( ]
    this->sub_reader = this->n.subscribe("robot/pose", 1000, &RobotPose::robotPosePrint, this);
  }

  void robotPosePrint(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // To read data in msg i checked the documentation online http://wiki.ros.org/geometry_msgs
    // There you can find the definition of the PoseStamped type of message.
    // It is divided in two categories: 'header' and 'pose'.
    // Here I decided to print only the pose (for our project i think is the relevant one).
    // The 'pose' is composed like this:
    // position(x,y,z) and orientation(x,y,z,w)
    // to access values you need to do like down here


    ROS_INFO("pose:");
    ROS_INFO(" position:");
    ROS_INFO("\t x: [%lf]", msg->pose.position.x);
    ROS_INFO("\t y: [%lf]", msg->pose.position.y);
    ROS_INFO("\t z: [%lf]", msg->pose.position.z);
    ROS_INFO(" orientation:");
    ROS_INFO("\t x: [%lf]", msg->pose.orientation.x);
    ROS_INFO("\t y: [%lf]", msg->pose.orientation.y);
    ROS_INFO("\t z: [%lf]", msg->pose.orientation.z);
    ROS_INFO("\t z: [%lf]", msg->pose.orientation.w);
  }

  void run() {
    ros::spin();
  }

private:
  ros::NodeHandle n;
  ros::Subscriber sub_reader;

};

int main(int argc, char **argv) {
  // I don't know if it's relevant by i tend to use the name also in the 'type' category of the launch file
  ros::init(argc, argv, "pose_reader");

  RobotPose read_pose;
  read_pose.run();

  return 0;

}
