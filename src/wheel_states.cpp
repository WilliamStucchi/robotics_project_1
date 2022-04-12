#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <sstream>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

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
    this->sub_pose = this->n.subscribe("robot/pose", 1000, &WheelState::robotPoseCheck, this);
    this->pub_velocity = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1000);
    this->pub_odometry = this->n.advertise<nav_msgs::Odometry>("odom", 1000);
    this->first_read = 0;
  }

  void publishVelocity() {
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.stamp = ros::Time::now();
    cmd_vel.header.frame_id = "world";
    cmd_vel.twist.linear.x = this->v_bx_ticks;
    cmd_vel.twist.linear.y = this->v_by_ticks;
    cmd_vel.twist.linear.z = 0;
    cmd_vel.twist.angular.x = 0;
    cmd_vel.twist.angular.y = 0;
    cmd_vel.twist.angular.z = this->w_bz_ticks;

    this->pub_velocity.publish(cmd_vel);
  }

  void robotPoseCheck(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    float x_diff_eu = this->curr_x_eu - msg->pose.position.x;
    float y_diff_eu = this->curr_y_eu - msg->pose.position.y;

    float x_diff_ru = this->curr_x_ru - msg->pose.position.x;
    float y_diff_ru = this->curr_y_ru - msg->pose.position.y;

    ROS_INFO("[%d]\n\tcurrX:[%lf]\n\t-eulerX:[%lf]\n\t-rungeX[%lf]", this->seq_number, this->curr_x_ru, x_diff_eu, x_diff_ru);
    ROS_INFO("[%d]\n\tcurrY:[%lf]\n\t-eluerY:[%lf]\n\t-rungeY[%lf]", this->seq_number, this->curr_y_ru, y_diff_eu, y_diff_ru);
    ROS_INFO("[%d]\n\tTH: [%lf]", this->seq_number, this->curr_theta);
    //ROS_INFO("[%d]currX: [%lf]\n", this->seq_number, this->curr_x_ru);
    //ROS_INFO("[%d]currY: [%lf]\n", this->seq_number, this->curr_y_ru);

    this->seq_number++;

    geometry_msgs::TransformStamped transformStamped;
    tf2::Quaternion q;
    q.setRPY(0, 0, this->curr_theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.y();
    transformStamped.transform.rotation.w = q.w();
/*
    ROS_INFO("[%d]x: [%lf]", this->seq_number, transformStamped.transform.rotation.x);
    ROS_INFO("[%d]y: [%lf]", this->seq_number, transformStamped.transform.rotation.y);
    ROS_INFO("[%d]z: [%lf]", this->seq_number, transformStamped.transform.rotation.z);
    ROS_INFO("[%d]w: [%lf]", this->seq_number, transformStamped.transform.rotation.w);
*/

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "wolrd";
    odom.child_frame_id = "robot";
    odom.pose.pose.position.x = this->curr_x_ru;
    odom.pose.pose.position.y = this->curr_y_ru;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = this->v_bx_ticks;
    odom.twist.twist.linear.y = this->v_by_ticks;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = this->w_bz_ticks;

    this->pub_odometry.publish(odom);
  }



  void wheelStatePrint(const sensor_msgs::JointState::ConstPtr &msg) {

    // ------------------- V & W
    // To read data in msg i checked the documentation online http://wiki.ros.org/sensor_msgs
    // There you can find the definition of the JointState type of message.
    // It is divided in four categories: 'name', 'position', 'velocity' and 'effort'.
    // Here I decided to print only the position and velocity (for our project i think they are the relevant ones).
    // These categories are arrays so the content is accessed like down here.

    int curr_ticks_fl = 0;
    int curr_ticks_fr = 0;
    int curr_ticks_rl = 0;
    int curr_ticks_rr = 0;
    int delta_ticks_fl = 0;
    int delta_ticks_fr = 0;
    int delta_ticks_rl = 0;
    int delta_ticks_rr = 0;

    int curr_sec = 0;
    int curr_nsec = 0;
    //int delta_sec = 0;
    float delta_nsec = 0;
    float delta_nsec_norm;

    if(first_read == 0) {
      this->curr_x_eu = 0;
      this->curr_y_eu = 0;
      this->curr_x_ru = 0;
      this->curr_y_ru = 0;

      this->prev_ticks_fl = msg->position[0];
      this->prev_ticks_fr = msg->position[1];
      this->prev_ticks_rl = msg->position[2];
      this->prev_ticks_rr = msg->position[3];
      this->prev_sec = msg->header.stamp.sec;
      this->prev_nsec = 1000000000*this->prev_sec + msg->header.stamp.nsec;
      this->first_read = 1;
    } else {
      curr_ticks_fl = msg->position[0];
      curr_ticks_fr = msg->position[1];
      curr_ticks_rl = msg->position[2];
      curr_ticks_rr = msg->position[3];

      delta_ticks_fl = curr_ticks_fl - this->prev_ticks_fl;
      delta_ticks_fr = curr_ticks_fr - this->prev_ticks_fr;
      delta_ticks_rl = curr_ticks_rl - this->prev_ticks_rl;
      delta_ticks_rr = curr_ticks_rr - this->prev_ticks_rr;

/*
      ROS_INFO("FL: %d", delta_ticks_fl);
      ROS_INFO("FR: %d", delta_ticks_fr);
      ROS_INFO("RL: %d", delta_ticks_rl);
      ROS_INFO("RR: %d", delta_ticks_rr);
*/
      this->prev_ticks_fl = curr_ticks_fl;
      this->prev_ticks_fr = curr_ticks_fr;
      this->prev_ticks_rl = curr_ticks_rl;
      this->prev_ticks_rr = curr_ticks_rr;

      curr_sec = msg->header.stamp.sec;
      curr_nsec = 1000000000*curr_sec + msg->header.stamp.nsec;
      //delta_sec = curr_sec - this->prev_sec;
      delta_nsec = curr_nsec - this->prev_nsec;

      //ROS_INFO("sec: %d", delta_sec);
      //ROS_INFO("nsec: %d", delta_nsec);

      this->prev_sec = curr_sec;
      this->prev_nsec = curr_nsec;


      delta_nsec_norm = (0.000000001*delta_nsec);


      //ROS_INFO("scala: %lf", ((delta_ticks_fl/(0.000000001*delta_nsec))/(N*T)));

      // TICKS
      float w_fl_ticks = ((delta_ticks_fl/delta_nsec_norm)/(N*T))*2*3.14;
      float w_fr_ticks = ((delta_ticks_fr/delta_nsec_norm)/(N*T))*2*3.14;
      float w_rl_ticks = ((delta_ticks_rl/delta_nsec_norm)/(N*T))*2*3.14;
      float w_rr_ticks = ((delta_ticks_rr/delta_nsec_norm)/(N*T))*2*3.14;

      this->w_bz_ticks = (R * (w_rr_ticks - w_fl_ticks))/(2 * (L+W));
      this->v_bx_ticks = 0.5 * R * (w_fr_ticks + w_fl_ticks);
      this->v_by_ticks = (-W-L) * w_bz_ticks + v_bx_ticks - R * w_fl_ticks;

/*
    // RPM

      //somebody says this formulas are without 2*pi (because of wrong data definition in bags)
      float w_fl_rpm = (((msg->velocity[0])/60)*2*3.14)/T;
      float w_fr_rpm = (((msg->velocity[1])/60)*2*3.14)/T;
      float w_rl_rpm = (((msg->velocity[2])/60)*2*3.14)/T;
      float w_rr_rpm = (((msg->velocity[3])/60)*2*3.14)/T;

      float lv_fl_rpm = w_fl_rpm*R;
      float lv_fr_rpm = w_fr_rpm*R;
      float lv_rl_rpm = w_rl_rpm*R;
      float lv_rr_rpm = w_rr_rpm*R;

      // from the book in the slides (page 541 reversed)
      float w_bz_rpm = (R * (w_rr_rpm - w_fl_rpm))/(2 * (L+W));
      float v_bx_rpm = 0.5 * R * (w_fr_rpm + w_fl_rpm);
      float v_by_rpm = (-W-L) * w_bz_rpm + v_bx_rpm - R * w_fl_rpm;
*/
  /*
      ROS_INFO("Wheel_state:");
      ROS_INFO(" fl: {p:[%lf],v:[%lf]->[%lf]\nticks:[%lf]}", msg->position[0], msg->velocity[0], w_fl_ticks, w_fl_ticks);
      ROS_INFO(" fr: {p:[%lf],v:[%lf]->[%lf]\nticks:[%lf]}", msg->position[1], msg->velocity[1], w_fr_ticks, w_fr_ticks);
      ROS_INFO(" rl: {p:[%lf],v:[%lf]->[%lf]\nticks:[%lf]}", msg->position[2], msg->velocity[2], w_rl_ticks, w_rl_ticks);
      ROS_INFO(" rr: {p:[%lf],v:[%lf]->[%lf]\nticks:[%lf]}", msg->position[3], msg->velocity[3], w_rr_ticks, w_rr_ticks);
      ROS_INFO(" w_bz: [%lf]", w_bz_ticks);
      ROS_INFO(" v_bx: [%lf]", v_bx_ticks);
      ROS_INFO(" v_by: [%lf]", v_by_ticks);
  */

      publishVelocity();


      // ------------------- Odometry
      // add service for initial position
      // add parameter for integeration method

      float robot_velocity = sqrt(pow(this->v_bx_ticks, 2) + pow(this->v_by_ticks, 2));
      float cos_x = cos(this->curr_theta + ((this->w_bz_ticks*delta_nsec_norm)/2.0));
      float sin_x = sin(this->curr_theta + ((this->w_bz_ticks*delta_nsec_norm)/2.0));
/*
      ROS_INFO("\n[%d]velocity: [%lf]\n", this->seq_number, robot_velocity);
      ROS_INFO("[%d]cos: [%lf]\n", this->seq_number, cos_x);
      ROS_INFO("[%d]sin: [%lf]\n", this->seq_number, sin_x);
      ROS_INFO("[%d]delta: [%lf]\n", this->seq_number, delta_nsec_norm);
*/
      float next_x_eu = this->curr_x_eu + this->v_bx_ticks*delta_nsec_norm;
      float next_y_eu = this->curr_y_eu + this->v_by_ticks*delta_nsec_norm;

      float next_x_ru = this->curr_x_ru + (robot_velocity * delta_nsec_norm * cos_x);
      float next_y_ru = this->curr_y_ru + (robot_velocity * delta_nsec_norm * sin_x);

      float next_theta = this->curr_theta + this->w_bz_ticks*delta_nsec_norm;

      this->curr_x_eu = next_x_eu;
      this->curr_y_eu = next_y_eu;
      this->curr_x_ru = next_x_ru;
      this->curr_y_ru = next_y_ru;
      this->curr_theta = next_theta;
    }
  }

  void run() {
    ros::spin();
  }

private:
  ros::NodeHandle n;
  ros::Subscriber sub_reader;
  ros::Subscriber sub_pose;
  ros::Publisher pub_velocity;
  ros::Publisher pub_odometry;
  float w_bz_ticks;
  float v_bx_ticks;
  float v_by_ticks;

  int seq_number = 143997;

  int first_read;
  int prev_ticks_fl;
  int prev_ticks_fr;
  int prev_ticks_rl;
  int prev_ticks_rr;
  int prev_sec;
  int prev_nsec;

  float curr_x_eu;
  float curr_y_eu;
  float curr_x_ru;
  float curr_y_ru;
  float curr_theta;

};

int main(int argc, char **argv) {
  // I don't know if it's relevant by i tend to use the name also in the 'type' category of the launch file
  ros::init(argc, argv, "wheel_state_reader");

  WheelState wheel_state;
  wheel_state.run();

  return 0;

}
