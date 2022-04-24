#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "test_pkg/Position.h"

#include <dynamic_reconfigure/server.h>
#include <test_pkg/parametersConfig.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#define _USE_MATH_DEFINES
#include <sstream>
#include <cmath>


#define T 5.0
#define N 41.0 // 41.0 after adjustment
#define R 0.076 //0.076 after adjustment
#define L 0.2
#define W 0.169

class WheelState {
public:

  WheelState() {
    // Subscribers
    this->sub_reader = this->n.subscribe("wheel_states", 1, &WheelState::wheelStatePrint, this);
    this->sub_pose = this->n.subscribe("robot/pose", 1, &WheelState::robotPoseCheck, this);

    // Publishers
    this->pub_velocity = this->n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1);
    this->pub_odometry = this->n.advertise<nav_msgs::Odometry>("odom", 1);

    // Service server
    this->position_server = this->n.advertiseService("reset_pos", &WheelState::positionService, this);

    // Dynamic reconfigure
    this->f = boost::bind(&WheelState::dynConfig, this, _1, _2);
    this->dynServer.setCallback(f);


    // Utility
    this->first_read = 0;

    this->n.getParam("/pos_x", this->curr_x_ru);
    this->n.getParam("/pos_y", this->curr_y_ru);
    this->n.getParam("/theta", this->curr_theta);

    this->curr_x_eu = this->curr_x_ru;
    this->curr_y_eu = this->curr_y_ru;

  }

  // Dynamic reconfigure callback
  void dynConfig(test_pkg::parametersConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request: %d - Level %d",
              config.mth,
              level);

    this->integration_mth = config.mth;
  }

  // Service server callback
  bool positionService(test_pkg::Position::Request &req, test_pkg::Position::Response &res) {
    res.old_x_pos = this->curr_x_ru;
    res.old_y_pos = this->curr_y_ru;
    res.old_theta = this->curr_theta;

    this->curr_x_ru = req.new_x_pos;
    this->curr_y_ru = req.new_y_pos;
    this->curr_theta = req.new_theta_pos;

    this->curr_x_eu = this->curr_x_ru;
    this->curr_y_eu = this->curr_y_ru;

    ROS_INFO("Request to reset position to X:[%lf], Y:[%lf], TH:[%lf] - Responding with old position: X:[%lf], Y:[%lf], TH:[%lf]",
            req.new_x_pos, req.new_y_pos, req.new_theta_pos, res.old_x_pos, res.old_y_pos, res.old_theta);

    return true;

  }

  // Odometry publisher callback
  void robotPoseCheck(const geometry_msgs::PoseStamped::ConstPtr &msg) {

    // Just for debug
    float x_diff_eu = this->curr_x_eu - msg->pose.position.x;
    float y_diff_eu = this->curr_y_eu - msg->pose.position.y;

    if(this->max_x_eu < x_diff_eu) {
      this->max_x_eu = x_diff_eu;
    }
    if(this->max_y_eu < y_diff_eu) {
      this->max_y_eu = y_diff_eu;
    }

    float x_diff_ru = this->curr_x_ru - msg->pose.position.x;
    float y_diff_ru = this->curr_y_ru - msg->pose.position.y;

    if(this->max_x_ru < x_diff_ru) {
      this->max_x_ru = x_diff_ru;
    }
    if(this->max_y_ru < y_diff_ru) {
      this->max_y_ru = y_diff_ru;
    }
    // -------------------

/*
    ROS_INFO("[%d] maxEUX: [%f] --- maxRUX: [%f]", this->seq_number, this->max_x_eu, this->max_x_ru);
    ROS_INFO("[%d] maxEUY: [%f] --- maxRUY: [%f]", this->seq_number, this->max_y_eu, this->max_y_ru);
*/
/*
    ROS_INFO("[%d]\n\tcurrX:[%lf]\n\t-eulerX:[%lf]\n\t-rungeX[%lf]\n\tdiffX[%lf]", this->seq_number, this->curr_x_ru, x_diff_eu, x_diff_ru, x_diff_eu - x_diff_ru);
    ROS_INFO("[%d]\n\tcurrY:[%lf]\n\t-eluerY:[%lf]\n\t-rungeY[%lf]\n\tdiffY[%lf]", this->seq_number, this->curr_y_ru, y_diff_eu, y_diff_ru, y_diff_eu - y_diff_ru);
    ROS_INFO("[%d]\n\tTH: [%lf]", this->seq_number, this->curr_theta);
*/
/*
    ROS_INFO("[%d]currX: [%lf]\n", this->seq_number, this->curr_x_ru);
    ROS_INFO("[%d]currY: [%lf]\n", this->seq_number, this->curr_y_ru);
*/
/*
    geometry_msgs::TransformStamped transformStamped;
    tf2::Quaternion q;
    q.setRPY(0, 0, this->curr_theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.y();
    transformStamped.transform.rotation.w = q.w();
*/
/*
    ROS_INFO("[%d]x: [%lf]", this->seq_number, transformStamped.transform.rotation.x);
    ROS_INFO("[%d]y: [%lf]", this->seq_number, transformStamped.transform.rotation.y);
    ROS_INFO("[%d]z: [%lf]", this->seq_number, transformStamped.transform.rotation.z);
    ROS_INFO("[%d]w: [%lf]", this->seq_number, transformStamped.transform.rotation.w);
*/

    // --- ROBOT TRUE POSE (for debug)
    if(this->true_pose) {
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = "odom";
      transformStamped.child_frame_id = "base_link";


      if(this->integration_mth == 0) {
        transformStamped.transform.translation.x = msg->pose.position.x;
        transformStamped.transform.translation.y = msg->pose.position.y;
      } else {
        transformStamped.transform.translation.x = this->curr_x_ru;
        transformStamped.transform.translation.y = this->curr_y_ru;
      }

      transformStamped.transform.translation.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, this->curr_theta);
      transformStamped.transform.rotation.x = msg->pose.orientation.x;
      transformStamped.transform.rotation.y = msg->pose.orientation.y;
      transformStamped.transform.rotation.z = msg->pose.orientation.z;
      transformStamped.transform.rotation.w = msg->pose.orientation.w;
      br.sendTransform(transformStamped);
    }

    this->seq_number++;
  }

  void publishOdom() {
    // Publish Odometry
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "wolrd";
    odom.child_frame_id = "robot";

    if(this->integration_mth == 0) {
      odom.pose.pose.position.x = this->curr_x_eu;
      odom.pose.pose.position.y = this->curr_y_eu;
    } else {
      odom.pose.pose.position.x = this->curr_x_ru;
      odom.pose.pose.position.y = this->curr_y_ru;
    }

    tf2::Quaternion q;
    q.setRPY(0, 0, this->curr_theta);

    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = this->vx_base_ticks;
    odom.twist.twist.linear.y = this->vy_base_ticks;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = this->w_base_ticks;

    this->pub_odometry.publish(odom);
  }

  void publishTF() {
    // TF Broadcast
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";


    if(this->integration_mth == 0) {
      transformStamped.transform.translation.x = this->curr_x_eu;
      transformStamped.transform.translation.y = this->curr_y_eu;
    } else {
      transformStamped.transform.translation.x = this->curr_x_ru;
      transformStamped.transform.translation.y = this->curr_y_ru;
    }

    transformStamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, this->curr_theta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
  }

  // Velocity publisher callback
  void publishVelocity() {
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.stamp = ros::Time::now();
    cmd_vel.header.frame_id = "world";
    cmd_vel.twist.linear.x = this->vx_base_ticks;
    cmd_vel.twist.linear.y = this->vy_base_ticks;
    cmd_vel.twist.linear.z = 0;
    cmd_vel.twist.angular.x = 0;
    cmd_vel.twist.angular.y = 0;
    cmd_vel.twist.angular.z = this->w_base_ticks;

    this->pub_velocity.publish(cmd_vel);
  }

  float computeWheelW(float delta_ticks, float delta_nsec) {
    return ((delta_ticks/delta_nsec)/(N*T))*2*M_PI;
  }

/*
  float computeRobotW(float w_rr, float w_fl) {
    return (R * (w_rr - w_fl)) / (2 * (L+W));
  }

  float computeRobotVX(float w_fr, float w_fl) {
    return 0.5 * R * (w_fr + w_fl);
  }

  float computeRobotVY(float w_bz, float v_bx, float w_fl) {
    return (-W-L) * w_bz + v_bx - R * w_fl;
  }
*/

  float computeRobotVX(float w_fl_ticks, float w_fr_ticks, float w_rl_ticks, float w_rr_ticks) {
    return (w_fl_ticks + w_fr_ticks + w_rl_ticks + w_rr_ticks) * R / 4;
  }

  float computeRobotVY(float w_fl_ticks, float w_fr_ticks, float w_rl_ticks, float w_rr_ticks) {
    return (-w_fl_ticks + w_fr_ticks + w_rl_ticks - w_rr_ticks) * R / 4;
  }

  float computeRobotW(float w_fl_ticks, float w_fr_ticks, float w_rl_ticks, float w_rr_ticks) {
    return (-w_fl_ticks + w_fr_ticks - w_rl_ticks + w_rr_ticks) * R / (4 * (L + W));
  }

  float computeRobotVelocity(float vx, float vy) {
    return sqrt(pow(vx, 2) + pow(vy, 2));
  }

  // Main body of the node
  void wheelStatePrint(const sensor_msgs::JointState::ConstPtr &msg) {

    int curr_ticks_fl = 0;
    int curr_ticks_fr = 0;
    int curr_ticks_rl = 0;
    int curr_ticks_rr = 0;
    int delta_ticks_fl = 0;
    int delta_ticks_fr = 0;
    int delta_ticks_rl = 0;
    int delta_ticks_rr = 0;

    long int curr_sec = 0;
    long int curr_nsec = 0;
    //int delta_sec = 0;
    float delta_nsec = 0;
    float delta_nsec_norm;

    if(first_read == 0) {
      // ------------------- Initilization

      // ticks
      this->prev_ticks_fl = msg->position[0];
      this->prev_ticks_fr = msg->position[1];
      this->prev_ticks_rl = msg->position[2];
      this->prev_ticks_rr = msg->position[3];

      // time
      this->prev_sec = msg->header.stamp.sec;
      this->prev_nsec = pow(10, 9)*this->prev_sec + msg->header.stamp.nsec;
      //ROS_INFO("sec: %ld", this->prev_sec);
      //ROS_INFO("nsec: %ld", this->prev_nsec);

      this->first_read = 1;

    } else {
      // ------------------- compute

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

      // time
      curr_sec = msg->header.stamp.sec;
      curr_nsec = pow(10, 9)*curr_sec + msg->header.stamp.nsec;
      delta_nsec = curr_nsec - this->prev_nsec;
      delta_nsec_norm = (pow(10, -9) * delta_nsec);
      //ROS_INFO("nsec: %lf", delta_nsec);
      //ROS_INFO("nsec: %f", delta_nsec_norm);

      this->prev_sec = curr_sec;
      this->prev_nsec = curr_nsec;


      // ------------------- TICKS
      float w_fl_ticks = computeWheelW(delta_ticks_fl, delta_nsec_norm);
      float w_fr_ticks = computeWheelW(delta_ticks_fr, delta_nsec_norm);
      float w_rl_ticks = computeWheelW(delta_ticks_rl, delta_nsec_norm);
      float w_rr_ticks = computeWheelW(delta_ticks_rr, delta_nsec_norm);
      //ROS_INFO("w_rr: %f", w_rr_ticks);

      // ------------------- Robot Velocity
      /*
      this->w_base_ticks = computeRobotW(w_rr_ticks, w_fl_ticks);
      this->vx_base_ticks = computeRobotVX(w_fr_ticks, w_fl_ticks);
      this->vy_base_ticks = computeRobotVY(this->w_base_ticks, this->vx_base_ticks, w_fl_ticks);
      */
      this->w_base_ticks = computeRobotW(w_fl_ticks, w_fr_ticks, w_rl_ticks, w_rr_ticks);
      this->vx_base_ticks = computeRobotVX(w_fl_ticks, w_fr_ticks, w_rl_ticks, w_rr_ticks);
      this->vy_base_ticks = computeRobotVY(w_fl_ticks, w_fr_ticks, w_rl_ticks, w_rr_ticks);
      //ROS_INFO("v_bx: %f", this->v_bx_ticks);
      //ROS_INFO("v_by: %f", this->v_by_ticks);

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

      // ------------------- World Velocity (maybe this needs to be printed int odom, not the base one)
      float pi_rad = (90 * M_PI) / 180;
      this->vx_world = vx_base_ticks * cos(this->curr_theta) + vy_base_ticks * cos(pi_rad + this->curr_theta);
      this->vy_world = vx_base_ticks * sin(this->curr_theta) + vy_base_ticks * sin(pi_rad + this->curr_theta);

      // ------------------- Odometry
      float robot_velocity = computeRobotVelocity(this->vx_world, this->vy_world);
      float cos_x = cos(this->curr_theta + ((this->w_base_ticks * delta_nsec_norm) / 2.0));
      float sin_x = sin(this->curr_theta + ((this->w_base_ticks * delta_nsec_norm) / 2.0));

/*
      ROS_INFO("cos: [%f] --- sin:[%f]", cos_x, sin_x);
      ROS_INFO("theta: [%f]", this->curr_theta);
*/
/*
      ROS_INFO("\n[%d]velocity: [%lf]\n", this->seq_number, robot_velocity);
      ROS_INFO("[%d]cos: [%lf]\n", this->seq_number, cos_x);
      ROS_INFO("[%d]sin: [%lf]\n", this->seq_number, sin_x);
      ROS_INFO("[%d]deltaT: [%lf]\n", this->seq_number, delta_nsec_norm);
*/

      float next_x_eu = this->curr_x_eu + this->vx_world * delta_nsec_norm;
      float next_y_eu = this->curr_y_eu + this->vy_world * delta_nsec_norm;

      float next_x_ru = this->curr_x_ru + (robot_velocity * delta_nsec_norm * cos_x);
      float next_y_ru = this->curr_y_ru + (robot_velocity * delta_nsec_norm * sin_x);

      float next_theta = this->curr_theta + this->w_base_ticks * delta_nsec_norm;

      this->curr_x_eu = next_x_eu;
      this->curr_y_eu = next_y_eu;
      this->curr_x_ru = next_x_ru;
      this->curr_y_ru = next_y_ru;
      this->curr_theta = next_theta;


      // ------------------- Publishing
      publishVelocity();
      publishOdom();
      publishTF();
      this->true_pose = false;
    }
  }

  void run() {
    ros::spin();
  }

private:
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber sub_reader;
  ros::Subscriber sub_pose;

  // Publishers
  ros::Publisher pub_velocity;
  ros::Publisher pub_odometry;

  // Service server
  ros::ServiceServer position_server;

  // Dynamic reconfigure
  dynamic_reconfigure::Server<test_pkg::parametersConfig> dynServer;
  dynamic_reconfigure::Server<test_pkg::parametersConfig>::CallbackType f;

  // TF Broadcaster
  tf2_ros::TransformBroadcaster br;
  //tf::Transform transform;

  // Utility
  float w_base_ticks = 0;
  float vx_base_ticks = 0;
  float vy_base_ticks = 0;
  float vx_world = 0;
  float vy_world = 0;

  // Just for debug
  int seq_number = 143997;
  // -----

  int first_read = 0;
  int prev_ticks_fl = 0;
  int prev_ticks_fr = 0;
  int prev_ticks_rl = 0;
  int prev_ticks_rr = 0;
  long int prev_sec = 0;
  long int prev_nsec = 0;

  float curr_x_eu = 0;
  float curr_y_eu = 0;
  float curr_x_ru = 0;
  float curr_y_ru = 0;
  float curr_theta = 0;

  int integration_mth = 0;
  bool true_pose = true;

  float max_x_eu = 0;
  float max_y_eu = 0;
  float max_x_ru = 0;
  float max_y_ru = 0;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "wheel_states");

  WheelState wheel_state;
  wheel_state.run();

  return 0;

}
