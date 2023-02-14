#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Int32MultiArray.h"

//AD1
const double PI = 3.14159265;
const double DistancePerCount = (2 * PI * 0.1) / 1500 ;// (2*pi*r)/ppr
const double WHEELRADIUS = 0.1; // Wheel radius in meters
const double WHEEL2WHEELWIDTH = 0.33; // Center of left tire to center of right tire base :0.33

ros::Subscriber sub_encoder;

// Create odometry data publishers
ros::Publisher odom_pub;

nav_msgs::Odometry odom;

double last_position_x,last_position_y, last_angle ;

// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;

double g_distance_r = 0;
double g_distance_l = 0;

ros::Time current_time, last_time;




void encoder_cb(const std_msgs::Int32MultiArray::ConstPtr& encoder)
{

  int encoder_r = encoder->data[0];
  int encoder_l = encoder->data[1];

  //ROS_INFO("encoder msg = %d , %d", encoder_r , encoder_l);

  g_distance_r = encoder_r * DistancePerCount ;
  g_distance_l = encoder_l * DistancePerCount ;

  //ROS_INFO("wheel distance msg = %f , %f", g_distance_r , g_distance_l);
}


// Update odometry information
void update_odom() {

  tf::TransformBroadcaster odom_broadcaster;
  current_time = ros::Time::now();

  // Calculate the average distance
  double robot_distance = (g_distance_r + g_distance_l) / 2;

  // Calculate the number of radians the robot has turned

  double robot_angle = (g_distance_r-g_distance_l)/WHEEL2WHEELWIDTH;

  //ROS_INFO("robot_angle: %f", robot_angle*(180.0/PI));

  double x,y;
  x = cos(robot_angle)*robot_distance;
  y = sin(robot_angle)*robot_distance;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_angle);


  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  double vx = (x-last_position_x)/((current_time-last_time).toSec());
  double vy = (y-last_position_y)/((current_time-last_time).toSec());
  double vth =(robot_angle - last_angle)/((current_time-last_time).toSec());

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  //publish the message
  odom_pub.publish(odom);

  last_position_x = x;
  last_position_y = y;
  last_angle = robot_angle;
  last_time = current_time;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ad1_odometry");
  ros::NodeHandle nh;

  last_position_x = initialX;
  last_position_y = initialX;
  last_angle = initialTheta;



  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

  sub_encoder = nh.subscribe("encoder_pos", 1000, encoder_cb);

  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate loop_rete(10);

  while(ros::ok())
  {
    update_odom();

    ros::spinOnce();
    loop_rete.sleep();
  }

}
