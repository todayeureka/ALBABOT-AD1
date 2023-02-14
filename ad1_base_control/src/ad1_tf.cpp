#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "ad1_tf");
  ros::NodeHandle n;

  ros::Rate r(10);

  tf::Quaternion q;



 q.setRPY(0,0,3.141592); //90*(3.141592/180.0)

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
      broadcaster.sendTransform(
        tf::StampedTransform(
          tf::Transform(
            tf::Quaternion(0, 0, 0, 1),
            tf::Vector3(0.0, 0.0, 0.01)),
            ros::Time::now(),
            "base_footprint",
            "base_link"));



      broadcaster.sendTransform(
        tf::StampedTransform(
          tf::Transform(
            q,
            tf::Vector3(0.01, 0.0, 0.04)),
            ros::Time::now(),
            "base_link",
            "base_scan"));


    r.sleep();
  }
}
