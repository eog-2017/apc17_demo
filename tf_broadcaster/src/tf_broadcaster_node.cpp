#include<iostream>
#include<ros/ros.h>
#include<tf/transform_broadcaster.h>

int main(int argc, char **argv)
{


    ros::init(argc,argv,"broadcasting");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(10.0);

    tf::TransformBroadcaster br;
    tf::Transform ee_point;


    while(ros::ok())
    {
    ee_point.setOrigin(tf::Vector3(0.0895,0.0,0.0));
    ee_point.setRotation(tf::Quaternion(0,0,0,1));
    br.sendTransform(tf::StampedTransform(ee_point,ros::Time::now(),"ee_link","point"));


r.sleep();
    }


    return 0;
}
