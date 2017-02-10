#include <iostream>
#include <ros/ros.h>
#include<ur10_motion/movej.h>
#include<ur10_motion/movel.h>
#include <signal.h>
#include <sensor_msgs/JointState.h>
#include<std_msgs/String.h>
#include "/home/ravi/apc17ws/src/apc17_demo/ur10_motion/include/ur5model.h"


#define pose 0
#define joint 1

using namespace MODUR5;
MODUR5::UR5 robot;

ros::Publisher command_pub;
std::vector<double> current_jts;
std::vector<double> current_pose;

void signal_callback_handler(int signum)
{
    std::cout << "Caught Signal" << signum << std::endl;
    std::cout << "Exiting ur10 motion" << std::endl;

    exit(0);
}

void ftoa(double x, char* text)
{
    double y = x;

    int i=0;

    if(x < 0.0)
    {
        text[i++] = '-';
        y=-x;
    }

    text[i++] = (int64_t)y+48;
    text[i++] = '.';
    y = y - (int64_t)y;
    y *= 10.0;

    while(i<5)
    {
        text[i++] = (int64_t)y+48;
        y = y - (int64_t)y;
        y *= 10.0;
    }
    text[5] = '\0';
}


void pos_to_movej_cmd(std::vector<double> position, double a, double v, std_msgs::String& movej_cmd)
{

    std::stringstream pos_command;

    char text[6];
#if(pose)
    pos_command << "movej(p[";
#endif
#if(joint)
    pos_command << "movej([";
#endif
    //loop over position

    for(int i=0;i<6;i++)
    {
        ftoa(position[i],text);
        pos_command << text;
        if(i<5)
            pos_command <<", ";
    }

    pos_command << "], ";

    //insert a
    pos_command << "a=";
    ftoa(a,text);
    pos_command << text;
    pos_command <<", ";

    //insert v
    pos_command << "v=";
    ftoa(v,text);
    pos_command << text;
    pos_command << ")";

    movej_cmd.data= pos_command.str();
std::cout<<movej_cmd.data<<std::endl;

}

void pos_to_movel_cmd(std::vector<double> position, double a, double v, std_msgs::String& movel_cmd)
{

    std::stringstream pos_command;

    char text[6];
#if(pose)
    pos_command << "movel(p[";
#endif
#if(joint)
    pos_command << "movel([";
#endif
    //loop over position

    for(int i=0;i<6;i++)
    {
        ftoa(position[i],text);
        pos_command << text;
        if(i<5)
            pos_command <<", ";
    }

    pos_command << "], ";

    //insert a
    pos_command << "a=";
    ftoa(a,text);
    pos_command << text;
    pos_command <<", ";

    //insert v
    pos_command << "v=";
    ftoa(v,text);
    pos_command << text;
    pos_command << ")";

    movel_cmd.data= pos_command.str();
std::cout<<movel_cmd.data<<std::endl;

}


void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{

double theta[6];
double eepose[6];

        current_jts.clear();
        current_pose.clear();
        for(int i=0;i<msg->position.size();i++)
        {
            theta[i]=msg->position[i];
            current_jts.push_back(msg->position[i]);
        }



        robot.set_joint_angles(theta);
        robot.fwd_kin_pose(eepose);
        for(int j=0;j<6;j++)
            current_pose.push_back(eepose[j]);

    return;
}




bool movejControlCallback(ur10_motion::movej::Request &req, ur10_motion::movej::Response &res)
{
   std::vector<double> ur10_desired_pose(6);
    std::cout << "Reached service to move robot by movej" << std::endl;
    std_msgs::String movej_cmd;
    for(int i=0;i<req.joints.size();i++)
    {
        std::cout << req.joints[i] << "  ";
        ur10_desired_pose[i]=req.joints[i];

    }
    std::cout << std::endl;
    pos_to_movej_cmd(ur10_desired_pose,0.200,0.8500,movej_cmd);
    int j=0;

    ros::Rate rate =3;
    while(j++<2)
            {
              command_pub.publish(movej_cmd);
              rate.sleep();
            }

    bool goal_achieved = false;

std::cout << std::endl;
    while(!goal_achieved)
    {
        ros::spinOnce();


        double avg_pos_diff = 0;
//******************************when ur10_desired_pose is joint position*****************************************************
#if(joint)
        for(int i = 0; i < ur10_desired_pose.size(); i++)
            avg_pos_diff += fabs((current_jts[i] - ur10_desired_pose[i]));
        avg_pos_diff /= ur10_desired_pose.size();
#endif
//******************************when ur10_desired_pose is position*****************************************************
#if(pose)
        for(int i = 0; i < 3; i++)
            avg_pos_diff = fabs((current_pose[0] - ur10_desired_pose[0]))+fabs((current_pose[1] - ur10_desired_pose[1]))+fabs((current_pose[2] - ur10_desired_pose[2]));
        avg_pos_diff /= 3;
#endif
        std::cout<<avg_pos_diff<<std::endl;
        // wait until avg joint position diff is approx. 5.7 degrees
        if(avg_pos_diff < 0.0135)
            goal_achieved = true;
        else
        {
            goal_achieved = false;

        }
        signal(SIGINT, signal_callback_handler);

    }

    std::cout<<" movejcallback exiting"<<std::endl;
   ros::Duration(1).sleep(); // sleep for half a second
    if(goal_achieved)
    {

        return true;
    }
    else
        return false;








return true;




}

bool movelControlCallback(ur10_motion::movel::Request &req, ur10_motion::movel::Response &res)
{
   std::vector<double> ur10_desired_pose(6);
    std::cout << "Reached service to move robot by movel" << std::endl;
    std_msgs::String movel_cmd;
    for(int i=0;i<req.joints.size();i++)
    {
        std::cout << req.joints[i] << "  ";
        ur10_desired_pose[i]=req.joints[i];
//        std::cout << ur10_desired_pose[i] << "  ";

    }
    std::cout << std::endl;
    pos_to_movel_cmd(ur10_desired_pose,0.10,0.30,movel_cmd);
    int j=0;

    ros::Rate rate =3;
    while(j++<2)
            {
              command_pub.publish(movel_cmd);
        //      ros::spinOnce();
              rate.sleep();
            }

    std::cout<<" movelcallback exiting"<<std::endl;

return true;




}



int main(int argc, char **argv)
{
    std::cout<<" main"<<std::endl;

signal(SIGINT, signal_callback_handler);
ros::init(argc,argv,"ur10_motion");
ros::NodeHandle nh;

 ros::ServiceServer service_movej_motion = nh.advertiseService("/motion/movej", movejControlCallback);
ros::ServiceServer service_movel_motion = nh.advertiseService("/motion/movel", movelControlCallback);


std::vector<double> current_pose(6);
//ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("/joint_states",1,boost::bind(jointStateCallback,_1,boost::ref(current_pose)));

//ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>
//        ("/joint_states",1,boost::bind(jointStateCallback,_1,boost::ref(current_pose)));
ros::Subscriber sub = nh.subscribe("/joint_states", 1, jointStateCallback);
command_pub = nh.advertise<std_msgs::String>("/ur_driver/URScript", 1);
ros::MultiThreadedSpinner spinner(4);

spinner.spin();

return 0;
}
