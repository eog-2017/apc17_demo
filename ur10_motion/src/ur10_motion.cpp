#include <iostream>
#include <ros/ros.h>
#include<ur10_motion/movej.h>
#include<ur10_motion/movel.h>
#include <signal.h>
#include <sensor_msgs/JointState.h>
#include<std_msgs/String.h>
//#include</home/ravi/apc17ws/src/apc17_demo/ur10_motion/include/ur10_motion.h>



sensor_msgs::JointState current_pose;
ros::Publisher command_pub;

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
    pos_command << "movej(p[";
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



//void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
//{
////    std::cout<<" cahttercallback reached"<<std::endl;


////        std::cout << "Position =  ";
//        for(int i=0;i<msg->position.size();i++)
//        {
////            std::cout << msg->position[i] << "  ";
//            current_pose.position.push_back(msg->position[i]);
//        }
//        std::cout << std::endl;
////        std::cout<<" cahttercallback exiting"<<std::endl;
//    return;
//}

bool movejControlCallback(ur10_motion::movej::Request &req, ur10_motion::movej::Response &res)
{
   std::vector<double> ur10_desired_pose(6);
    std::cout << "Reached service to move robot by movej" << std::endl;
    std_msgs::String movej_cmd;
    for(int i=0;i<req.joints.size();i++)
    {
        std::cout << req.joints[i] << "  ";
        ur10_desired_pose[i]=req.joints[i];
//        std::cout << ur10_desired_pose[i] << "  ";

    }
    std::cout << std::endl;
    pos_to_movej_cmd(ur10_desired_pose,1.0,1.0,movej_cmd);
    int j=0;

    ros::Rate rate =3;
    while(j++<2)
            {
              command_pub.publish(movej_cmd);
        //      ros::spinOnce();
              rate.sleep();
            }

    std::cout<<" movejcallback exiting"<<std::endl;

return true;




}



int main(int argc, char **argv)
{
signal(SIGINT, signal_callback_handler);
ros::init(argc,argv,"ur10_motion");
ros::NodeHandle nh;

 ros::ServiceServer service_movej_motion = nh.advertiseService("/motion/movej", movejControlCallback);
//ros::ServiceServer service_movel_motion = nh.advertiseService("/motion/movel", movelControlCallback);



// ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("/joint_states",1,jointStateCallback);

//ros::Subscriber sub = nh.subscribe("/joint_states", 1000, chatterCallback);
command_pub = nh.advertise<std_msgs::String>("/ur_driver/URScript", 1);
ros::MultiThreadedSpinner spinner(4);

spinner.spin();

return 0;
}
