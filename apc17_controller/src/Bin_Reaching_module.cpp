#include<iostream>
#include<apc17_controller/joint_movej.h>
#include<apc17_controller/joint_movel.h>
#include<apc17_controller/pose_movej.h>
#include<apc17_controller/pose_movel.h>
#include <ros/ros.h>
#include <signal.h>
#include <unistd.h>
#include <fstream>
#define DEG2RAD(x) M_PI*x/180.0


void signal_callback_handler(int signum)
{
    std::cout << "Caught Signal" << signum << std::endl;


    exit(0);
}




int main(int argc, char **argv)
{
    ros::init(argc,argv,"apc17_controller");
    ros::NodeHandle nh;



    ros::ServiceClient joint_movejService = nh.serviceClient<apc17_controller::joint_movej>("/joints/movej");
    ros::ServiceClient joint_movelService = nh.serviceClient<apc17_controller::joint_movel>("/joints/movel");

    ros::ServiceClient pose_movejService = nh.serviceClient<apc17_controller::pose_movej>("/pose/movej");
    ros::ServiceClient pose_movelService = nh.serviceClient<apc17_controller::pose_movel>("/pose/movel");



    while(ros::ok())
    {


        signal(SIGINT, signal_callback_handler);

        int input;
        std::cout << "\n Press 0 for Start Position "
                  << "\n Press 1  testing joint_movej"
                  << "\n Press 2  testing joint_movel"
                  << "\n Press 3  testing pose_movej"
                  << "\n Press 4  testing pose_movel"
                  << std::endl;
        std::cin>>input;
        getchar();







        switch(input)
        {
        case 0:

        {
            apc17_controller::joint_movej test_start;
            double p0[6]={DEG2RAD(185.6), DEG2RAD(-44.01), DEG2RAD(-129.62),DEG2RAD(7.62),DEG2RAD(-267.6), DEG2RAD(-0.37)};
//            double p0_i[6];
//            for(int i=0;i<6;i++)
//                p0_i[i]=p0[i];
//            p0_i[1] += 0.100;
            for(int i=0;i<6;i++)
                test_start.request.joints.push_back(p0[i]);
            if(joint_movejService.call(test_start))
                std::cout << "Start  position reached " << std::endl;
            else
                ROS_ERROR("Failed to call service");


        }break;

        case 1:

        {
            apc17_controller::joint_movej test_joint_movej1;
            double p1[6]={2.39169,-1.4114,-1.79109,0.103913,-3.55616,-0.00640089};
            for(int i=0;i<6;i++)
                test_joint_movej1.request.joints.push_back(p1[i]);
            if(joint_movejService.call(test_joint_movej1))
                std::cout << "moved by joint_movej --- waypoint 1" << std::endl;
            else
                ROS_ERROR("Failed to call service");

            apc17_controller::joint_movej test_joint_movej2;

            double p2[6]={DEG2RAD(281), DEG2RAD(-84), DEG2RAD(-99),DEG2RAD(-5.5),DEG2RAD(-34), DEG2RAD(-0.37)};
            for(int i=0;i<6;i++)
                test_joint_movej2.request.joints.push_back(p2[i]);
            if(joint_movejService.call(test_joint_movej2))
                std::cout << "moved by joint_movej --- waypoint 2" << std::endl;
            else
                ROS_ERROR("Failed to call service");
        }break;

        case 2:

        {
            std::cout<<"No need of this move"<<std::endl;

            apc17_controller::joint_movel test_joint_movel1;
            double p5[6]={2.39169,-1.4114,-1.79109,0.103913,-3.55616,-0.00640089};
            for(int i=0;i<6;i++)
                test_joint_movel1.request.joints.push_back(p5[i]);
            if(joint_movelService.call(test_joint_movel1))
                std::cout << "moved by joint_movel --- waypoint 1" << std::endl;
            else
                ROS_ERROR("Failed to call service");

            apc17_controller::joint_movej test_joint_movel2;

            double p6[6]={DEG2RAD(281), DEG2RAD(-84), DEG2RAD(-99),DEG2RAD(-5.5),DEG2RAD(-34), DEG2RAD(-0.37)};
            for(int i=0;i<6;i++)
                test_joint_movel2.request.joints.push_back(p6[i]);
            if(joint_movelService.call(test_joint_movel2))
                std::cout << "moved by joint_movel --- waypoint 2" << std::endl;
            else
                ROS_ERROR("Failed to call service");

        }break;

        case 3:

        {
            apc17_controller::pose_movej test_pose_movej1;
            double p3[6]={-0.421,0.023639,0.77826811,2.450,2.1150,-2.7560};
            for(int i=0;i<6;i++)
                test_pose_movej1.request.pose.push_back(p3[i]);
            if(pose_movejService.call(test_pose_movej1))
                std::cout << "moved by pose_movej --- waypoint 1" << std::endl;
            else
                ROS_ERROR("Failed to call service");

            apc17_controller::pose_movej test_pose_movej2;
            double p4[6]={-0.7281743,-0.31913,0.8610038,2.370,2.30,-2.6370};
            for(int i=0;i<6;i++)
                test_pose_movej2.request.pose.push_back(p4[i]);
            if(pose_movejService.call(test_pose_movej2))
                std::cout << "moved by pose_movej --- waypoint 2" << std::endl;
            else
                ROS_ERROR("Failed to call service");
        }break;

        case 4:

        {
            apc17_controller::pose_movel test_pose_movel1;
            double p7[6]={-0.421,0.023639,0.77826811,2.450,2.1150,-2.7560};
            for(int i=0;i<6;i++)
                test_pose_movel1.request.pose.push_back(p7[i]);
            if(pose_movelService.call(test_pose_movel1))
                std::cout << "moved by pose_movel --- waypoint 1" << std::endl;
            else
                ROS_ERROR("Failed to call service");

            apc17_controller::pose_movel test_pose_movel2;
            double p4[6]={-0.7281743,-0.31913,0.8610038,2.370,2.30,-2.6370};
            for(int i=0;i<6;i++)
                test_pose_movel2.request.pose.push_back(p4[i]);
            if(pose_movelService.call(test_pose_movel2))
                std::cout << "moved by pose_movel --- waypoint 2" << std::endl;
            else
                ROS_ERROR("Failed to call service");
        }break;





        }

    }





    return 0;
}
