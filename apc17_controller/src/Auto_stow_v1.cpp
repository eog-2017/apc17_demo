#include<iostream>
#include<apc17_controller/joint_movej.h>
#include<apc17_controller/joint_movel.h>
#include<apc17_controller/pose_movej.h>
#include<apc17_controller/pose_movel.h>
#include <ros/ros.h>
#include <signal.h>
#include <unistd.h>
#include <fstream>

#include<apc17_controller/rack_detect.h>
#include<apc17_controller/object_detect.h>

#define DEG2RAD(x) M_PI*x/180.0


#define MACRO_NUM_BINS 6
#define MACRO_NUM_OBJECTS 6

#define MACRO_JOINT 1
#define MACRO_POSE 0
#define MACRO_ENERGY 1
#define MACRO_LINEAR 0

#define MACRO_AUTO_STOW 0


typedef struct _rack_info_
{
    float bins[MACRO_NUM_BINS][3];
}rack_info;


typedef struct _obj_info_
{
    float location[3];
    float normal[3];
}obj_info;


void signal_callback_handler(int signum)
{
    std::cout << "Caught Signal" << signum << std::endl;


    exit(0);
}



bool srv_call_rack_detection(ros::NodeHandle& nh, rack_info& rack)
{

    ros::ServiceClient srv_rack_detect =   nh.serviceClient<apc17_controller::rack_detect>("/srv_rack_detect");

    apc17_controller::rack_detect rack_detect;

    if(srv_rack_detect.call(rack_detect))
    {
        for(int i=0;i<MACRO_NUM_BINS;i++)
        {
            rack.bins[i][0] = rack_detect.response.bin_locations.data[i*3+0];
            rack.bins[i][1] = rack_detect.response.bin_locations.data[i*3+1];
            rack.bins[i][2] = rack_detect.response.bin_locations.data[i*3+2];
        }
        return true;
    }
    else
        return false;

}

bool srv_call_object_detect(ros::NodeHandle& nh, obj_info& object_info)
{
    ros::ServiceClient srv_obj_detect =   nh.serviceClient<apc17_controller::object_detect>("/srv_obj_detect");

    apc17_controller::object_detect obj_detect;

    if(srv_obj_detect.call(obj_detect))
    {
        object_info.location[0] = obj_detect.response.object_location.data[0];
        object_info.location[1] = obj_detect.response.object_location.data[1];
        object_info.location[2] = obj_detect.response.object_location.data[2];

        object_info.normal[0] = obj_detect.response.object_normal.data[0];
        object_info.normal[1] = obj_detect.response.object_normal.data[1];
        object_info.normal[2] = obj_detect.response.object_normal.data[2];

        return true;
    }
    else
        return false;

}

bool srv_call_move_robot(ros::NodeHandle& nh, std::vector<float>& joint_pose, bool joint_or_pose, bool energy_or_linear)
{


    int motion_type = -1;

    //    if(joint_or_pose)
    //        motion_type = 1;
    //    else
    //        motion_type = 3;

    //    if(energy_or_linear)
    //        motion_type += 1 ;
    //    else
    //        motion_type += 1;


    if(joint_or_pose && energy_or_linear)
        motion_type = 1;
    else if(joint_or_pose && !energy_or_linear)
        motion_type = 2;
    else if(!joint_or_pose && energy_or_linear)
        motion_type = 3;
    else if(!joint_or_pose && !energy_or_linear)
        motion_type = 4;

std::cout << "MOTION TYPE = " << motion_type << std::endl;

    switch(motion_type)
    {
    case 1:
    {
        ros::ServiceClient joint_movejService = nh.serviceClient<apc17_controller::joint_movej>("/joints/movej");

        apc17_controller::joint_movej joint_movej;

        for(int i=0;i<6;i++)
            joint_movej.request.joints.push_back(joint_pose[i]);

        if(joint_movejService.call(joint_movej))
        {
            return true;
        }
        else
            return false;

    }
        break;
    case 2:
    {
        ros::ServiceClient joint_movelService = nh.serviceClient<apc17_controller::joint_movel>("/joints/movel");
        apc17_controller::joint_movel joint_movel;

        for(int i=0;i<6;i++)
            joint_movel.request.joints.push_back(joint_pose[i]);

        if(joint_movelService.call(joint_movel))
        {
            return true;
        }
        else
            return false;
    }
        break;
    case 3:
    {
        ros::ServiceClient pose_movejService = nh.serviceClient<apc17_controller::pose_movej>("/pose/movej");

        apc17_controller::pose_movej pose_movej;

        for(int i=0;i<6;i++)
            pose_movej.request.pose.push_back(joint_pose[i]);

        if(pose_movejService.call(pose_movej))
        {
            return true;
        }
        else
            return false;
    }
        break;
    case 4:
    {
        ros::ServiceClient pose_movelService = nh.serviceClient<apc17_controller::pose_movel>("/pose/movel");

        apc17_controller::pose_movel pose_movel;

        for(int i=0;i<6;i++)
            pose_movel.request.pose.push_back(joint_pose[i]);

        if(pose_movelService.call(pose_movel))
        {
            return true;
        }
        else
            return false;

    }
    }

}


const float const_joints_rack_detect[]= {1.57457,-1.68438,-1.93159,-0.767926,1.53725,0.445088};
const float const_joints_tote_view[]= {2.30682,-2.49865,-1.59493,-0.707245,1.5372,0.444932};
const float const_joints_rack_view[]= {1.46068,-2.58579,-0.761579,-1.31002,1.53762,0.444165};

const float const_pose_obj[]= {-0.52,0.83,-0.343,2,1,1.5};
const float const_pose_bin[]= {0.03,1.05,-0.122,3,0,0};



int main(int argc, char **argv)
{
    ros::init(argc,argv,"apc17_controller");
    ros::NodeHandle nh;


    rack_info rack;


    while(ros::ok())
    {

        signal(SIGINT, signal_callback_handler);



        std::cout << "PRESS \n 0 rack detection \n 1 for stow test \n \n";




        int work_type = 7;
        fflush(stdin);
        std::cin >> work_type;



        switch(work_type)
        {
        case 1:
        {

            std::vector<float> joint;
            for(int i=0;i<6;i++)
                joint.push_back(const_joints_rack_detect[i]);

            bool srv_status = srv_call_move_robot(nh,joint,MACRO_JOINT,MACRO_ENERGY);

            if(srv_status)
            {
                ROS_INFO("RACK DETECTION VIEW REACHED\n");

                srv_status =  srv_call_rack_detection(nh, rack);

                if(srv_status)
                {
                    ROS_INFO(" RACK DETECTED\n");
                    if(MACRO_AUTO_STOW)work_type++;
                }
                else
                {
                    ROS_ERROR("RACK DETECTION FAILED\n");
                    ros::shutdown();
                }
            }
        }
            break;

        case 2:
        {
            std::vector<float> joint_tote_view;
            for(int i=0;i<6;i++)
                joint_tote_view.push_back(const_joints_tote_view[i]);

            std::vector<float> joint_rack_view;
            for(int i=0;i<6;i++)
                joint_rack_view.push_back(const_joints_rack_view[i]);

            std::vector<float> pose_obj_test;
            for(int i=0;i<6;i++)
                pose_obj_test.push_back(const_pose_obj[i]);
            std::vector<float> pose_bin_test;
            for(int i=0;i<6;i++)
                pose_bin_test.push_back(const_pose_bin[i]);

            for(int i=0;i<MACRO_NUM_OBJECTS;i++)
            {

                bool srv_status = srv_call_move_robot(nh,joint_tote_view,MACRO_JOINT,MACRO_ENERGY);

                if(srv_status)
                {
                    ROS_INFO("TOTE VIEW REACHED\n");
                    obj_info object_info;

//                    srv_status = srv_call_object_detect(nh, object_info);

                    if(srv_status)
                    {
                        ROS_INFO("OBJECT DETECTION DONE\n");

                        std::vector<float> obj_location(6);
                        for(int j=0;j<3;j++)
                        {
                            obj_location[j]= object_info.location[j];
                            obj_location[3+j]= object_info.normal[j];
                        }

//                        srv_status = srv_call_move_robot(nh,obj_location,MACRO_POSE, MACRO_LINEAR);
                        srv_status = srv_call_move_robot(nh,pose_obj_test,MACRO_POSE, MACRO_LINEAR);


                        if(srv_status)
                        {
                            ROS_INFO("REACHED OBJECT\n");


                        }
                        else
                        {
                            ROS_ERROR("OBJECT CNN'T BE REACHED\n");
                            continue;
                        }

                        srv_status = srv_call_move_robot(nh,joint_tote_view,MACRO_JOINT,MACRO_LINEAR);

                        if(srv_status)
                        {
                            ROS_INFO("REACHED TOTE VIEW AFTER PICKING THE OBJECT\n");
                        }
                        else
                        {
                            ROS_ERROR("REACHING TOTE VIEW AFTER PICKING THE OBJECT FAILED\n");
                            continue;
                        }

                        srv_status = srv_call_move_robot(nh,joint_rack_view,MACRO_JOINT,MACRO_ENERGY);

                        if(srv_status)
                        {
                            ROS_INFO("REACHED ABOVE RACK TO REACH BIN\n");
                        }
                        else
                        {
                            ROS_ERROR("REACHING ABOVE RACK TO REACH BIN FAILED\n");
                            continue;
                        }

                        int bin_no = i;

//                        std::vector<float> bin_location;

//                        for(int j=0;j<3;j++)
//                            bin_location.push_back(rack.bins[bin_no][j]);

//                        srv_status = srv_call_move_robot(nh,bin_location,MACRO_JOINT,MACRO_LINEAR);
                        srv_status = srv_call_move_robot(nh,pose_bin_test,MACRO_POSE,MACRO_LINEAR);


                        if(srv_status)
                        {
                            ROS_INFO("REACHED BIN\n");
                        }
                        else
                        {
                            ROS_ERROR("REACHING BIN FAILED\n");
                            continue;
                        }
                        srv_status = srv_call_move_robot(nh,joint_rack_view,MACRO_JOINT,MACRO_LINEAR);

                        if(srv_status)
                        {
                            ROS_INFO("REACHED ABOVE RACK TO REACH BIN\n");
                        }
                        else
                        {
                            ROS_ERROR("REACHING ABOVE RACK TO REACH BIN FAILED\n");
                            continue;
                        }

                    }
                    else
                    {
                        ROS_ERROR("OBJECT DETECTION FAILED\n");
                        continue;
                    }
                }
            }
        }
            break;

        }
    }


    return 0;
}
