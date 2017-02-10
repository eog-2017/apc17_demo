#ifndef UR10_MOTION_H
#define UR10_MOTION_H

#include <ros/ros.h>
#include <signal.h>
#include <unistd.h>
#include <fstream>



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


}



bool movejControlCallback(ur10_motion::movej::Request &req, ur10_motion::movej::Response &res)
{
   std::vector<double> ur10_desired_pose(6);
    std::cout << "Reached service to move robot by movej" << std::endl;
    for(int i=0;i<req.joints.size();i++)
    {
        std::cout << req.joints[i] << "  ";
        ur10_desired_pose[i]=req.joints[i];
    }
    std::cout << std::endl;
    std::cout<<" movejcallback exiting"<<std::endl;

return true;




}






void ur10JointStateCallback(const sensor_msgs::JointState::ConstPtr &jt_state_msg, std::vector<double> &pose_state)
{
    double jt_state[6];
    std::cout << "Position =  ";
    for(int i = 0; i < jt_state_msg->position.size(); i++)
    {
//        std::cout << jt_state_msg->position[i] << "  ";
        jt_state[i] = jt_state_msg->position[i];
    }
    std::cout << std::endl;
    double T[16]; double test_jt[6]={0,0,0,0,0,0};
    double eetrans[3];double eerot[3];
//   #define UR10_PARAMS
//    forward(jt_state, T);
//    forward(test_jt, T);

//    extractfrom_mat44(T,eetrans,eerot);
    pose_state[0]=eetrans[0];    pose_state[1]=eetrans[1];    pose_state[2]=eetrans[2];
    pose_state[3]=eerot[0];    pose_state[4]=eerot[1];    pose_state[5]=eerot[2];

    for(int i = 0; i < pose_state.size(); i++)
    {
        std::cout << pose_state[i] << "  ";
    }    std::cout << std::endl;

    return;
}


////bool checkJointsReached(vector<double> &desired_joints_state, vector<double> &current_joints_state)
//{
//    bool goal_achieved = false;
//    double current_jts[7];

//    double duration;

//    int start_jt = 1;
//    while(!goal_achieved)
//    {
//        ros::spinOnce();
//        for(int i = 0; i < current_joints_state.size(); i++)
//            current_jts[i] = current_joints_state[i];

//        double avg_pos_diff = 0;
//        for(int i = start_jt; i < desired_joints_state.size(); i++)
//            avg_pos_diff += fabs((current_jts[i] - desired_joints_state[i]));
//        avg_pos_diff /= desired_joints_state.size();

//        // wait until avg joint position diff is approx. 5.7 degrees
//        if(avg_pos_diff < 0.025)
//            goal_achieved = true;
//        else
//        {
//            goal_achieved = false;

//        }

//        duration = ros::Time::now().toSec() - time;
//        if(duration > 8) // if in loop for more than 60 secs then break out
//        {
//            ROS_INFO("Time limit exceeded to reach desired position");
//            goal_achieved = true;
//            break;
//        }
////        cout << "Desired: [";
////        for(int i=0; i<7 ; i++)
////            cout << desired_joints_state[i] << " ";
////        cout << "]\n";
////        cout << "Current: [";
////        for(int i=0; i<7 ; i++)
////            cout << current_joints_state[i] << " ";
////        cout << "]";
//    }
//    if(goal_achieved)
//    {
//        cout << "WAM desired position achieved in: " << duration << "seconds" << endl;
//        return true;
//    }
//    else
//        return false;
//}
#endif

