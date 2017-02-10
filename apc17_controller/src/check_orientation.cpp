#include<iostream>
#include<apc17_controller/movej.h>
#include<apc17_controller/movel.h>
#include<apc17_controller/Kinect3D.h>
#include<apc17_controller/srv_calib_test.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <signal.h>
#include <unistd.h>
#include <fstream>
#include<eigen3/Eigen/Eigen>
//#include <eigen3/Eigen/SVD>
#include "/home/ravi/apc17ws/src/apc17_demo/ur10_motion/include/ur5model.h"



#include<pcl/registration/transformation_estimation_svd.h>

using namespace MODUR5;
MODUR5::UR5 robot;







#define NO_OF_JOINTS 6

#define NO_OF_POINTS_Marker_View 101




void signal_callback_handler(int signum)
{
    std::cout << "Caught Signal" << signum << std::endl;


    exit(0);
}






template<typename T>
void MakeMatrix(T ** &temp,unsigned rows, unsigned cols)
{
    temp = new T*[rows];

    for(unsigned int i = 0; i < rows; ++i)
        temp[i] = new T[cols];
}


void LoadData_jts(const char *filename, long double ** &Weights)
{
    std::string line;
    std::ifstream file( filename );
    char temp;
    if(file.good())
    {
        for(unsigned int ii = 0;ii<NO_OF_POINTS_Marker_View;ii++)
        {
            for(unsigned int jj = 0;jj<NO_OF_JOINTS;jj++)
            {
                getline ( file, line, ',' );
                Weights[ii][jj] = atof(line.c_str());
                //                std::cout<<Weights[ii][jj]<<"  "<<" "<<ii+1<<" "<< jj+1<<" "<<std::endl;

            }
            //            std::cout<<std::endl<<std::endl;
            //std::cin >> temp;
        }
    }
    else
    {
        std::cout<<"Cannot find "<<filename<<std::endl;
        exit(0);
    }



}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"apc17_controller");
    ros::NodeHandle nh;

    ros::ServiceClient movejService = nh.serviceClient<apc17_controller::movej>("/motion/movej");
    ros::ServiceClient movelService = nh.serviceClient<apc17_controller::movel>("/motion/movel");
    ros::ServiceClient Kinect3DService = nh.serviceClient<apc17_controller::Kinect3D>("kinect2/get3Dpoint");
    ros::ServiceClient service_test_point = nh.serviceClient<apc17_controller::srv_calib_test>("calib/test_3d_point");




    while(ros::ok())
    {
        std::cout << "PRESS \n 0 for database generation \n \n";

        int calib_or_test = -1;
        fflush(stdin);
        //        scanf("%d",&calib_or_test);
        std::cin >> calib_or_test;


        switch(calib_or_test)
        {
        case 0:
        {

            long double **MarkerPoints_wrt_Base_actual;
            long double **MarkerView_joint_angles;





            MakeMatrix<long double>(MarkerView_joint_angles,NO_OF_POINTS_Marker_View,NO_OF_JOINTS); // Allocate memory for weights
            LoadData_jts("/home/ravi/apc17ws/object_detection1.txt",MarkerView_joint_angles);

            for(unsigned int ii = 0;ii<NO_OF_POINTS_Marker_View;ii++)
            {
                for(unsigned int jj = 0;jj<NO_OF_JOINTS;jj++)
                {


                    std::cout<<MarkerView_joint_angles[ii][jj]<<"  "<<" "<<ii+1<<" "<< jj+1<<" "<<std::endl;

                }
                std::cout<<std::endl<<std::endl;
            }

            std::cout<<"press any key to move robot to start position"<<std::endl;
//                        fflush(stdin);
//                        getchar();

            apc17_controller::movej ur10_movej_start;
            double p0[6]={-3.32961,-1.4674,2.08827,-1.86459,4.72845,-0.00664026};//if in joint



            for(int i=0;i<6;i++)
                ur10_movej_start.request.joints.push_back(p0[i]);
            if(movejService.call(ur10_movej_start))
                std::cout << "Start  position reached " << std::endl;
            else
                ROS_ERROR("Failed to call service");




            for(int count=0;count<NO_OF_POINTS_Marker_View;count++)
            {


                signal(SIGINT, signal_callback_handler);

                int input;
                std::cout <<"\n Press any key for next marker view position"<< std::endl;
//                                fflush(stdin);
//                                getchar();

                std::cout<<"Moving towards Marker View : "<< count << std::endl;
                apc17_controller::movej next_marker;
                for(int i=0;i<6;i++)
                    next_marker.request.joints.push_back(MarkerView_joint_angles[count][i]);
                if(movejService.call(next_marker))
                    std::cout << "Marker View"<< count << "reached " << std::endl;
                else
                    ROS_ERROR("Failed to call service");
                ros::Duration(1).sleep();


                std::cout<<"kinect service"<<std::endl;
                apc17_controller::Kinect3D marker_point_kinect;



//                if(Kinect3DService.call(marker_point_kinect))
//                {



//                    std::cout << "Kinect service done reached " << std::endl;
//                }
//                else
//                    ROS_ERROR("Failed to call service");
            }





            break;

        }
        }

    }
    return 0;
}
