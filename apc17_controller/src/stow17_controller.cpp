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


#define DEG2RAD(x) M_PI*x/180.0


#define NO_OF_POINTS_Marker 23

#define NO_OF_JOINTS 6

#define NO_OF_POINTS_Marker_View 3

#define NO_OF_POSE_VARIABLE 3


const char* file_name_pcl_tx = "/home/ravi/apc17ws/kinect_T_wr3_pcl.txt";
const char* file_name_tx = "/home/ravi/apc17ws/kinect_T_wr3_tx.txt";

void signal_callback_handler(int signum)
{
    std::cout << "Caught Signal" << signum << std::endl;


    exit(0);
}

Eigen::MatrixXf find_rotation_matrix(Eigen::MatrixXf points_wrt_base, Eigen::MatrixXf points_wrt_kinect)
{
    int no_of_points = points_wrt_base.rows()-1;

    Eigen::MatrixXf difference_kinect_data_matrix(no_of_points,3);
    Eigen::MatrixXf difference_base_data_matrix(no_of_points,3);
    Eigen::MatrixXf M(3,3);
    Eigen::MatrixXf final_r_mat(3,3);

    for(int i=0;i<no_of_points;i++)
    {
        difference_base_data_matrix.row(i) = points_wrt_base.row(i)-points_wrt_base.row(i+1);
        difference_kinect_data_matrix.row(i) = points_wrt_kinect.row(i)-points_wrt_kinect.row(i+1);
    }
    M = difference_base_data_matrix.transpose()*difference_kinect_data_matrix;

    Eigen::JacobiSVD<Eigen::MatrixXf> svd( M, Eigen::ComputeFullV | Eigen::ComputeFullU );

    final_r_mat = svd.matrixU()*(svd.matrixV().transpose());

    return final_r_mat;
}

Eigen::MatrixXf find_translation_matrix(Eigen::MatrixXf points_wrt_base,
                                        Eigen::MatrixXf points_wrt_kinect,
                                        Eigen::MatrixXf final_r_mat)
{
    Eigen::MatrixXf final_t_mat(3,1);
    final_t_mat = points_wrt_base - (final_r_mat*points_wrt_kinect);
    return final_t_mat;
}


void compute_txformation_matrix(Eigen::MatrixXf& points_wrt_base, Eigen::MatrixXf& points_wrt_kinect,
                                Eigen::MatrixXf& tx)
{
    Eigen::MatrixXf  final_r_mat(3,3), final_t_mat(3,1);

    final_r_mat = find_rotation_matrix(points_wrt_base, points_wrt_kinect);

    final_t_mat = find_translation_matrix(points_wrt_base.row(0).transpose(), points_wrt_kinect.row(0).transpose(), final_r_mat);

    tx.setZero();
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            tx(i,j)=final_r_mat(i,j);

    for(int i=0;i<3;i++)
        tx(i,3)=final_t_mat(i,0);

    tx(3,3)=1.0f;

}



template<typename T>
void MakeMatrix(T ** &temp,unsigned rows, unsigned cols)
{
    temp = new T*[rows];

    for(unsigned int i = 0; i < rows; ++i)
        temp[i] = new T[cols];
}

void LoadData_pose(const char *filename, long double ** &Weights)
{
    std::string line;
    std::ifstream file( filename );
    char temp;
    if(file.good())
    {
        for(unsigned int ii = 0;ii<NO_OF_POINTS_Marker;ii++)
        {
            for(unsigned int jj = 0;jj<NO_OF_POSE_VARIABLE;jj++)
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
        std::cout << "PRESS \n 0 for calibration \n 1 for calibration test \n \n";

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


            MakeMatrix<long double>(MarkerPoints_wrt_Base_actual,NO_OF_POINTS_Marker,NO_OF_POSE_VARIABLE); // Allocate memory for weights
            LoadData_pose("/home/ravi/apc17ws/current_pose1.txt",MarkerPoints_wrt_Base_actual);



            MakeMatrix<long double>(MarkerView_joint_angles,NO_OF_POINTS_Marker_View,NO_OF_JOINTS); // Allocate memory for weights
            LoadData_jts("/home/ravi/apc17ws/current_jts1.txt",MarkerView_joint_angles);
            for(unsigned int ii = 0;ii<NO_OF_POINTS_Marker_View;ii++)
            {
                for(unsigned int jj = 0;jj<NO_OF_JOINTS;jj++)
                {


                    std::cout<<MarkerView_joint_angles[ii][jj]<<"  "<<" "<<ii+1<<" "<< jj+1<<" "<<std::endl;

                }
                std::cout<<std::endl<<std::endl;
                //std::cin >> temp;
            }

            std::cout<<"press any key to move robot to start position"<<std::endl;
            //            fflush(stdin);
            //            getchar();

            apc17_controller::movej ur10_movej_start;
            //        double p0[6]={-0.094,-0.200,0.580,0.4,-2.416622,2.38};//if in pose
            double p0[6]={3.47565,-0.602627,-2.59696,-1.20724,1.56201,0.444069};//if in joint


            for(int i=0;i<6;i++)
                ur10_movej_start.request.joints.push_back(p0[i]);
            if(movejService.call(ur10_movej_start))
                std::cout << "Start intermediate position reached " << std::endl;
            else
                ROS_ERROR("Failed to call service");



            std::vector<double> points_wrt_kinect;
            std::vector<int> no_of_points_wrt_kinect;
            std::vector<tf::StampedTransform> view_transforms;

            for(int count=0;count<NO_OF_POINTS_Marker_View;count++)
            {


                signal(SIGINT, signal_callback_handler);

                int input;
                std::cout <<"\n Press any key for next marker view position"<< std::endl;
                                fflush(stdin);
                                getchar();

                std::cout<<"Moving towards Marker View : "<< count << std::endl;
                apc17_controller::movej next_marker;
                for(int i=0;i<6;i++)
                    next_marker.request.joints.push_back(MarkerView_joint_angles[count][i]);
                if(movejService.call(next_marker))
                    std::cout << "Marker View"<< count << "reached " << std::endl;
                else
                    ROS_ERROR("Failed to call service");
                ros::Duration(3).sleep();


                std::cout<<"kinect service"<<std::endl;
                apc17_controller::Kinect3D marker_point_kinect;

                tf::TransformListener transform_listener;
                tf::StampedTransform transform_kinect;


                //            transform_listener.waitForTransform("/base", "/wrist_3_link",  ros::Time(0), ros::Duration(3));
                //            transform_listener.lookupTransform("/base", "/wrist_3_link", ros::Time(0), transform_kinect);

                transform_listener.waitForTransform("/wrist_3_link" , "/base",   ros::Time(0), ros::Duration(3));
                transform_listener.lookupTransform( "/wrist_3_link" , "/base", ros::Time(0), transform_kinect);

                view_transforms.push_back(transform_kinect);

                if(Kinect3DService.call(marker_point_kinect))
                {

                    for(int i=0;i<marker_point_kinect.response.point_3D.data.size();i++)
                        points_wrt_kinect.push_back(marker_point_kinect.response.point_3D.data[i]);
                    no_of_points_wrt_kinect.push_back(marker_point_kinect.response.point_3D.data.size()/NO_OF_POSE_VARIABLE);

                    std::cout << "Kinect service done reached " << std::endl;
                }
                else
                    ROS_ERROR("Failed to call service");
            }


            Eigen::MatrixXf mtx_points_wrt_base(NO_OF_POINTS_Marker,NO_OF_POSE_VARIABLE);

            for(unsigned int ii = 0;ii<NO_OF_POINTS_Marker;ii++)
                for(unsigned int jj = 0;jj<NO_OF_POSE_VARIABLE;jj++)
                    mtx_points_wrt_base(ii,jj) = MarkerPoints_wrt_Base_actual[ii][jj];


            Eigen::MatrixXf mtx_points_wrt_kinect(NO_OF_POINTS_Marker,NO_OF_POSE_VARIABLE);

            for(int i=0;i<NO_OF_POINTS_Marker;i++)
                for(unsigned int j = 0;j<NO_OF_POSE_VARIABLE;j++)
                    mtx_points_wrt_kinect(i,j) = points_wrt_kinect[i*3+j];


            std::cout << mtx_points_wrt_base << "\n" << std::endl;
            std::cout << mtx_points_wrt_kinect << "\n" << std::endl;

            std::cout<< points_wrt_kinect.size()<< "\n" << std::endl;
            std::cout<< no_of_points_wrt_kinect.size() << "\n" << std::endl;


            Eigen::MatrixXf mtx_points_wrt_wrist(NO_OF_POINTS_Marker,NO_OF_POSE_VARIABLE);

            int no_of_points_processed=0;

            for (int i=0;i<view_transforms.size();i++)
            {
                for(int j=0;j<no_of_points_wrt_kinect[i];j++)
                {
                    tf::Vector3 point(mtx_points_wrt_base(no_of_points_processed,0),
                                      mtx_points_wrt_base(no_of_points_processed,1),
                                      mtx_points_wrt_base(no_of_points_processed,2));

                    point.m_floats[3] =1.0f;

                    tf::Vector3 txfrmd_point = view_transforms[i]*point;

                    mtx_points_wrt_wrist(no_of_points_processed,0)= txfrmd_point.m_floats[0];
                    mtx_points_wrt_wrist(no_of_points_processed,1)= txfrmd_point.m_floats[1];
                    mtx_points_wrt_wrist(no_of_points_processed,2)= txfrmd_point.m_floats[2];
                    no_of_points_processed++;
                }
            }

            std::cout << mtx_points_wrt_wrist << "\n" << std::endl;

            Eigen::MatrixXf tx(4,4);
            compute_txformation_matrix(mtx_points_wrt_wrist,mtx_points_wrt_kinect,tx);
            //        compute_txformation_matrix(mtx_points_wrt_kinect,mtx_points_wrt_wrist,tx);
            std::cout << " TRANSFORMATION MTX \n" <<  tx << "\n\n";



            pcl::PointCloud<pcl::PointXYZ> cloud_wrist;
            pcl::PointCloud<pcl::PointXYZ> cloud_kinect;

            for(int i=0;i<mtx_points_wrt_kinect.rows();i++)
            {
                pcl::PointXYZ kinect_point;
                kinect_point.x = mtx_points_wrt_kinect(i,0);
                kinect_point.y = mtx_points_wrt_kinect(i,1);
                kinect_point.z = mtx_points_wrt_kinect(i,2);

                pcl::PointXYZ wrist_point;
                wrist_point.x = mtx_points_wrt_wrist(i,0);
                wrist_point.y = mtx_points_wrt_wrist(i,1);
                wrist_point.z = mtx_points_wrt_wrist(i,2);

                cloud_wrist.push_back(wrist_point);
                cloud_kinect.push_back(kinect_point);
            }


            Eigen::Matrix4f pcl_tx;
            pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> estimate_transformation;

            estimate_transformation.estimateRigidTransformation(cloud_kinect,cloud_wrist,pcl_tx);
            //                estimate_transformation.estimateRigidTransformation(cloud_wrist,cloud_kinect,pcl_tx);


            std::cout << " PCL TRANSFORMATION\n" << pcl_tx << std::endl;
            // std::cout << " PCL TRANSFORMATION\n" << pcl_tx << std::endl;



            tf::TransformListener transform_listener;
            tf::StampedTransform transform_kinect;

            //        transform_listener.waitForTransform("/wrist_3_link", "/base" ,ros::Time(0), ros::Duration(3));
            //        transform_listener.lookupTransform("/wrist_3_link", "/base", ros::Time(0), transform_kinect);

            transform_listener.waitForTransform("/base", "/wrist_3_link", ros::Time(0), ros::Duration(3));
            transform_listener.lookupTransform("/base" , "/wrist_3_link", ros::Time(0), transform_kinect);

          float sum=0.0;
            for(int i=16;i<NO_OF_POINTS_Marker;i++)
            {
                int point_index = i;//points_wrt_kinect.size()/NO_OF_POSE_VARIABLE-1;

                Eigen::Vector4f point_kinect;
                point_kinect(0,0) = mtx_points_wrt_kinect(point_index,0);
                point_kinect(1,0) = mtx_points_wrt_kinect(point_index,1);
                point_kinect(2,0) = mtx_points_wrt_kinect(point_index,2);
                point_kinect(3,0) = 1.0f;

                Eigen::Vector4f txfrmd_wrt_wr2 = tx * point_kinect;
                //            Eigen::Vector4f txfrmd_wrt_wr2 = pcl_tx * point_kinect;

                //        tf::Vector3 point(0.0f,0.0f,0.0f);

                tf::Vector3 point(txfrmd_wrt_wr2(0,0),
                                  txfrmd_wrt_wr2(1,0),
                                  txfrmd_wrt_wr2(2,0));

                point.m_floats[3]=1.0f;

                tf::Vector3 txfrmd_point = transform_kinect*point;
                //        tf::Vector3 txfrmd_point = view_transforms[2]*point;


                std::cout << mtx_points_wrt_base(point_index,0) << " "
                          << mtx_points_wrt_base(point_index,1) << " "
                          << mtx_points_wrt_base(point_index,2) << " " << std::endl;


                std::cout << txfrmd_point.x() <<  " " << txfrmd_point.y() << " " << txfrmd_point.z() << std::endl;

                float dx = abs(mtx_points_wrt_base(point_index,0) - txfrmd_point.x());
                   float dy = abs(mtx_points_wrt_base(point_index,1) - txfrmd_point.y());
                   float dz = abs(mtx_points_wrt_base(point_index,2) - txfrmd_point.z());

                   float err= std::sqrt((dx*dx+dy*dy+dz*dz));
                   std::cout<<"error in catesian coord"<<" "<<err<<std::endl;
                    sum+=err;
            }
            sum/=7;

            std::cout<<"avg error"<<" "<<sum<<std::endl;


            std::ofstream transformation_file_pcl;

            transformation_file_pcl.open(file_name_pcl_tx);

            for(int i=0;i<4;i++)
            {
                for(int j=0;j<4;j++)
                    transformation_file_pcl << pcl_tx(i,j) << ",";
                transformation_file_pcl << "\n";
            }

            transformation_file_pcl.close();


            std::ofstream transformation_file_tx;

            transformation_file_tx.open(file_name_tx);

            for(int i=0;i<4;i++)
            {
                for(int j=0;j<4;j++)
                    transformation_file_tx << tx(i,j) << ",";
                transformation_file_tx << "\n";
            }

            transformation_file_tx.close();
        }
            break;

        case 1:
        {


            Eigen::Matrix4f pcl_kinect_T_wr3;
            Eigen::Matrix4f tx_kinect_T_wr3;

            std::ifstream transformation_file_pcl;

            transformation_file_pcl.open(file_name_pcl_tx,std::ios_base::in);

            for(int i=0;i<4;i++)
                for(int j=0;j<4;j++)
                {
                    std::string line;
                    getline(transformation_file_pcl, line, ',' );
                    pcl_kinect_T_wr3(i,j) = atof(line.c_str());

                }
            transformation_file_pcl.close();


            std::ifstream transformation_file_tx;

            transformation_file_tx.open(file_name_tx,std::ios_base::in);

            for(int i=0;i<4;i++)
                for(int j=0;j<4;j++)
                {

                    std::string line;
                    getline(transformation_file_tx, line, ',' );
                    tx_kinect_T_wr3(i,j) = atof(line.c_str());
                }

            transformation_file_tx.close();

            apc17_controller::movej ur10_movej_test;
            //        double p0[6]={-0.094,-0.200,0.580,0.4,-2.416622,2.38};//if in pose
            double p0[6]={DEG2RAD(260.78), DEG2RAD(-52.67), DEG2RAD(-142.94), DEG2RAD(-1), DEG2RAD(83.86), DEG2RAD(-0.3)};//if in joint


            std::cout << pcl_kinect_T_wr3 << std::endl;
            std::cout << tx_kinect_T_wr3 << std::endl;



            for(int i=0;i<6;i++)
                ur10_movej_test.request.joints.push_back(p0[i]);
            if(movejService.call(ur10_movej_test))
                std::cout << "test position reached " << std::endl;
            else
                ROS_ERROR("Failed to call service");




            apc17_controller::srv_calib_test calib_test;
            ROS_INFO("Sending service for calib test");

            if(service_test_point.call(calib_test))
            {
                std::cout << "POINT TO REACH is = ";



                for(int i=0;i<3;i++)
                    std::cout << calib_test.response.Point_3D.data[i] << " ";
                std::cout << "\n";

                Eigen::Vector4f point_kinect;
                point_kinect(0,0) = calib_test.response.Point_3D.data[0];
                point_kinect(1,0) = calib_test.response.Point_3D.data[1];
                point_kinect(2,0) = calib_test.response.Point_3D.data[2];
                point_kinect(3,0) = 1.0f;

                Eigen::Vector4f txfrmd_wrt_wr2 = pcl_kinect_T_wr3 * point_kinect;
                //            Eigen::Vector4f txfrmd_wrt_wr2 = pcl_tx * point_kinect;

                tf::Vector3 point(txfrmd_wrt_wr2(0,0),
                                  txfrmd_wrt_wr2(1,0),
                                  txfrmd_wrt_wr2(2,0));

                point.m_floats[3]=1.0f;

                tf::TransformListener transform_listener;
                tf::StampedTransform transform_kinect;

                //        transform_listener.waitForTransform("/wrist_3_link", "/base" ,ros::Time(0), ros::Duration(3));
                //        transform_listener.lookupTransform("/wrist_3_link", "/base", ros::Time(0), transform_kinect);

                transform_listener.waitForTransform("/base", "/wrist_3_link", ros::Time(0), ros::Duration(3));
                transform_listener.lookupTransform("/base" , "/wrist_3_link", ros::Time(0), transform_kinect);

                tf::Vector3 txfrmd_point = transform_kinect*point;
                //        tf::Vector3 txfrmd_point = view_transforms[2]*point;


                std::cout << txfrmd_point.x() <<  " " << txfrmd_point.y() << " " << txfrmd_point.z() << std::endl;


                txfrmd_wrt_wr2 = tx_kinect_T_wr3 * point_kinect;
                //            Eigen::Vector4f txfrmd_wrt_wr2 = pcl_tx * point_kinect;

                point.m_floats[0]=txfrmd_wrt_wr2(0,0);
                point.m_floats[1]=txfrmd_wrt_wr2(1,0);
                point.m_floats[2]=txfrmd_wrt_wr2(2,0);
                point.m_floats[3]=1.0f;


                txfrmd_point = transform_kinect*point;
                //        tf::Vector3 txfrmd_point = view_transforms[2]*point;


                std::cout << txfrmd_point.x() <<  " " << txfrmd_point.y() << " " << txfrmd_point.z() << std::endl;
                //MOVE THE ROBOT WITH MOTION PLANNING








                int num = 2000;
                double angle[num][6];

                double init_config[6] = {DEG2RAD(260.78), DEG2RAD(-52.67), DEG2RAD(-142.94), DEG2RAD(-1), DEG2RAD(83.86), DEG2RAD(-0.3)};
                //double pref_config[NL] = {DEG2RAD(-30), DEG2RAD(20), DEG2RAD(-80), 0, 0, 0}; // minimize angle norm
                double pref_config[6] = {DEG2RAD(-0.31), DEG2RAD(-50.8), DEG2RAD(-127.7), 0, DEG2RAD(92.5), DEG2RAD(21)}; // minimize angle norm

                double theta[6];


                double pose_t[6];
                pose_t[0]=txfrmd_point[0];
                pose_t[1]=txfrmd_point[1];
                pose_t[2]=txfrmd_point[2];
                pose_t[3]=0;
                pose_t[4]=0;
                pose_t[5]=0;



                double err = robot.ik_traj_withpose(init_config, pose_t, pref_config, angle, num);


                cout << "error = " <<  err << endl;

//                for(int i = 0; i < NW; ++i)
//                    f1 << pose_t[i] << "\t";
//                f1 << endl;

                double pose[NW];
                cv::Mat Jpos(NR,3,CV_64F,0.0);
                for(int cnt = 0; cnt < num; ++cnt)
                {
                    for(int i = 0; i < NL; ++i)
                    {
                        theta[i] = angle[cnt][i];
                    }


                }

            std::cout << "\nJoint Angle Computed (in Degrees) = " ;
            for(int i = 0; i < NL; i++)
                cout << RAD2DEG(theta[i]) << "\t";// $$$$$$$$$$$$$$$$$$$$$$$$          This is the output of IK    $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
            std::cout << endl;






            apc17_controller::movej ur10_movej_test1;
//        double p0[6]={-0.094,-0.200,0.580,0.4,-2.416622,2.38};//if in pose
//            double p01[6]={DEG2RAD(183.12), DEG2RAD(-118.8), DEG2RAD(-48.5), DEG2RAD(-10.8), DEG2RAD(-266.5), DEG2RAD(-0.3)};//if in joint


for(int i=0;i<6;i++)
    ur10_movej_test1.request.joints.push_back(theta[i]);
if(movejService.call(ur10_movej_test1))
    std::cout << "test position reached " << std::endl;
else
    ROS_ERROR("Failed to call service");

             }
            else
                ROS_ERROR("Failed to call service");










        }

        }

    }
    return 0;
}
