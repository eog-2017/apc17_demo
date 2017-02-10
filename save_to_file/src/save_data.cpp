#include<iostream>
#include<ros/ros.h>
#include <tf/transform_listener.h>


#include<signal.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <sensor_msgs/JointState.h>
#include<std_msgs/String.h>


#define pose 0
#define joints 1



std::vector<double> current_jts;
std::vector<double> current_pose;
using namespace std;
void signal_callback_handler(int signum)
{
    cout << "Caught Signal" << signum << endl;

    exit(0);
}

void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{



        current_jts.clear();

        for(int i=0;i<msg->position.size();i++)
        {

            current_jts.push_back(msg->position[i]);
        }


#if (joints)
        std::cout<<current_jts[0]<< "," << current_jts[1] << "," << current_jts[2]<< "," <<current_jts[3]<< "," << current_jts[4] << "," << current_jts[5]<< "," <<  std::endl;

#endif

    return;
}









void waitForEnter() {
    static std::string line;
    std::getline(std::cin, line);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"save_data");
    ros::NodeHandle nh;
    signal(SIGINT, signal_callback_handler);



    std::ofstream myfile;

#if (pose)

    myfile.open ("/home/ravi/Desktop/current_pose.txt");

#endif

#if (joints)
    myfile.open ("/home/ravi/Desktop/record_jts.txt");

#endif


    ros::Subscriber sub = nh.subscribe("/joint_states", 1, jointStateCallback);
    std::cout<<"Press [Enter] to start displaying joint positions.\n"<<std::endl;
        int i;
        for(i=0; i<25;i++)
        {
        waitForEnter();
        {
            ros::spinOnce();





            tf::TransformListener transform_listener;
            tf::StampedTransform transform_ee;


            transform_listener.waitForTransform("/base", "/point", ros::Time(0), ros::Duration(3));
            transform_listener.lookupTransform("/base" , "/point", ros::Time(0), transform_ee);

           std::cout<<transform_ee.getOrigin().x()<<" "<<transform_ee.getOrigin().y()<<" "<<transform_ee.getOrigin().z()<<" "<<std::endl;





#if (pose)

            myfile <<transform_ee.getOrigin().x()<< "," << transform_ee.getOrigin().y()<< "," << transform_ee.getOrigin().z()<< "," << std::endl;

#endif

#if (joints)
            myfile <<current_jts[0]<< "," << current_jts[1] << "," << current_jts[2]<< "," <<current_jts[3]<< "," << current_jts[4] << "," << current_jts[5]<< "," <<  std::endl;

#endif
//            std::cout <<current_pose[0]<< "\t" << current_pose[1] << "\t" << current_pose[2] <<'\n';

        }
        }

      myfile.close();
//        ros::MultiThreadedSpinner spinner(4);

//        spinner.spin();


return 0;
}
