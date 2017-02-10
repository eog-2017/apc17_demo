/* Forward Kinematics: We use the transformation matrix given in Vidyasagar's book
 *
 *
 * */

#include </home/ravi/apc17ws/src/apc17_demo/ur10_motion/include/ur5model-inl.h>
#include<cmath>

using namespace std;
using namespace cv;
using namespace MODUR5;


//----------------------------
inline bool UR5::exist(const std::string& name)
{
    std::ifstream file(name.c_str());
    if(!file)            // If the file was not found, then file is 0, i.e. !file=1 or true.
        return false;    // The file was not found.
    else                 // If the file was found, then file is non-0.
        return true;     // The file was found.
}
//-------------------------------
UR5::UR5()
{


    theta_max = std::vector<double>{M_PI, M_PI, M_PI, M_PI, M_PI, M_PI};
    theta_min = std::vector<double>{-M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI};
    c_min = std::vector<double>{-0.8, -0.8, -0.5};
    c_max = std::vector<double>{0.8, 0.8, 0.8};

    offset = std::vector<double>{0, -M_PI/2.0, 0, -M_PI/2.0, 0, 0};



    J  = cv::Mat(NW, NL, CV_64F, 0.0);
    Jg  = cv::Mat(NW, NL, CV_64F, 0.0);
    Jp = cv::Mat(NC,NL,CV_64F, 0.0);
    Jo = cv::Mat(NC,NL,CV_64F, 0.0);
    Jw = cv::Mat(NC,NL,CV_64F, 0.0);
    Jv = cv::Mat(NC, NL, CV_64F, 0.0);
    R = cv::Mat(NC, NC, CV_64F, 0.0);

}
//---------------------------------------
void UR5::set_joint_angles(const double Th[])
{
    for(int i = 0; i < NL; i++)
        theta[i+1] = Th[i];
}
//------------------------------------------
void UR5::get_joint_angles(double Th[])
{
    for(int i = 0; i < NL; i++)
        Th[i] = theta[i+1];

}
void UR5::get_ee_pose(double xp[])
{
    for(int i = 0; i < NW; i++)
        xp[i] = pose[i];
}
void UR5::get_jacobian(cv::Mat &Jac)
{
    J.copyTo(Jac);
}

//--------------------------------------------------------
// Forward kinematics
// Th - Joint angle values in radians
// x - Cartesian position of end-effector in meters
void UR5::fwd_kin_posn(double xef[])
{

    // theta index starts from 1


  posn[0] = (sin(theta[1])*cos(theta[5])-((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+
          (cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*sin(theta[5]))*d6+(
          (cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-
          (-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5+sin(theta[1])*d4-cos(theta[1])*
          sin(theta[2])*sin(theta[3])*a3+cos(theta[1])*cos(theta[2])*cos(theta[3])*a3+cos(theta[1])*cos(theta[2])*a2;

  posn[1] = (-((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+
          (sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*sin(theta[5])-cos(theta[1])*cos(theta[5]))*d6+(
          (sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-
          (-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5-cos(theta[1])*d4-sin(theta[1])*sin(theta[2])
          *sin(theta[3])*a3+sin(theta[1])*cos(theta[2])*cos(theta[3])*a3+sin(theta[1])*cos(theta[2])*a2;

  posn[2] = -
              ((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*sin(theta[4])+(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*cos(theta[4]))*
              sin(theta[5])*d6+
              ((cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*sin(theta[4])-(cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*cos(theta[4]))*d5
              +d1+cos(theta[2])*sin(theta[3])*a3+sin(theta[2])*cos(theta[3])*a3+sin(theta[2])*a2;


  if(xef  != NULL)
  {
      for(int i = 0; i < NC; i++)
          xef[i] = posn[i];
  }

}

//====================================================


//-----------------------------------------------
// Gives the position of various joints
// Theta starts with index 1
// X should have 6 rows for UR5
void UR5::joint_position(cv::Mat &X)
{


    double t1[4][4] = { { cos(theta[1]),0,sin(theta[1]),0},
                        {sin(theta[1]),0,-cos(theta[1]),0},
                        {0,1,0,d1},{0,0,0,1} };


    double t2[4][4] = { {cos(theta[2]),-sin(theta[2]),0,cos(theta[2])*a2},
                        {sin(theta[2]),cos(theta[2]),0,sin(theta[2])*a2},
                        {0,0,1,0},{0,0,0,1} };

    double t3[4][4] = { {cos(theta[3]),-sin(theta[3]),0,cos(theta[3])*a3},
                        {sin(theta[3]),cos(theta[3]),0,sin(theta[3])*a3},
                        {0,0,1,0},{0,0,0,1} };

    double t4[4][4] = { { cos(theta[4]),0,sin(theta[4]),0},
                        {sin(theta[4]),0,-cos(theta[4]),0},
                        {0,1,0,d4},{0,0,0,1} };


    double t5[4][4] = { { cos(theta[5]),0,-sin(theta[5]),0},
                        {sin(theta[5]),0,cos(theta[5]),0},
                        {0,-1,0,d5},{0,0,0,1} };

    double t6[4][4] = { {cos(theta[6]),-sin(theta[6]),0,0},
                        {sin(theta[6]),cos(theta[6]),0,0},
                        {0,0,1,d6},{0,0,0,1} };


    Mat T1 = Mat(4, 4, CV_64F, t1);
    Mat T2 = Mat(4, 4, CV_64F, t2);
    Mat T3 = Mat(4, 4, CV_64F, t3);
    Mat T4 = Mat(4, 4, CV_64F, t4);
    Mat T5 = Mat(4, 4, CV_64F, t5);
    Mat T6 = Mat(4, 4, CV_64F, t6);

    if(X.rows < NR || X.cols < NC)
    {
        cerr << "Improper dimension for X. Use a 7x3 matrix. Exiting ..." << endl;
        cerr << "Error at Location:" << __FILE__ <<":" << __LINE__ << endl;
        exit(-1);
    }
    else
    {
        //This is base of the robot
        for(int i = 0; i < NC; i++)
            X.at<double>(0,i) = 0.0;

        Mat TF1 = T1;
        X.row(1) = TF1.col(3).rowRange(0,3).t();

        Mat TF2 = TF1*T2;
        X.row(2) = TF2.col(3).rowRange(0,3).t();

        Mat TF3 = TF2*T3;
        X.row(3) = TF3.col(3).rowRange(0,3).t();

        Mat TF4 = TF3*T4;
        X.row(4) = TF4.col(3).rowRange(0,3).t();

        Mat TF5 = TF4*T5;
        X.row(5) = TF5.col(3).rowRange(0,3).t();

        Mat TF6 = TF5*T6;
        X.row(6) = TF6.col(3).rowRange(0,3).t();
    }
}
//=============================================================
// Computes the 3x3 rotation matrix for the end-effector coordinate system wrt the base frame
// Input: joint angle vector: Th[7]
// Output: Roatation matrix: R[3][3]
//----------------------------------------------

void UR5::rotation_matrix(cv::Mat &RM)
{

    R.at<double>(0,0) = ((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4])-(cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4]))*sin(theta[6])
            +(sin(theta[1])*sin(theta[5])+
            ((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*cos(theta[5]))*
            cos(theta[6]);



    R.at<double>(0,1) = ((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4])-(cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4]))*cos(theta[6])
            -(sin(theta[1])*sin(theta[5])+
            ((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*cos(theta[5]))*
            sin(theta[6]);


    R.at<double>(0,2) = sin(theta[1])*cos(theta[5])-
                ((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*sin(theta[5]);

    R.at<double>(1,0) = ((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4])-(sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4]))*sin(theta[6])
            +(((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*cos(theta[5])-
            cos(theta[1])*sin(theta[5]))*cos(theta[6]);


    R.at<double>(1,1) = ((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4])-(sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4]))*cos(theta[6])
            -(((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*cos(theta[5])-
            cos(theta[1])*sin(theta[5]))*sin(theta[6]);


    R.at<double>(1,2) = -((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*
            sin(theta[5])-cos(theta[1])*cos(theta[5]);


    R.at<double>(2,0) = ((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*cos(theta[4])-(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*sin(theta[4]))*sin(theta[6])+
            ((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*sin(theta[4])+(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*cos(theta[4]))*cos(theta[5])*cos(theta[6]);


    R.at<double>(2,1) = ((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*cos(theta[4])-(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*sin(theta[4]))*cos(theta[6])-
                ((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*sin(theta[4])+(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*cos(theta[4]))*cos(theta[5])*sin(theta[6]);


    R.at<double>(2,2) = -((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*sin(theta[4])+(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*cos(theta[4]))*sin(theta[5]);

    if(&RM != NULL)
    {
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 3; j++)
                RM.at<double>(i,j) = R.at<double>(i,j);
    }

}

//==========================================================

// UR5 forward kinematics that returns the position and
// orientation of the end-effector
// Input: Joint angle vector: theta[7]
// Output: end-effector pose: xpose[6];
// Orientation is in terms of X-Y-Z fixed angles (gamma -->roll, beta --> pitch, alpha --> Yaw)
//---------------------------------------------
void UR5::fwd_kin_pose(double xp[])
{
    pose[0] = (sin(theta[1])*cos(theta[5])-
                ((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*sin(theta[5]))*d6+
                ((cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5+sin(theta[1])*d4-
                cos(theta[1])*sin(theta[2])*sin(theta[3])*a3+cos(theta[1])*cos(theta[2])*cos(theta[3])*a3+cos(theta[1])*cos(theta[2])*a2;

    pose[1] = (-((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*
            sin(theta[5])-cos(theta[1])*cos(theta[5]))*d6+
            ((sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5-cos(theta[1])*d4-
            sin(theta[1])*sin(theta[2])*sin(theta[3])*a3+sin(theta[1])*cos(theta[2])*cos(theta[3])*a3+sin(theta[1])*cos(theta[2])*a2;

    pose[2] = -((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*sin(theta[4])+(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*cos(theta[4]))*sin(theta[5])*d6+
            ((cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*sin(theta[4])-(cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*cos(theta[4]))*d5+d1+cos(theta[2])*sin(theta[3])*a3+sin(theta[2])*cos(theta[3])*a3+
            sin(theta[2])*a2;

    rotation_matrix(); // Pass the original Th[] variable.

    // Beta - Pitch - Rotation about Y-axis. consider the positive value of the sqrt
    double Beta = atan2(-R.at<double>(2,0), fabs(sqrt(pow(R.at<double>(0,0), 2) + pow(R.at<double>(1,0), 2.0) )) );

    double Alpha, Gamma;



    if (Beta == M_PI/2.0)
    {
        Alpha = 0.0;  //alpha
        Gamma = atan2(R.at<double>(0,1), R.at<double>(1,1)); // gamma
    }
    else if(Beta == -M_PI/2.0)
    {
        Alpha = 0.0;
        Gamma = -atan2(R.at<double>(0,1), R.at<double>(1,1));
    }
    else
    {
        // Alpha - Roll- Rotation about z-axis
        Alpha = atan2(R.at<double>(1,0)/cos(pose[3]), R.at<double>(0,0)/cos(pose[3]));

        // Gamma - Yaw - Rotation about X-axis
        Gamma = atan2(R.at<double>(2,1)/cos(pose[3]), R.at<double>(2,2)/cos(pose[3]));
    }

    pose[3] = Gamma;  // Rx
    pose[4] = Beta;   // Ry
    pose[5] = Alpha;  // Rz

    // Roll-Pitch-Yaw Angles

    rpy[0] = Alpha; // Alpha - Roll - Rotation about z-axis
    rpy[1] = Beta; // Beta - Pitch - Rotation about y-axis
    rpy[2] = Gamma; // Gamma - Yaw - Rotation about x-axis

    posn[0] = pose[0];
    posn[1] = pose[1];
    posn[2] = pose[2];

    // Rotation about x, y and z axes

    rxyz[0] = Gamma;  //Rx
    rxyz[1] = Beta;   //Ry
    rxyz[2] = Alpha;  //Rz

    if(xp != NULL)
    {
        for(int i = 0; i < NW; i++)
            xp[i] = pose[i];
    }
}
//=========================================
// Generates input-output data for the manipulator
void UR5::generate_data(double Uc[], double Th[])
{
    do
    {
        for(int i = 0; i < NL; i++)
        {
            theta[i+1] = theta_min[i] + (theta_max[i] - theta_min[i]) *
                    (rand()/(double)RAND_MAX);
            Th[i] = theta[i+1];
        }

        //So that Ut is in the workspace of robot
        fwd_kin_posn();

        for(int i = 0; i < NC; i++)
            Uc[i] = posn[i];

        if( (Uc[0] >= c_min[0]) && (Uc[0] < c_max[0]) &&
                (Uc[1] >= c_min[1]) && (Uc[1] < c_max[1]) &&
                (Uc[2] >= c_min[2]) && (Uc[2] < c_max[2]) )
            break;
    }while(1);
}


//============================================================

// Generates input-output data for the manipulator
// Input: Joint angles
// Output: End-effector pose in 6D
void UR5::generate_pose_data(double Uc[], double Th[])
{
    do
    {
        for(int i = 0; i < NL; i++)
        {
           theta[i+1] = theta_min[i] + (theta_max[i] - theta_min[i]) *
                    (rand()/(double)RAND_MAX);
           Th[i] = theta[i+1];
        }

        // find the end-effector pose for a given theta vector
        // forward kinematics
        fwd_kin_pose();
        for(int i = 0; i < NW; i++)
            Uc[i] = pose[i];


        if( (Uc[0] >= c_min[0]) && (Uc[0] < c_max[0]) &&
                (Uc[1] >= c_min[1]) && (Uc[1] < c_max[1]) &&
                (Uc[2] >= c_min[2]) && (Uc[2] < c_max[2]) )
            break;

    }while(1);
}

//=================================================================
// Jp : dX /d theta : 3x7
// input: current joint angle vector: theta (7x1)
void UR5::position_jacobian(cv::Mat &Jposn)
{



   Jp.at<double>(0,0) = (cos(theta[1])*cos(theta[5])-
               ((sin(theta[1])*cos(theta[2])*sin(theta[3])+sin(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(sin(theta[1])*sin(theta[2])*sin(theta[3])-sin(theta[1])*cos(theta[2])*cos(theta[3]))*cos(theta[4]))*sin(theta[5]))*d6+
               ((sin(theta[1])*sin(theta[2])*sin(theta[3])-sin(theta[1])*cos(theta[2])*cos(theta[3]))*sin(theta[4])-(sin(theta[1])*cos(theta[2])*sin(theta[3])+sin(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5+cos(theta[1])*d4+
               sin(theta[1])*sin(theta[2])*sin(theta[3])*a3-sin(theta[1])*cos(theta[2])*cos(theta[3])*a3-sin(theta[1])*cos(theta[2])*a2;



   Jp.at<double>(0,1) = -((cos(theta[1])*sin(theta[2])*sin(theta[3])-cos(theta[1])*cos(theta[2])*cos(theta[3]))*sin(theta[4])+(-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*
           sin(theta[5])*d6+((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])-(cos(theta[1])*sin(theta[2])*sin(theta[3])-cos(theta[1])*cos(theta[2])*cos(theta[3]))*cos(theta[4]))*d5-
           cos(theta[1])*cos(theta[2])*sin(theta[3])*a3-cos(theta[1])*sin(theta[2])*cos(theta[3])*a3-cos(theta[1])*sin(theta[2])*a2;



   Jp.at<double>(0,2) = -((cos(theta[1])*sin(theta[2])*sin(theta[3])-cos(theta[1])*cos(theta[2])*cos(theta[3]))*sin(theta[4])+(-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*
           sin(theta[5])*d6+((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])-(cos(theta[1])*sin(theta[2])*sin(theta[3])-cos(theta[1])*cos(theta[2])*cos(theta[3]))*cos(theta[4]))*d5-
           cos(theta[1])*cos(theta[2])*sin(theta[3])*a3-cos(theta[1])*sin(theta[2])*cos(theta[3])*a3;


   Jp.at<double>(0,3) = ((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*d5-
               ((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4])-(cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4]))*sin(theta[5])*d6;

   Jp.at<double>(0,4) = (-sin(theta[1])*sin(theta[5])-
               ((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*cos(theta[5]))*d6;

   Jp.at<double>(0,5) = 0.0;


   //-----------------

   Jp.at<double>(1,0) = (sin(theta[1])*cos(theta[5])-
               ((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*sin(theta[5]))*d6+
               ((cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5+sin(theta[1])*d4-
               cos(theta[1])*sin(theta[2])*sin(theta[3])*a3+cos(theta[1])*cos(theta[2])*cos(theta[3])*a3+cos(theta[1])*cos(theta[2])*a2;

   Jp.at<double>(1,1) = -((sin(theta[1])*sin(theta[2])*sin(theta[3])-sin(theta[1])*cos(theta[2])*cos(theta[3]))*sin(theta[4])+(-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*
           sin(theta[5])*d6+((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])-(sin(theta[1])*sin(theta[2])*sin(theta[3])-sin(theta[1])*cos(theta[2])*cos(theta[3]))*cos(theta[4]))*d5-
           sin(theta[1])*cos(theta[2])*sin(theta[3])*a3-sin(theta[1])*sin(theta[2])*cos(theta[3])*a3-sin(theta[1])*sin(theta[2])*a2;

   Jp.at<double>(1,2) =  -((sin(theta[1])*sin(theta[2])*sin(theta[3])-sin(theta[1])*cos(theta[2])*cos(theta[3]))*sin(theta[4])+(-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*
           sin(theta[5])*d6+((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])-(sin(theta[1])*sin(theta[2])*sin(theta[3])-sin(theta[1])*cos(theta[2])*cos(theta[3]))*cos(theta[4]))*d5-
           sin(theta[1])*cos(theta[2])*sin(theta[3])*a3-sin(theta[1])*sin(theta[2])*cos(theta[3])*a3;

   Jp.at<double>(1,3) = ((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*d5-
               ((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4])-(sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4]))*sin(theta[5])*d6;


   Jp.at<double>(1,4) = (cos(theta[1])*sin(theta[5])-
               ((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*cos(theta[5]))*d6;

   Jp.at<double>(1,5) = 0;

   //-----------------------------


   Jp.at<double>(2,0) = 0.0;

   Jp.at<double>(2,1) =  -((-cos(theta[2])*sin(theta[3])-sin(theta[2])*cos(theta[3]))*sin(theta[4])+(cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*cos(theta[4]))*sin(theta[5])*d6+
           ((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-cos(theta[2])*sin(theta[3])-sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5-sin(theta[2])*sin(theta[3])*a3+cos(theta[2])*cos(theta[3])*a3+
           cos(theta[2])*a2;

   Jp.at<double>(2,2) = -((-cos(theta[2])*sin(theta[3])-sin(theta[2])*cos(theta[3]))*sin(theta[4])+(cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*cos(theta[4]))*sin(theta[5])*d6+
           ((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-cos(theta[2])*sin(theta[3])-sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5-sin(theta[2])*sin(theta[3])*a3+cos(theta[2])*cos(theta[3])*a3;

   Jp.at<double>(2,3) = ((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*sin(theta[4])+(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5-
               ((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*cos(theta[4])-(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*sin(theta[4]))*sin(theta[5])*d6;

   Jp.at<double>(2,4) = -((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*sin(theta[4])+(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*cos(theta[4]))*cos(theta[5])*d6;

   Jp.at<double>(2,5) = 0.0;


   if(&Jposn != NULL)
   {
       for(int i = 0; i < NC; i++)
           for(int j = 0; j < NL; j++)
               Jposn.at<double>(i,j) = Jp.at<double>(i,j);
   }
}

//------------------------------------------------

// This is obtained by using Euler Angle representation

void UR5::orientation_jacobian(cv::Mat &Jort)
{


    // Rotation Matrix
    rotation_matrix();

    double r21 = R.at<double>(1,0);
    double r11 = R.at<double>(0,0);
    double r31 = R.at<double>(2,0);
    double r32 = R.at<double>(2,1);
    double r33 = R.at<double>(2,2);
    double r22 = R.at<double>(1,1);
    double r12 = R.at<double>(0,1);
    double r13 = R.at<double>(0,2);
    double r23 = R.at<double>(1,2);


    Jo.at<double>(0,0) = 0.0;

    Jo.at<double>(0,1) = ((8*sin(theta[5]+theta[4]+theta[3]+theta[2])+8*sin(theta[5]-theta[4]-theta[3]-theta[2]))*r32-
                sin(theta[6]+2*theta[5]+2*theta[4]+2*theta[3]+2*theta[2])+sin(theta[6]+2*theta[5]-2*theta[4]-2*theta[3]-2*theta[2])-2*
                sin(theta[6]+theta[5]+2*theta[4]+2*theta[3]+2*theta[2])-2*sin(theta[6]+theta[5]-2*theta[4]-2*theta[3]-2*theta[2])+4*sin(theta[6]+theta[5])+2*
                sin(theta[6]-theta[5]+2*theta[4]+2*theta[3]+2*theta[2])+2*sin(theta[6]-theta[5]-2*theta[4]-2*theta[3]-2*theta[2])-4*sin(theta[6]-theta[5])+
                sin(theta[6]-2*theta[5]+2*theta[4]+2*theta[3]+2*theta[2])-sin(theta[6]-2*theta[5]-2*theta[4]-2*theta[3]-2*theta[2]))*pow(16*pow(r32,2)+2*
                cos(2*theta[5]+2*theta[4]+2*theta[3]+2*theta[2])+2*cos(2*theta[5]-2*theta[4]-2*theta[3]-2*theta[2])-4*cos(2*theta[5])-4*
                cos(2*theta[4]+2*theta[3]+2*theta[2])+4,-1);





    Jo.at<double>(0,2) = ((8*sin(theta[5]+theta[4]+theta[3]+theta[2])+8*sin(theta[5]-theta[4]-theta[3]-theta[2]))*r32-
                sin(theta[6]+2*theta[5]+2*theta[4]+2*theta[3]+2*theta[2])+sin(theta[6]+2*theta[5]-2*theta[4]-2*theta[3]-2*theta[2])-2*
                sin(theta[6]+theta[5]+2*theta[4]+2*theta[3]+2*theta[2])-2*sin(theta[6]+theta[5]-2*theta[4]-2*theta[3]-2*theta[2])+4*sin(theta[6]+theta[5])+2*
                sin(theta[6]-theta[5]+2*theta[4]+2*theta[3]+2*theta[2])+2*sin(theta[6]-theta[5]-2*theta[4]-2*theta[3]-2*theta[2])-4*sin(theta[6]-theta[5])+
                sin(theta[6]-2*theta[5]+2*theta[4]+2*theta[3]+2*theta[2])-sin(theta[6]-2*theta[5]-2*theta[4]-2*theta[3]-2*theta[2]))*pow(16*pow(r32,2)+2*
                cos(2*theta[5]+2*theta[4]+2*theta[3]+2*theta[2])+2*cos(2*theta[5]-2*theta[4]-2*theta[3]-2*theta[2])-4*cos(2*theta[5])-4*
                cos(2*theta[4]+2*theta[3]+2*theta[2])+4,-1);





    Jo.at<double>(0,3) = ((8*sin(theta[5]+theta[4]+theta[3]+theta[2])+8*sin(theta[5]-theta[4]-theta[3]-theta[2]))*r32-
                sin(theta[6]+2*theta[5]+2*theta[4]+2*theta[3]+2*theta[2])+sin(theta[6]+2*theta[5]-2*theta[4]-2*theta[3]-2*theta[2])-2*
                sin(theta[6]+theta[5]+2*theta[4]+2*theta[3]+2*theta[2])-2*sin(theta[6]+theta[5]-2*theta[4]-2*theta[3]-2*theta[2])+4*sin(theta[6]+theta[5])+2*
                sin(theta[6]-theta[5]+2*theta[4]+2*theta[3]+2*theta[2])+2*sin(theta[6]-theta[5]-2*theta[4]-2*theta[3]-2*theta[2])-4*sin(theta[6]-theta[5])+
                sin(theta[6]-2*theta[5]+2*theta[4]+2*theta[3]+2*theta[2])-sin(theta[6]-2*theta[5]-2*theta[4]-2*theta[3]-2*theta[2]))*pow(16*pow(r32,2)+2*
                cos(2*theta[5]+2*theta[4]+2*theta[3]+2*theta[2])+2*cos(2*theta[5]-2*theta[4]-2*theta[3]-2*theta[2])-4*cos(2*theta[5])-4*
                cos(2*theta[4]+2*theta[3]+2*theta[2])+4,-1);






    Jo.at<double>(0,4) = ((8*sin(theta[5]+theta[4]+theta[3]+theta[2])-8*sin(theta[5]-theta[4]-theta[3]-theta[2]))*r32-
                sin(theta[6]+2*theta[5]+2*theta[4]+2*theta[3]+2*theta[2])-sin(theta[6]+2*theta[5]-2*theta[4]-2*theta[3]-2*theta[2])+2*sin(theta[6]+2*theta[5])-
                sin(theta[6]-2*theta[5]+2*theta[4]+2*theta[3]+2*theta[2])-sin(theta[6]-2*theta[5]-2*theta[4]-2*theta[3]-2*theta[2])+2*sin(theta[6]-2*theta[5])+2*
                sin(theta[6]+2*theta[4]+2*theta[3]+2*theta[2])+2*sin(theta[6]-2*theta[4]-2*theta[3]-2*theta[2])-4*sin(theta[6]))*pow(16*pow(r32,2)+2*
                cos(2*theta[5]+2*theta[4]+2*theta[3]+2*theta[2])+2*cos(2*theta[5]-2*theta[4]-2*theta[3]-2*theta[2])-4*cos(2*theta[5])-4*
                cos(2*theta[4]+2*theta[3]+2*theta[2])+4,-1);




    Jo.at<double>(0,5) = -(sin(theta[6]+2*theta[5]+2*theta[4]+2*theta[3]+2*theta[2])+sin(theta[6]+2*theta[5]-2*theta[4]-2*theta[3]-2*theta[2])-2*
                                                                                                                                               sin(theta[6]+2*theta[5])+2*sin(theta[6]+theta[5]+2*theta[4]+2*theta[3]+2*theta[2])-2*sin(theta[6]+theta[5]-2*theta[4]-2*theta[3]-2*theta[2])-2*
                                                                                                                                               sin(theta[6]-theta[5]+2*theta[4]+2*theta[3]+2*theta[2])+2*sin(theta[6]-theta[5]-2*theta[4]-2*theta[3]-2*theta[2])-
                                                                                                                                               sin(theta[6]-2*theta[5]+2*theta[4]+2*theta[3]+2*theta[2])-sin(theta[6]-2*theta[5]-2*theta[4]-2*theta[3]-2*theta[2])+2*sin(theta[6]-2*theta[5]))*pow(16*
                                                                                                                                               pow(r32,2)+2*cos(2*theta[5]+2*theta[4]+2*theta[3]+2*theta[2])+2*cos(2*theta[5]-2*theta[4]-2*theta[3]-2*theta[2])-4*cos(2*theta[5])-4*
                                                                                                                                               cos(2*theta[4]+2*theta[3]+2*theta[2])+4,-1);




    //------------------

    Jo.at<double>(1,0) = (8*r11*r21+(-sin(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]+theta[1])+sin(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-
                sin(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+sin(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+2*sin(theta[6]+theta[5]+theta[1])
                +2*sin(theta[6]+theta[5]-theta[1])-sin(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]+theta[1])+
                sin(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-sin(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+
                sin(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]-theta[1])-2*sin(theta[6]-theta[5]+theta[1])-2*sin(theta[6]-theta[5]-theta[1])-2*
                sin(theta[6]+theta[4]+theta[3]+theta[2]+theta[1])+2*sin(theta[6]+theta[4]+theta[3]+theta[2]-theta[1])+2*
                sin(theta[6]-theta[4]-theta[3]-theta[2]+theta[1])-2*sin(theta[6]-theta[4]-theta[3]-theta[2]-theta[1]))*r11)*pow(pow(r21,2)+pow(r11,2),1/2)*r31*
                pow((8*pow(r21,2)+8*pow(r11,2))*pow(r31,2)+8*pow(r21,4)+16*pow(r11,2)*pow(r21,2)+8*pow(r11,4),-1);



    Jo.at<double>(1,1) = pow(pow(r21,2)+pow(r11,2),1/2)*(((cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-
                cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+
                cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-
                cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+
                cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+2*cos(theta[6]+theta[4]+theta[3]+theta[2]+theta[1])-2*
                cos(theta[6]+theta[4]+theta[3]+theta[2]-theta[1])+2*cos(theta[6]-theta[4]-theta[3]-theta[2]+theta[1])-2*
                cos(theta[6]-theta[4]-theta[3]-theta[2]-theta[1]))*r21+(-sin(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-
                sin(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]-theta[1])+sin(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+
                sin(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]-theta[1])-sin(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-
                sin(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]-theta[1])+sin(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+
                sin(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]-theta[1])-2*sin(theta[6]+theta[4]+theta[3]+theta[2]+theta[1])-2*
                sin(theta[6]+theta[4]+theta[3]+theta[2]-theta[1])-2*sin(theta[6]-theta[4]-theta[3]-theta[2]+theta[1])-2*
                sin(theta[6]-theta[4]-theta[3]-theta[2]-theta[1]))*r11)*r31+(-2*cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2])-2*
                cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2])-2*cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2])-2*
                cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2])-4*cos(theta[6]+theta[4]+theta[3]+theta[2])+4*cos(theta[6]-theta[4]-theta[3]-theta[2]))*pow(r21,2)+
                (-2*cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2])-2*cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2])-2*
                cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2])-2*cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2])-4*cos(theta[6]+theta[4]+theta[3]+theta[2])+4*
                cos(theta[6]-theta[4]-theta[3]-theta[2]))*pow(r11,2))*
                pow((8*pow(r21,2)+8*pow(r11,2))*pow(r31,2)+8*pow(r21,4)+16*pow(r11,2)*pow(r21,2)+8*pow(r11,4),-1);





    Jo.at<double>(1,2) = pow(pow(r21,2)+pow(r11,2),1/2)*(((cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-
                cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+
                cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-
                cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+
                cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+2*cos(theta[6]+theta[4]+theta[3]+theta[2]+theta[1])-2*
                cos(theta[6]+theta[4]+theta[3]+theta[2]-theta[1])+2*cos(theta[6]-theta[4]-theta[3]-theta[2]+theta[1])-2*
                cos(theta[6]-theta[4]-theta[3]-theta[2]-theta[1]))*r21+(-sin(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-
                sin(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]-theta[1])+sin(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+
                sin(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]-theta[1])-sin(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-
                sin(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]-theta[1])+sin(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+
                sin(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]-theta[1])-2*sin(theta[6]+theta[4]+theta[3]+theta[2]+theta[1])-2*
                sin(theta[6]+theta[4]+theta[3]+theta[2]-theta[1])-2*sin(theta[6]-theta[4]-theta[3]-theta[2]+theta[1])-2*
                sin(theta[6]-theta[4]-theta[3]-theta[2]-theta[1]))*r11)*r31+(-2*cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2])-2*
                cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2])-2*cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2])-2*
                cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2])-4*cos(theta[6]+theta[4]+theta[3]+theta[2])+4*cos(theta[6]-theta[4]-theta[3]-theta[2]))*pow(r21,2)+
                (-2*cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2])-2*cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2])-2*
                cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2])-2*cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2])-4*cos(theta[6]+theta[4]+theta[3]+theta[2])+4*
                cos(theta[6]-theta[4]-theta[3]-theta[2]))*pow(r11,2))*
                pow((8*pow(r21,2)+8*pow(r11,2))*pow(r31,2)+8*pow(r21,4)+16*pow(r11,2)*pow(r21,2)+8*pow(r11,4),-1);



    Jo.at<double>(1,3) = pow(pow(r21,2)+pow(r11,2),1/2)*(((cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-
                cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+
                cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-
                cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+
                cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+2*cos(theta[6]+theta[4]+theta[3]+theta[2]+theta[1])-2*
                cos(theta[6]+theta[4]+theta[3]+theta[2]-theta[1])+2*cos(theta[6]-theta[4]-theta[3]-theta[2]+theta[1])-2*
                cos(theta[6]-theta[4]-theta[3]-theta[2]-theta[1]))*r21+(-sin(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-
                sin(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]-theta[1])+sin(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+
                sin(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]-theta[1])-sin(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-
                sin(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]-theta[1])+sin(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+
                sin(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]-theta[1])-2*sin(theta[6]+theta[4]+theta[3]+theta[2]+theta[1])-2*
                sin(theta[6]+theta[4]+theta[3]+theta[2]-theta[1])-2*sin(theta[6]-theta[4]-theta[3]-theta[2]+theta[1])-2*
                sin(theta[6]-theta[4]-theta[3]-theta[2]-theta[1]))*r11)*r31+(-2*cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2])-2*
                cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2])-2*cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2])-2*
                cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2])-4*cos(theta[6]+theta[4]+theta[3]+theta[2])+4*cos(theta[6]-theta[4]-theta[3]-theta[2]))*pow(r21,2)+
                (-2*cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2])-2*cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2])-2*
                cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2])-2*cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2])-4*cos(theta[6]+theta[4]+theta[3]+theta[2])+4*
                cos(theta[6]-theta[4]-theta[3]-theta[2]))*pow(r11,2))*
                pow((8*pow(r21,2)+8*pow(r11,2))*pow(r31,2)+8*pow(r21,4)+16*pow(r11,2)*pow(r21,2)+8*pow(r11,4),-1);




    Jo.at<double>(1,4) = pow(pow(r21,2)+pow(r11,2),1/2)*((4*cos(theta[6])*r21*r23+4*cos(theta[6])*r11*r13)*r31+(-
                                                                                                                    cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2])+cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2])+cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2])-
                                                                                                                    cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]))*pow(r21,2)+(-cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2])+
                                                                                                                    cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2])+cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2])-cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]))*
                                                                                                                    pow(r11,2))*pow((4*pow(r21,2)+4*pow(r11,2))*pow(r31,2)+4*pow(r21,4)+8*pow(r11,2)*pow(r21,2)+4*pow(r11,4),-1);




    Jo.at<double>(1,5) = -pow(pow(r21,2)+pow(r11,2),1/2)*
                          pow((pow(r21,2)+pow(r11,2))*pow(r31,2)+pow(r21,4)+2*pow(r11,2)*pow(r21,2)+pow(r11,4),-1)*
                          ((pow(r21,2)+pow(r11,2))*r32+(-r21*r22-r11*r12)*r31);



    //----------------------------

    Jo.at<double>(2,0) = ((sin(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-sin(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]-theta[1])+
            sin(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]+theta[1])-sin(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]-theta[1])-2*sin(theta[6]+theta[5]+theta[1])-
            2*sin(theta[6]+theta[5]-theta[1])+sin(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-sin(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]-theta[1])
            +sin(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]+theta[1])-sin(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+2*sin(theta[6]-theta[5]+theta[1])
            +2*sin(theta[6]-theta[5]-theta[1])+2*sin(theta[6]+theta[4]+theta[3]+theta[2]+theta[1])-2*sin(theta[6]+theta[4]+theta[3]+theta[2]-theta[1])-2*
            sin(theta[6]-theta[4]-theta[3]-theta[2]+theta[1])+2*sin(theta[6]-theta[4]-theta[3]-theta[2]-theta[1]))*r21+8*pow(r11,2))*
            pow(8*pow(r21,2)+8*pow(r11,2),-1);



    Jo.at<double>(2,1) = ((sin(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]+theta[1])+sin(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-
                sin(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]+theta[1])-sin(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+
                sin(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]+theta[1])+sin(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-
                sin(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]+theta[1])-sin(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+2*
                sin(theta[6]+theta[4]+theta[3]+theta[2]+theta[1])+2*sin(theta[6]+theta[4]+theta[3]+theta[2]-theta[1])+2*
                sin(theta[6]-theta[4]-theta[3]-theta[2]+theta[1])+2*sin(theta[6]-theta[4]-theta[3]-theta[2]-theta[1]))*r21+(
                cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-
                cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+
                cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-
                cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+2*
                cos(theta[6]+theta[4]+theta[3]+theta[2]+theta[1])-2*cos(theta[6]+theta[4]+theta[3]+theta[2]-theta[1])+2*
                cos(theta[6]-theta[4]-theta[3]-theta[2]+theta[1])-2*cos(theta[6]-theta[4]-theta[3]-theta[2]-theta[1]))*r11)*pow(8*pow(r21,2)+8*pow(r11,2),-1);




    Jo.at<double>(2,2) = ((sin(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]+theta[1])+sin(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-
                sin(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]+theta[1])-sin(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+
                sin(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]+theta[1])+sin(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-
                sin(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]+theta[1])-sin(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+2*
                sin(theta[6]+theta[4]+theta[3]+theta[2]+theta[1])+2*sin(theta[6]+theta[4]+theta[3]+theta[2]-theta[1])+2*
                sin(theta[6]-theta[4]-theta[3]-theta[2]+theta[1])+2*sin(theta[6]-theta[4]-theta[3]-theta[2]-theta[1]))*r21+(
                cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-
                cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+
                cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-
                cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+2*
                cos(theta[6]+theta[4]+theta[3]+theta[2]+theta[1])-2*cos(theta[6]+theta[4]+theta[3]+theta[2]-theta[1])+2*
                cos(theta[6]-theta[4]-theta[3]-theta[2]+theta[1])-2*cos(theta[6]-theta[4]-theta[3]-theta[2]-theta[1]))*r11)*pow(8*pow(r21,2)+8*pow(r11,2),-1);




    Jo.at<double>(2,3) = ((sin(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]+theta[1])+sin(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-
                sin(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]+theta[1])-sin(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+
                sin(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]+theta[1])+sin(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-
                sin(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]+theta[1])-sin(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+2*
                sin(theta[6]+theta[4]+theta[3]+theta[2]+theta[1])+2*sin(theta[6]+theta[4]+theta[3]+theta[2]-theta[1])+2*
                sin(theta[6]-theta[4]-theta[3]-theta[2]+theta[1])+2*sin(theta[6]-theta[4]-theta[3]-theta[2]-theta[1]))*r21+(
                cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-cos(theta[6]+theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-
                cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+cos(theta[6]+theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+
                cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]+theta[1])-cos(theta[6]-theta[5]+theta[4]+theta[3]+theta[2]-theta[1])-
                cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]+theta[1])+cos(theta[6]-theta[5]-theta[4]-theta[3]-theta[2]-theta[1])+2*
                cos(theta[6]+theta[4]+theta[3]+theta[2]+theta[1])-2*cos(theta[6]+theta[4]+theta[3]+theta[2]-theta[1])+2*
                cos(theta[6]-theta[4]-theta[3]-theta[2]+theta[1])-2*cos(theta[6]-theta[4]-theta[3]-theta[2]-theta[1]))*r11)*pow(8*pow(r21,2)+8*pow(r11,2),-1);






    Jo.at<double>(2,4) = pow(pow(r21,2)+pow(r11,2),-1)*(cos(theta[6])*r11*r23-cos(theta[6])*r13*r21);


    Jo.at<double>(2,5) = pow(pow(r21,2)+pow(r11,2),-1)*(r11*r22-r12*r21);




    //-------------------------------

    if(&Jort != NULL)
    {
        for(int i = 0; i < NC; i++)
            for(int j = 0; j < NL; j++)
                Jort.at<double>(i,j) = Jo.at<double>(i,j);
    }

}
//=======================================================
void UR5::compute_jacobian(cv::Mat &Jacob)
{
    position_jacobian();
    orientation_jacobian();

    cv::vconcat(Jp, Jo, J);

    if(&Jacob != NULL)
    {
        for(int i = 0; i < NW; i++)
            for(int j = 0; j < NL; j++)
                Jacob.at<double>(i,j) = J.at<double>(i,j);
    }
}

//------------------------------------------------
void UR5::compute_geometric_jacobian(cv::Mat &Jacob)
{
    linear_velocity_jacobian();
    angular_velocity_jacobian();

    cv::vconcat(Jv, Jw, Jg);

    if(&Jacob != NULL)
    {
        for(int i = 0; i < NW; i++)
            for(int j = 0; j < NL; j++)
                Jacob.at<double>(i,j) = Jg.at<double>(i,j);
    }
}

//=================================================
// It gives the rotated end-effector coordinate axes
// xp2[3][3] - coordinate axes after rotation
// t[3][3] - translation matrix
// xp2 = Rz * Ry * Rx * xp + t
//----------------------------------------------------------------
void UR5::rotated_EE_axis_with_rpy(const double dx[], cv::Mat &XE)
{

    //Base Coordinate Frame

    double xorg[3][3] ={
        {dx[0], 0, 0},
        {0, dx[1], 0},
        {0, 0, dx[2]}
    };

    cv::Mat XB = cv::Mat(3,3,CV_64F, &xorg);


    double Alpha = rpy[0]; // roll - rotation about z axis
    double Beta = rpy[1]; // Pitch - rotation about y-axis
    double Gamma = rpy[2]; // Yaw - rotation about x-axis


    //Rotation about z axis
    double rz[3][3] = {
        {cos(Alpha), -sin(Alpha), 0},
        {sin(Alpha), cos(Alpha), 0},
        {0, 0, 1}
    };
    Mat RZ = Mat(3,3,CV_64F, &rz);

    // Rotation about y axis
    double ry[3][3] = {
        {cos(Beta), 0, sin(Beta)},
        {0, 1, 0},
        {-sin(Beta), 0, cos(Beta)}
    };
    Mat RY = Mat(3,3,CV_64F, &ry);


    // Rotation about X axis
    double rx[3][3] = {
        {1, 0, 0},
        {0, cos(Gamma), -sin(Gamma)},
        {0, sin(Gamma), cos(Gamma)}
    };
    Mat RX = Mat(3,3,CV_64F, &rx);


    double tx[3][3] = {
        {posn[0], posn[0], posn[0]},
        {posn[1], posn[1], posn[1]},
        {posn[2], posn[2], posn[2]}
    };

    cv::Mat T = cv::Mat(3,3, CV_64F, &tx);

    // Transformed axes

    XE = RZ * RY * RX * XB + T;

}
//===================================================================

// In this case, rpy is provided externally
// Rx = Gamma, Ry = Beta and Rz = Alpha
// Output: End-effector coordinate axes after rotation.
//--------------------------------------
void UR5::rotated_EE_axis_with_rpy(const double dx[], const double rpye[], cv::Mat &XE)
{

    //Base Coordinate Frame

    double xorg[3][3] ={
        {dx[0], 0, 0},
        {0, dx[1], 0},
        {0, 0, dx[2]}
    };

    cv::Mat XB = cv::Mat(3,3,CV_64F, &xorg);


    double Alpha = rpye[0]; // roll - rotation about z axis
    double Beta = rpye[1]; // Pitch - rotation about y-axis
    double Gamma = rpye[2]; // Yaw - rotation about x-axis


    //Rotation about z axis
    double rz[3][3] = {
        {cos(Alpha), -sin(Alpha), 0},
        {sin(Alpha), cos(Alpha), 0},
        {0, 0, 1}
    };
    Mat RZ = Mat(3,3,CV_64F, &rz);

    // Rotation about y axis
    double ry[3][3] = {
        {cos(Beta), 0, sin(Beta)},
        {0, 1, 0},
        {-sin(Beta), 0, cos(Beta)}
    };
    Mat RY = Mat(3,3,CV_64F, &ry);


    // Rotation about X axis
    double rx[3][3] = {
        {1, 0, 0},
        {0, cos(Gamma), -sin(Gamma)},
        {0, sin(Gamma), cos(Gamma)}
    };
    Mat RX = Mat(3,3,CV_64F, &rx);


    double tx[3][3] = {
        {posn[0], posn[0], posn[0]},
        {posn[1], posn[1], posn[1]},
        {posn[2], posn[2], posn[2]}
    };

    cv::Mat T = cv::Mat(3,3, CV_64F, &tx);

    // Transformed axes

    XE = RZ * RY * RX * XB + T;

}

//=============================================================
void UR5::rotated_EE_axis_with_rot_matrix(const double dx[], cv::Mat &XE)
{

    //Base Coordinate Frame
    double xorg[3][3] = {
        {dx[0], 0, 0},
        {0, dx[1], 0},
        {0, 0, dx[2]}
    };


    cv::Mat XB = cv::Mat(3, 3, CV_64F, &xorg);

    //cout << "XB = " << XB << endl << endl;

    //Translation Matrix
    double tx[3][3] = {
        {posn[0], posn[0], posn[0]},
        {posn[1], posn[1], posn[1]},
        {posn[2], posn[2], posn[2]}
    };

    cv::Mat T = cv::Mat(3,3, CV_64F, &tx);

   // cout << "T = " << T << endl << endl;

    XE = R * XB + T;

    //cout << "XE = " << XE << endl;
    //getchar();
}

//================================================================
/* -----------------------------------------
 * pose_t: Desired pose of the end-effector (6x1)
 * pref_config: The solution should stay close to preferred configuration (pref_config)
 * jtangle: Output of the function - set of joint angles as robot trajectory: num x 7
 * num: Number of joint angle vectors required
 * Output: Mean pose error for the end-effector
 * -------------------------------------------------------------------------------- */

double UR5::ik_traj_withpose(const double init_config[NL],  double pose_t[NW], const double pref_config[NL], double jtangles[][NL], int num)
{

    cout << "\n .........................................." << endl;
#ifdef PD_CONTROL
    cout << "\n Inverse Kinematic Solution using PD-Control Technique" << endl;
#elif defined JTM
    cout << "\n Inverse Kinematic Solution using Jacobian Transpose Method" << endl;
#else
    cout << "\n Inverse Kinematic Solution using Pseudo-inverse Method with Null Space Optimization" << endl;
#endif
    cout << "\n ..............................................." << endl;

    cv::Mat Pose_T(NW,1, CV_64F, pose_t);

#ifdef PD_CONTROL
    cv::Mat Pose_T_dot(NW,1, CV_64F, 0.0);
#endif


    // Initial values
    for(int i = 0; i < NL; ++i)
        theta[i+1] = init_config[i];


   // Forward Kinematics
    fwd_kin_pose();

    cv::Mat Pose_C(NW,1, CV_64F, pose);
    cv::Mat Theta_dot(NL,1,CV_64F, 0.0);
    cv::Mat Error(NW,1,CV_64F, 0.0);
    cv::Mat Jpinv(NL,NW, CV_64F, 0.0);
    cv::Mat q0_dot(NL,1,CV_64F,0.0); // Self-motion velocity

    // Gains
    cv::Mat Kp(NW,NW,CV_64F,0.0);
    Kp =  cv::Mat::eye(Kp.rows, Kp.cols, CV_64F);


    int nr = floor(TMAX / dt);

    if(num > nr)
    {
        cerr << __FILE__ << ":" << __LINE__ << ":" << "The number of data points requested is more than the maximum number of data available in the array. \n Aborting ..." << endl;
        exit(-1);
    }

    ofstream f1("error.txt");

    double angle_values[nr][NL];
    int cnt = 0;
    for(double t = 0; t < TMAX; t = t + dt)
    {

        Error = Pose_T - Pose_C;

        //Jacobian
        compute_jacobian();

        cv::invert(J, Jpinv, cv::DECOMP_SVD);

        // Here w(q) = 1/n* \sum_i{(theta[i]/(theta_max[i] - theta_min[i]))^2}
        // dw/dq_i = theta[i];
        // null space optimization for obtained a desired joint configuration
        for(int i = 0; i < NL; ++i)
        {
            q0_dot.at<double>(i,0) = 2*(theta[i+1]-pref_config[i]) / (NL * pow((theta_max[i] - theta_min[i]),2.0));
            //q0_dot.at<double>(i,0) = 2*(theta[i+1]) / (NL * pow((theta_max[i] - theta_min[i]),2.0));
        }


#ifdef JTM
        // Jacobian Transpose method

        //step size : alpha
        double alpha = cv::norm(J.t()*Error) / cv::norm(J*J.t()*Error);

        // Jacobian Transpose Method for computing theta_dot
        Theta_dot = alpha * J.t() * Error;
#endif

#ifdef PD_CONTROL
        // Joint angle velocity using Self-motion component
        // pseudoinverse solution with nullspace optimization
            Theta_dot = Jpinv * (Pose_T_dot + Kp*Error) - 1.5*(cv::Mat::eye(NL,NL,CV_64F)-Jpinv*J) * q0_dot;
#endif


#ifdef PI_NSO
        // Pseudo-inverse with Null Space Optimzation (NSO) = PI+NSO
        Theta_dot = Jpinv * Error - (cv::Mat::eye(NL,NL,CV_64F)-Jpinv*J) * q0_dot;
#endif

        // update joint angles
        for(int i = 0; i < NL; ++i)
        {
            theta[i+1] = theta[i+1] + dt * Theta_dot.at<double>(i);

//            if(theta[i] > theta_max[i]) theta[i] = theta_max[i];
//            else if(theta[i] < theta_min[i]) theta[i] = theta_min[i];


            angle_values[cnt][i] = theta[i+1];
        }


        // new robot pose
        fwd_kin_pose();


        for(int i = 0; i < NW; ++i)
            Pose_C.at<double>(i) = pose[i];


        double s1 = 0.0, s2 = 0.0;
        for(int i = 0; i < NC; i++)
        {
            s1 += pow(Error.at<double>(i), 2.0);
            s2 += pow(Error.at<double>(i+NC), 2.0);
        }
        s1 = sqrt(s1/NC);
        s2 = sqrt(s2/NC);

        f1 << s1 << "\t" << s2 << endl;


      cnt++;

    } // control-time-loop

    f1.close();

    int rowcount = 0;
    for(int i = nr-num; i < nr; ++i)
    {

        for(int j = 0; j < NL; ++j)
            jtangles[rowcount][j] = angle_values[i][j];
       rowcount++;
    }


    // Return the pose error
    return(sqrt(cv::Mat(Error.t()*Error).at<double>(0,0)/NW));
}
//=============================================================

void UR5::get_angle_range(double th_max[], double th_min[])
{
    for(int i = 0; i < NL; i++)
    {
        th_max[i] = theta_max[i];
        th_min[i] = theta_min[i];
    }
}

//=================================================
void UR5::display_variables()
{
    cout << "\ntheta = ";
    for(int i = 0; i < NL; i++)
        cout << theta[i+1] << "\t";
    cout << endl;

    cout << "------------------" << endl;
    cout << "\n R = " << R << endl;

    cout << "\n Jp = " << Jp << endl;

    cout << "\n Jo = " << Jo << endl;

    cout << "-----------------------" << endl;
    cout << "\n Jv = " << Jv << endl;
    cout << "\n Jw = " << Jw << endl;
}
//====================================================
// Geometric Method for computing Angular Velocity Jacobian
// Jw = [z00, z01, z02, z03, z04, z05]
// z01 = R01*z00;
// z02 = R01*R12*z00; and so on ...
//-------------------------------------------------
void UR5::angular_velocity_jacobian(cv::Mat &Jaw)
{
    Jw.at<double>(0,0) = 0.0;
    Jw.at<double>(0,1) = sin(theta[1]);
    Jw.at<double>(0,2) = sin(theta[1]);

    Jw.at<double>(0,3) = sin(theta[1]);
    Jw.at<double>(0,4) = (cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]);

    Jw.at<double>(0,5) = sin(theta[1])*cos(theta[5])-
                ((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*sin(theta[5]);


    //------------------------------------
    Jw.at<double>(1,0) = 0.0;
    Jw.at<double>(1,1) = -cos(theta[1]);
    Jw.at<double>(1,2) = -cos(theta[1]);
    Jw.at<double>(1,3) = -cos(theta[1]);

    Jw.at<double>(1,4) = (sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]);
    Jw.at<double>(1,5) = -((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*
            sin(theta[5])-cos(theta[1])*cos(theta[5]);

    //-------------------------------------------------------
    Jw.at<double>(2,0) = 1.0;
    Jw.at<double>(2,1) = 0.0;
    Jw.at<double>(2,2) = 0.0;
    Jw.at<double>(2,3) = 0.0;
    Jw.at<double>(2,4) = (cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*sin(theta[4])-(cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*cos(theta[4]);
    Jw.at<double>(2,5) = -((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*sin(theta[4])+(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*cos(theta[4]))*sin(theta[5]);

    //-----------------------------------------

    if(&Jaw != NULL)
    {
        for(int i = 0; i < NC; i++)
            for(int j = 0; j < NL; j++)
                Jaw.at<double>(i,j) = Jw.at<double>(i,j);
    }

}
//--------------------------------------
// This is same as position_jacobian but computed using Vector Product Method
// Still under testing.
void UR5::linear_velocity_jacobian(cv::Mat &Jav)
{

    Jv.at<double>(0,0) = -(-((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*
            sin(theta[5])-cos(theta[1])*cos(theta[5]))*d6-
            ((sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5+cos(theta[1])*d4+
            sin(theta[1])*sin(theta[2])*sin(theta[3])*a3-sin(theta[1])*cos(theta[2])*cos(theta[3])*a3-sin(theta[1])*cos(theta[2])*a2;


    Jv.at<double>(0,1) = -cos(theta[1])*(-((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*sin(theta[4])+(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*cos(theta[4]))*sin(theta[5])*d6+
            ((cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*sin(theta[4])-(cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*cos(theta[4]))*d5+cos(theta[2])*sin(theta[3])*a3+sin(theta[2])*cos(theta[3])*a3+
            sin(theta[2])*a2);



    Jv.at<double>(0,2) = -cos(theta[1])*(-((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*sin(theta[4])+(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*cos(theta[4]))*sin(theta[5])*d6+
            ((cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*sin(theta[4])-(cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*cos(theta[4]))*d5+cos(theta[2])*sin(theta[3])*a3+sin(theta[2])*cos(theta[3])*a3);




    Jv.at<double>(0,3) = -cos(theta[1])*(((cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*sin(theta[4])-(cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*cos(theta[4]))*d5-
                ((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*sin(theta[4])+(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*cos(theta[4]))*sin(theta[5])*d6);

    Jv.at<double>(0,4) = ((sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*(
                ((cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*sin(theta[4])-(cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*cos(theta[4]))*d5-
                ((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*sin(theta[4])+(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*cos(theta[4]))*sin(theta[5])*d6)-
                ((cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*sin(theta[4])-(cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*cos(theta[4]))*((-
                ((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*sin(theta[5])-
                cos(theta[1])*cos(theta[5]))*d6+
                ((sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5);


    Jv.at<double>(0,5) = 0.0;



    //-----------------------------------------------

    Jv.at<double>(1,0) = (sin(theta[1])*cos(theta[5])-
                ((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*sin(theta[5]))*d6+
                ((cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5+sin(theta[1])*d4-
                cos(theta[1])*sin(theta[2])*sin(theta[3])*a3+cos(theta[1])*cos(theta[2])*cos(theta[3])*a3+cos(theta[1])*cos(theta[2])*a2;

    Jv.at<double>(1,1) = -sin(theta[1])*(-((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*sin(theta[4])+(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*cos(theta[4]))*sin(theta[5])*d6+
            ((cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*sin(theta[4])-(cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*cos(theta[4]))*d5+cos(theta[2])*sin(theta[3])*a3+sin(theta[2])*cos(theta[3])*a3+
            sin(theta[2])*a2);


    Jv.at<double>(1,2) = -sin(theta[1])*(-((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*sin(theta[4])+(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*cos(theta[4]))*sin(theta[5])*d6+
            ((cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*sin(theta[4])-(cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*cos(theta[4]))*d5+cos(theta[2])*sin(theta[3])*a3+sin(theta[2])*cos(theta[3])*a3);


    Jv.at<double>(1,3) = -sin(theta[1])*(((cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*sin(theta[4])-(cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*cos(theta[4]))*d5-
                ((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*sin(theta[4])+(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*cos(theta[4]))*sin(theta[5])*d6);


    Jv.at<double>(1,4) = ((cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*sin(theta[4])-(cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*cos(theta[4]))*((sin(theta[1])*cos(theta[5])-
                ((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*sin(theta[5]))*d6+
                ((cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5)-
                ((cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*(
                ((cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*sin(theta[4])-(cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*cos(theta[4]))*d5-
                ((cos(theta[2])*cos(theta[3])-sin(theta[2])*sin(theta[3]))*sin(theta[4])+(cos(theta[2])*sin(theta[3])+sin(theta[2])*cos(theta[3]))*cos(theta[4]))*sin(theta[5])*d6);

    Jv.at<double>(1,5) = 0.0;
    //------------------------------------------

    Jv.at<double>(2,0) = 0.0;

    Jv.at<double>(2,1) = sin(theta[1])*((-
                                             ((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*sin(theta[5])-
                                             cos(theta[1])*cos(theta[5]))*d6+
                                             ((sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5-cos(theta[1])*d4-
                                             sin(theta[1])*sin(theta[2])*sin(theta[3])*a3+sin(theta[1])*cos(theta[2])*cos(theta[3])*a3+sin(theta[1])*cos(theta[2])*a2)+cos(theta[1])*((sin(theta[1])*cos(theta[5])-
                                             ((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*sin(theta[5]))*d6+
                                             ((cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5+sin(theta[1])*d4-
                                             cos(theta[1])*sin(theta[2])*sin(theta[3])*a3+cos(theta[1])*cos(theta[2])*cos(theta[3])*a3+cos(theta[1])*cos(theta[2])*a2);

    Jv.at<double>(2,2) = sin(theta[1])*((-
                                             ((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*sin(theta[5])-
                                             cos(theta[1])*cos(theta[5]))*d6+
                                             ((sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5-cos(theta[1])*d4-
                                             sin(theta[1])*sin(theta[2])*sin(theta[3])*a3+sin(theta[1])*cos(theta[2])*cos(theta[3])*a3)+cos(theta[1])*((sin(theta[1])*cos(theta[5])-
                                             ((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*sin(theta[5]))*d6+
                                             ((cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5+sin(theta[1])*d4-
                                             cos(theta[1])*sin(theta[2])*sin(theta[3])*a3+cos(theta[1])*cos(theta[2])*cos(theta[3])*a3);


    Jv.at<double>(2,3) = sin(theta[1])*((-
                                             ((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*sin(theta[5])-
                                             cos(theta[1])*cos(theta[5]))*d6+
                                             ((sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5-cos(theta[1])*d4)
                                             +cos(theta[1])*((sin(theta[1])*cos(theta[5])-
                                             ((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*sin(theta[5]))*d6+
                                             ((cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5+sin(theta[1])*d4);


    Jv.at<double>(2,4) = ((cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*((-
                                                                                                                                                                                                                                          ((-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*sin(theta[5])-
                                                                                                                                                                                                                                          cos(theta[1])*cos(theta[5]))*d6+
                                                                                                                                                                                                                                          ((sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5)-
                                                                                                                                                                                                                                          ((sin(theta[1])*cos(theta[2])*cos(theta[3])-sin(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-sin(theta[1])*cos(theta[2])*sin(theta[3])-sin(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*((sin(theta[1])*
                                                                                                                                                                                                                                          cos(theta[5])-((-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*sin(theta[4])+(cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*cos(theta[4]))*
                                                                                                                                                                                                                                          sin(theta[5]))*d6+((cos(theta[1])*cos(theta[2])*cos(theta[3])-cos(theta[1])*sin(theta[2])*sin(theta[3]))*sin(theta[4])-(-cos(theta[1])*cos(theta[2])*sin(theta[3])-cos(theta[1])*sin(theta[2])*cos(theta[3]))*cos(theta[4]))*d5
                                                                                                                                                                                                                                          );


    Jv.at<double>(2,5) = 0.0;


    if(&Jav != NULL)
    {
        for(int i = 0; i < NC; i++)
            for(int j = 0; j < NL; j++)
                Jav.at<double>(i,j) = Jv.at<double>(i,j);
    }

}
//--------------------------------------

// This module is not working. Need to be tested
/*
double UR5::ik_traj_withpose2(const double init_config[NL],  double pose_t[NW], const double pref_config[NL], double jtangles[][NL], int num)
{
    cv::Mat Pose_T(NW,1, CV_64F, pose_t);

#ifdef PD_CONTROL
    cv::Mat Pose_T_dot(NW,1, CV_64F, 0.0);
#endif

    // Initial values
    for(int i = 0; i < NL; ++i)
        theta[i+1] = init_config[i];

   // Forward Kinematics
    fwd_kin_pose();

    cv::Mat Pose_C(NW,1, CV_64F, pose);
    cv::Mat Theta_dot(NL,1,CV_64F, 0.0);
    cv::Mat Error(NW,1,CV_64F, 0.0);
    cv::Mat Jpinv(NL,NW, CV_64F, 0.0);
    cv::Mat q0_dot(NL,1,CV_64F,0.0); // Self-motion velocity

    // Gains
    cv::Mat Kp(NW,NW,CV_64F,0.0);
    Kp =  cv::Mat::eye(Kp.rows, Kp.cols, CV_64F);


    int nr = floor(TMAX / dt);

    if(num > nr)
    {
        cerr << __FILE__ << ":" << __LINE__ << ":" << "The number of data points requested is more than the maximum number of data available in the array. \n Aborting ..." << endl;
        exit(-1);
    }

    double angle_values[nr][NL];
    int cnt = 0;
    for(double t = 0; t < TMAX; t = t + dt)
    {

        Error = Pose_T - Pose_C;

        //Jacobian
        compute_jacobian2();


        cv::invert(J, Jpinv, cv::DECOMP_SVD);

        // Here w(q) = 1/n* \sum_i{(theta[i]/(theta_max[i] - theta_min[i]))^2}
        // dw/dq_i = theta[i];
        // null space optimization for obtained a desired joint configuration
        for(int i = 0; i < NL; ++i)
        {
            q0_dot.at<double>(i,0) = 2*(theta[i]-pref_config[i]) / (NL * pow((theta_max[i] - theta_min[i]),2.0));
            //q0_dot.at<double>(i,0) = 2*(theta[i]) / (NL * pow((theta_max[i] - theta_min[i]),2.0));
        }


#ifdef JTM
        // Jacobian Transpose method

        //step size : alpha
        double alpha = cv::norm(J.t()*Error) / cv::norm(J*J.t()*Error);

        // Jacobian Transpose Method for computing theta_dot
        Theta_dot = alpha * J.t() * Error;
#endif

#ifdef PD_CONTROL
        // Joint angle velocity using Self-motion component
        // pseudoinverse solution with nullspace optimization
            Theta_dot = Jpinv * (Pose_T_dot + Kp*Error) - 1.5*(cv::Mat::eye(NL,NL,CV_64F)-Jpinv*J) * q0_dot;
#endif


#ifdef PI_NSO
        // Pseudo-inverse with Null Space Optimzation (NSO) = PI+NSO
        Theta_dot = Jpinv * Error - (cv::Mat::eye(NL,NL,CV_64F)-Jpinv*J) * q0_dot;
#endif

        // update joint angles
        for(int i = 0; i < NL; ++i)
        {
            theta[i+1] = theta[i+1] + dt * Theta_dot.at<double>(i);

//            if(theta[i] > theta_max[i]) theta[i] = theta_max[i];
//            else if(theta[i] < theta_min[i]) theta[i] = theta_min[i];


            angle_values[cnt][i] = theta[i+1];
        }


        // new robot pose
        fwd_kin_pose();


        for(int i = 0; i < NW; ++i)
            Pose_C.at<double>(i) = pose[i];


      cnt++;

    } // control-time-loop

    int rowcount = 0;
    for(int i = nr-num; i < nr; ++i)
    {

        for(int j = 0; j < NL; ++j)
            jtangles[rowcount][j] = angle_values[i][j];
       rowcount++;
    }

    return(sqrt(cv::Mat(Error.t()*Error).at<double>(0,0)/NW));
}
//=================================================
*/
