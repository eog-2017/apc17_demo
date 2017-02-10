#ifndef UR5_MOD_H
#define UR5_MOD_H

#include <cmath>
#include <fstream>
#include <opencv2/opencv.hpp>


namespace MODUR5
{
//Algorithm Settings
//#define PD_CONTROL    // use only one option at a time
//#define JTM
#define PI_NSO

#define DEG2RAD(x) M_PI*x/180.0
#define RAD2DEG(x) 180.0*x/M_PI

//Configuration Settings
#define NL 6   // Dimension of joint space
#define NW 6   // Dimension of workspace ( 3 - for only position, 6 - for pose = position + orientation)
#define NR 7   // Dimension of Robot config matrix
#define NC 3
#define dt 0.01 // Sampling time in Seconds
#define TMAX 20 // Total Simulation time in seconds



class UR5
{
private:

    bool FLAG;

    // Only theta starts with index 1. Rest all starts with index 0.

    double theta[NL+1]; // Joint angle vector (6x1)
    double pose[NW]; // End-effector pose (6x1)
    double posn[NC]; // End-effector position (3x1)
    double rpy[NC];  // orientation in roll-pitch-yaw format
    double rxyz[NC]; // orientation in [Rx, Ry, Rz] format

    cv::Mat Jp;  // position Jacobian (3x6)
    cv::Mat Jo;  // orientation Jacobian (3x6)
    cv::Mat Jw;  // angular velocity Jacobian (3x6)
    cv::Mat Jv;  // Linear velocity Jacobian
    cv::Mat J;   // Full analytic Jacobian (6x7)
    cv::Mat R;   // Rotation Matrix (3x3)
    cv::Mat Jg;  // Full geometric Jacobian (6x7)

    std::vector<double> theta_max, theta_min, c_min, c_max;
    std::vector<double> offset;

    // D-H parameters
    //const double d1 = 0.089159, d4 = 0.10915, d5 = 0.09465, d6 = 0.0823;
    //const double a2 = -0.425, a3 = -0.39225;
// FOR UR5 ROBOT
//    const double d1 = 0.08920, d4 = 0.109, d5 = 0.093, d6 = 0.0823;
//    const double a2 = -0.425, a3 = -0.39243;

// FOR UR10 ROBOT
    const double d1 = 0.1273, d4 = 0.163941, d5 = 0.1157, d6 = 0.0922;
    const double a2 = -0.612, a3 = -0.5723;

public:
    UR5();
    void set_joint_angles(const double Th[]);
    void get_joint_angles(double Th[]);
    void get_ee_pose(double xp[]);
    void get_jacobian(cv::Mat &J);
    void get_angle_range(double th_max[], double th_min[]);
    void display_variables();
    inline bool exist(const std::string& name);
    void fwd_kin_posn(double xef[]=NULL);
    void rotation_matrix(cv::Mat &RM = *(cv::Mat*)NULL);
    void fwd_kin_pose(double xp[]=NULL);
    void generate_data(double Uc[], double Th[]);
    void generate_pose_data(double Uc[], double Th[]);
    void joint_position(cv::Mat &X);
    void position_jacobian(cv::Mat &Jp=*(cv::Mat*)NULL);
    void orientation_jacobian(cv::Mat &Jo=*(cv::Mat*)NULL);
    void compute_jacobian(cv::Mat &Jac=*(cv::Mat*)NULL); // Analytic Jacobian
    void angular_velocity_jacobian(cv::Mat &Jav=*(cv::Mat*)NULL); // testing
    void linear_velocity_jacobian(cv::Mat &Jaw=*(cv::Mat*)NULL);
    void compute_geometric_jacobian(cv::Mat &J2=*(cv::Mat*)NULL); //Geometric Jacobian
    void rotated_EE_axis_with_rpy(const double dx[], cv::Mat &XP2);
    void rotated_EE_axis_with_rpy(const double dx[], const double rpye[], cv::Mat &XE);
    void rotated_EE_axis_with_rot_matrix(const double dx[], cv::Mat &XP2);
    double ik_traj_withpose(const double init_config[NL], double pose_t[NW], const double pref_config[NL], double jtangles[][NL], int num);
    //double ik_traj_withpose2(const double init_config[NL], double pose_t[NW], const double pref_config[NL], double jtangles[][NL], int num);
};


}
#endif // UR5_MOD_H
