#include "kdl_ros_control/kdl_robot.h"
#include "kdl_ros_control/kdl_control.h"
#include "kdl_ros_control/kdl_planner.h"

#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"
#include <std_srvs/Empty.h>
#include "eigen_conversions/eigen_kdl.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "gazebo_msgs/SetModelConfiguration.h"
//---------------------------------------------------------------- Step 2a --------------------------------------------------------------
#include "math.h"        // Inclusion of the "math.h" library to use M_PI
//---------------------------------------------------------------------------------------------------------------------------------------


// Global variables
std::vector<double> jnt_pos(7,0.0), init_jnt_pos(7,0.0), jnt_vel(7,0.0), aruco_pose(7,0.0);
bool robot_state_available = false, aruco_pose_available = false;
double lambda = 10*0.2;
double KP = 15;

// Functions
KDLRobot createRobot(std::string robot_string)
{
    KDL::Tree robot_tree;
    urdf::Model my_model;
    if (!my_model.initFile(robot_string))
    {
        printf("Failed to parse urdf robot model \n");
    }
    if (!kdl_parser::treeFromUrdfModel(my_model, robot_tree))
    {
        printf("Failed to construct kdl tree \n");
    }
    
    KDLRobot robot(robot_tree);
    return robot;
}

void jointStateCallback(const sensor_msgs::JointState & msg)
{
    robot_state_available = true;
    // Update joints
    jnt_pos.clear();
    jnt_vel.clear();
    for (int i = 0; i < msg.position.size(); i++)
    {
        jnt_pos.push_back(msg.position[i]);
        jnt_vel.push_back(msg.velocity[i]);
    }
}

void arucoPoseCallback(const geometry_msgs::PoseStamped & msg)
{
    aruco_pose_available = true;
    aruco_pose.clear();
    aruco_pose.push_back(msg.pose.position.x);
    aruco_pose.push_back(msg.pose.position.y);
    aruco_pose.push_back(msg.pose.position.z);
    aruco_pose.push_back(msg.pose.orientation.x);
    aruco_pose.push_back(msg.pose.orientation.y);
    aruco_pose.push_back(msg.pose.orientation.z);
    aruco_pose.push_back(msg.pose.orientation.w);
}


// Main
int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("Please, provide a path to a URDF file...\n");
        return 0;
    }

    // Init node
    ros::init(argc, argv, "kdl_ros_control_node");
    ros::NodeHandle n;

    // Rate
    ros::Rate loop_rate(500);

    // Subscribers
    ros::Subscriber aruco_pose_sub = n.subscribe("/aruco_single/pose", 1, arucoPoseCallback);
    ros::Subscriber joint_state_sub = n.subscribe("/iiwa/joint_states", 1, jointStateCallback);

    // Publishers
    ros::Publisher joint1_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J1_controller/command", 1);
    ros::Publisher joint2_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J2_controller/command", 1);
    ros::Publisher joint3_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J3_controller/command", 1);
    ros::Publisher joint4_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J4_controller/command", 1);
    ros::Publisher joint5_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J5_controller/command", 1);
    ros::Publisher joint6_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J6_controller/command", 1);
    ros::Publisher joint7_dq_pub = n.advertise<std_msgs::Float64>("/iiwa/VelocityJointInterface_J7_controller/command", 1);    

    ros::Publisher sx = n.advertise<std_msgs::Float64>("/iiwa/sx", 1);      // Publishers defined for step 2b
    ros::Publisher sy = n.advertise<std_msgs::Float64>("/iiwa/sy", 1);
    ros::Publisher sz = n.advertise<std_msgs::Float64>("/iiwa/sz", 1);

    // Services
    ros::ServiceClient robot_set_state_srv = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    ros::ServiceClient pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");

    // Initial desired robot state
    init_jnt_pos[0] = 0.0;
    init_jnt_pos[1] = 1.57;
    init_jnt_pos[2] = -1.57;
    init_jnt_pos[3] = -1.57;
    init_jnt_pos[4] = 1.57;
    init_jnt_pos[5] = -1.57;
    init_jnt_pos[6] = +1.57;
    Eigen::VectorXd qdi = toEigen(init_jnt_pos);

    // Create robot
    KDLRobot robot = createRobot(argv[1]);

    // Messages
    std_msgs::Float64 dq1_msg, dq2_msg, dq3_msg, dq4_msg, dq5_msg, dq6_msg, dq7_msg;
    std_srvs::Empty pauseSrv;
    std_msgs::Float64 s_x, s_y, s_z;                                        // Step 2b: in order to plot the components of s, I need to define the corrisponding ROS messages

    // Joints
    KDL::JntArray qd(robot.getNrJnts()), dqd(robot.getNrJnts()), ddqd(robot.getNrJnts());
    qd.data.setZero();
    dqd.data.setZero();
    ddqd.data.setZero();

    // Wait for robot and object state
    while (!(robot_state_available))
    {
        ROS_INFO_STREAM_ONCE("Robot/object state not available yet.");
        ROS_INFO_STREAM_ONCE("Please start gazebo simulation.");
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    // Bring robot in a desired initial configuration with velocity control
    ROS_INFO("Robot going into initial configuration....");
    double jnt_position_error_norm = computeJointErrorNorm(toEigen(init_jnt_pos),toEigen(jnt_pos));
    while (jnt_position_error_norm > 0.01)
    {
        dqd.data = KP*(toEigen(init_jnt_pos) - toEigen(jnt_pos));

        // Set joints
        dq1_msg.data = dqd.data[0];
        dq2_msg.data = dqd.data[1];
        dq3_msg.data = dqd.data[2];
        dq4_msg.data = dqd.data[3];
        dq5_msg.data = dqd.data[4];
        dq6_msg.data = dqd.data[5];
        dq7_msg.data = dqd.data[6];

        // Publish
        joint1_dq_pub.publish(dq1_msg);
        joint2_dq_pub.publish(dq2_msg);
        joint3_dq_pub.publish(dq3_msg);
        joint4_dq_pub.publish(dq4_msg);
        joint5_dq_pub.publish(dq5_msg);
        joint6_dq_pub.publish(dq6_msg);
        joint7_dq_pub.publish(dq7_msg);

        jnt_position_error_norm = computeJointErrorNorm(toEigen(init_jnt_pos),toEigen(jnt_pos));
        std::cout << "jnt_position_error_norm: " << jnt_position_error_norm << "\n" << std::endl;
        ros::spinOnce();
        loop_rate.sleep();

    }

    // Specify an end-effector: camera in flange transform
    KDL::Frame ee_T_cam;
    ee_T_cam.M = KDL::Rotation::RotY(1.57)*KDL::Rotation::RotZ(-1.57);
    ee_T_cam.p = KDL::Vector(0,0,0.025);
    robot.addEE(ee_T_cam);

    // Update robot
    robot.update(jnt_pos, jnt_vel);

    // Retrieve initial simulation time
    ros::Time begin = ros::Time::now();
    ROS_INFO_STREAM_ONCE("Starting control loop ...");

    // Retrieve initial ee pose
    KDL::Frame Fi = robot.getEEFrame();
    Eigen::Vector3d pdi = toEigen(Fi.p);

    while (true)
    {
        if (robot_state_available && aruco_pose_available)
        {
            // Update robot
            robot.update(jnt_pos, jnt_vel);
        
            // Update time
            double t = (ros::Time::now()-begin).toSec();
            std::cout << "time: " << t << std::endl;

            // compute current jacobians
            KDL::Jacobian J_cam = robot.getEEJacobian();
            KDL::Frame cam_T_object(KDL::Rotation::Quaternion(aruco_pose[3], aruco_pose[4], aruco_pose[5], aruco_pose[6]), KDL::Vector(aruco_pose[0], aruco_pose[1], aruco_pose[2]));

//----------------------------------------------------------------------------------- Step 2a ----------------------------------------------------------------------------------

            KDL::Frame marker = cam_T_object;                         // New frame for the marker with reference to the camera ---> I will use it to correct the orientation
                                                                      // of the marker with respect to the camera and as the frame the control system will actually pursuit

            KDL::Rotation X_correction = KDL::Rotation::RotX(M_PI);   // Rotation of Pi around the X axis that I will use to align the frame on the marker to the frame on the camera
            KDL::Vector offset(0.0, 0.0, 0.5);                        // Arbitrary offset on axis Z, since the "marker" and "robot.getEEFrame()" frames will be aligned along Z
            KDL::Frame marker_correction(X_correction, offset);       // Frame defined to apply in a single shot both the desired offset and the orientation correction to the "marker" frame 
            marker = marker*marker_correction;                        // Application of the correction to the "marker" frame
            
            KDL::Frame base_T_object = robot.getEEFrame()*marker;     // Transformation to express the "marker" frame I want to follow in base reference

            /*---------------- Original look_at_point algorithm (UNCOMMENT to restore) -----------------------------------------------
            // look at point: compute rotation error from angle/axis   (Unmodified for step 2a)
            Eigen::Matrix<double,3,1> aruco_pos_n = toEigen(cam_T_object.p); //(aruco_pose[0],aruco_pose[1],aruco_pose[2]);
            aruco_pos_n.normalize();
            Eigen::Vector3d r_o = skew(Eigen::Vector3d(0,0,1))*aruco_pos_n;
            double aruco_angle = std::acos(Eigen::Vector3d(0,0,1).dot(aruco_pos_n));
            KDL::Rotation Re = KDL::Rotation::Rot(KDL::Vector(r_o[0], r_o[1], r_o[2]), aruco_angle);
            */

    //------------------------------------------------------------------------------- Step 2b ---------------------------------------------------------------------------
            // look_at_point algorithm for step 2b

            Eigen::Vector3d cPo = toEigen(cam_T_object.p);                    // Definition of the vector from the camera to the marker
            Eigen::Vector3d s_ = cPo / cPo.norm();                            // Normalization of cPo (I tried s_ = cPo.normalize() but it does not work)
            Eigen::Matrix<double,3,3> I3 = Eigen::MatrixXd::Identity(3, 3);   // 3x3 Identity matrix

            Eigen::MatrixXd L_left = -(I3 - s_*s_.transpose()) / cPo.norm();  // Left  block for the matrix L
            Eigen::MatrixXd L_right = skew(s_);                               // Right block for the matrix L

            Eigen::Matrix<double,3,3> Rc = toEigen(robot.getEEFrame().M);     // 3x3 Rc matrix definition (to be used to compose the block diagonal 6x6 matrix R) 
            Eigen::Matrix<double,6,6> R(6,6);                                 // Block diagonal 6x6 rotation matrix R definiion 
            R.setZero();
            R.block(0,0,3,3) = Rc;                                            // Block diagonal 6x6 matrix R filling 
            R.block(3,3,3,3) = Rc;

            Eigen::Matrix<double,3,6> L = Eigen::Matrix<double,3,6>::Zero();  // 3x6 L matrix definition
            
            L << L_left, L_right; L = L*R.transpose();                        // L matrix filling and multiplicating by the transpose of R (in order to convert from camera reference to the base reference)

            Eigen::Matrix<double,3,1> s_d; s_d << 0, 0, 1;                    // Desired s_d value setting
            Eigen::Matrix<double,7,7> I7 = Eigen::MatrixXd::Identity(7, 7);   // 7x7 Identity matrix   
            Eigen::MatrixXd LJ = L * toEigen(J_cam);                          // L*J matrix product computation 
            Eigen::MatrixXd LJ_dag = LJ.completeOrthogonalDecomposition().pseudoInverse();   // Computation of the pseudo-inverse of LJ
            Eigen::MatrixXd N = I7 - LJ_dag * LJ;                             // Computation of the matrix N, which spans the null space of the LJ matrix
           
            double k = 3;                                                     // Gain
            
            Eigen::Matrix<double,7,1> q_dot_zero = Eigen::Matrix<double,7,1>::Zero(); 

            q_dot_zero = (qdi - toEigen(jnt_pos));                            // Working q_dot_zero
            // q_dot_zero << 0, 0.1, 0.1, 0, 0.1, 0, 0.1;                     // Arbitrary velocities (not working)

            Eigen::Matrix<double,7,1> q_dot = k*LJ_dag*s_d + N*q_dot_zero;    // Joint velocities computation
            dqd.data = q_dot;

            s_x.data=s_[0];                        // Filling of the messages to be published and plotted
            s_y.data=s_[1];
            s_z.data=s_[2];

    //----------------------------------------------------------------------------- End of Step 2b ----------------------------------------------------------------------
    
            // This "compute errors" section is unused during step 2b
            // compute errors (edited for step 2a)
            Eigen::Matrix<double,3,1> e_p = computeLinearError(toEigen(base_T_object.p),toEigen(robot.getEEFrame().p));         // New linear error computed with respect to the "marker" frame in base reference
            Eigen::Matrix<double,3,1> e_o = computeOrientationError(toEigen(base_T_object.M) , toEigen(robot.getEEFrame().M));  // New orientation error (I forgot to put this line in evidence in the Report)

//--------------------------------------------------------------------------------- End of Step 2a --------------------------------------------------------------------------------
           
            // compute errors (unedited)
            Eigen::Matrix<double,3,1> e_o_w = computeOrientationError(toEigen(Fi.M), toEigen(robot.getEEFrame().M));
            Eigen::Matrix<double,6,1> x_tilde; x_tilde << e_p,  e_o_w[0], e_o[1], e_o[2];

            /*---------------------- Original errors computation + velocity computation (UNCOMMENT to restore)--------------------------------------------
            // compute errors
            Eigen::Matrix<double,3,1> e_o = computeOrientationError(toEigen(robot.getEEFrame().M*Re), toEigen(robot.getEEFrame().M));
            Eigen::Matrix<double,3,1> e_o_w = computeOrientationError(toEigen(Fi.M), toEigen(robot.getEEFrame().M));
            Eigen::Matrix<double,3,1> e_p = computeLinearError(pdi,toEigen(robot.getEEFrame().p));
            Eigen::Matrix<double,6,1> x_tilde; x_tilde << e_p,  e_o_w[0], e_o[1], e_o[2];

            // resolved velocity control low
            Eigen::MatrixXd J_pinv = J_cam.data.completeOrthogonalDecomposition().pseudoInverse();
            dqd.data = lambda*J_pinv*x_tilde + 10*(Eigen::Matrix<double,7,7>::Identity() - J_pinv*J_cam.data)*(qdi - toEigen(jnt_pos));
           */

            /* debug (UNCOMMENT to restore - was already commented)
            // std::cout << "x_tilde: " << std::endl << x_tilde << std::endl;
            // std::cout << "Rd: " << std::endl << toEigen(robot.getEEFrame().M*Re) << std::endl;
            // std::cout << "aruco_pos_n: " << std::endl << aruco_pos_n << std::endl;
            // std::cout << "aruco_pos_n.norm(): " << std::endl << aruco_pos_n.norm() << std::endl;
            // std::cout << "Re: " << std::endl << Re << std::endl;
            // std::cout << "jacobian: " << std::endl << robot.getEEJacobian().data << std::endl;
            // std::cout << "jsim: " << std::endl << robot.getJsim() << std::endl;
            // std::cout << "c: " << std::endl << robot.getCoriolis().transpose() << std::endl;
            // std::cout << "g: " << std::endl << robot.getGravity().transpose() << std::endl;
            // std::cout << "qd: " << std::endl << qd.data.transpose() << std::endl;
            // std::cout << "q: " << std::endl << robot.getJntValues().transpose() << std::endl;
            // std::cout << "tau: " << std::endl << tau.transpose() << std::endl;
            // std::cout << "desired_pose: " << std::endl << des_pose << std::endl;
            // std::cout << "current_pose: " << std::endl << robot.getEEFrame() << std::endl;
            */
        }
        else{
            dqd.data = KP*(toEigen(init_jnt_pos) - toEigen(jnt_pos));
        }
        // Set joints
        dq1_msg.data = dqd.data[0];
        dq2_msg.data = dqd.data[1];
        dq3_msg.data = dqd.data[2];
        dq4_msg.data = dqd.data[3];
        dq5_msg.data = dqd.data[4];
        dq6_msg.data = dqd.data[5];
        dq7_msg.data = dqd.data[6];

        // Publish
        joint1_dq_pub.publish(dq1_msg);
        joint2_dq_pub.publish(dq2_msg);
        joint3_dq_pub.publish(dq3_msg);
        joint4_dq_pub.publish(dq4_msg);
        joint5_dq_pub.publish(dq5_msg);
        joint6_dq_pub.publish(dq6_msg);
        joint7_dq_pub.publish(dq7_msg);

        sx.publish(s_x);                       // Step 2b: publishing of the s components' messages  
        sy.publish(s_y);
        sz.publish(s_z);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}