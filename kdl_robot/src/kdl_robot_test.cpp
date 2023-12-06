
////////////////// All the modifications I made for Homework 3 can be found by searching the key words "HW03 Step 2c" ///////////////////////

#include "kdl_ros_control/kdl_robot.h"
#include "kdl_ros_control/kdl_control.h"
#include "kdl_ros_control/kdl_planner.h"

#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"
#include <std_srvs/Empty.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/SetModelConfiguration.h"
//----------------------------------------------------------- HW03 Step 2c --------------------------------------------------------------
#include "math.h"        // Inclusion of the "math.h" library to use M_PI
#include "geometry_msgs/PoseStamped.h"
#include "eigen_conversions/eigen_kdl.h"
//---------------------------------------------------------------------------------------------------------------------------------------

// Global variables
std::vector<double> jnt_pos(7,0.0), jnt_vel(7,0.0), obj_pos(6,0.0),  obj_vel(6,0.0);
bool robot_state_available = false;
//--------------------------------- Global variables for HW03 Step 2c from kdl_robot_vision_control.cpp ---------------------------------
std::vector<double>  aruco_pose(7,0.0), init_jnt_pos(7,0.0);
bool aruco_pose_available = false;
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
    jnt_pos.clear();              // Update joints
    jnt_vel.clear();
    for (int i = 0; i < msg.position.size(); i++)
    {
        jnt_pos.push_back(msg.position[i]);
        jnt_vel.push_back(msg.velocity[i]);
    }
}

void arucoPoseCallback(const geometry_msgs::PoseStamped & msg)   // Function needed for HW03 Step 2c copied from kdl_robot_vision_control.cpp
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
    ros::Subscriber joint_state_sub = n.subscribe("/iiwa/joint_states", 1, jointStateCallback);
    ros::Subscriber aruco_pose_sub = n.subscribe("/aruco_single/pose", 1, arucoPoseCallback);  // Subscriber added for HW03 Step 2c

    // Publishers
    ros::Publisher joint1_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_1_effort_controller/command", 1);
    ros::Publisher joint2_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_2_effort_controller/command", 1);
    ros::Publisher joint3_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_3_effort_controller/command", 1);
    ros::Publisher joint4_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_4_effort_controller/command", 1);
    ros::Publisher joint5_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_5_effort_controller/command", 1);
    ros::Publisher joint6_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_6_effort_controller/command", 1);
    ros::Publisher joint7_effort_pub = n.advertise<std_msgs::Float64>("/iiwa/iiwa_joint_7_effort_controller/command", 1);    

    // Services
    ros::ServiceClient robot_set_state_srv = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
    ros::ServiceClient pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");

    // Set robot state
    gazebo_msgs::SetModelConfiguration robot_init_config;
    robot_init_config.request.model_name = "iiwa";
    robot_init_config.request.urdf_param_name = "robot_description";
    robot_init_config.request.joint_names.push_back("iiwa_joint_1");
    robot_init_config.request.joint_names.push_back("iiwa_joint_2");
    robot_init_config.request.joint_names.push_back("iiwa_joint_3");
    robot_init_config.request.joint_names.push_back("iiwa_joint_4");
    robot_init_config.request.joint_names.push_back("iiwa_joint_5");
    robot_init_config.request.joint_names.push_back("iiwa_joint_6");
    robot_init_config.request.joint_names.push_back("iiwa_joint_7");
    robot_init_config.request.joint_positions.push_back(0.0);
    robot_init_config.request.joint_positions.push_back(1.57);
    robot_init_config.request.joint_positions.push_back(-1.57);
    robot_init_config.request.joint_positions.push_back(-1.2);
    robot_init_config.request.joint_positions.push_back(1.57);
    robot_init_config.request.joint_positions.push_back(-1.57);
    //robot_init_config.request.joint_positions.push_back(-0.37);
    robot_init_config.request.joint_positions.push_back(1.57);     //  HW03 Step 2c: To make the camera face the marker


// ----------- HW03 Step 2c: Portion of code imported from kdl_robot_vision_control.cpp to implement the vision task -----------------------

    // init_jnt_pos[0] = 0.0;
    // init_jnt_pos[1] = 1.57;
    // init_jnt_pos[2] = -1.57;
    // init_jnt_pos[3] = -1.2;
    // init_jnt_pos[4] = 1.57;
    // init_jnt_pos[5] = -1.57;
    // //init_jnt_pos[6] = -0.37;
    // init_jnt_pos[6] = +1.57;
    // Eigen::VectorXd qdi = toEigen(init_jnt_pos);

// --------------------- HW03 Step 2c: End of the portion of code imported from kdl_robot_vision_control.cpp ----------------------------
    
    // Create robot
    KDLRobot robot = createRobot(argv[1]);
    robot.update(jnt_pos, jnt_vel);
    int nrJnts = robot.getNrJnts();

    if(robot_set_state_srv.call(robot_init_config))
        ROS_INFO("Robot state set.");
    else
        ROS_INFO("Failed to set robot state.");

    // Messages
    std_msgs::Float64 tau1_msg, tau2_msg, tau3_msg, tau4_msg, tau5_msg, tau6_msg, tau7_msg;
    std_srvs::Empty pauseSrv;

    // Joints
    KDL::JntArray qd(robot.getNrJnts()),dqd(robot.getNrJnts()),ddqd(robot.getNrJnts());
    qd.data.setZero();           //  HW03 Step 2c: line added from kdl_robot_vision_control.cpp
    dqd.data.setZero();
    ddqd.data.setZero();

    // Wait for robot and object state
    while (!(robot_state_available))
    {
        ROS_INFO_STREAM_ONCE("Robot/object state not available yet.");
        ROS_INFO_STREAM_ONCE("Please start gazebo simulation.");
        if (!(robot_set_state_srv.call(robot_init_config)))
            ROS_INFO("Failed to set robot state.");            
        
        ros::spinOnce();
        //loop_rate.sleep();     //  HW03 Step 2c: line added from the same loop I saw in kdl_robot_vision_control.cpp
    }

// ----------- HW03 Step 2c: Portion of code imported from kdl_robot_vision_control.cpp to implement the vision task -----------------------

    //  // Bring robot in a desired initial configuration with velocity control
    // ROS_INFO("Robot going into initial configuration....");
    // double jnt_position_error_norm = computeJointErrorNorm(toEigen(init_jnt_pos),toEigen(jnt_pos));
    // while (jnt_position_error_norm > 0.01)
    // {
    //     dqd.data = KP*(toEigen(init_jnt_pos) - toEigen(jnt_pos));

    //     // Set joints
    //     // dq1_msg.data = dqd.data[0];
    //     // dq2_msg.data = dqd.data[1];
    //     // dq3_msg.data = dqd.data[2];
    //     // dq4_msg.data = dqd.data[3];
    //     // dq5_msg.data = dqd.data[4];
    //     // dq6_msg.data = dqd.data[5];
    //     // dq7_msg.data = dqd.data[6];

    //     // Publish
    //     // joint1_dq_pub.publish(dq1_msg);
    //     // joint2_dq_pub.publish(dq2_msg);
    //     // joint3_dq_pub.publish(dq3_msg);
    //     // joint4_dq_pub.publish(dq4_msg);
    //     // joint5_dq_pub.publish(dq5_msg);
    //     // joint6_dq_pub.publish(dq6_msg);
    //     // joint7_dq_pub.publish(dq7_msg);

    //     jnt_position_error_norm = computeJointErrorNorm(toEigen(init_jnt_pos),toEigen(jnt_pos));
    //     std::cout << "jnt_position_error_norm: " << jnt_position_error_norm << "\n" << std::endl;
    //     ros::spinOnce();
    //     loop_rate.sleep();

    //}

   // --------------------- HW03 Step 2c: End of the portion of code imported from kdl_robot_vision_control.cpp ----------------------------

    // Specify an end-effector    ----->  HW03 Step 2c: I used the End Effector as defined in kdl_robot_vision_control.cpp (camera in flange transform)
    robot.addEE(KDL::Frame::Identity());
    KDL::Frame ee_T_cam;
    //ee_T_cam.M = KDL::Rotation::RotZ(1.570006);
    //ee_T_cam.M = KDL::Rotation::RotY(1.57)*KDL::Rotation::RotZ(-1.57)*KDL::Rotation::RotX(M_PI_2);
    ee_T_cam.M = KDL::Rotation::RotY(1.57)*KDL::Rotation::RotZ(-1.57);
    ee_T_cam.p = KDL::Vector(0,0,0.025);
    robot.addEE(ee_T_cam);

    // Torques
    Eigen::VectorXd tau;
    tau.resize(robot.getNrJnts());

    // Update robot
    robot.update(jnt_pos, jnt_vel);

    // Init controller
    KDLController controller_(robot);

    // EE's trajectory initial position
    KDL::Frame init_cart_pose = robot.getEEFrame();
    Eigen::Vector3d init_position(init_cart_pose.p.data);
    
    KDL::Frame Fi = init_cart_pose;      // HW03 Step 2c: renaming of these 2 elements to make the imported code work
    Eigen::Vector3d pdi = toEigen(Fi.p);

    // EE trajectory end position
    Eigen::Vector3d end_position;
    end_position << init_cart_pose.p.x(), -init_cart_pose.p.y(), init_cart_pose.p.z();

    // Plan trajectory
    double traj_duration = 4.0, acc_duration = 1.5, t = 0.0, init_time_slot = 1.0; // HW03 Step 2c: traj_duration = 1.5 --> 4.0, acc_duration = 0.5 --> 1.5
    double radius = 0.1;  // Original value = 0.18

//-------------------------------- HW02: Modifications for Step 3a --------------------------------------------

    // Constructors
    //KDLPlanner planner(traj_duration, acc_duration, init_position, end_position); // Linear Trajectory (selection = 1, 2, 3)
    KDLPlanner planner(traj_duration, init_position, radius); // Circular Trajectory (selection = 4, 5)
    
    // Retrieve the first trajectory point
    trajectory_point p = planner.compute_trajectory(t);
    
/*------------------- This commented code does not work because of scope issues -------------------------------

    int selection = 2;  // !! REMEMBER TO SELECT THE RIGHT TRAJECTORY IN "kdl_planner.cpp" (line 171)!!

    //  selection = 1 --> Original Linear Trajectory from Prof. Selvaggio
    //  selection = 2 --> Linear Trajectory with Trapezoidal Velocity Profile
    //  selection = 3 --> Linear Trajectory with Cubic Polynomial Velocity Profile
    //  selection = 4 --> Circular Trajectory with Trapezoidal Velocity Profile
    //  selection = 5 --> Circular Trajectory with Cubic Polynomial Velocity Profile

    //trajectory_point p;

    if (selection > 5) //  if CASE to be sure I am running a legit trajectory
    {
         selection = 2;
    }
    else if(selection == 1)  // Original Linear Trajectory
    {
        // Constructor for Linear Trajectory
        KDLPlanner planner(traj_duration, acc_duration, init_position, end_position);
            // Retrieve the first trajectory point
        //p = planner.compute_trajectory(t, selection);
    }
    else if(selection == 2)  // Linear Trajectory with Trapezoidal Velocity Profile
    {
        // Constructor for Linear Trajectory
        KDLPlanner planner(traj_duration, acc_duration, init_position, end_position);
            // Retrieve the first trajectory point
        //p = planner.compute_trajectory(t, selection);
    }
    else if(selection == 3)  // Linear Trajectory with Cubic Polynomial Velocity Profile
    {
        // Constructor for Linear Trajectory
        KDLPlanner planner(traj_duration, acc_duration, init_position, end_position);
            // Retrieve the first trajectory point
        //p = planner.compute_trajectory(t, selection);
    }
    else if(selection == 4)  // Circular Trajectory with Trapezoidal Velocity Profile
    {
        // Constructor for Circular Trajectory
        KDLPlanner planner(traj_duration, init_position, radius);
            // Retrieve the first trajectory point
        //p = planner.compute_trajectory(t, selection);
    }
    else if(selection == 5)  // Circular Trajectory with Cubic Polynomial Velocity Profile
    {
        // Constructor for Circular Trajectory
        KDLPlanner planner(traj_duration, init_position, radius);
            // Retrieve the first trajectory point
        //p = planner.compute_trajectory(t, selection);
    }

    // Retrieve the first trajectory point
    trajectory_point p = planner.compute_trajectory(t, selection);
    */

    // Gains
    double Kp = 20, Kd = sqrt(Kp);  //  Original Kp = 50

//-------------------------- HW02: End of the section to modify for Step 3a ---------------------------------

    // Retrieve initial simulation time
    ros::Time begin = ros::Time::now();
    ROS_INFO_STREAM_ONCE("Starting control loop ...");

    // Init trajectory
    KDL::Frame des_pose = KDL::Frame::Identity(); 
    KDL::Twist des_cart_vel = KDL::Twist::Zero(), des_cart_acc = KDL::Twist::Zero();
    
    des_pose.M = robot.getEEFrame().M;

    while ((ros::Time::now()-begin).toSec() < 2*traj_duration + init_time_slot)
    {
        //if (robot_state_available)
        if (robot_state_available && aruco_pose_available)    // HW03 Step 2c: I used the condition from kdl_robot_vision_control.cpp since now I am handling also the Aruco Marker
        {
            // Update robot
            robot.update(jnt_pos, jnt_vel);

            // Update time
            t = (ros::Time::now()-begin).toSec();
            std::cout << "time: " << t << std::endl;
        
            // Extract desired pose
            des_cart_vel = KDL::Twist::Zero();
            des_cart_acc = KDL::Twist::Zero();
            if (t <= init_time_slot) // wait a second
            {
                p = planner.compute_trajectory(0.0);
            }
            else if(t > init_time_slot && t <= traj_duration + init_time_slot)
            {
                p = planner.compute_trajectory(t-init_time_slot);
                
                des_cart_vel = KDL::Twist(KDL::Vector(p.vel[0], p.vel[1], p.vel[2]),KDL::Vector::Zero());
                des_cart_acc = KDL::Twist(KDL::Vector(p.acc[0], p.acc[1], p.acc[2]),KDL::Vector::Zero());
            }
            else
            {
                ROS_INFO_STREAM_ONCE("trajectory terminated");
                break;
            }

            des_pose.p = KDL::Vector(p.pos[0],p.pos[1],p.pos[2]);

    // ----------- HW03 Step 2c: Portion of code imported from kdl_robot_vision_control.cpp to implement the vision task -----------------------

            // compute current jacobians 
            KDL::Jacobian J_cam = robot.getEEJacobian();
            KDL::Frame cam_T_object(KDL::Rotation::Quaternion(aruco_pose[3], aruco_pose[4], aruco_pose[5], aruco_pose[6]), KDL::Vector(aruco_pose[0], aruco_pose[1], aruco_pose[2]));

            // look at point: compute rotation error from angle/axis
            Eigen::Matrix<double,3,1> aruco_pos_n = toEigen(cam_T_object.p); //(aruco_pose[0],aruco_pose[1],aruco_pose[2]);
            aruco_pos_n.normalize();
            Eigen::Vector3d r_o = skew(Eigen::Vector3d(0,0,1))*aruco_pos_n;
            double aruco_angle = std::acos(Eigen::Vector3d(0,0,1).dot(aruco_pos_n));
            KDL::Rotation Re = KDL::Rotation::Rot(KDL::Vector(r_o[0], r_o[1], r_o[2]), aruco_angle);

            des_pose.M = robot.getEEFrame().M*Re;
            
            // compute errors
            // Eigen::Matrix<double,3,1> e_o = computeOrientationError(toEigen(robot.getEEFrame().M*Re), toEigen(robot.getEEFrame().M));
            // Eigen::Matrix<double,3,1> e_o_w = computeOrientationError(toEigen(Fi.M), toEigen(robot.getEEFrame().M));
            // Eigen::Matrix<double,3,1> e_p = computeLinearError(pdi,toEigen(robot.getEEFrame().p));
            // Eigen::Matrix<double,6,1> x_tilde; x_tilde << e_p,  e_o_w[0], e_o[1], e_o[2];

            // resolved velocity control low
            //Eigen::MatrixXd J_pinv = J_cam.data.completeOrthogonalDecomposition().pseudoInverse();
            //dqd.data = lambda*J_pinv*x_tilde + 10*(Eigen::Matrix<double,7,7>::Identity() - J_pinv*J_cam.data)*(qdi - toEigen(jnt_pos));

    // --------------------- HW03 Step 2c: End of the portion of code imported from kdl_robot_vision_control.cpp ----------------------------


//----------------- Switch between control logics for HW02 Step 4 in this section of code ------------------

            //double error;
            // inverse kinematics
            qd.data << jnt_pos[0], jnt_pos[1], jnt_pos[2], jnt_pos[3], jnt_pos[4], jnt_pos[5], jnt_pos[6];
            //qd = robot.getInvKin(qd, des_pose);   // HW03 Step 2c: Original inverse kinematics
           
            qd = robot.getInvKin(qd, des_pose*robot.getFlangeEE().Inverse());   // HW03 Step 2c: New inverse kinematics for position...
            //dqd = robot.getInvKin(qd,des_cart_vel);                               // ... and velocity
            // resolved velocity control low
            // Eigen::MatrixXd J_pinv = J_cam.data.completeOrthogonalDecomposition().pseudoInverse();
            // dqd.data = lambda*J_pinv*x_tilde + 10*(Eigen::Matrix<double,7,7>::Identity() - J_pinv*J_cam.data)*(qdi - toEigen(jnt_pos));

            // joint space inverse dynamics control
            tau = controller_.idCntr(qd, dqd, ddqd, Kp, Kd);

            // double Kp = 50;  //  Original Kp = 1000 --> These values are too high
            // double Ko = 5;  //  Original Ko = 1000

            // // Cartesian space inverse dynamics control
            // tau = controller_.idCntr(des_pose, des_cart_vel, des_cart_acc,Kp, Ko, 2*sqrt(Kp), 2*sqrt(Ko));
            
//--------------------------- HW02: End of the section to modify for Step 4 ---------------------------------
        }
       
        else{      // "else" case introduced from kdl_robot_vision_control.cpp
            dqd.data = KP*(toEigen(init_jnt_pos) - toEigen(jnt_pos));
        }
            // Set torques
            tau1_msg.data = tau[0];
            tau2_msg.data = tau[1];
            tau3_msg.data = tau[2];
            tau4_msg.data = tau[3];
            tau5_msg.data = tau[4];
            tau6_msg.data = tau[5];
            tau7_msg.data = tau[6];

            // Publish
            joint1_effort_pub.publish(tau1_msg);
            joint2_effort_pub.publish(tau2_msg);
            joint3_effort_pub.publish(tau3_msg);
            joint4_effort_pub.publish(tau4_msg);
            joint5_effort_pub.publish(tau5_msg);
            joint6_effort_pub.publish(tau6_msg);
            joint7_effort_pub.publish(tau7_msg);

            ros::spinOnce();
            loop_rate.sleep();
    }    
    
    if(pauseGazebo.call(pauseSrv))
        ROS_INFO("Simulation paused.");
    else
        ROS_INFO("Failed to pause simulation.");

    return 0;
}