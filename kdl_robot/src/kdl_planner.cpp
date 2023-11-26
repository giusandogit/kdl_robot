#include "kdl_ros_control/kdl_planner.h"
#include <math.h>

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

//-------------------------------- Step 2a: New Constructor ---------------------------------------------------

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _radius)
{
 trajDuration_ = _trajDuration;
 trajInit_ = _trajInit;
 radius_=_radius; 
}

//-------------------------------------- End of Step 2a -----------------------------------------------


void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

//-------------------------------- Step 1a: Trapezoidal Velocity Profile computation ---------------------------------------------------

void KDLPlanner::trapezoidal_vel (double time, double &s, double &s_d, double &s_dd)
{
  double s_f=std::sqrt(std::pow(trajEnd_(0),2)+std::pow(trajEnd_(1),2)+std::pow(trajEnd_(2),2));
  double s_i=std::sqrt(std::pow(trajInit_(0),2)+std::pow(trajInit_(1),2)+std::pow(trajInit_(2),2));

  double ddot_traj_c = -(s_f-s_i)/(std::pow(accDuration_,2)+trajDuration_*accDuration_);

   if(time <= accDuration_)
  {
    s = s_i + 0.5*ddot_traj_c*std::pow(time,2);
    s_d = ddot_traj_c*time;
    s_dd = ddot_traj_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    s = s_i + ddot_traj_c*accDuration_*(time-accDuration_/2);
    s_d = ddot_traj_c*accDuration_;
    s_dd = 0;
  }
  else
  {
    s = s_f - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    s_d = ddot_traj_c*(trajDuration_-time);
    s_dd = -ddot_traj_c;
  }
  
}
//---------------------------------------------- End of Step 1a -----------------------------------------------

//-------------------------------- Step 1b: Cubic Polynomial Velocity Profile computation ------------------------------------------------

void KDLPlanner::cubic_polinomial (double time, double &s, double &s_d, double &s_dd)
{
  double s_i=std::sqrt(std::pow(trajInit_(0),2)+std::pow(trajInit_(1),2)+std::pow(trajInit_(2),2));
  double s_f=std::sqrt(std::pow(trajEnd_(0),2)+std::pow(trajEnd_(1),2)+std::pow(trajEnd_(2),2));

  // Coefficients computed offline based on boundary conditions
        double a0 = s_i;
        double a1 = 0;
        double a2 = 3*(s_f-s_i)/std::pow(trajDuration_,2);
        double a3 = -2*(s_f-s_i)/std::pow(trajDuration_,3);

  // Calculate s(t) = curvilinear abscissa (arc length) 
        s = a3*std::pow(time,3) + a2* std::pow(time,2) + a1*time + a0;
 
  // Calculate the first derivative of s(t)
        s_d = 3*a3* std::pow(time,2) + 2*a2*time + a1;
 
  // Calculate the second derivative of s(t)
        s_dd = 6*a3*time + 2*a2;
    
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}
//---------------------------------------------- End of Step 1b -----------------------------------------------

//--------------------------- Original compute_trajectory (UNCOMMENT to restore) --------------------------------------

// trajectory_point KDLPlanner::compute_trajectory(double time)
// {
//   /*trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
//   time = current time
//   trajDuration_  = final time
//   accDuration_   = acceleration time
//   trajInit_ = trajectory initial point
//   trajEnd_  = trajectory final point
// */

//   trajectory_point traj;

//   Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);

//   if(time <= accDuration_)
//   {
//     traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
//     traj.vel = ddot_traj_c*time;
//     traj.acc = ddot_traj_c;
//   }
//   else if(time <= trajDuration_-accDuration_)
//   {
//     traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
//     traj.vel = ddot_traj_c*accDuration_;
//     traj.acc = Eigen::Vector3d::Zero();
//   }
//   else
//   {
//     traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
//     traj.vel = ddot_traj_c*(trajDuration_-time);
//     traj.acc = -ddot_traj_c;
//   }

//   return traj;
// }

//--------------------------------- Modified compute_trajectory for Step 2 --------------------------------------------

trajectory_point KDLPlanner::compute_trajectory(double time)
{
int selection = 4; // !! REMEMBER TO UNCOMMENT THE RIGHT CONSTRUCTOR IN "kdl_robot_test.cpp" (lines 157, 158)!!
/*
    selection = 1 --> Original Linear Trajectory from Prof. Selvaggio
    selection = 2 --> Linear Trajectory with Trapezoidal Velocity Profile
    selection = 3 --> Linear Trajectory with Cubic Polynomial Velocity Profile
    selection = 4 --> Circular Trajectory with Trapezoidal Velocity Profile
    selection = 5 --> Circular Trajectory with Cubic Polynomial Velocity Profile
*/

  if (selection == 1)  // Original Linear Trajectory
{
  /*trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
  time = current time
  trajDuration_  = final time
  accDuration_   = acceleration time
  trajInit_ = trajectory initial point
  trajEnd_  = trajectory final point
*/
  trajectory_point traj;

  Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);

  if(time <= accDuration_)
  {
    traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
    traj.vel = ddot_traj_c*time;
    traj.acc = ddot_traj_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
    traj.vel = ddot_traj_c*accDuration_;
    traj.acc = Eigen::Vector3d::Zero();
  }
  else
  {
    traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    traj.vel = ddot_traj_c*(trajDuration_-time);
    traj.acc = -ddot_traj_c;
  }

  return traj;

}
    //---------------- Step 2c: Linear Trajectory computation -----------------------------------

else if (selection == 2)  // Linear Trajectory with Trapezoidal Velocity Profile
{
 trajectory_point traj;

  double s,s_d,s_dd;
  trapezoidal_vel(time, s, s_d, s_dd);    

  traj.pos = trajInit_ + s * (trajEnd_ - trajInit_)/(trajEnd_-trajInit_).norm();
  traj.vel = s_d * (trajEnd_ - trajInit_)/(trajEnd_-trajInit_).norm();
  traj.acc = s_dd * (trajEnd_ - trajInit_)/(trajEnd_-trajInit_).norm();

  return traj; 
    
}

else if (selection == 3)  // Linear Trajectory with Cubic Polynomial Velocity Profile
{
  trajectory_point traj;

  double s,s_d,s_dd;
  cubic_polinomial(time, s, s_d, s_dd);    

  traj.pos = trajInit_ + s * (trajEnd_ - trajInit_)/(trajEnd_-trajInit_).norm();
  traj.vel = s_d * (trajEnd_ - trajInit_)/(trajEnd_-trajInit_).norm();
  traj.acc = s_dd * (trajEnd_ - trajInit_)/(trajEnd_-trajInit_).norm();

  return traj; 

}

    //---------------------------------- End of Step 2c -------------------------------------------

    //----------------------- Step 2b: Circular Trajectory computation ----------------------------
    
else  if (selection == 4)  // Circular Trajectory with Trapezoidal Velocity Profile
{
  trajectory_point traj;

  double s,s_d,s_dd;
  trapezoidal_vel(time, s, s_d, s_dd);

  traj.pos(0) = trajInit_(0);
  traj.pos(1) = trajInit_(1)- radius_*(std::cos(2*M_PI*s));
  traj.pos(2) = trajInit_(2) - radius_*(std::sin(2*M_PI*s));

  traj.vel(0) = 0;
  traj.vel(1) = 2*M_PI*radius_*std::sin(2*M_PI*s)*s_d;
  traj.vel(2) = -2*M_PI*radius_*std::cos(2*M_PI*s)*s_d;
  
  traj.acc(0) = 0;
  traj.acc(1) = 2*M_PI*radius_*(std::cos(2*M_PI*s)*2*M_PI*std::pow(s_d,2)+std::sin(2*M_PI*s)*s_dd);
  traj.acc(2) = 2*M_PI*radius_*(std::sin(2*M_PI*s)*2*M_PI*std::pow(s_d,2)-std::cos(2*M_PI*s)*s_dd);

  return traj;
  
}

else if (selection == 5) // Circular Trajectory with Cubic Polynomial Velocity Profile
{
  trajectory_point traj;

  double s,s_d,s_dd;
  cubic_polinomial(time, s, s_d, s_dd);

  traj.pos(0) = trajInit_(0);
  traj.pos(1) = trajInit_(1)- radius_*(std::cos(2*M_PI*s));
  traj.pos(2) = trajInit_(2) - radius_*(std::sin(2*M_PI*s));

  traj.vel(0) = 0;
  traj.vel(1) = 2*M_PI*radius_*std::sin(2*M_PI*s)*s_d;
  traj.vel(2) = -2*M_PI*radius_*std::cos(2*M_PI*s)*s_d;
  
  traj.acc(0) = 0;
  traj.acc(1) = 2*M_PI*radius_*(std::cos(2*M_PI*s)*2*M_PI*std::pow(s_d,2)+std::sin(2*M_PI*s)*s_dd);
  traj.acc(2) = 2*M_PI*radius_*(std::sin(2*M_PI*s)*2*M_PI*std::pow(s_d,2)-std::cos(2*M_PI*s)*s_dd);

  return traj;

}

    //---------------------------------- End of Step 2b -------------------------------------------
//-------------------------------------- End of Step 2 -----------------------------------------------


}
