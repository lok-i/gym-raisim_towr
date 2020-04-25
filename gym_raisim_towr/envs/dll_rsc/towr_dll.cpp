/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <cmath>
#include <iostream>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>
#include <xpp_inv/hyqleg_inverse_kinematics.h>
#include <xpp_inv/cartesian_declarations.h>

//#define base_height_initial 0.54
#define towr_initial_output false
using namespace towr;

// A minimal example how to build a trajectory optimization problem using TOWR.
//
// The more advanced example that includes ROS integration, GUI, rviz
// visualization and plotting can be found here:
// towr_ros/src/towr_ros_app.cc
struct Trajectory_data
{

float base_linear[3];
float base_angular[3];
float ee_liner[3];
float ee_force[3];
//float tau[3];

};





void towr_trajectory(NlpFormulation &formulation,SplineHolder &solution,Eigen::Vector3d target,float base_height_initial,Trajectory_data* data,int no_of_samples)
{


  // terrain
  formulation.terrain_ = std::make_shared<FlatGround>(0.0);

  // Kinematic limits and dynamic parameters of the hopper
  formulation.model_ = RobotModel(RobotModel::Monoped);

  // set the initial position of the hopper
  formulation.initial_base_.lin.at(kPos).z() = base_height_initial;
  formulation.initial_ee_W_.push_back(Eigen::Vector3d::Zero());

  // define the desired goal state of the hopper
  formulation.final_base_.lin.at(towr::kPos) =target;

  // Parameters that define the motion. See c'tor for default values or
  // other values that can be modified.
  // First we define the initial phase durations, that can however be changed
  // by the optimizer. The number of swing and stance phases however is fixed.
  // alternating stance and swing:     ____-----_____-----_____-----_____
  formulation.params_.ee_phase_durations_.push_back({0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.2});
  formulation.params_.ee_in_contact_at_start_.push_back(true);

  // Initialize the nonlinear-programming problem with the variables,
  // constraints and costs.
  ifopt::Problem nlp;

  for (auto c : formulation.GetVariableSets(solution))
    nlp.AddVariableSet(c);
  for (auto c : formulation.GetConstraints(solution))
    nlp.AddConstraintSet(c);
  for (auto c : formulation.GetCosts())
    nlp.AddCostSet(c);

  // You can add your own elements to the nlp as well, simply by calling:
  // nlp.AddVariablesSet(your_custom_variables);
  // nlp.AddConstraintSet(your_custom_constraints);

  // Choose ifopt solver (IPOPT or SNOPT), set some parameters and solve.
  // solver->SetOption("derivative_test", "first-order");
  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  solver->SetOption("max_cpu_time", 20.0);
  solver->Solve(nlp);

  // Can directly view the optimization variables through:
  // Eigen::VectorXd x = nlp.GetVariableValues()
  // However, it's more convenient to access the splines constructed from these
  // variables and query their values at specific times:
  using namespace std;
  cout.precision(2);
  nlp.PrintCurrent(); // view variable-set, constraint violations, indices,...
  cout << fixed;
  




 
  cout << "\n====================\nMonoped trajectory:\n====================\n";

  double t = 0.0;

  //possible bug-might not get to the xact target position
  //need to rectify
  double time_step = (solution.base_linear_->GetTotalTime() + 1e-5)/no_of_samples;
  //std::cout<<"\ntotal_Time:\t"<<solution.base_linear_->GetTotalTime() + 1e-5<<"\n";
int i =0;

  //while (t<=solution.base_linear_->GetTotalTime() + 1e-5)
  while(i<=no_of_samples)
  {
    if(t>2)
      t=2;
    //base angles to radian
    Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
    rad =  rad/M_PI*180;


    for(int j=0;j<3;j++)
        {    

             (data+i)->base_linear[j] = solution.base_linear_->GetPoint(t).p().transpose()[j];
             
             
             (data+i)->base_angular[j] = rad[j];
             
             (data+i)->ee_liner[j] = solution.ee_motion_.at(0)->GetPoint(t).p().transpose()[j];

             (data+i)->ee_force[j] = solution.ee_force_.at(0)->GetPoint(t).p().transpose()[j];            

        }
    //cout<<"\n"<<t<<"\n";
    i+=1;
    t+=time_step;

  }

//cout<<"\ni:"<<i<<"\n";

 if(towr_initial_output)
  {
   t= 0.0;
  while (t<=solution.base_linear_->GetTotalTime() + 1e-5) {
     if(t>2)
      t=2;
    cout << "t=" << t << "\n";
    cout << "Base linear position x,y,z:   \t";
    cout << solution.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;

    cout << "Base Euler roll, pitch, yaw:  \t";
    Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
    cout << (rad/M_PI*180).transpose() << "\t[deg]" << endl;


    
    cout << "Foot position x,y,z:          \t";
    cout << solution.ee_motion_.at(0)->GetPoint(t).p().transpose() << "\t[m]" << endl;
 

    cout << "Contact force x,y,z:          \t";
    cout << solution.ee_force_.at(0)->GetPoint(t).p().transpose() << "\t[N]" << endl;

    bool contact = solution.phase_durations_.at(0)->IsContactPhase(t);
    std::string foot_in_contact = contact? "yes" : "no";
    cout << "Foot in contact:              \t" + foot_in_contact << endl;

    cout << endl;

    t += time_step;
  }}
}



extern "C"{

void Trajectory(Trajectory_data* data,float i_h,float t[3],int no_of_samples)
{
  

   
  NlpFormulation formulation;
  SplineHolder solution;
  Eigen::Vector3d target;
  //std::cout <<"\nEnter Target Co-ordinates:\n";
  
  target(0)=t[0];
  target(1)=t[1];
  target(2)=t[3]; //doesnt matter coz plain terrain - always 0.54

  

  towr_trajectory(formulation,solution,target,i_h,data,no_of_samples);


}
}
/*
int main()
{
int no_of_samples = 11;
Trajectory_data t[no_of_samples];
float target[3];
target[0]=1;
target[1]=1;
target[2]=0.54;

Trajectory(t,0.54,target,no_of_samples);

for(int j =0 ;j<no_of_samples+1;j++)
{  
std::cout<<"\n\nSample_No:"<<j<<"\n";

for (int i=0;i<3;i++){
              
             std::cout<<"\n\n"<<t[j].base_linear[i] ;
             
             
             std::cout<<"\n\n"<<t[j].base_angular[i] ;
             
             std::cout<<"\n\n"<<t[j].ee_liner[i] ;

             std::cout<<"\n\n"<<t[j].ee_force[i] ;  

}}

}*/