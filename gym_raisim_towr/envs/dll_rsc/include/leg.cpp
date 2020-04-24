#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#define PI  3.141592
//MONOPED LINK LENGTHS
#define L1 0.08
#define L2 0.35
#define L3 0.33

void forward_kinematics(Eigen::Vector3d &end_effector_pos, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Performs forward kinematics for a 3R spatial manipulator.
    thetas = [abduction, hip, knee] angles in radians
    end_effector_pos = [x,y,z] in base frame
    l1, l2, l3 = link lengths
    */
    end_effector_pos[0] = cos(thetas[0])*(l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]));
    end_effector_pos[1] = (l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]))*sin(thetas[0]);
    end_effector_pos[2] = l1 + l2*sin(thetas[1]) + l3*sin(thetas[1] + thetas[2]);
}


void forward_kinematics_2(Eigen::Vector3d &end_effector_pos, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Performs forward kinematics for a 3R spatial manipulator.
    thetas = [abduction, hip, knee] angles in radians
    end_effector_pos = [x,y,z] in base frame
    l1, l2, l3 = link lengths
    */
    end_effector_pos[0] = cos(thetas[0])*(l1+l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]));
    end_effector_pos[1] = sin(thetas[0])*(l1+l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]));
    end_effector_pos[2] = l2*sin(thetas[1]) + l3*sin(thetas[1] + thetas[2]);
}





void inverse_kinematics_branch1(Eigen::Vector3d &thetas, Eigen::Vector3d &end_effector_pos,  float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Performs inverse kinematics for a 3R spatial manipulator -- BRANCH1
    thetas = [abduction, hip, knee] angles in radians
    end_effector_pos = [x,y,z] in base frame
    l1, l2, l3 = link lengths
    */
   float theta23, t, r;
   float x = end_effector_pos[0];
   float y = end_effector_pos[1];
   float z = end_effector_pos[2];
   thetas[0] = atan2(y, x);
   r = sqrt(x*x + y*y);
   t = (-4*l1*l3 + 4*l3*z - sqrt(pow(4*l1*l3 - 4*l3*z,2) - 4*(pow(l1,2) - pow(l2,2) + pow(l3,2) - 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2))*
        (pow(l1,2) - pow(l2,2) + pow(l3,2) + 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2))))/(2.*(pow(l1,2) - pow(l2,2) + pow(l3,2) + 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2)));
   theta23 = atan2(2*t, 1-t*t);
   thetas[1] = atan2(-l1 + z - l3*sin(theta23), r - l3*cos(theta23));
   thetas[2] = theta23 - thetas[1];
}

void inverse_kinematics_branch2(Eigen::Vector3d &thetas, Eigen::Vector3d &end_effector_pos,  float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Performs inverse kinematics for a 3R spatial manipulator -- BRANCH2
    thetas = [abduction, hip, knee] angles in radians
    end_effector_pos = [x,y,z] in base frame
    l1, l2, l3 = link lengths
    */
   float theta23, t, r;
   float x = end_effector_pos[0];
   float y = end_effector_pos[1];
   float z = end_effector_pos[2];
   thetas[0] = atan2(y, x);
   r = sqrt(x*x + y*y);
   t = (-4*l1*l3 + 4*l3*z + sqrt(pow(4*l1*l3 - 4*l3*z,2) - 4*(pow(l1,2) - pow(l2,2) + pow(l3,2) - 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2))*
        (pow(l1,2) - pow(l2,2) + pow(l3,2) + 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2))))/(2.*(pow(l1,2) - pow(l2,2) + pow(l3,2) + 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2)));
   theta23 = atan2(2*t, 1-t*t);
   thetas[1] = atan2(-l1 + z - l3*sin(theta23), r - l3*cos(theta23));
   thetas[2] = theta23 - thetas[1];
}
void inverse_kinematics_branch3(Eigen::Vector3d &thetas, Eigen::Vector3d &end_effector_pos,  float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Performs inverse kinematics for a 3R spatial manipulator -- BRANCH3
    thetas = [abduction, hip, knee] angles in radians
    end_effector_pos = [x,y,z] in base frame
    l1, l2, l3 = link lengths
    */
   float theta23, t, r;
   float x = end_effector_pos[0];
   float y = end_effector_pos[1];
   float z = end_effector_pos[2];
   thetas[0] = PI + atan2(y, x);
   r = -1*sqrt(x*x + y*y);
   t = (-4*l1*l3 + 4*l3*z - sqrt(pow(4*l1*l3 - 4*l3*z,2) - 4*(pow(l1,2) - pow(l2,2) + pow(l3,2) - 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2))*
        (pow(l1,2) - pow(l2,2) + pow(l3,2) + 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2))))/(2.*(pow(l1,2) - pow(l2,2) + pow(l3,2) + 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2)));
   theta23 = atan2(2*t, 1-t*t);
   thetas[1] = atan2(-l1 + z - l3*sin(theta23), r - l3*cos(theta23));
   thetas[2] = theta23 - thetas[1];
}
void inverse_kinematics_branch4(Eigen::Vector3d &thetas, Eigen::Vector3d &end_effector_pos,  float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Performs inverse kinematics for a 3R spatial manipulator -- BRANCH3
    thetas = [abduction, hip, knee] angles in radians
    end_effector_pos = [x,y,z] in base frame
    l1, l2, l3 = link lengths
    */
   float theta23, t, r;
   float x = end_effector_pos[0];
   float y = end_effector_pos[1];
   float z = end_effector_pos[2];
   thetas[0] = PI + atan2(y, x);
   r = -1*sqrt(x*x + y*y);
   t = (-4*l1*l3 + 4*l3*z + sqrt(pow(4*l1*l3 - 4*l3*z,2) - 4*(pow(l1,2) - pow(l2,2) + pow(l3,2) - 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2))*
        (pow(l1,2) - pow(l2,2) + pow(l3,2) + 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2))))/(2.*(pow(l1,2) - pow(l2,2) + pow(l3,2) + 2*l3*r + pow(r,2) - 2*l1*z + pow(z,2)));
   theta23 = atan2(2*t, 1-t*t);
   thetas[1] = atan2(-l1 + z - l3*sin(theta23), r - l3*cos(theta23));
   thetas[2] = theta23 - thetas[1];
}
void inverse_kinematics(Eigen::Vector3d &thetas, Eigen::Vector3d &end_effector_pos, int branch,  float l1 = L1, float l2 = L2, float l3 = L3)
{
    if(branch == 1)
    {
        inverse_kinematics_branch1(thetas, end_effector_pos, l1, l2, l3);
    }
    if(branch == 2)
    {
        inverse_kinematics_branch2(thetas, end_effector_pos, l1, l2, l3);
    }
    if(branch == 3)
    {
        inverse_kinematics_branch3(thetas, end_effector_pos, l1, l2, l3);
    }
    if(branch == 4)
    {
        inverse_kinematics_branch4(thetas, end_effector_pos, l1, l2, l3);
    }
}


void jacobian(Eigen::Matrix<double, 3, 3> &jacob, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Finds Jacobian of the end effector given a certain configuration of the robot
    thetas = [abduction, hip, knee] angles in radians
    jacob = Dq/Dthetas where q = [x,y,z]
    l1, l2, l3 = link lengths
    */
   jacob << -(l2*cos(thetas[1])*sin(thetas[0])) - l3*cos(thetas[1] + thetas[2])*sin(thetas[0]),-(l2*cos(thetas[0])*sin(thetas[1])) - l3*cos(thetas[0])*sin(thetas[1] + thetas[2]),-(l3*cos(thetas[0])*sin(thetas[1] + thetas[2])),
   l2*cos(thetas[0])*cos(thetas[1]) + l3*cos(thetas[0])*cos(thetas[1] + thetas[2]),-(l2*sin(thetas[0])*sin(thetas[1])) - l3*sin(thetas[0])*sin(thetas[1] + thetas[2]),-(l3*sin(thetas[0])*sin(thetas[1] + thetas[2])),
   0,l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2]),l3*cos(thetas[1] + thetas[2]);
}


void jacobian_2(Eigen::Matrix<double, 3, 3> &jacob, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Finds Jacobian of the end effector given a certain configuration of the robot
    thetas = [abduction, hip, knee] angles in radians
    jacob = Dq/Dthetas where q = [x,y,z]
    l1, l2, l3 = link lengths
    */
   jacob << -sin(thetas[0])*(l1+l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2])),//dx/d01
            
            -cos(thetas[0])*(   l2*sin(thetas[1]) + l3*sin(thetas[1] + thetas[2])),//dx/d02
            
            -cos(thetas[0])*l3*sin(thetas[1]+thetas[2]), //dx/d03

            
             cos(thetas[0])*(l1+l2*cos(thetas[1]) + l3*cos(thetas[1] + thetas[2])),//dy/d01
            
             -sin(thetas[0])*(   l2*sin(thetas[1]) + l3*sin(thetas[1] + thetas[2])),//dy/d02
            
             -sin(thetas[0])*l3*sin(thetas[1]+thetas[2]),//dy/d03
             
             0,//dz/d01
            
             l2*cos(thetas[1]) + l3*cos(thetas[1]+thetas[2]),//dz/d02
            
             l3*cos(thetas[1]+thetas[2]);//dz/d03



}





void inverse_dynamics(Eigen::Vector3d &torques, Eigen::Vector3d &force, Eigen::Vector3d &thetas, float l1 = L1, float l2 = L2, float l3 = L3)
{
    /*
    Finds Joint torques for desired force at end effector
    torques =  [abduction, hip, knee]
    force = [x,y,z]
    thetas = [abduction, hip, knee] angles in radians
    l1, l2, l3 = link lengths
    */
    Eigen::Matrix<double,3,3> jacob;
    jacobian(jacob, thetas, l1, l2, l3);
    torques = jacob.transpose()*force; 
}  


void chk_forward_k(Eigen::Vector3d q0 )
{
Eigen::Vector3d EE_co_l;
forward_kinematics_2(EE_co_l,q0);
std::cout<<std::endl<<"\nEE_co_predicted by fk in leg frame:\n"<<EE_co_l;
Eigen::Matrix<double,3,3> Rot_hl;
                  Rot_hl << 0, 0,-1,
                            0,-1, 0,
                           -1, 0, 0;
std::cout<<std::endl<<"\nEE_co_predicted by fk in hip frame:\n"<<Rot_hl*EE_co_l;


}



/*
int main()
{
    Eigen::Vector3d thetas(0,0,0);

    for(int i=0;i<3;i++)
    {
      std::cout<<"\n"<<"theta"<<i<<":";
      std::cin>>thetas(i);
    }
    float l1 = 1.;
    float l2 = 1.;
    float l3 = 1.;
    Eigen::Vector3d pos;
    Eigen::Vector3d thetas2;
    //Testing inverse Kinematics
    forward_kinematics_2(pos, thetas);
    std::cout<<"\nLeg_Frame:\t"<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
    double initial_height = 0.91;
    Eigen::Vector3d base(0,0,initial_height);
    std::cout<<"\nbase in raisim is @:\n"<<base<<std::endl;
    
    //std::cout<<"\nBase_Frame:\t"<<pos[2]<<" "<<pos[1]<<" "<<pos[0]+0.23<<std::endl;
    Eigen::Matrix<double,3,3> Rot_legframe_to_baseframe;
    Rot_legframe_to_baseframe << 0, 0,-1,
                                 0,-1, 0,
                                -1, 0, 0;

    pos = Rot_legframe_to_baseframe*pos;
    std::cout<<"\nwrt_hip_frame:\t"<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;

    /*inverse_kinematics_branch1(thetas, pos, l1, l2, l3);
    forward_kinematics(pos, thetas);
    std::cout<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
    inverse_kinematics_branch2(thetas, pos, l1, l2, l3);
    forward_kinematics(pos, thetas);
    std::cout<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
    inverse_kinematics_branch3(thetas, pos, l1, l2, l3);
    forward_kinematics(pos, thetas);
    std::cout<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
    inverse_kinematics_branch4(thetas, pos, l1, l2, l3);
    forward_kinematics(pos, thetas);
    std::cout<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
    inverse_kinematics(thetas, pos, 1);
    forward_kinematics(pos, thetas);
    std::cout<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
    
    //Testing Jacobian -- eqs are correct
    Eigen::Matrix<double, 3, 3> jacob;
    jacobian_2(jacob, thetas);
    std::cout<<std::endl<<"Jacobian:"<<std::endl;
    std::cout<<jacob<<std::endl;
    
    Eigen::Matrix<double, 3, 3> jacob_inv = (Rot_legframe_to_baseframe*jacob).inverse();

    Eigen::Vector3d tou(-0.926493,-0.0524843,-7.40437e-07);//0.2,0.2,-0.6
    std::cout<<std::endl<<"Generalized_force:"<<std::endl<<tou;

   std::cout<<std::endl<<"((R*J)^-1)^T * tou"<<std::endl<<jacob_inv.transpose()*tou; 

    //Testing Inverse Dynamics-- eqs are correct
    Eigen::Vector3d forces(1,1,1);
    Eigen::Vector3d torques;
    inverse_dynamics(torques, forces,thetas);
    std::cout<<"torques: "<<torques<<std::endl;
    return 0;
}*/