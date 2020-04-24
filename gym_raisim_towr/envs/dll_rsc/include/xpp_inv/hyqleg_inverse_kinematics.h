/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

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

#ifndef XPP_VIS_HYQLEG_INVERSE_KINEMATICS_H_
#define XPP_VIS_HYQLEG_INVERSE_KINEMATICS_H_

#include <Eigen/Dense>
#include <xpp_inv/cartesian_declarations.h>

namespace xpp {

enum HyqJointID {HAA=0, HFE, KFE, HyqlegJointCount};

/**
 * @brief Converts a hyq foot position to joint angles.
 */
class HyqlegInverseKinematics {
public:
  using Vector3d = Eigen::Vector3d;
  enum KneeBend { Forward, Backward };

  /**
   * @brief Default c'tor initializing leg lengths with standard values.
   */
  HyqlegInverseKinematics () = default;
  virtual ~HyqlegInverseKinematics () = default;

  /**
   * @brief Returns the joint angles to reach a Cartesian foot position.
   * @param ee_pos_H  Foot position xyz expressed in the frame attached
   * at the hip-aa (H).
   */
  Vector3d GetJointAngles(Vector3d& ee_pos_H, KneeBend bend=Forward) const
  {
  double q_HAA_bf, q_HAA_br, q_HFE_br; // rear bend of knees
  
  double q_HFE_bf, q_KFE_br, q_KFE_bf; // forward bend of knees

  Eigen::Vector3d xr;
  Eigen::Matrix3d R;

  // translate to the local coordinate of the attachment of the leg
  // and flip coordinate signs such that all computations can be done
  // for the front-left leg
  xr = ee_pos_H;

  // compute the HAA angle
  q_HAA_bf = q_HAA_br = -atan2(xr[Y],-xr[Z]);

  // rotate into the HFE coordinate system (rot around X)
  R << 1.0, 0.0, 0.0, 0.0, cos(q_HAA_bf), -sin(q_HAA_bf), 0.0, sin(q_HAA_bf), cos(q_HAA_bf);

  xr = (R * xr).eval();

  // translate into the HFE coordinate system (along Z axis)
  xr += hfe_to_haa_z;  //distance of HFE to HAA in z direction

  // compute square of length from HFE to foot
  double tmp1 = pow(xr[X],2)+pow(xr[Z],2);


  // compute temporary angles (with reachability check)
  double lu = length_thigh;  // length of upper leg
  double ll = length_shank;  // length of lower leg
  double alpha = atan2(-xr[Z],xr[X]) - 0.5*M_PI;  //  flip and rotate to match HyQ joint definition


  double some_random_value_for_beta = (pow(lu,2)+tmp1-pow(ll,2))/(2.*lu*sqrt(tmp1)); // this must be between -1 and 1
  if (some_random_value_for_beta > 1) {
    some_random_value_for_beta = 1;
  }
  if (some_random_value_for_beta < -1) {
    some_random_value_for_beta = -1;
  }
  double beta = acos(some_random_value_for_beta);

  // compute Hip FE angle
  q_HFE_bf = q_HFE_br = alpha + beta;


  double some_random_value_for_gamma = (pow(ll,2)+pow(lu,2)-tmp1)/(2.*ll*lu);
  // law of cosines give the knee angle
  if (some_random_value_for_gamma > 1) {
    some_random_value_for_gamma = 1;
  }
  if (some_random_value_for_gamma < -1) {
    some_random_value_for_gamma = -1;
  }
  double gamma  = acos(some_random_value_for_gamma);


  q_KFE_bf = q_KFE_br = gamma - M_PI;

  // forward knee bend
  EnforceLimits(q_HAA_bf, HAA);
  EnforceLimits(q_HFE_bf, HFE);
  EnforceLimits(q_KFE_bf, KFE);

  // backward knee bend
  EnforceLimits(q_HAA_br, HAA);
  EnforceLimits(q_HFE_br, HFE);
  EnforceLimits(q_KFE_br, KFE);

  if (bend==Forward)
    return Vector3d(q_HAA_bf, q_HFE_bf, q_KFE_bf);
  else // backward
    return Vector3d(q_HAA_br, -q_HFE_br, -q_KFE_br);
}

  /**
   * @brief Restricts the joint angles to lie inside the feasible range
   * @param q[in/out]  Current joint angle that is adapted if it exceeds
   * the specified range.
   * @param joint  Which joint (HAA, HFE, KFE) this value represents.
   */
  void EnforceLimits(double& val, HyqJointID joint) const
  {
  // totally exaggerated joint angle limits
  const static double haa_min = -180;
  const static double haa_max =  90;

  const static double hfe_min = -90;
  const static double hfe_max =  90;

  const static double kfe_min = -180;
  const static double kfe_max =  0;

  // reduced joint angles for optimization
  static const std::map<HyqJointID, double> max_range {
    {HAA, haa_max/180.0*M_PI},
    {HFE, hfe_max/180.0*M_PI},
    {KFE, kfe_max/180.0*M_PI}
  };

  // reduced joint angles for optimization
  static const std::map<HyqJointID, double> min_range {
    {HAA, haa_min/180.0*M_PI},
    {HFE, hfe_min/180.0*M_PI},
    {KFE, kfe_min/180.0*M_PI}
  };

  double max = max_range.at(joint);
  val = val>max? max : val;

  double min = min_range.at(joint);
  val = val<min? min : val;
}

private:
  Vector3d hfe_to_haa_z = Vector3d(0.0, 0.0, 0.08); //distance of HFE to HAA in z direction
  double length_thigh = 0.35; // length of upper leg
  double length_shank = 0.33; // length of lower leg
};

} /* namespace xpp */

#endif /* XPP_VIS_HYQLEG_INVERSE_KINEMATICS_H_ */
