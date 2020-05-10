#ifndef IIT_ROBOT_ANYMAL_MODEL_CONSTANTS_H_
#define IIT_ROBOT_ANYMAL_MODEL_CONSTANTS_H_

#include "rbd_types.h"

/**
 * \file
 * This file contains the definitions of all the non-zero numerical
 * constants of the robot model (i.e. the numbers appearing in the
 * .kindsl file).
 *
 * Varying these values (and recompiling) is a quick & dirty
 * way to vary the kinematics/dynamics model. For a much more
 * flexible way of exploring variations of the model, consider
 * using the parametrization feature of RobCoGen (see the wiki).
 *
 * Beware of inconsistencies when changing any of the inertia
 * properties.
 */

namespace iit {
namespace anymal {

// Do not use 'constexpr' to allow for non-literal scalar types

const Scalar tx_LF_HAA = 0.2770000100135803;
const Scalar ty_LF_HAA = 0.11599999666213989;
const Scalar ty_LF_HFE = 0.04100000113248825;
const Scalar tz_LF_HFE = 0.06350000202655792;
const Scalar tx_LF_KFE = 0.25;
const Scalar tz_LF_KFE = 0.10899999737739563;
const Scalar ty_LF_SHANK_TO_ADAPTER = -0.10000000149011612;
const Scalar tz_LF_SHANK_TO_ADAPTER = -0.019999999552965164;
const Scalar tz_LF_ADAPTER_TO_FOOT = -0.32124999165534973;
const Scalar tx_RF_HAA = 0.2770000100135803;
const Scalar ty_RF_HAA = -0.11599999666213989;
const Scalar ty_RF_HFE = -0.04100000113248825;
const Scalar tz_RF_HFE = 0.06350000202655792;
const Scalar tx_RF_KFE = 0.25;
const Scalar tz_RF_KFE = -0.10899999737739563;
const Scalar ty_RF_SHANK_TO_ADAPTER = -0.10000000149011612;
const Scalar tz_RF_SHANK_TO_ADAPTER = 0.019999999552965164;
const Scalar tz_RF_ADAPTER_TO_FOOT = -0.32124999165534973;
const Scalar tx_LH_HAA = -0.2770000100135803;
const Scalar ty_LH_HAA = 0.11599999666213989;
const Scalar ty_LH_HFE = 0.04100000113248825;
const Scalar tz_LH_HFE = -0.06350000202655792;
const Scalar tx_LH_KFE = 0.25;
const Scalar tz_LH_KFE = 0.10899999737739563;
const Scalar ty_LH_SHANK_TO_ADAPTER = 0.10000000149011612;
const Scalar tz_LH_SHANK_TO_ADAPTER = -0.019999999552965164;
const Scalar tz_LH_ADAPTER_TO_FOOT = -0.32124999165534973;
const Scalar tx_RH_HAA = -0.2770000100135803;
const Scalar ty_RH_HAA = -0.11599999666213989;
const Scalar ty_RH_HFE = -0.04100000113248825;
const Scalar tz_RH_HFE = -0.06350000202655792;
const Scalar tx_RH_KFE = 0.25;
const Scalar tz_RH_KFE = -0.10899999737739563;
const Scalar ty_RH_SHANK_TO_ADAPTER = 0.10000000149011612;
const Scalar tz_RH_SHANK_TO_ADAPTER = 0.019999999552965164;
const Scalar tz_RH_ADAPTER_TO_FOOT = -0.32124999165534973;
const Scalar m_base_inertia = 16.793500900268555;
const Scalar comx_base_inertia = -0.001961000030860305;
const Scalar comy_base_inertia = -0.0014130000490695238;
const Scalar comz_base_inertia = 0.050207000225782394;
const Scalar ix_base_inertia = 0.25975701212882996;
const Scalar ixy_base_inertia = 0.0013749999925494194;
const Scalar ixz_base_inertia = 6.290000164881349E-4;
const Scalar iy_base_inertia = 0.6818289756774902;
const Scalar iyz_base_inertia = 1.8899999849963933E-4;
const Scalar iz_base_inertia = 0.6242390275001526;
const Scalar m_LF_HIP = 1.4246200323104858;
const Scalar comx_LF_HIP = 1.5199999324977398E-4;
const Scalar comy_LF_HIP = -0.0037869999650865793;
const Scalar comz_LF_HIP = 0.06451600044965744;
const Scalar ix_LF_HIP = 0.007930999621748924;
const Scalar ixy_LF_HIP = 2.5999999706982635E-5;
const Scalar ixz_LF_HIP = -9.999999974752427E-7;
const Scalar iy_LF_HIP = 0.008232000283896923;
const Scalar iyz_LF_HIP = -3.330000035930425E-4;
const Scalar iz_LF_HIP = 0.0024510000366717577;
const Scalar m_LF_THIGH = 1.6349799633026123;
const Scalar comx_LF_THIGH = 0.21458299458026886;
const Scalar comy_LF_THIGH = 0.0038980001118034124;
const Scalar comz_LF_THIGH = 0.05422699823975563;
const Scalar ix_LF_THIGH = 0.007327000144869089;
const Scalar ixy_LF_THIGH = 0.0013000000035390258;
const Scalar ixz_LF_THIGH = 0.017619000747799873;
const Scalar iy_LF_THIGH = 0.09212899953126907;
const Scalar iyz_LF_THIGH = 4.130000015720725E-4;
const Scalar iz_LF_THIGH = 0.08737300336360931;
const Scalar m_LF_SHANK = 0.20720399916172028;
const Scalar comx_LF_SHANK = -8.929999894462526E-4;
const Scalar comy_LF_SHANK = -0.030817000195384026;
const Scalar comz_LF_SHANK = -0.004616999998688698;
const Scalar ix_LF_SHANK = 7.459999760612845E-4;
const Scalar ixy_LF_SHANK = 6.199999916134402E-5;
const Scalar iy_LF_SHANK = 2.1499999274965376E-4;
const Scalar iyz_LF_SHANK = -2.700000004551839E-5;
const Scalar iz_LF_SHANK = 8.730000117793679E-4;
const Scalar m_LF_ADAPTER = 0.14017100632190704;
const Scalar comz_LF_ADAPTER = -0.2443459928035736;
const Scalar ix_LF_ADAPTER = 0.009968000464141369;
const Scalar iy_LF_ADAPTER = 0.009968000464141369;
const Scalar iz_LF_ADAPTER = 5.400000009103678E-5;
const Scalar m_RF_HIP = 1.4246200323104858;
const Scalar comx_RF_HIP = 1.5199999324977398E-4;
const Scalar comy_RF_HIP = 0.0037869999650865793;
const Scalar comz_RF_HIP = 0.06451600044965744;
const Scalar ix_RF_HIP = 0.007930999621748924;
const Scalar ixy_RF_HIP = -2.5999999706982635E-5;
const Scalar ixz_RF_HIP = 2.9000000722589903E-5;
const Scalar iy_RF_HIP = 0.008232000283896923;
const Scalar iyz_RF_HIP = 3.330000035930425E-4;
const Scalar iz_RF_HIP = 0.0024510000366717577;
const Scalar m_RF_THIGH = 1.6349799633026123;
const Scalar comx_RF_THIGH = 0.21458299458026886;
const Scalar comy_RF_THIGH = 0.0038980001118034124;
const Scalar comz_RF_THIGH = -0.05422699823975563;
const Scalar ix_RF_THIGH = 0.007327000144869089;
const Scalar ixy_RF_THIGH = 0.0014349999837577343;
const Scalar ixz_RF_THIGH = -0.017619000747799873;
const Scalar iy_RF_THIGH = 0.09212899953126907;
const Scalar iyz_RF_THIGH = -4.130000015720725E-4;
const Scalar iz_RF_THIGH = 0.08737300336360931;
const Scalar m_RF_SHANK = 0.20720399916172028;
const Scalar comx_RF_SHANK = -8.929999894462526E-4;
const Scalar comy_RF_SHANK = -0.030817000195384026;
const Scalar comz_RF_SHANK = 0.004616999998688698;
const Scalar ix_RF_SHANK = 7.459999760612845E-4;
const Scalar ixy_RF_SHANK = -5.0999999075429514E-5;
const Scalar iy_RF_SHANK = 2.1499999274965376E-4;
const Scalar iyz_RF_SHANK = 2.700000004551839E-5;
const Scalar iz_RF_SHANK = 8.730000117793679E-4;
const Scalar m_RF_ADAPTER = 0.14017100632190704;
const Scalar comz_RF_ADAPTER = -0.2443459928035736;
const Scalar ix_RF_ADAPTER = 0.009968000464141369;
const Scalar iy_RF_ADAPTER = 0.009968000464141369;
const Scalar iz_RF_ADAPTER = 5.400000009103678E-5;
const Scalar m_LH_HIP = 1.4246200323104858;
const Scalar comx_LH_HIP = 1.5199999324977398E-4;
const Scalar comy_LH_HIP = -0.0037869999650865793;
const Scalar comz_LH_HIP = -0.06451600044965744;
const Scalar ix_LH_HIP = 0.007930999621748924;
const Scalar ixy_LH_HIP = 2.5999999706982635E-5;
const Scalar ixz_LH_HIP = 9.999999974752427E-7;
const Scalar iy_LH_HIP = 0.008232000283896923;
const Scalar iyz_LH_HIP = 3.330000035930425E-4;
const Scalar iz_LH_HIP = 0.0024510000366717577;
const Scalar m_LH_THIGH = 1.6349799633026123;
const Scalar comx_LH_THIGH = 0.21458299458026886;
const Scalar comy_LH_THIGH = -0.0038980001118034124;
const Scalar comz_LH_THIGH = 0.05422699823975563;
const Scalar ix_LH_THIGH = 0.007327000144869089;
const Scalar ixy_LH_THIGH = -0.0013000000035390258;
const Scalar ixz_LH_THIGH = 0.017619000747799873;
const Scalar iy_LH_THIGH = 0.09212899953126907;
const Scalar iyz_LH_THIGH = -4.130000015720725E-4;
const Scalar iz_LH_THIGH = 0.08737300336360931;
const Scalar m_LH_SHANK = 0.20720399916172028;
const Scalar comx_LH_SHANK = -8.929999894462526E-4;
const Scalar comy_LH_SHANK = 0.030817000195384026;
const Scalar comz_LH_SHANK = -0.004616999998688698;
const Scalar ix_LH_SHANK = 7.459999760612845E-4;
const Scalar ixy_LH_SHANK = -6.199999916134402E-5;
const Scalar iy_LH_SHANK = 2.1499999274965376E-4;
const Scalar iyz_LH_SHANK = 2.700000004551839E-5;
const Scalar iz_LH_SHANK = 8.730000117793679E-4;
const Scalar m_LH_ADAPTER = 0.14017100632190704;
const Scalar comz_LH_ADAPTER = -0.2443459928035736;
const Scalar ix_LH_ADAPTER = 0.009968000464141369;
const Scalar iy_LH_ADAPTER = 0.009968000464141369;
const Scalar iz_LH_ADAPTER = 5.400000009103678E-5;
const Scalar m_RH_HIP = 1.4246200323104858;
const Scalar comx_RH_HIP = 1.5199999324977398E-4;
const Scalar comy_RH_HIP = 0.0037869999650865793;
const Scalar comz_RH_HIP = -0.06451600044965744;
const Scalar ix_RH_HIP = 0.007930999621748924;
const Scalar ixy_RH_HIP = -2.5999999706982635E-5;
const Scalar ixz_RH_HIP = -2.9000000722589903E-5;
const Scalar iy_RH_HIP = 0.008232000283896923;
const Scalar iyz_RH_HIP = -3.330000035930425E-4;
const Scalar iz_RH_HIP = 0.0024510000366717577;
const Scalar m_RH_THIGH = 1.6349799633026123;
const Scalar comx_RH_THIGH = 0.21458299458026886;
const Scalar comy_RH_THIGH = -0.0038980001118034124;
const Scalar comz_RH_THIGH = -0.05422699823975563;
const Scalar ix_RH_THIGH = 0.007327000144869089;
const Scalar ixy_RH_THIGH = -0.0014349999837577343;
const Scalar ixz_RH_THIGH = -0.017619000747799873;
const Scalar iy_RH_THIGH = 0.09212899953126907;
const Scalar iyz_RH_THIGH = 4.130000015720725E-4;
const Scalar iz_RH_THIGH = 0.08737300336360931;
const Scalar m_RH_SHANK = 0.20720399916172028;
const Scalar comx_RH_SHANK = -8.929999894462526E-4;
const Scalar comy_RH_SHANK = 0.030817000195384026;
const Scalar comz_RH_SHANK = 0.004616999998688698;
const Scalar ix_RH_SHANK = 7.459999760612845E-4;
const Scalar ixy_RH_SHANK = 5.0999999075429514E-5;
const Scalar iy_RH_SHANK = 2.1499999274965376E-4;
const Scalar iyz_RH_SHANK = -2.700000004551839E-5;
const Scalar iz_RH_SHANK = 8.730000117793679E-4;
const Scalar m_RH_ADAPTER = 0.14017100632190704;
const Scalar comz_RH_ADAPTER = -0.2443459928035736;
const Scalar ix_RH_ADAPTER = 0.009968000464141369;
const Scalar iy_RH_ADAPTER = 0.009968000464141369;
const Scalar iz_RH_ADAPTER = 5.400000009103678E-5;

}
}
#endif
