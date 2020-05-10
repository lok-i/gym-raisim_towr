#include <iit/rbd/robcogen_commons.h>

#include "inverse_dynamics.h"
#include "inertia_properties.h"
#ifndef EIGEN_NO_DEBUG
    #include <iostream>
#endif
using namespace std;
using namespace iit::rbd;
using namespace iit::anymal::dyn;

// Initialization of static-const data
const iit::anymal::dyn::InverseDynamics::ExtForces
iit::anymal::dyn::InverseDynamics::zeroExtForces(Force::Zero());

iit::anymal::dyn::InverseDynamics::InverseDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    base_inertia_I(inertiaProps->getTensor_base_inertia() ),
    LF_HIP_I(inertiaProps->getTensor_LF_HIP() ),
    LF_THIGH_I(inertiaProps->getTensor_LF_THIGH() ),
    LF_SHANK_I(inertiaProps->getTensor_LF_SHANK() ),
    LF_ADAPTER_I(inertiaProps->getTensor_LF_ADAPTER() ),
    LF_FOOT_I(inertiaProps->getTensor_LF_FOOT() ),
    RF_HIP_I(inertiaProps->getTensor_RF_HIP() ),
    RF_THIGH_I(inertiaProps->getTensor_RF_THIGH() ),
    RF_SHANK_I(inertiaProps->getTensor_RF_SHANK() ),
    RF_ADAPTER_I(inertiaProps->getTensor_RF_ADAPTER() ),
    RF_FOOT_I(inertiaProps->getTensor_RF_FOOT() ),
    LH_HIP_I(inertiaProps->getTensor_LH_HIP() ),
    LH_THIGH_I(inertiaProps->getTensor_LH_THIGH() ),
    LH_SHANK_I(inertiaProps->getTensor_LH_SHANK() ),
    LH_ADAPTER_I(inertiaProps->getTensor_LH_ADAPTER() ),
    LH_FOOT_I(inertiaProps->getTensor_LH_FOOT() ),
    RH_HIP_I(inertiaProps->getTensor_RH_HIP() ),
    RH_THIGH_I(inertiaProps->getTensor_RH_THIGH() ),
    RH_SHANK_I(inertiaProps->getTensor_RH_SHANK() ),
    RH_ADAPTER_I(inertiaProps->getTensor_RH_ADAPTER() ),
    RH_FOOT_I(inertiaProps->getTensor_RH_FOOT() )
    {
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot anymal, InverseDynamics::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    base_inertia_v.setZero();
    LF_HIP_v.setZero();
    LF_THIGH_v.setZero();
    LF_SHANK_v.setZero();
    LF_ADAPTER_v.setZero();
    LF_FOOT_v.setZero();
    RF_HIP_v.setZero();
    RF_THIGH_v.setZero();
    RF_SHANK_v.setZero();
    RF_ADAPTER_v.setZero();
    RF_FOOT_v.setZero();
    LH_HIP_v.setZero();
    LH_THIGH_v.setZero();
    LH_SHANK_v.setZero();
    LH_ADAPTER_v.setZero();
    LH_FOOT_v.setZero();
    RH_HIP_v.setZero();
    RH_THIGH_v.setZero();
    RH_SHANK_v.setZero();
    RH_ADAPTER_v.setZero();
    RH_FOOT_v.setZero();

    vcross.setZero();
}

void iit::anymal::dyn::InverseDynamics::id(
    JointState& jForces,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    firstPass(qd, qdd, fext);
    secondPass(jForces);
}

void iit::anymal::dyn::InverseDynamics::G_terms(JointState& jForces)
{
    // Link 'base_inertia'
    base_inertia_a = (xm->fr_base_inertia_X_fr_base).col(iit::rbd::LZ) * anymal::g;
    base_inertia_f = base_inertia_I * base_inertia_a;
    // Link 'LF_HIP'
    LF_HIP_a = (xm->fr_LF_HIP_X_fr_base).col(iit::rbd::LZ) * anymal::g;
    LF_HIP_f = LF_HIP_I * LF_HIP_a;
    // Link 'LF_THIGH'
    LF_THIGH_a = (xm->fr_LF_THIGH_X_fr_LF_HIP) * LF_HIP_a;
    LF_THIGH_f = LF_THIGH_I * LF_THIGH_a;
    // Link 'LF_SHANK'
    LF_SHANK_a = (xm->fr_LF_SHANK_X_fr_LF_THIGH) * LF_THIGH_a;
    LF_SHANK_f = LF_SHANK_I * LF_SHANK_a;
    // Link 'LF_ADAPTER'
    LF_ADAPTER_a = (xm->fr_LF_ADAPTER_X_fr_LF_SHANK) * LF_SHANK_a;
    LF_ADAPTER_f = LF_ADAPTER_I * LF_ADAPTER_a;
    // Link 'LF_FOOT'
    LF_FOOT_a = (xm->fr_LF_FOOT_X_fr_LF_ADAPTER) * LF_ADAPTER_a;
    LF_FOOT_f = LF_FOOT_I * LF_FOOT_a;
    // Link 'RF_HIP'
    RF_HIP_a = (xm->fr_RF_HIP_X_fr_base).col(iit::rbd::LZ) * anymal::g;
    RF_HIP_f = RF_HIP_I * RF_HIP_a;
    // Link 'RF_THIGH'
    RF_THIGH_a = (xm->fr_RF_THIGH_X_fr_RF_HIP) * RF_HIP_a;
    RF_THIGH_f = RF_THIGH_I * RF_THIGH_a;
    // Link 'RF_SHANK'
    RF_SHANK_a = (xm->fr_RF_SHANK_X_fr_RF_THIGH) * RF_THIGH_a;
    RF_SHANK_f = RF_SHANK_I * RF_SHANK_a;
    // Link 'RF_ADAPTER'
    RF_ADAPTER_a = (xm->fr_RF_ADAPTER_X_fr_RF_SHANK) * RF_SHANK_a;
    RF_ADAPTER_f = RF_ADAPTER_I * RF_ADAPTER_a;
    // Link 'RF_FOOT'
    RF_FOOT_a = (xm->fr_RF_FOOT_X_fr_RF_ADAPTER) * RF_ADAPTER_a;
    RF_FOOT_f = RF_FOOT_I * RF_FOOT_a;
    // Link 'LH_HIP'
    LH_HIP_a = (xm->fr_LH_HIP_X_fr_base).col(iit::rbd::LZ) * anymal::g;
    LH_HIP_f = LH_HIP_I * LH_HIP_a;
    // Link 'LH_THIGH'
    LH_THIGH_a = (xm->fr_LH_THIGH_X_fr_LH_HIP) * LH_HIP_a;
    LH_THIGH_f = LH_THIGH_I * LH_THIGH_a;
    // Link 'LH_SHANK'
    LH_SHANK_a = (xm->fr_LH_SHANK_X_fr_LH_THIGH) * LH_THIGH_a;
    LH_SHANK_f = LH_SHANK_I * LH_SHANK_a;
    // Link 'LH_ADAPTER'
    LH_ADAPTER_a = (xm->fr_LH_ADAPTER_X_fr_LH_SHANK) * LH_SHANK_a;
    LH_ADAPTER_f = LH_ADAPTER_I * LH_ADAPTER_a;
    // Link 'LH_FOOT'
    LH_FOOT_a = (xm->fr_LH_FOOT_X_fr_LH_ADAPTER) * LH_ADAPTER_a;
    LH_FOOT_f = LH_FOOT_I * LH_FOOT_a;
    // Link 'RH_HIP'
    RH_HIP_a = (xm->fr_RH_HIP_X_fr_base).col(iit::rbd::LZ) * anymal::g;
    RH_HIP_f = RH_HIP_I * RH_HIP_a;
    // Link 'RH_THIGH'
    RH_THIGH_a = (xm->fr_RH_THIGH_X_fr_RH_HIP) * RH_HIP_a;
    RH_THIGH_f = RH_THIGH_I * RH_THIGH_a;
    // Link 'RH_SHANK'
    RH_SHANK_a = (xm->fr_RH_SHANK_X_fr_RH_THIGH) * RH_THIGH_a;
    RH_SHANK_f = RH_SHANK_I * RH_SHANK_a;
    // Link 'RH_ADAPTER'
    RH_ADAPTER_a = (xm->fr_RH_ADAPTER_X_fr_RH_SHANK) * RH_SHANK_a;
    RH_ADAPTER_f = RH_ADAPTER_I * RH_ADAPTER_a;
    // Link 'RH_FOOT'
    RH_FOOT_a = (xm->fr_RH_FOOT_X_fr_RH_ADAPTER) * RH_ADAPTER_a;
    RH_FOOT_f = RH_FOOT_I * RH_FOOT_a;

    secondPass(jForces);
}

void iit::anymal::dyn::InverseDynamics::C_terms(JointState& jForces, const JointState& qd)
{
    // Link 'base_inertia'
    base_inertia_v(iit::rbd::AZ) = qd(BASE_TO_BASE_INERTIA);   // base_inertia_v = vJ, for the first link of a fixed base robot
    
    base_inertia_f = vxIv(qd(BASE_TO_BASE_INERTIA), base_inertia_I);
    
    // Link 'LF_HIP'
    LF_HIP_v(iit::rbd::AZ) = qd(LF_HAA);   // LF_HIP_v = vJ, for the first link of a fixed base robot
    
    LF_HIP_f = vxIv(qd(LF_HAA), LF_HIP_I);
    
    // Link 'LF_THIGH'
    LF_THIGH_v = ((xm->fr_LF_THIGH_X_fr_LF_HIP) * LF_HIP_v);
    LF_THIGH_v(iit::rbd::AZ) += qd(LF_HFE);
    
    motionCrossProductMx<Scalar>(LF_THIGH_v, vcross);
    
    LF_THIGH_a = (vcross.col(iit::rbd::AZ) * qd(LF_HFE));
    
    LF_THIGH_f = LF_THIGH_I * LF_THIGH_a + vxIv(LF_THIGH_v, LF_THIGH_I);
    
    // Link 'LF_SHANK'
    LF_SHANK_v = ((xm->fr_LF_SHANK_X_fr_LF_THIGH) * LF_THIGH_v);
    LF_SHANK_v(iit::rbd::AZ) += qd(LF_KFE);
    
    motionCrossProductMx<Scalar>(LF_SHANK_v, vcross);
    
    LF_SHANK_a = (xm->fr_LF_SHANK_X_fr_LF_THIGH) * LF_THIGH_a + vcross.col(iit::rbd::AZ) * qd(LF_KFE);
    
    LF_SHANK_f = LF_SHANK_I * LF_SHANK_a + vxIv(LF_SHANK_v, LF_SHANK_I);
    
    // Link 'LF_ADAPTER'
    LF_ADAPTER_v = ((xm->fr_LF_ADAPTER_X_fr_LF_SHANK) * LF_SHANK_v);
    LF_ADAPTER_v(iit::rbd::AZ) += qd(LF_SHANK_TO_ADAPTER);
    
    motionCrossProductMx<Scalar>(LF_ADAPTER_v, vcross);
    
    LF_ADAPTER_a = (xm->fr_LF_ADAPTER_X_fr_LF_SHANK) * LF_SHANK_a + vcross.col(iit::rbd::AZ) * qd(LF_SHANK_TO_ADAPTER);
    
    LF_ADAPTER_f = LF_ADAPTER_I * LF_ADAPTER_a + vxIv(LF_ADAPTER_v, LF_ADAPTER_I);
    
    // Link 'LF_FOOT'
    LF_FOOT_v = ((xm->fr_LF_FOOT_X_fr_LF_ADAPTER) * LF_ADAPTER_v);
    LF_FOOT_v(iit::rbd::AZ) += qd(LF_ADAPTER_TO_FOOT);
    
    motionCrossProductMx<Scalar>(LF_FOOT_v, vcross);
    
    LF_FOOT_a = (xm->fr_LF_FOOT_X_fr_LF_ADAPTER) * LF_ADAPTER_a + vcross.col(iit::rbd::AZ) * qd(LF_ADAPTER_TO_FOOT);
    
    LF_FOOT_f = LF_FOOT_I * LF_FOOT_a + vxIv(LF_FOOT_v, LF_FOOT_I);
    
    // Link 'RF_HIP'
    RF_HIP_v(iit::rbd::AZ) = qd(RF_HAA);   // RF_HIP_v = vJ, for the first link of a fixed base robot
    
    RF_HIP_f = vxIv(qd(RF_HAA), RF_HIP_I);
    
    // Link 'RF_THIGH'
    RF_THIGH_v = ((xm->fr_RF_THIGH_X_fr_RF_HIP) * RF_HIP_v);
    RF_THIGH_v(iit::rbd::AZ) += qd(RF_HFE);
    
    motionCrossProductMx<Scalar>(RF_THIGH_v, vcross);
    
    RF_THIGH_a = (vcross.col(iit::rbd::AZ) * qd(RF_HFE));
    
    RF_THIGH_f = RF_THIGH_I * RF_THIGH_a + vxIv(RF_THIGH_v, RF_THIGH_I);
    
    // Link 'RF_SHANK'
    RF_SHANK_v = ((xm->fr_RF_SHANK_X_fr_RF_THIGH) * RF_THIGH_v);
    RF_SHANK_v(iit::rbd::AZ) += qd(RF_KFE);
    
    motionCrossProductMx<Scalar>(RF_SHANK_v, vcross);
    
    RF_SHANK_a = (xm->fr_RF_SHANK_X_fr_RF_THIGH) * RF_THIGH_a + vcross.col(iit::rbd::AZ) * qd(RF_KFE);
    
    RF_SHANK_f = RF_SHANK_I * RF_SHANK_a + vxIv(RF_SHANK_v, RF_SHANK_I);
    
    // Link 'RF_ADAPTER'
    RF_ADAPTER_v = ((xm->fr_RF_ADAPTER_X_fr_RF_SHANK) * RF_SHANK_v);
    RF_ADAPTER_v(iit::rbd::AZ) += qd(RF_SHANK_TO_ADAPTER);
    
    motionCrossProductMx<Scalar>(RF_ADAPTER_v, vcross);
    
    RF_ADAPTER_a = (xm->fr_RF_ADAPTER_X_fr_RF_SHANK) * RF_SHANK_a + vcross.col(iit::rbd::AZ) * qd(RF_SHANK_TO_ADAPTER);
    
    RF_ADAPTER_f = RF_ADAPTER_I * RF_ADAPTER_a + vxIv(RF_ADAPTER_v, RF_ADAPTER_I);
    
    // Link 'RF_FOOT'
    RF_FOOT_v = ((xm->fr_RF_FOOT_X_fr_RF_ADAPTER) * RF_ADAPTER_v);
    RF_FOOT_v(iit::rbd::AZ) += qd(RF_ADAPTER_TO_FOOT);
    
    motionCrossProductMx<Scalar>(RF_FOOT_v, vcross);
    
    RF_FOOT_a = (xm->fr_RF_FOOT_X_fr_RF_ADAPTER) * RF_ADAPTER_a + vcross.col(iit::rbd::AZ) * qd(RF_ADAPTER_TO_FOOT);
    
    RF_FOOT_f = RF_FOOT_I * RF_FOOT_a + vxIv(RF_FOOT_v, RF_FOOT_I);
    
    // Link 'LH_HIP'
    LH_HIP_v(iit::rbd::AZ) = qd(LH_HAA);   // LH_HIP_v = vJ, for the first link of a fixed base robot
    
    LH_HIP_f = vxIv(qd(LH_HAA), LH_HIP_I);
    
    // Link 'LH_THIGH'
    LH_THIGH_v = ((xm->fr_LH_THIGH_X_fr_LH_HIP) * LH_HIP_v);
    LH_THIGH_v(iit::rbd::AZ) += qd(LH_HFE);
    
    motionCrossProductMx<Scalar>(LH_THIGH_v, vcross);
    
    LH_THIGH_a = (vcross.col(iit::rbd::AZ) * qd(LH_HFE));
    
    LH_THIGH_f = LH_THIGH_I * LH_THIGH_a + vxIv(LH_THIGH_v, LH_THIGH_I);
    
    // Link 'LH_SHANK'
    LH_SHANK_v = ((xm->fr_LH_SHANK_X_fr_LH_THIGH) * LH_THIGH_v);
    LH_SHANK_v(iit::rbd::AZ) += qd(LH_KFE);
    
    motionCrossProductMx<Scalar>(LH_SHANK_v, vcross);
    
    LH_SHANK_a = (xm->fr_LH_SHANK_X_fr_LH_THIGH) * LH_THIGH_a + vcross.col(iit::rbd::AZ) * qd(LH_KFE);
    
    LH_SHANK_f = LH_SHANK_I * LH_SHANK_a + vxIv(LH_SHANK_v, LH_SHANK_I);
    
    // Link 'LH_ADAPTER'
    LH_ADAPTER_v = ((xm->fr_LH_ADAPTER_X_fr_LH_SHANK) * LH_SHANK_v);
    LH_ADAPTER_v(iit::rbd::AZ) += qd(LH_SHANK_TO_ADAPTER);
    
    motionCrossProductMx<Scalar>(LH_ADAPTER_v, vcross);
    
    LH_ADAPTER_a = (xm->fr_LH_ADAPTER_X_fr_LH_SHANK) * LH_SHANK_a + vcross.col(iit::rbd::AZ) * qd(LH_SHANK_TO_ADAPTER);
    
    LH_ADAPTER_f = LH_ADAPTER_I * LH_ADAPTER_a + vxIv(LH_ADAPTER_v, LH_ADAPTER_I);
    
    // Link 'LH_FOOT'
    LH_FOOT_v = ((xm->fr_LH_FOOT_X_fr_LH_ADAPTER) * LH_ADAPTER_v);
    LH_FOOT_v(iit::rbd::AZ) += qd(LH_ADAPTER_TO_FOOT);
    
    motionCrossProductMx<Scalar>(LH_FOOT_v, vcross);
    
    LH_FOOT_a = (xm->fr_LH_FOOT_X_fr_LH_ADAPTER) * LH_ADAPTER_a + vcross.col(iit::rbd::AZ) * qd(LH_ADAPTER_TO_FOOT);
    
    LH_FOOT_f = LH_FOOT_I * LH_FOOT_a + vxIv(LH_FOOT_v, LH_FOOT_I);
    
    // Link 'RH_HIP'
    RH_HIP_v(iit::rbd::AZ) = qd(RH_HAA);   // RH_HIP_v = vJ, for the first link of a fixed base robot
    
    RH_HIP_f = vxIv(qd(RH_HAA), RH_HIP_I);
    
    // Link 'RH_THIGH'
    RH_THIGH_v = ((xm->fr_RH_THIGH_X_fr_RH_HIP) * RH_HIP_v);
    RH_THIGH_v(iit::rbd::AZ) += qd(RH_HFE);
    
    motionCrossProductMx<Scalar>(RH_THIGH_v, vcross);
    
    RH_THIGH_a = (vcross.col(iit::rbd::AZ) * qd(RH_HFE));
    
    RH_THIGH_f = RH_THIGH_I * RH_THIGH_a + vxIv(RH_THIGH_v, RH_THIGH_I);
    
    // Link 'RH_SHANK'
    RH_SHANK_v = ((xm->fr_RH_SHANK_X_fr_RH_THIGH) * RH_THIGH_v);
    RH_SHANK_v(iit::rbd::AZ) += qd(RH_KFE);
    
    motionCrossProductMx<Scalar>(RH_SHANK_v, vcross);
    
    RH_SHANK_a = (xm->fr_RH_SHANK_X_fr_RH_THIGH) * RH_THIGH_a + vcross.col(iit::rbd::AZ) * qd(RH_KFE);
    
    RH_SHANK_f = RH_SHANK_I * RH_SHANK_a + vxIv(RH_SHANK_v, RH_SHANK_I);
    
    // Link 'RH_ADAPTER'
    RH_ADAPTER_v = ((xm->fr_RH_ADAPTER_X_fr_RH_SHANK) * RH_SHANK_v);
    RH_ADAPTER_v(iit::rbd::AZ) += qd(RH_SHANK_TO_ADAPTER);
    
    motionCrossProductMx<Scalar>(RH_ADAPTER_v, vcross);
    
    RH_ADAPTER_a = (xm->fr_RH_ADAPTER_X_fr_RH_SHANK) * RH_SHANK_a + vcross.col(iit::rbd::AZ) * qd(RH_SHANK_TO_ADAPTER);
    
    RH_ADAPTER_f = RH_ADAPTER_I * RH_ADAPTER_a + vxIv(RH_ADAPTER_v, RH_ADAPTER_I);
    
    // Link 'RH_FOOT'
    RH_FOOT_v = ((xm->fr_RH_FOOT_X_fr_RH_ADAPTER) * RH_ADAPTER_v);
    RH_FOOT_v(iit::rbd::AZ) += qd(RH_ADAPTER_TO_FOOT);
    
    motionCrossProductMx<Scalar>(RH_FOOT_v, vcross);
    
    RH_FOOT_a = (xm->fr_RH_FOOT_X_fr_RH_ADAPTER) * RH_ADAPTER_a + vcross.col(iit::rbd::AZ) * qd(RH_ADAPTER_TO_FOOT);
    
    RH_FOOT_f = RH_FOOT_I * RH_FOOT_a + vxIv(RH_FOOT_v, RH_FOOT_I);
    

    secondPass(jForces);
}


void iit::anymal::dyn::InverseDynamics::firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    // First pass, link 'base_inertia'
    base_inertia_a = (xm->fr_base_inertia_X_fr_base).col(iit::rbd::LZ) * anymal::g;
    base_inertia_a(iit::rbd::AZ) += qdd(BASE_TO_BASE_INERTIA);
    base_inertia_v(iit::rbd::AZ) = qd(BASE_TO_BASE_INERTIA);   // base_inertia_v = vJ, for the first link of a fixed base robot
    
    base_inertia_f = base_inertia_I * base_inertia_a + vxIv(qd(BASE_TO_BASE_INERTIA), base_inertia_I)  - fext[BASE_INERTIA];
    
    // First pass, link 'LF_HIP'
    LF_HIP_a = (xm->fr_LF_HIP_X_fr_base).col(iit::rbd::LZ) * anymal::g;
    LF_HIP_a(iit::rbd::AZ) += qdd(LF_HAA);
    LF_HIP_v(iit::rbd::AZ) = qd(LF_HAA);   // LF_HIP_v = vJ, for the first link of a fixed base robot
    
    LF_HIP_f = LF_HIP_I * LF_HIP_a + vxIv(qd(LF_HAA), LF_HIP_I)  - fext[LF_HIP];
    
    // First pass, link 'LF_THIGH'
    LF_THIGH_v = ((xm->fr_LF_THIGH_X_fr_LF_HIP) * LF_HIP_v);
    LF_THIGH_v(iit::rbd::AZ) += qd(LF_HFE);
    
    motionCrossProductMx<Scalar>(LF_THIGH_v, vcross);
    
    LF_THIGH_a = (xm->fr_LF_THIGH_X_fr_LF_HIP) * LF_HIP_a + vcross.col(iit::rbd::AZ) * qd(LF_HFE);
    LF_THIGH_a(iit::rbd::AZ) += qdd(LF_HFE);
    
    LF_THIGH_f = LF_THIGH_I * LF_THIGH_a + vxIv(LF_THIGH_v, LF_THIGH_I) - fext[LF_THIGH];
    
    // First pass, link 'LF_SHANK'
    LF_SHANK_v = ((xm->fr_LF_SHANK_X_fr_LF_THIGH) * LF_THIGH_v);
    LF_SHANK_v(iit::rbd::AZ) += qd(LF_KFE);
    
    motionCrossProductMx<Scalar>(LF_SHANK_v, vcross);
    
    LF_SHANK_a = (xm->fr_LF_SHANK_X_fr_LF_THIGH) * LF_THIGH_a + vcross.col(iit::rbd::AZ) * qd(LF_KFE);
    LF_SHANK_a(iit::rbd::AZ) += qdd(LF_KFE);
    
    LF_SHANK_f = LF_SHANK_I * LF_SHANK_a + vxIv(LF_SHANK_v, LF_SHANK_I) - fext[LF_SHANK];
    
    // First pass, link 'LF_ADAPTER'
    LF_ADAPTER_v = ((xm->fr_LF_ADAPTER_X_fr_LF_SHANK) * LF_SHANK_v);
    LF_ADAPTER_v(iit::rbd::AZ) += qd(LF_SHANK_TO_ADAPTER);
    
    motionCrossProductMx<Scalar>(LF_ADAPTER_v, vcross);
    
    LF_ADAPTER_a = (xm->fr_LF_ADAPTER_X_fr_LF_SHANK) * LF_SHANK_a + vcross.col(iit::rbd::AZ) * qd(LF_SHANK_TO_ADAPTER);
    LF_ADAPTER_a(iit::rbd::AZ) += qdd(LF_SHANK_TO_ADAPTER);
    
    LF_ADAPTER_f = LF_ADAPTER_I * LF_ADAPTER_a + vxIv(LF_ADAPTER_v, LF_ADAPTER_I) - fext[LF_ADAPTER];
    
    // First pass, link 'LF_FOOT'
    LF_FOOT_v = ((xm->fr_LF_FOOT_X_fr_LF_ADAPTER) * LF_ADAPTER_v);
    LF_FOOT_v(iit::rbd::AZ) += qd(LF_ADAPTER_TO_FOOT);
    
    motionCrossProductMx<Scalar>(LF_FOOT_v, vcross);
    
    LF_FOOT_a = (xm->fr_LF_FOOT_X_fr_LF_ADAPTER) * LF_ADAPTER_a + vcross.col(iit::rbd::AZ) * qd(LF_ADAPTER_TO_FOOT);
    LF_FOOT_a(iit::rbd::AZ) += qdd(LF_ADAPTER_TO_FOOT);
    
    LF_FOOT_f = LF_FOOT_I * LF_FOOT_a + vxIv(LF_FOOT_v, LF_FOOT_I) - fext[LF_FOOT];
    
    // First pass, link 'RF_HIP'
    RF_HIP_a = (xm->fr_RF_HIP_X_fr_base).col(iit::rbd::LZ) * anymal::g;
    RF_HIP_a(iit::rbd::AZ) += qdd(RF_HAA);
    RF_HIP_v(iit::rbd::AZ) = qd(RF_HAA);   // RF_HIP_v = vJ, for the first link of a fixed base robot
    
    RF_HIP_f = RF_HIP_I * RF_HIP_a + vxIv(qd(RF_HAA), RF_HIP_I)  - fext[RF_HIP];
    
    // First pass, link 'RF_THIGH'
    RF_THIGH_v = ((xm->fr_RF_THIGH_X_fr_RF_HIP) * RF_HIP_v);
    RF_THIGH_v(iit::rbd::AZ) += qd(RF_HFE);
    
    motionCrossProductMx<Scalar>(RF_THIGH_v, vcross);
    
    RF_THIGH_a = (xm->fr_RF_THIGH_X_fr_RF_HIP) * RF_HIP_a + vcross.col(iit::rbd::AZ) * qd(RF_HFE);
    RF_THIGH_a(iit::rbd::AZ) += qdd(RF_HFE);
    
    RF_THIGH_f = RF_THIGH_I * RF_THIGH_a + vxIv(RF_THIGH_v, RF_THIGH_I) - fext[RF_THIGH];
    
    // First pass, link 'RF_SHANK'
    RF_SHANK_v = ((xm->fr_RF_SHANK_X_fr_RF_THIGH) * RF_THIGH_v);
    RF_SHANK_v(iit::rbd::AZ) += qd(RF_KFE);
    
    motionCrossProductMx<Scalar>(RF_SHANK_v, vcross);
    
    RF_SHANK_a = (xm->fr_RF_SHANK_X_fr_RF_THIGH) * RF_THIGH_a + vcross.col(iit::rbd::AZ) * qd(RF_KFE);
    RF_SHANK_a(iit::rbd::AZ) += qdd(RF_KFE);
    
    RF_SHANK_f = RF_SHANK_I * RF_SHANK_a + vxIv(RF_SHANK_v, RF_SHANK_I) - fext[RF_SHANK];
    
    // First pass, link 'RF_ADAPTER'
    RF_ADAPTER_v = ((xm->fr_RF_ADAPTER_X_fr_RF_SHANK) * RF_SHANK_v);
    RF_ADAPTER_v(iit::rbd::AZ) += qd(RF_SHANK_TO_ADAPTER);
    
    motionCrossProductMx<Scalar>(RF_ADAPTER_v, vcross);
    
    RF_ADAPTER_a = (xm->fr_RF_ADAPTER_X_fr_RF_SHANK) * RF_SHANK_a + vcross.col(iit::rbd::AZ) * qd(RF_SHANK_TO_ADAPTER);
    RF_ADAPTER_a(iit::rbd::AZ) += qdd(RF_SHANK_TO_ADAPTER);
    
    RF_ADAPTER_f = RF_ADAPTER_I * RF_ADAPTER_a + vxIv(RF_ADAPTER_v, RF_ADAPTER_I) - fext[RF_ADAPTER];
    
    // First pass, link 'RF_FOOT'
    RF_FOOT_v = ((xm->fr_RF_FOOT_X_fr_RF_ADAPTER) * RF_ADAPTER_v);
    RF_FOOT_v(iit::rbd::AZ) += qd(RF_ADAPTER_TO_FOOT);
    
    motionCrossProductMx<Scalar>(RF_FOOT_v, vcross);
    
    RF_FOOT_a = (xm->fr_RF_FOOT_X_fr_RF_ADAPTER) * RF_ADAPTER_a + vcross.col(iit::rbd::AZ) * qd(RF_ADAPTER_TO_FOOT);
    RF_FOOT_a(iit::rbd::AZ) += qdd(RF_ADAPTER_TO_FOOT);
    
    RF_FOOT_f = RF_FOOT_I * RF_FOOT_a + vxIv(RF_FOOT_v, RF_FOOT_I) - fext[RF_FOOT];
    
    // First pass, link 'LH_HIP'
    LH_HIP_a = (xm->fr_LH_HIP_X_fr_base).col(iit::rbd::LZ) * anymal::g;
    LH_HIP_a(iit::rbd::AZ) += qdd(LH_HAA);
    LH_HIP_v(iit::rbd::AZ) = qd(LH_HAA);   // LH_HIP_v = vJ, for the first link of a fixed base robot
    
    LH_HIP_f = LH_HIP_I * LH_HIP_a + vxIv(qd(LH_HAA), LH_HIP_I)  - fext[LH_HIP];
    
    // First pass, link 'LH_THIGH'
    LH_THIGH_v = ((xm->fr_LH_THIGH_X_fr_LH_HIP) * LH_HIP_v);
    LH_THIGH_v(iit::rbd::AZ) += qd(LH_HFE);
    
    motionCrossProductMx<Scalar>(LH_THIGH_v, vcross);
    
    LH_THIGH_a = (xm->fr_LH_THIGH_X_fr_LH_HIP) * LH_HIP_a + vcross.col(iit::rbd::AZ) * qd(LH_HFE);
    LH_THIGH_a(iit::rbd::AZ) += qdd(LH_HFE);
    
    LH_THIGH_f = LH_THIGH_I * LH_THIGH_a + vxIv(LH_THIGH_v, LH_THIGH_I) - fext[LH_THIGH];
    
    // First pass, link 'LH_SHANK'
    LH_SHANK_v = ((xm->fr_LH_SHANK_X_fr_LH_THIGH) * LH_THIGH_v);
    LH_SHANK_v(iit::rbd::AZ) += qd(LH_KFE);
    
    motionCrossProductMx<Scalar>(LH_SHANK_v, vcross);
    
    LH_SHANK_a = (xm->fr_LH_SHANK_X_fr_LH_THIGH) * LH_THIGH_a + vcross.col(iit::rbd::AZ) * qd(LH_KFE);
    LH_SHANK_a(iit::rbd::AZ) += qdd(LH_KFE);
    
    LH_SHANK_f = LH_SHANK_I * LH_SHANK_a + vxIv(LH_SHANK_v, LH_SHANK_I) - fext[LH_SHANK];
    
    // First pass, link 'LH_ADAPTER'
    LH_ADAPTER_v = ((xm->fr_LH_ADAPTER_X_fr_LH_SHANK) * LH_SHANK_v);
    LH_ADAPTER_v(iit::rbd::AZ) += qd(LH_SHANK_TO_ADAPTER);
    
    motionCrossProductMx<Scalar>(LH_ADAPTER_v, vcross);
    
    LH_ADAPTER_a = (xm->fr_LH_ADAPTER_X_fr_LH_SHANK) * LH_SHANK_a + vcross.col(iit::rbd::AZ) * qd(LH_SHANK_TO_ADAPTER);
    LH_ADAPTER_a(iit::rbd::AZ) += qdd(LH_SHANK_TO_ADAPTER);
    
    LH_ADAPTER_f = LH_ADAPTER_I * LH_ADAPTER_a + vxIv(LH_ADAPTER_v, LH_ADAPTER_I) - fext[LH_ADAPTER];
    
    // First pass, link 'LH_FOOT'
    LH_FOOT_v = ((xm->fr_LH_FOOT_X_fr_LH_ADAPTER) * LH_ADAPTER_v);
    LH_FOOT_v(iit::rbd::AZ) += qd(LH_ADAPTER_TO_FOOT);
    
    motionCrossProductMx<Scalar>(LH_FOOT_v, vcross);
    
    LH_FOOT_a = (xm->fr_LH_FOOT_X_fr_LH_ADAPTER) * LH_ADAPTER_a + vcross.col(iit::rbd::AZ) * qd(LH_ADAPTER_TO_FOOT);
    LH_FOOT_a(iit::rbd::AZ) += qdd(LH_ADAPTER_TO_FOOT);
    
    LH_FOOT_f = LH_FOOT_I * LH_FOOT_a + vxIv(LH_FOOT_v, LH_FOOT_I) - fext[LH_FOOT];
    
    // First pass, link 'RH_HIP'
    RH_HIP_a = (xm->fr_RH_HIP_X_fr_base).col(iit::rbd::LZ) * anymal::g;
    RH_HIP_a(iit::rbd::AZ) += qdd(RH_HAA);
    RH_HIP_v(iit::rbd::AZ) = qd(RH_HAA);   // RH_HIP_v = vJ, for the first link of a fixed base robot
    
    RH_HIP_f = RH_HIP_I * RH_HIP_a + vxIv(qd(RH_HAA), RH_HIP_I)  - fext[RH_HIP];
    
    // First pass, link 'RH_THIGH'
    RH_THIGH_v = ((xm->fr_RH_THIGH_X_fr_RH_HIP) * RH_HIP_v);
    RH_THIGH_v(iit::rbd::AZ) += qd(RH_HFE);
    
    motionCrossProductMx<Scalar>(RH_THIGH_v, vcross);
    
    RH_THIGH_a = (xm->fr_RH_THIGH_X_fr_RH_HIP) * RH_HIP_a + vcross.col(iit::rbd::AZ) * qd(RH_HFE);
    RH_THIGH_a(iit::rbd::AZ) += qdd(RH_HFE);
    
    RH_THIGH_f = RH_THIGH_I * RH_THIGH_a + vxIv(RH_THIGH_v, RH_THIGH_I) - fext[RH_THIGH];
    
    // First pass, link 'RH_SHANK'
    RH_SHANK_v = ((xm->fr_RH_SHANK_X_fr_RH_THIGH) * RH_THIGH_v);
    RH_SHANK_v(iit::rbd::AZ) += qd(RH_KFE);
    
    motionCrossProductMx<Scalar>(RH_SHANK_v, vcross);
    
    RH_SHANK_a = (xm->fr_RH_SHANK_X_fr_RH_THIGH) * RH_THIGH_a + vcross.col(iit::rbd::AZ) * qd(RH_KFE);
    RH_SHANK_a(iit::rbd::AZ) += qdd(RH_KFE);
    
    RH_SHANK_f = RH_SHANK_I * RH_SHANK_a + vxIv(RH_SHANK_v, RH_SHANK_I) - fext[RH_SHANK];
    
    // First pass, link 'RH_ADAPTER'
    RH_ADAPTER_v = ((xm->fr_RH_ADAPTER_X_fr_RH_SHANK) * RH_SHANK_v);
    RH_ADAPTER_v(iit::rbd::AZ) += qd(RH_SHANK_TO_ADAPTER);
    
    motionCrossProductMx<Scalar>(RH_ADAPTER_v, vcross);
    
    RH_ADAPTER_a = (xm->fr_RH_ADAPTER_X_fr_RH_SHANK) * RH_SHANK_a + vcross.col(iit::rbd::AZ) * qd(RH_SHANK_TO_ADAPTER);
    RH_ADAPTER_a(iit::rbd::AZ) += qdd(RH_SHANK_TO_ADAPTER);
    
    RH_ADAPTER_f = RH_ADAPTER_I * RH_ADAPTER_a + vxIv(RH_ADAPTER_v, RH_ADAPTER_I) - fext[RH_ADAPTER];
    
    // First pass, link 'RH_FOOT'
    RH_FOOT_v = ((xm->fr_RH_FOOT_X_fr_RH_ADAPTER) * RH_ADAPTER_v);
    RH_FOOT_v(iit::rbd::AZ) += qd(RH_ADAPTER_TO_FOOT);
    
    motionCrossProductMx<Scalar>(RH_FOOT_v, vcross);
    
    RH_FOOT_a = (xm->fr_RH_FOOT_X_fr_RH_ADAPTER) * RH_ADAPTER_a + vcross.col(iit::rbd::AZ) * qd(RH_ADAPTER_TO_FOOT);
    RH_FOOT_a(iit::rbd::AZ) += qdd(RH_ADAPTER_TO_FOOT);
    
    RH_FOOT_f = RH_FOOT_I * RH_FOOT_a + vxIv(RH_FOOT_v, RH_FOOT_I) - fext[RH_FOOT];
    
}

void iit::anymal::dyn::InverseDynamics::secondPass(JointState& jForces)
{
    // Link 'RH_FOOT'
    jForces(RH_ADAPTER_TO_FOOT) = RH_FOOT_f(iit::rbd::AZ);
    RH_ADAPTER_f += xm->fr_RH_FOOT_X_fr_RH_ADAPTER.transpose() * RH_FOOT_f;
    // Link 'RH_ADAPTER'
    jForces(RH_SHANK_TO_ADAPTER) = RH_ADAPTER_f(iit::rbd::AZ);
    RH_SHANK_f += xm->fr_RH_ADAPTER_X_fr_RH_SHANK.transpose() * RH_ADAPTER_f;
    // Link 'RH_SHANK'
    jForces(RH_KFE) = RH_SHANK_f(iit::rbd::AZ);
    RH_THIGH_f += xm->fr_RH_SHANK_X_fr_RH_THIGH.transpose() * RH_SHANK_f;
    // Link 'RH_THIGH'
    jForces(RH_HFE) = RH_THIGH_f(iit::rbd::AZ);
    RH_HIP_f += xm->fr_RH_THIGH_X_fr_RH_HIP.transpose() * RH_THIGH_f;
    // Link 'RH_HIP'
    jForces(RH_HAA) = RH_HIP_f(iit::rbd::AZ);
    // Link 'LH_FOOT'
    jForces(LH_ADAPTER_TO_FOOT) = LH_FOOT_f(iit::rbd::AZ);
    LH_ADAPTER_f += xm->fr_LH_FOOT_X_fr_LH_ADAPTER.transpose() * LH_FOOT_f;
    // Link 'LH_ADAPTER'
    jForces(LH_SHANK_TO_ADAPTER) = LH_ADAPTER_f(iit::rbd::AZ);
    LH_SHANK_f += xm->fr_LH_ADAPTER_X_fr_LH_SHANK.transpose() * LH_ADAPTER_f;
    // Link 'LH_SHANK'
    jForces(LH_KFE) = LH_SHANK_f(iit::rbd::AZ);
    LH_THIGH_f += xm->fr_LH_SHANK_X_fr_LH_THIGH.transpose() * LH_SHANK_f;
    // Link 'LH_THIGH'
    jForces(LH_HFE) = LH_THIGH_f(iit::rbd::AZ);
    LH_HIP_f += xm->fr_LH_THIGH_X_fr_LH_HIP.transpose() * LH_THIGH_f;
    // Link 'LH_HIP'
    jForces(LH_HAA) = LH_HIP_f(iit::rbd::AZ);
    // Link 'RF_FOOT'
    jForces(RF_ADAPTER_TO_FOOT) = RF_FOOT_f(iit::rbd::AZ);
    RF_ADAPTER_f += xm->fr_RF_FOOT_X_fr_RF_ADAPTER.transpose() * RF_FOOT_f;
    // Link 'RF_ADAPTER'
    jForces(RF_SHANK_TO_ADAPTER) = RF_ADAPTER_f(iit::rbd::AZ);
    RF_SHANK_f += xm->fr_RF_ADAPTER_X_fr_RF_SHANK.transpose() * RF_ADAPTER_f;
    // Link 'RF_SHANK'
    jForces(RF_KFE) = RF_SHANK_f(iit::rbd::AZ);
    RF_THIGH_f += xm->fr_RF_SHANK_X_fr_RF_THIGH.transpose() * RF_SHANK_f;
    // Link 'RF_THIGH'
    jForces(RF_HFE) = RF_THIGH_f(iit::rbd::AZ);
    RF_HIP_f += xm->fr_RF_THIGH_X_fr_RF_HIP.transpose() * RF_THIGH_f;
    // Link 'RF_HIP'
    jForces(RF_HAA) = RF_HIP_f(iit::rbd::AZ);
    // Link 'LF_FOOT'
    jForces(LF_ADAPTER_TO_FOOT) = LF_FOOT_f(iit::rbd::AZ);
    LF_ADAPTER_f += xm->fr_LF_FOOT_X_fr_LF_ADAPTER.transpose() * LF_FOOT_f;
    // Link 'LF_ADAPTER'
    jForces(LF_SHANK_TO_ADAPTER) = LF_ADAPTER_f(iit::rbd::AZ);
    LF_SHANK_f += xm->fr_LF_ADAPTER_X_fr_LF_SHANK.transpose() * LF_ADAPTER_f;
    // Link 'LF_SHANK'
    jForces(LF_KFE) = LF_SHANK_f(iit::rbd::AZ);
    LF_THIGH_f += xm->fr_LF_SHANK_X_fr_LF_THIGH.transpose() * LF_SHANK_f;
    // Link 'LF_THIGH'
    jForces(LF_HFE) = LF_THIGH_f(iit::rbd::AZ);
    LF_HIP_f += xm->fr_LF_THIGH_X_fr_LF_HIP.transpose() * LF_THIGH_f;
    // Link 'LF_HIP'
    jForces(LF_HAA) = LF_HIP_f(iit::rbd::AZ);
    // Link 'base_inertia'
    jForces(BASE_TO_BASE_INERTIA) = base_inertia_f(iit::rbd::AZ);
}
