#include "forward_dynamics.h"

#include <Eigen/Cholesky>
#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

// Initialization of static-const data
const iit::anymal::dyn::ForwardDynamics::ExtForces
    iit::anymal::dyn::ForwardDynamics::zeroExtForces(Force::Zero());

iit::anymal::dyn::ForwardDynamics::ForwardDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    base_inertia_v.setZero();
    base_inertia_c.setZero();
    LF_HIP_v.setZero();
    LF_HIP_c.setZero();
    LF_THIGH_v.setZero();
    LF_THIGH_c.setZero();
    LF_SHANK_v.setZero();
    LF_SHANK_c.setZero();
    LF_ADAPTER_v.setZero();
    LF_ADAPTER_c.setZero();
    LF_FOOT_v.setZero();
    LF_FOOT_c.setZero();
    RF_HIP_v.setZero();
    RF_HIP_c.setZero();
    RF_THIGH_v.setZero();
    RF_THIGH_c.setZero();
    RF_SHANK_v.setZero();
    RF_SHANK_c.setZero();
    RF_ADAPTER_v.setZero();
    RF_ADAPTER_c.setZero();
    RF_FOOT_v.setZero();
    RF_FOOT_c.setZero();
    LH_HIP_v.setZero();
    LH_HIP_c.setZero();
    LH_THIGH_v.setZero();
    LH_THIGH_c.setZero();
    LH_SHANK_v.setZero();
    LH_SHANK_c.setZero();
    LH_ADAPTER_v.setZero();
    LH_ADAPTER_c.setZero();
    LH_FOOT_v.setZero();
    LH_FOOT_c.setZero();
    RH_HIP_v.setZero();
    RH_HIP_c.setZero();
    RH_THIGH_v.setZero();
    RH_THIGH_c.setZero();
    RH_SHANK_v.setZero();
    RH_SHANK_c.setZero();
    RH_ADAPTER_v.setZero();
    RH_ADAPTER_c.setZero();
    RH_FOOT_v.setZero();
    RH_FOOT_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

void iit::anymal::dyn::ForwardDynamics::fd(
    JointState& qdd,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    base_inertia_AI = inertiaProps->getTensor_base_inertia();
    base_inertia_p = - fext[BASE_INERTIA];
    LF_HIP_AI = inertiaProps->getTensor_LF_HIP();
    LF_HIP_p = - fext[LF_HIP];
    LF_THIGH_AI = inertiaProps->getTensor_LF_THIGH();
    LF_THIGH_p = - fext[LF_THIGH];
    LF_SHANK_AI = inertiaProps->getTensor_LF_SHANK();
    LF_SHANK_p = - fext[LF_SHANK];
    LF_ADAPTER_AI = inertiaProps->getTensor_LF_ADAPTER();
    LF_ADAPTER_p = - fext[LF_ADAPTER];
    LF_FOOT_AI = inertiaProps->getTensor_LF_FOOT();
    LF_FOOT_p = - fext[LF_FOOT];
    RF_HIP_AI = inertiaProps->getTensor_RF_HIP();
    RF_HIP_p = - fext[RF_HIP];
    RF_THIGH_AI = inertiaProps->getTensor_RF_THIGH();
    RF_THIGH_p = - fext[RF_THIGH];
    RF_SHANK_AI = inertiaProps->getTensor_RF_SHANK();
    RF_SHANK_p = - fext[RF_SHANK];
    RF_ADAPTER_AI = inertiaProps->getTensor_RF_ADAPTER();
    RF_ADAPTER_p = - fext[RF_ADAPTER];
    RF_FOOT_AI = inertiaProps->getTensor_RF_FOOT();
    RF_FOOT_p = - fext[RF_FOOT];
    LH_HIP_AI = inertiaProps->getTensor_LH_HIP();
    LH_HIP_p = - fext[LH_HIP];
    LH_THIGH_AI = inertiaProps->getTensor_LH_THIGH();
    LH_THIGH_p = - fext[LH_THIGH];
    LH_SHANK_AI = inertiaProps->getTensor_LH_SHANK();
    LH_SHANK_p = - fext[LH_SHANK];
    LH_ADAPTER_AI = inertiaProps->getTensor_LH_ADAPTER();
    LH_ADAPTER_p = - fext[LH_ADAPTER];
    LH_FOOT_AI = inertiaProps->getTensor_LH_FOOT();
    LH_FOOT_p = - fext[LH_FOOT];
    RH_HIP_AI = inertiaProps->getTensor_RH_HIP();
    RH_HIP_p = - fext[RH_HIP];
    RH_THIGH_AI = inertiaProps->getTensor_RH_THIGH();
    RH_THIGH_p = - fext[RH_THIGH];
    RH_SHANK_AI = inertiaProps->getTensor_RH_SHANK();
    RH_SHANK_p = - fext[RH_SHANK];
    RH_ADAPTER_AI = inertiaProps->getTensor_RH_ADAPTER();
    RH_ADAPTER_p = - fext[RH_ADAPTER];
    RH_FOOT_AI = inertiaProps->getTensor_RH_FOOT();
    RH_FOOT_p = - fext[RH_FOOT];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link base_inertia
    //  - The spatial velocity:
    base_inertia_v(AZ) = qd(BASE_TO_BASE_INERTIA);
    
    //  - The bias force term:
    base_inertia_p += vxIv(qd(BASE_TO_BASE_INERTIA), base_inertia_AI);
    
    // + Link LF_HIP
    //  - The spatial velocity:
    LF_HIP_v(AZ) = qd(LF_HAA);
    
    //  - The bias force term:
    LF_HIP_p += vxIv(qd(LF_HAA), LF_HIP_AI);
    
    // + Link LF_THIGH
    //  - The spatial velocity:
    LF_THIGH_v = (motionTransforms-> fr_LF_THIGH_X_fr_LF_HIP) * LF_HIP_v;
    LF_THIGH_v(AZ) += qd(LF_HFE);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(LF_THIGH_v, vcross);
    LF_THIGH_c = vcross.col(AZ) * qd(LF_HFE);
    
    //  - The bias force term:
    LF_THIGH_p += vxIv(LF_THIGH_v, LF_THIGH_AI);
    
    // + Link LF_SHANK
    //  - The spatial velocity:
    LF_SHANK_v = (motionTransforms-> fr_LF_SHANK_X_fr_LF_THIGH) * LF_THIGH_v;
    LF_SHANK_v(AZ) += qd(LF_KFE);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(LF_SHANK_v, vcross);
    LF_SHANK_c = vcross.col(AZ) * qd(LF_KFE);
    
    //  - The bias force term:
    LF_SHANK_p += vxIv(LF_SHANK_v, LF_SHANK_AI);
    
    // + Link LF_ADAPTER
    //  - The spatial velocity:
    LF_ADAPTER_v = (motionTransforms-> fr_LF_ADAPTER_X_fr_LF_SHANK) * LF_SHANK_v;
    LF_ADAPTER_v(AZ) += qd(LF_SHANK_TO_ADAPTER);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(LF_ADAPTER_v, vcross);
    LF_ADAPTER_c = vcross.col(AZ) * qd(LF_SHANK_TO_ADAPTER);
    
    //  - The bias force term:
    LF_ADAPTER_p += vxIv(LF_ADAPTER_v, LF_ADAPTER_AI);
    
    // + Link LF_FOOT
    //  - The spatial velocity:
    LF_FOOT_v = (motionTransforms-> fr_LF_FOOT_X_fr_LF_ADAPTER) * LF_ADAPTER_v;
    LF_FOOT_v(AZ) += qd(LF_ADAPTER_TO_FOOT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(LF_FOOT_v, vcross);
    LF_FOOT_c = vcross.col(AZ) * qd(LF_ADAPTER_TO_FOOT);
    
    //  - The bias force term:
    LF_FOOT_p += vxIv(LF_FOOT_v, LF_FOOT_AI);
    
    // + Link RF_HIP
    //  - The spatial velocity:
    RF_HIP_v(AZ) = qd(RF_HAA);
    
    //  - The bias force term:
    RF_HIP_p += vxIv(qd(RF_HAA), RF_HIP_AI);
    
    // + Link RF_THIGH
    //  - The spatial velocity:
    RF_THIGH_v = (motionTransforms-> fr_RF_THIGH_X_fr_RF_HIP) * RF_HIP_v;
    RF_THIGH_v(AZ) += qd(RF_HFE);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(RF_THIGH_v, vcross);
    RF_THIGH_c = vcross.col(AZ) * qd(RF_HFE);
    
    //  - The bias force term:
    RF_THIGH_p += vxIv(RF_THIGH_v, RF_THIGH_AI);
    
    // + Link RF_SHANK
    //  - The spatial velocity:
    RF_SHANK_v = (motionTransforms-> fr_RF_SHANK_X_fr_RF_THIGH) * RF_THIGH_v;
    RF_SHANK_v(AZ) += qd(RF_KFE);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(RF_SHANK_v, vcross);
    RF_SHANK_c = vcross.col(AZ) * qd(RF_KFE);
    
    //  - The bias force term:
    RF_SHANK_p += vxIv(RF_SHANK_v, RF_SHANK_AI);
    
    // + Link RF_ADAPTER
    //  - The spatial velocity:
    RF_ADAPTER_v = (motionTransforms-> fr_RF_ADAPTER_X_fr_RF_SHANK) * RF_SHANK_v;
    RF_ADAPTER_v(AZ) += qd(RF_SHANK_TO_ADAPTER);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(RF_ADAPTER_v, vcross);
    RF_ADAPTER_c = vcross.col(AZ) * qd(RF_SHANK_TO_ADAPTER);
    
    //  - The bias force term:
    RF_ADAPTER_p += vxIv(RF_ADAPTER_v, RF_ADAPTER_AI);
    
    // + Link RF_FOOT
    //  - The spatial velocity:
    RF_FOOT_v = (motionTransforms-> fr_RF_FOOT_X_fr_RF_ADAPTER) * RF_ADAPTER_v;
    RF_FOOT_v(AZ) += qd(RF_ADAPTER_TO_FOOT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(RF_FOOT_v, vcross);
    RF_FOOT_c = vcross.col(AZ) * qd(RF_ADAPTER_TO_FOOT);
    
    //  - The bias force term:
    RF_FOOT_p += vxIv(RF_FOOT_v, RF_FOOT_AI);
    
    // + Link LH_HIP
    //  - The spatial velocity:
    LH_HIP_v(AZ) = qd(LH_HAA);
    
    //  - The bias force term:
    LH_HIP_p += vxIv(qd(LH_HAA), LH_HIP_AI);
    
    // + Link LH_THIGH
    //  - The spatial velocity:
    LH_THIGH_v = (motionTransforms-> fr_LH_THIGH_X_fr_LH_HIP) * LH_HIP_v;
    LH_THIGH_v(AZ) += qd(LH_HFE);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(LH_THIGH_v, vcross);
    LH_THIGH_c = vcross.col(AZ) * qd(LH_HFE);
    
    //  - The bias force term:
    LH_THIGH_p += vxIv(LH_THIGH_v, LH_THIGH_AI);
    
    // + Link LH_SHANK
    //  - The spatial velocity:
    LH_SHANK_v = (motionTransforms-> fr_LH_SHANK_X_fr_LH_THIGH) * LH_THIGH_v;
    LH_SHANK_v(AZ) += qd(LH_KFE);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(LH_SHANK_v, vcross);
    LH_SHANK_c = vcross.col(AZ) * qd(LH_KFE);
    
    //  - The bias force term:
    LH_SHANK_p += vxIv(LH_SHANK_v, LH_SHANK_AI);
    
    // + Link LH_ADAPTER
    //  - The spatial velocity:
    LH_ADAPTER_v = (motionTransforms-> fr_LH_ADAPTER_X_fr_LH_SHANK) * LH_SHANK_v;
    LH_ADAPTER_v(AZ) += qd(LH_SHANK_TO_ADAPTER);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(LH_ADAPTER_v, vcross);
    LH_ADAPTER_c = vcross.col(AZ) * qd(LH_SHANK_TO_ADAPTER);
    
    //  - The bias force term:
    LH_ADAPTER_p += vxIv(LH_ADAPTER_v, LH_ADAPTER_AI);
    
    // + Link LH_FOOT
    //  - The spatial velocity:
    LH_FOOT_v = (motionTransforms-> fr_LH_FOOT_X_fr_LH_ADAPTER) * LH_ADAPTER_v;
    LH_FOOT_v(AZ) += qd(LH_ADAPTER_TO_FOOT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(LH_FOOT_v, vcross);
    LH_FOOT_c = vcross.col(AZ) * qd(LH_ADAPTER_TO_FOOT);
    
    //  - The bias force term:
    LH_FOOT_p += vxIv(LH_FOOT_v, LH_FOOT_AI);
    
    // + Link RH_HIP
    //  - The spatial velocity:
    RH_HIP_v(AZ) = qd(RH_HAA);
    
    //  - The bias force term:
    RH_HIP_p += vxIv(qd(RH_HAA), RH_HIP_AI);
    
    // + Link RH_THIGH
    //  - The spatial velocity:
    RH_THIGH_v = (motionTransforms-> fr_RH_THIGH_X_fr_RH_HIP) * RH_HIP_v;
    RH_THIGH_v(AZ) += qd(RH_HFE);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(RH_THIGH_v, vcross);
    RH_THIGH_c = vcross.col(AZ) * qd(RH_HFE);
    
    //  - The bias force term:
    RH_THIGH_p += vxIv(RH_THIGH_v, RH_THIGH_AI);
    
    // + Link RH_SHANK
    //  - The spatial velocity:
    RH_SHANK_v = (motionTransforms-> fr_RH_SHANK_X_fr_RH_THIGH) * RH_THIGH_v;
    RH_SHANK_v(AZ) += qd(RH_KFE);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(RH_SHANK_v, vcross);
    RH_SHANK_c = vcross.col(AZ) * qd(RH_KFE);
    
    //  - The bias force term:
    RH_SHANK_p += vxIv(RH_SHANK_v, RH_SHANK_AI);
    
    // + Link RH_ADAPTER
    //  - The spatial velocity:
    RH_ADAPTER_v = (motionTransforms-> fr_RH_ADAPTER_X_fr_RH_SHANK) * RH_SHANK_v;
    RH_ADAPTER_v(AZ) += qd(RH_SHANK_TO_ADAPTER);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(RH_ADAPTER_v, vcross);
    RH_ADAPTER_c = vcross.col(AZ) * qd(RH_SHANK_TO_ADAPTER);
    
    //  - The bias force term:
    RH_ADAPTER_p += vxIv(RH_ADAPTER_v, RH_ADAPTER_AI);
    
    // + Link RH_FOOT
    //  - The spatial velocity:
    RH_FOOT_v = (motionTransforms-> fr_RH_FOOT_X_fr_RH_ADAPTER) * RH_ADAPTER_v;
    RH_FOOT_v(AZ) += qd(RH_ADAPTER_TO_FOOT);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(RH_FOOT_v, vcross);
    RH_FOOT_c = vcross.col(AZ) * qd(RH_ADAPTER_TO_FOOT);
    
    //  - The bias force term:
    RH_FOOT_p += vxIv(RH_FOOT_v, RH_FOOT_AI);
    
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66 IaB;
    Force pa;
    
    // + Link RH_FOOT
    RH_FOOT_u = tau(RH_ADAPTER_TO_FOOT) - RH_FOOT_p(AZ);
    RH_FOOT_U = RH_FOOT_AI.col(AZ);
    RH_FOOT_D = RH_FOOT_U(AZ);
    
    compute_Ia_revolute(RH_FOOT_AI, RH_FOOT_U, RH_FOOT_D, Ia_r);  // same as: Ia_r = RH_FOOT_AI - RH_FOOT_U/RH_FOOT_D * RH_FOOT_U.transpose();
    pa = RH_FOOT_p + Ia_r * RH_FOOT_c + RH_FOOT_U * RH_FOOT_u/RH_FOOT_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RH_FOOT_X_fr_RH_ADAPTER, IaB);
    RH_ADAPTER_AI += IaB;
    RH_ADAPTER_p += (motionTransforms-> fr_RH_FOOT_X_fr_RH_ADAPTER).transpose() * pa;
    
    // + Link RH_ADAPTER
    RH_ADAPTER_u = tau(RH_SHANK_TO_ADAPTER) - RH_ADAPTER_p(AZ);
    RH_ADAPTER_U = RH_ADAPTER_AI.col(AZ);
    RH_ADAPTER_D = RH_ADAPTER_U(AZ);
    
    compute_Ia_revolute(RH_ADAPTER_AI, RH_ADAPTER_U, RH_ADAPTER_D, Ia_r);  // same as: Ia_r = RH_ADAPTER_AI - RH_ADAPTER_U/RH_ADAPTER_D * RH_ADAPTER_U.transpose();
    pa = RH_ADAPTER_p + Ia_r * RH_ADAPTER_c + RH_ADAPTER_U * RH_ADAPTER_u/RH_ADAPTER_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RH_ADAPTER_X_fr_RH_SHANK, IaB);
    RH_SHANK_AI += IaB;
    RH_SHANK_p += (motionTransforms-> fr_RH_ADAPTER_X_fr_RH_SHANK).transpose() * pa;
    
    // + Link RH_SHANK
    RH_SHANK_u = tau(RH_KFE) - RH_SHANK_p(AZ);
    RH_SHANK_U = RH_SHANK_AI.col(AZ);
    RH_SHANK_D = RH_SHANK_U(AZ);
    
    compute_Ia_revolute(RH_SHANK_AI, RH_SHANK_U, RH_SHANK_D, Ia_r);  // same as: Ia_r = RH_SHANK_AI - RH_SHANK_U/RH_SHANK_D * RH_SHANK_U.transpose();
    pa = RH_SHANK_p + Ia_r * RH_SHANK_c + RH_SHANK_U * RH_SHANK_u/RH_SHANK_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RH_SHANK_X_fr_RH_THIGH, IaB);
    RH_THIGH_AI += IaB;
    RH_THIGH_p += (motionTransforms-> fr_RH_SHANK_X_fr_RH_THIGH).transpose() * pa;
    
    // + Link RH_THIGH
    RH_THIGH_u = tau(RH_HFE) - RH_THIGH_p(AZ);
    RH_THIGH_U = RH_THIGH_AI.col(AZ);
    RH_THIGH_D = RH_THIGH_U(AZ);
    
    compute_Ia_revolute(RH_THIGH_AI, RH_THIGH_U, RH_THIGH_D, Ia_r);  // same as: Ia_r = RH_THIGH_AI - RH_THIGH_U/RH_THIGH_D * RH_THIGH_U.transpose();
    pa = RH_THIGH_p + Ia_r * RH_THIGH_c + RH_THIGH_U * RH_THIGH_u/RH_THIGH_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RH_THIGH_X_fr_RH_HIP, IaB);
    RH_HIP_AI += IaB;
    RH_HIP_p += (motionTransforms-> fr_RH_THIGH_X_fr_RH_HIP).transpose() * pa;
    
    // + Link RH_HIP
    RH_HIP_u = tau(RH_HAA) - RH_HIP_p(AZ);
    RH_HIP_U = RH_HIP_AI.col(AZ);
    RH_HIP_D = RH_HIP_U(AZ);
    
    
    // + Link LH_FOOT
    LH_FOOT_u = tau(LH_ADAPTER_TO_FOOT) - LH_FOOT_p(AZ);
    LH_FOOT_U = LH_FOOT_AI.col(AZ);
    LH_FOOT_D = LH_FOOT_U(AZ);
    
    compute_Ia_revolute(LH_FOOT_AI, LH_FOOT_U, LH_FOOT_D, Ia_r);  // same as: Ia_r = LH_FOOT_AI - LH_FOOT_U/LH_FOOT_D * LH_FOOT_U.transpose();
    pa = LH_FOOT_p + Ia_r * LH_FOOT_c + LH_FOOT_U * LH_FOOT_u/LH_FOOT_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LH_FOOT_X_fr_LH_ADAPTER, IaB);
    LH_ADAPTER_AI += IaB;
    LH_ADAPTER_p += (motionTransforms-> fr_LH_FOOT_X_fr_LH_ADAPTER).transpose() * pa;
    
    // + Link LH_ADAPTER
    LH_ADAPTER_u = tau(LH_SHANK_TO_ADAPTER) - LH_ADAPTER_p(AZ);
    LH_ADAPTER_U = LH_ADAPTER_AI.col(AZ);
    LH_ADAPTER_D = LH_ADAPTER_U(AZ);
    
    compute_Ia_revolute(LH_ADAPTER_AI, LH_ADAPTER_U, LH_ADAPTER_D, Ia_r);  // same as: Ia_r = LH_ADAPTER_AI - LH_ADAPTER_U/LH_ADAPTER_D * LH_ADAPTER_U.transpose();
    pa = LH_ADAPTER_p + Ia_r * LH_ADAPTER_c + LH_ADAPTER_U * LH_ADAPTER_u/LH_ADAPTER_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LH_ADAPTER_X_fr_LH_SHANK, IaB);
    LH_SHANK_AI += IaB;
    LH_SHANK_p += (motionTransforms-> fr_LH_ADAPTER_X_fr_LH_SHANK).transpose() * pa;
    
    // + Link LH_SHANK
    LH_SHANK_u = tau(LH_KFE) - LH_SHANK_p(AZ);
    LH_SHANK_U = LH_SHANK_AI.col(AZ);
    LH_SHANK_D = LH_SHANK_U(AZ);
    
    compute_Ia_revolute(LH_SHANK_AI, LH_SHANK_U, LH_SHANK_D, Ia_r);  // same as: Ia_r = LH_SHANK_AI - LH_SHANK_U/LH_SHANK_D * LH_SHANK_U.transpose();
    pa = LH_SHANK_p + Ia_r * LH_SHANK_c + LH_SHANK_U * LH_SHANK_u/LH_SHANK_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LH_SHANK_X_fr_LH_THIGH, IaB);
    LH_THIGH_AI += IaB;
    LH_THIGH_p += (motionTransforms-> fr_LH_SHANK_X_fr_LH_THIGH).transpose() * pa;
    
    // + Link LH_THIGH
    LH_THIGH_u = tau(LH_HFE) - LH_THIGH_p(AZ);
    LH_THIGH_U = LH_THIGH_AI.col(AZ);
    LH_THIGH_D = LH_THIGH_U(AZ);
    
    compute_Ia_revolute(LH_THIGH_AI, LH_THIGH_U, LH_THIGH_D, Ia_r);  // same as: Ia_r = LH_THIGH_AI - LH_THIGH_U/LH_THIGH_D * LH_THIGH_U.transpose();
    pa = LH_THIGH_p + Ia_r * LH_THIGH_c + LH_THIGH_U * LH_THIGH_u/LH_THIGH_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LH_THIGH_X_fr_LH_HIP, IaB);
    LH_HIP_AI += IaB;
    LH_HIP_p += (motionTransforms-> fr_LH_THIGH_X_fr_LH_HIP).transpose() * pa;
    
    // + Link LH_HIP
    LH_HIP_u = tau(LH_HAA) - LH_HIP_p(AZ);
    LH_HIP_U = LH_HIP_AI.col(AZ);
    LH_HIP_D = LH_HIP_U(AZ);
    
    
    // + Link RF_FOOT
    RF_FOOT_u = tau(RF_ADAPTER_TO_FOOT) - RF_FOOT_p(AZ);
    RF_FOOT_U = RF_FOOT_AI.col(AZ);
    RF_FOOT_D = RF_FOOT_U(AZ);
    
    compute_Ia_revolute(RF_FOOT_AI, RF_FOOT_U, RF_FOOT_D, Ia_r);  // same as: Ia_r = RF_FOOT_AI - RF_FOOT_U/RF_FOOT_D * RF_FOOT_U.transpose();
    pa = RF_FOOT_p + Ia_r * RF_FOOT_c + RF_FOOT_U * RF_FOOT_u/RF_FOOT_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RF_FOOT_X_fr_RF_ADAPTER, IaB);
    RF_ADAPTER_AI += IaB;
    RF_ADAPTER_p += (motionTransforms-> fr_RF_FOOT_X_fr_RF_ADAPTER).transpose() * pa;
    
    // + Link RF_ADAPTER
    RF_ADAPTER_u = tau(RF_SHANK_TO_ADAPTER) - RF_ADAPTER_p(AZ);
    RF_ADAPTER_U = RF_ADAPTER_AI.col(AZ);
    RF_ADAPTER_D = RF_ADAPTER_U(AZ);
    
    compute_Ia_revolute(RF_ADAPTER_AI, RF_ADAPTER_U, RF_ADAPTER_D, Ia_r);  // same as: Ia_r = RF_ADAPTER_AI - RF_ADAPTER_U/RF_ADAPTER_D * RF_ADAPTER_U.transpose();
    pa = RF_ADAPTER_p + Ia_r * RF_ADAPTER_c + RF_ADAPTER_U * RF_ADAPTER_u/RF_ADAPTER_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RF_ADAPTER_X_fr_RF_SHANK, IaB);
    RF_SHANK_AI += IaB;
    RF_SHANK_p += (motionTransforms-> fr_RF_ADAPTER_X_fr_RF_SHANK).transpose() * pa;
    
    // + Link RF_SHANK
    RF_SHANK_u = tau(RF_KFE) - RF_SHANK_p(AZ);
    RF_SHANK_U = RF_SHANK_AI.col(AZ);
    RF_SHANK_D = RF_SHANK_U(AZ);
    
    compute_Ia_revolute(RF_SHANK_AI, RF_SHANK_U, RF_SHANK_D, Ia_r);  // same as: Ia_r = RF_SHANK_AI - RF_SHANK_U/RF_SHANK_D * RF_SHANK_U.transpose();
    pa = RF_SHANK_p + Ia_r * RF_SHANK_c + RF_SHANK_U * RF_SHANK_u/RF_SHANK_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RF_SHANK_X_fr_RF_THIGH, IaB);
    RF_THIGH_AI += IaB;
    RF_THIGH_p += (motionTransforms-> fr_RF_SHANK_X_fr_RF_THIGH).transpose() * pa;
    
    // + Link RF_THIGH
    RF_THIGH_u = tau(RF_HFE) - RF_THIGH_p(AZ);
    RF_THIGH_U = RF_THIGH_AI.col(AZ);
    RF_THIGH_D = RF_THIGH_U(AZ);
    
    compute_Ia_revolute(RF_THIGH_AI, RF_THIGH_U, RF_THIGH_D, Ia_r);  // same as: Ia_r = RF_THIGH_AI - RF_THIGH_U/RF_THIGH_D * RF_THIGH_U.transpose();
    pa = RF_THIGH_p + Ia_r * RF_THIGH_c + RF_THIGH_U * RF_THIGH_u/RF_THIGH_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_RF_THIGH_X_fr_RF_HIP, IaB);
    RF_HIP_AI += IaB;
    RF_HIP_p += (motionTransforms-> fr_RF_THIGH_X_fr_RF_HIP).transpose() * pa;
    
    // + Link RF_HIP
    RF_HIP_u = tau(RF_HAA) - RF_HIP_p(AZ);
    RF_HIP_U = RF_HIP_AI.col(AZ);
    RF_HIP_D = RF_HIP_U(AZ);
    
    
    // + Link LF_FOOT
    LF_FOOT_u = tau(LF_ADAPTER_TO_FOOT) - LF_FOOT_p(AZ);
    LF_FOOT_U = LF_FOOT_AI.col(AZ);
    LF_FOOT_D = LF_FOOT_U(AZ);
    
    compute_Ia_revolute(LF_FOOT_AI, LF_FOOT_U, LF_FOOT_D, Ia_r);  // same as: Ia_r = LF_FOOT_AI - LF_FOOT_U/LF_FOOT_D * LF_FOOT_U.transpose();
    pa = LF_FOOT_p + Ia_r * LF_FOOT_c + LF_FOOT_U * LF_FOOT_u/LF_FOOT_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LF_FOOT_X_fr_LF_ADAPTER, IaB);
    LF_ADAPTER_AI += IaB;
    LF_ADAPTER_p += (motionTransforms-> fr_LF_FOOT_X_fr_LF_ADAPTER).transpose() * pa;
    
    // + Link LF_ADAPTER
    LF_ADAPTER_u = tau(LF_SHANK_TO_ADAPTER) - LF_ADAPTER_p(AZ);
    LF_ADAPTER_U = LF_ADAPTER_AI.col(AZ);
    LF_ADAPTER_D = LF_ADAPTER_U(AZ);
    
    compute_Ia_revolute(LF_ADAPTER_AI, LF_ADAPTER_U, LF_ADAPTER_D, Ia_r);  // same as: Ia_r = LF_ADAPTER_AI - LF_ADAPTER_U/LF_ADAPTER_D * LF_ADAPTER_U.transpose();
    pa = LF_ADAPTER_p + Ia_r * LF_ADAPTER_c + LF_ADAPTER_U * LF_ADAPTER_u/LF_ADAPTER_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LF_ADAPTER_X_fr_LF_SHANK, IaB);
    LF_SHANK_AI += IaB;
    LF_SHANK_p += (motionTransforms-> fr_LF_ADAPTER_X_fr_LF_SHANK).transpose() * pa;
    
    // + Link LF_SHANK
    LF_SHANK_u = tau(LF_KFE) - LF_SHANK_p(AZ);
    LF_SHANK_U = LF_SHANK_AI.col(AZ);
    LF_SHANK_D = LF_SHANK_U(AZ);
    
    compute_Ia_revolute(LF_SHANK_AI, LF_SHANK_U, LF_SHANK_D, Ia_r);  // same as: Ia_r = LF_SHANK_AI - LF_SHANK_U/LF_SHANK_D * LF_SHANK_U.transpose();
    pa = LF_SHANK_p + Ia_r * LF_SHANK_c + LF_SHANK_U * LF_SHANK_u/LF_SHANK_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LF_SHANK_X_fr_LF_THIGH, IaB);
    LF_THIGH_AI += IaB;
    LF_THIGH_p += (motionTransforms-> fr_LF_SHANK_X_fr_LF_THIGH).transpose() * pa;
    
    // + Link LF_THIGH
    LF_THIGH_u = tau(LF_HFE) - LF_THIGH_p(AZ);
    LF_THIGH_U = LF_THIGH_AI.col(AZ);
    LF_THIGH_D = LF_THIGH_U(AZ);
    
    compute_Ia_revolute(LF_THIGH_AI, LF_THIGH_U, LF_THIGH_D, Ia_r);  // same as: Ia_r = LF_THIGH_AI - LF_THIGH_U/LF_THIGH_D * LF_THIGH_U.transpose();
    pa = LF_THIGH_p + Ia_r * LF_THIGH_c + LF_THIGH_U * LF_THIGH_u/LF_THIGH_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_LF_THIGH_X_fr_LF_HIP, IaB);
    LF_HIP_AI += IaB;
    LF_HIP_p += (motionTransforms-> fr_LF_THIGH_X_fr_LF_HIP).transpose() * pa;
    
    // + Link LF_HIP
    LF_HIP_u = tau(LF_HAA) - LF_HIP_p(AZ);
    LF_HIP_U = LF_HIP_AI.col(AZ);
    LF_HIP_D = LF_HIP_U(AZ);
    
    
    // + Link base_inertia
    base_inertia_u = tau(BASE_TO_BASE_INERTIA) - base_inertia_p(AZ);
    base_inertia_U = base_inertia_AI.col(AZ);
    base_inertia_D = base_inertia_U(AZ);
    
    
    
    // ---------------------- THIRD PASS ---------------------- //
    base_inertia_a = (motionTransforms-> fr_base_inertia_X_fr_base).col(LZ) * (anymal::g);
    qdd(BASE_TO_BASE_INERTIA) = (base_inertia_u - base_inertia_U.dot(base_inertia_a)) / base_inertia_D;
    base_inertia_a(AZ) += qdd(BASE_TO_BASE_INERTIA);
    
    LF_HIP_a = (motionTransforms-> fr_LF_HIP_X_fr_base).col(LZ) * (anymal::g);
    qdd(LF_HAA) = (LF_HIP_u - LF_HIP_U.dot(LF_HIP_a)) / LF_HIP_D;
    LF_HIP_a(AZ) += qdd(LF_HAA);
    
    LF_THIGH_a = (motionTransforms-> fr_LF_THIGH_X_fr_LF_HIP) * LF_HIP_a + LF_THIGH_c;
    qdd(LF_HFE) = (LF_THIGH_u - LF_THIGH_U.dot(LF_THIGH_a)) / LF_THIGH_D;
    LF_THIGH_a(AZ) += qdd(LF_HFE);
    
    LF_SHANK_a = (motionTransforms-> fr_LF_SHANK_X_fr_LF_THIGH) * LF_THIGH_a + LF_SHANK_c;
    qdd(LF_KFE) = (LF_SHANK_u - LF_SHANK_U.dot(LF_SHANK_a)) / LF_SHANK_D;
    LF_SHANK_a(AZ) += qdd(LF_KFE);
    
    LF_ADAPTER_a = (motionTransforms-> fr_LF_ADAPTER_X_fr_LF_SHANK) * LF_SHANK_a + LF_ADAPTER_c;
    qdd(LF_SHANK_TO_ADAPTER) = (LF_ADAPTER_u - LF_ADAPTER_U.dot(LF_ADAPTER_a)) / LF_ADAPTER_D;
    LF_ADAPTER_a(AZ) += qdd(LF_SHANK_TO_ADAPTER);
    
    LF_FOOT_a = (motionTransforms-> fr_LF_FOOT_X_fr_LF_ADAPTER) * LF_ADAPTER_a + LF_FOOT_c;
    qdd(LF_ADAPTER_TO_FOOT) = (LF_FOOT_u - LF_FOOT_U.dot(LF_FOOT_a)) / LF_FOOT_D;
    LF_FOOT_a(AZ) += qdd(LF_ADAPTER_TO_FOOT);
    
    RF_HIP_a = (motionTransforms-> fr_RF_HIP_X_fr_base).col(LZ) * (anymal::g);
    qdd(RF_HAA) = (RF_HIP_u - RF_HIP_U.dot(RF_HIP_a)) / RF_HIP_D;
    RF_HIP_a(AZ) += qdd(RF_HAA);
    
    RF_THIGH_a = (motionTransforms-> fr_RF_THIGH_X_fr_RF_HIP) * RF_HIP_a + RF_THIGH_c;
    qdd(RF_HFE) = (RF_THIGH_u - RF_THIGH_U.dot(RF_THIGH_a)) / RF_THIGH_D;
    RF_THIGH_a(AZ) += qdd(RF_HFE);
    
    RF_SHANK_a = (motionTransforms-> fr_RF_SHANK_X_fr_RF_THIGH) * RF_THIGH_a + RF_SHANK_c;
    qdd(RF_KFE) = (RF_SHANK_u - RF_SHANK_U.dot(RF_SHANK_a)) / RF_SHANK_D;
    RF_SHANK_a(AZ) += qdd(RF_KFE);
    
    RF_ADAPTER_a = (motionTransforms-> fr_RF_ADAPTER_X_fr_RF_SHANK) * RF_SHANK_a + RF_ADAPTER_c;
    qdd(RF_SHANK_TO_ADAPTER) = (RF_ADAPTER_u - RF_ADAPTER_U.dot(RF_ADAPTER_a)) / RF_ADAPTER_D;
    RF_ADAPTER_a(AZ) += qdd(RF_SHANK_TO_ADAPTER);
    
    RF_FOOT_a = (motionTransforms-> fr_RF_FOOT_X_fr_RF_ADAPTER) * RF_ADAPTER_a + RF_FOOT_c;
    qdd(RF_ADAPTER_TO_FOOT) = (RF_FOOT_u - RF_FOOT_U.dot(RF_FOOT_a)) / RF_FOOT_D;
    RF_FOOT_a(AZ) += qdd(RF_ADAPTER_TO_FOOT);
    
    LH_HIP_a = (motionTransforms-> fr_LH_HIP_X_fr_base).col(LZ) * (anymal::g);
    qdd(LH_HAA) = (LH_HIP_u - LH_HIP_U.dot(LH_HIP_a)) / LH_HIP_D;
    LH_HIP_a(AZ) += qdd(LH_HAA);
    
    LH_THIGH_a = (motionTransforms-> fr_LH_THIGH_X_fr_LH_HIP) * LH_HIP_a + LH_THIGH_c;
    qdd(LH_HFE) = (LH_THIGH_u - LH_THIGH_U.dot(LH_THIGH_a)) / LH_THIGH_D;
    LH_THIGH_a(AZ) += qdd(LH_HFE);
    
    LH_SHANK_a = (motionTransforms-> fr_LH_SHANK_X_fr_LH_THIGH) * LH_THIGH_a + LH_SHANK_c;
    qdd(LH_KFE) = (LH_SHANK_u - LH_SHANK_U.dot(LH_SHANK_a)) / LH_SHANK_D;
    LH_SHANK_a(AZ) += qdd(LH_KFE);
    
    LH_ADAPTER_a = (motionTransforms-> fr_LH_ADAPTER_X_fr_LH_SHANK) * LH_SHANK_a + LH_ADAPTER_c;
    qdd(LH_SHANK_TO_ADAPTER) = (LH_ADAPTER_u - LH_ADAPTER_U.dot(LH_ADAPTER_a)) / LH_ADAPTER_D;
    LH_ADAPTER_a(AZ) += qdd(LH_SHANK_TO_ADAPTER);
    
    LH_FOOT_a = (motionTransforms-> fr_LH_FOOT_X_fr_LH_ADAPTER) * LH_ADAPTER_a + LH_FOOT_c;
    qdd(LH_ADAPTER_TO_FOOT) = (LH_FOOT_u - LH_FOOT_U.dot(LH_FOOT_a)) / LH_FOOT_D;
    LH_FOOT_a(AZ) += qdd(LH_ADAPTER_TO_FOOT);
    
    RH_HIP_a = (motionTransforms-> fr_RH_HIP_X_fr_base).col(LZ) * (anymal::g);
    qdd(RH_HAA) = (RH_HIP_u - RH_HIP_U.dot(RH_HIP_a)) / RH_HIP_D;
    RH_HIP_a(AZ) += qdd(RH_HAA);
    
    RH_THIGH_a = (motionTransforms-> fr_RH_THIGH_X_fr_RH_HIP) * RH_HIP_a + RH_THIGH_c;
    qdd(RH_HFE) = (RH_THIGH_u - RH_THIGH_U.dot(RH_THIGH_a)) / RH_THIGH_D;
    RH_THIGH_a(AZ) += qdd(RH_HFE);
    
    RH_SHANK_a = (motionTransforms-> fr_RH_SHANK_X_fr_RH_THIGH) * RH_THIGH_a + RH_SHANK_c;
    qdd(RH_KFE) = (RH_SHANK_u - RH_SHANK_U.dot(RH_SHANK_a)) / RH_SHANK_D;
    RH_SHANK_a(AZ) += qdd(RH_KFE);
    
    RH_ADAPTER_a = (motionTransforms-> fr_RH_ADAPTER_X_fr_RH_SHANK) * RH_SHANK_a + RH_ADAPTER_c;
    qdd(RH_SHANK_TO_ADAPTER) = (RH_ADAPTER_u - RH_ADAPTER_U.dot(RH_ADAPTER_a)) / RH_ADAPTER_D;
    RH_ADAPTER_a(AZ) += qdd(RH_SHANK_TO_ADAPTER);
    
    RH_FOOT_a = (motionTransforms-> fr_RH_FOOT_X_fr_RH_ADAPTER) * RH_ADAPTER_a + RH_FOOT_c;
    qdd(RH_ADAPTER_TO_FOOT) = (RH_FOOT_u - RH_FOOT_U.dot(RH_FOOT_a)) / RH_FOOT_D;
    RH_FOOT_a(AZ) += qdd(RH_ADAPTER_TO_FOOT);
    
    
}
