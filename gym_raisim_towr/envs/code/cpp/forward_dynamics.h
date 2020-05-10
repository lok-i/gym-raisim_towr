#ifndef IIT_ROBOT_ANYMAL_FORWARD_DYNAMICS_H_
#define IIT_ROBOT_ANYMAL_FORWARD_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace iit {
namespace anymal {
namespace dyn {

/**
 * The Forward Dynamics routine for the robot anymal.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */
class ForwardDynamics {
public:
    typedef LinkDataMap<Force> ExtForces;

    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot anymal, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(InertiaProperties& in, MotionTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
        JointState& qdd, // output parameter
        const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, // output parameter
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* motionTransforms;

    Matrix66 vcross; // support variable
    Matrix66 Ia_r;   // support variable, articulated inertia in the case of a revolute joint

    // Link 'base_inertia' :
    Matrix66 base_inertia_AI;
    Velocity base_inertia_a;
    Velocity base_inertia_v;
    Velocity base_inertia_c;
    Force    base_inertia_p;

    Column6 base_inertia_U;
    Scalar base_inertia_D;
    Scalar base_inertia_u;
    // Link 'LF_HIP' :
    Matrix66 LF_HIP_AI;
    Velocity LF_HIP_a;
    Velocity LF_HIP_v;
    Velocity LF_HIP_c;
    Force    LF_HIP_p;

    Column6 LF_HIP_U;
    Scalar LF_HIP_D;
    Scalar LF_HIP_u;
    // Link 'LF_THIGH' :
    Matrix66 LF_THIGH_AI;
    Velocity LF_THIGH_a;
    Velocity LF_THIGH_v;
    Velocity LF_THIGH_c;
    Force    LF_THIGH_p;

    Column6 LF_THIGH_U;
    Scalar LF_THIGH_D;
    Scalar LF_THIGH_u;
    // Link 'LF_SHANK' :
    Matrix66 LF_SHANK_AI;
    Velocity LF_SHANK_a;
    Velocity LF_SHANK_v;
    Velocity LF_SHANK_c;
    Force    LF_SHANK_p;

    Column6 LF_SHANK_U;
    Scalar LF_SHANK_D;
    Scalar LF_SHANK_u;
    // Link 'LF_ADAPTER' :
    Matrix66 LF_ADAPTER_AI;
    Velocity LF_ADAPTER_a;
    Velocity LF_ADAPTER_v;
    Velocity LF_ADAPTER_c;
    Force    LF_ADAPTER_p;

    Column6 LF_ADAPTER_U;
    Scalar LF_ADAPTER_D;
    Scalar LF_ADAPTER_u;
    // Link 'LF_FOOT' :
    Matrix66 LF_FOOT_AI;
    Velocity LF_FOOT_a;
    Velocity LF_FOOT_v;
    Velocity LF_FOOT_c;
    Force    LF_FOOT_p;

    Column6 LF_FOOT_U;
    Scalar LF_FOOT_D;
    Scalar LF_FOOT_u;
    // Link 'RF_HIP' :
    Matrix66 RF_HIP_AI;
    Velocity RF_HIP_a;
    Velocity RF_HIP_v;
    Velocity RF_HIP_c;
    Force    RF_HIP_p;

    Column6 RF_HIP_U;
    Scalar RF_HIP_D;
    Scalar RF_HIP_u;
    // Link 'RF_THIGH' :
    Matrix66 RF_THIGH_AI;
    Velocity RF_THIGH_a;
    Velocity RF_THIGH_v;
    Velocity RF_THIGH_c;
    Force    RF_THIGH_p;

    Column6 RF_THIGH_U;
    Scalar RF_THIGH_D;
    Scalar RF_THIGH_u;
    // Link 'RF_SHANK' :
    Matrix66 RF_SHANK_AI;
    Velocity RF_SHANK_a;
    Velocity RF_SHANK_v;
    Velocity RF_SHANK_c;
    Force    RF_SHANK_p;

    Column6 RF_SHANK_U;
    Scalar RF_SHANK_D;
    Scalar RF_SHANK_u;
    // Link 'RF_ADAPTER' :
    Matrix66 RF_ADAPTER_AI;
    Velocity RF_ADAPTER_a;
    Velocity RF_ADAPTER_v;
    Velocity RF_ADAPTER_c;
    Force    RF_ADAPTER_p;

    Column6 RF_ADAPTER_U;
    Scalar RF_ADAPTER_D;
    Scalar RF_ADAPTER_u;
    // Link 'RF_FOOT' :
    Matrix66 RF_FOOT_AI;
    Velocity RF_FOOT_a;
    Velocity RF_FOOT_v;
    Velocity RF_FOOT_c;
    Force    RF_FOOT_p;

    Column6 RF_FOOT_U;
    Scalar RF_FOOT_D;
    Scalar RF_FOOT_u;
    // Link 'LH_HIP' :
    Matrix66 LH_HIP_AI;
    Velocity LH_HIP_a;
    Velocity LH_HIP_v;
    Velocity LH_HIP_c;
    Force    LH_HIP_p;

    Column6 LH_HIP_U;
    Scalar LH_HIP_D;
    Scalar LH_HIP_u;
    // Link 'LH_THIGH' :
    Matrix66 LH_THIGH_AI;
    Velocity LH_THIGH_a;
    Velocity LH_THIGH_v;
    Velocity LH_THIGH_c;
    Force    LH_THIGH_p;

    Column6 LH_THIGH_U;
    Scalar LH_THIGH_D;
    Scalar LH_THIGH_u;
    // Link 'LH_SHANK' :
    Matrix66 LH_SHANK_AI;
    Velocity LH_SHANK_a;
    Velocity LH_SHANK_v;
    Velocity LH_SHANK_c;
    Force    LH_SHANK_p;

    Column6 LH_SHANK_U;
    Scalar LH_SHANK_D;
    Scalar LH_SHANK_u;
    // Link 'LH_ADAPTER' :
    Matrix66 LH_ADAPTER_AI;
    Velocity LH_ADAPTER_a;
    Velocity LH_ADAPTER_v;
    Velocity LH_ADAPTER_c;
    Force    LH_ADAPTER_p;

    Column6 LH_ADAPTER_U;
    Scalar LH_ADAPTER_D;
    Scalar LH_ADAPTER_u;
    // Link 'LH_FOOT' :
    Matrix66 LH_FOOT_AI;
    Velocity LH_FOOT_a;
    Velocity LH_FOOT_v;
    Velocity LH_FOOT_c;
    Force    LH_FOOT_p;

    Column6 LH_FOOT_U;
    Scalar LH_FOOT_D;
    Scalar LH_FOOT_u;
    // Link 'RH_HIP' :
    Matrix66 RH_HIP_AI;
    Velocity RH_HIP_a;
    Velocity RH_HIP_v;
    Velocity RH_HIP_c;
    Force    RH_HIP_p;

    Column6 RH_HIP_U;
    Scalar RH_HIP_D;
    Scalar RH_HIP_u;
    // Link 'RH_THIGH' :
    Matrix66 RH_THIGH_AI;
    Velocity RH_THIGH_a;
    Velocity RH_THIGH_v;
    Velocity RH_THIGH_c;
    Force    RH_THIGH_p;

    Column6 RH_THIGH_U;
    Scalar RH_THIGH_D;
    Scalar RH_THIGH_u;
    // Link 'RH_SHANK' :
    Matrix66 RH_SHANK_AI;
    Velocity RH_SHANK_a;
    Velocity RH_SHANK_v;
    Velocity RH_SHANK_c;
    Force    RH_SHANK_p;

    Column6 RH_SHANK_U;
    Scalar RH_SHANK_D;
    Scalar RH_SHANK_u;
    // Link 'RH_ADAPTER' :
    Matrix66 RH_ADAPTER_AI;
    Velocity RH_ADAPTER_a;
    Velocity RH_ADAPTER_v;
    Velocity RH_ADAPTER_c;
    Force    RH_ADAPTER_p;

    Column6 RH_ADAPTER_U;
    Scalar RH_ADAPTER_D;
    Scalar RH_ADAPTER_u;
    // Link 'RH_FOOT' :
    Matrix66 RH_FOOT_AI;
    Velocity RH_FOOT_a;
    Velocity RH_FOOT_v;
    Velocity RH_FOOT_c;
    Force    RH_FOOT_p;

    Column6 RH_FOOT_U;
    Scalar RH_FOOT_D;
    Scalar RH_FOOT_u;
private:
    static const ExtForces zeroExtForces;
};

inline void ForwardDynamics::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_base_inertia_X_fr_base)(q);
    (motionTransforms-> fr_LF_HIP_X_fr_base)(q);
    (motionTransforms-> fr_LF_THIGH_X_fr_LF_HIP)(q);
    (motionTransforms-> fr_LF_SHANK_X_fr_LF_THIGH)(q);
    (motionTransforms-> fr_LF_ADAPTER_X_fr_LF_SHANK)(q);
    (motionTransforms-> fr_LF_FOOT_X_fr_LF_ADAPTER)(q);
    (motionTransforms-> fr_RF_HIP_X_fr_base)(q);
    (motionTransforms-> fr_RF_THIGH_X_fr_RF_HIP)(q);
    (motionTransforms-> fr_RF_SHANK_X_fr_RF_THIGH)(q);
    (motionTransforms-> fr_RF_ADAPTER_X_fr_RF_SHANK)(q);
    (motionTransforms-> fr_RF_FOOT_X_fr_RF_ADAPTER)(q);
    (motionTransforms-> fr_LH_HIP_X_fr_base)(q);
    (motionTransforms-> fr_LH_THIGH_X_fr_LH_HIP)(q);
    (motionTransforms-> fr_LH_SHANK_X_fr_LH_THIGH)(q);
    (motionTransforms-> fr_LH_ADAPTER_X_fr_LH_SHANK)(q);
    (motionTransforms-> fr_LH_FOOT_X_fr_LH_ADAPTER)(q);
    (motionTransforms-> fr_RH_HIP_X_fr_base)(q);
    (motionTransforms-> fr_RH_THIGH_X_fr_RH_HIP)(q);
    (motionTransforms-> fr_RH_SHANK_X_fr_RH_THIGH)(q);
    (motionTransforms-> fr_RH_ADAPTER_X_fr_RH_SHANK)(q);
    (motionTransforms-> fr_RH_FOOT_X_fr_RH_ADAPTER)(q);
}

inline void ForwardDynamics::fd(
    JointState& qdd,
    const JointState& q,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    setJointStatus(q);
    fd(qdd, qd, tau, fext);
}

}
}
}

#endif
