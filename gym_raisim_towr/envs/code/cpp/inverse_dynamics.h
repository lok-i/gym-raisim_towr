#ifndef IIT_ANYMAL_INVERSE_DYNAMICS_H_
#define IIT_ANYMAL_INVERSE_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "inertia_properties.h"
#include "transforms.h"
#include "link_data_map.h"

namespace iit {
namespace anymal {
namespace dyn {

/**
 * The Inverse Dynamics routine for the robot anymal.
 *
 * In addition to the full Newton-Euler algorithm, specialized versions to compute
 * only certain terms are provided.
 * The parameters common to most of the methods are the joint status vector \c q, the
 * joint velocity vector \c qd and the acceleration vector \c qdd.
 *
 * Additional overloaded methods are provided without the \c q parameter. These
 * methods use the current configuration of the robot; they are provided for the
 * sake of efficiency, in case the motion transforms of the robot have already
 * been updated elsewhere with the most recent configuration (eg by a call to
 * setJointStatus()), so that it is useless to compute them again.
 *
 * Whenever present, the external forces parameter is a set of external
 * wrenches acting on the robot links. Each wrench must be expressed in
 * the reference frame of the link it is excerted on.
 */
class InverseDynamics {
public:
    typedef LinkDataMap<Force> ExtForces;

    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot anymal, which will be used by this instance
     *     to compute inverse-dynamics.
     */
    InverseDynamics(InertiaProperties& in, MotionTransforms& tr);

    /** \name Inverse dynamics
     * The full Newton-Euler algorithm for the inverse dynamics of this robot.
     *
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */
    ///@{
    void id(
        JointState& jForces,
        const JointState& q, const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    void id(
        JointState& jForces,
        const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    ///@}

    /** \name Gravity terms
     * The joint forces (linear or rotational) required to compensate
     * for the effect of gravity, in a specific configuration.
     */
    ///@{
    void G_terms(JointState& jForces, const JointState& q);
    void G_terms(JointState& jForces);
    ///@}

    /** \name Centrifugal and Coriolis terms
     * The forces (linear or rotational) acting on the joints due to centrifugal and
     * Coriolis effects, for a specific configuration.
     */
    ///@{
    void C_terms(JointState& jForces, const JointState& q, const JointState& qd);
    void C_terms(JointState& jForces, const JointState& qd);
    ///@}
    /** Updates all the kinematics transforms used by the inverse dynamics routine. */
    void setJointStatus(const JointState& q) const;

public:
    /** \name Getters
     * These functions return various spatial quantities used internally
     * by the inverse dynamics routines, like the spatial acceleration
     * of the links.
     *
     * The getters can be useful to retrieve the additional data that is not
     * returned explicitly by the inverse dynamics routines even though it
     * is computed. For example, after a call to the inverse dynamics,
     * the spatial velocity of all the links has been determined and
     * can be accessed.
     *
     * However, beware that certain routines might not use some of the
     * spatial quantities, which therefore would retain their last value
     * without being updated nor reset (for example, the spatial velocity
     * of the links is unaffected by the computation of the gravity terms).
     */
    ///@{
    const Velocity& getVelocity_base_inertia() const { return base_inertia_v; }
    const Acceleration& getAcceleration_base_inertia() const { return base_inertia_a; }
    const Force& getForce_base_inertia() const { return base_inertia_f; }
    const Velocity& getVelocity_LF_HIP() const { return LF_HIP_v; }
    const Acceleration& getAcceleration_LF_HIP() const { return LF_HIP_a; }
    const Force& getForce_LF_HIP() const { return LF_HIP_f; }
    const Velocity& getVelocity_LF_THIGH() const { return LF_THIGH_v; }
    const Acceleration& getAcceleration_LF_THIGH() const { return LF_THIGH_a; }
    const Force& getForce_LF_THIGH() const { return LF_THIGH_f; }
    const Velocity& getVelocity_LF_SHANK() const { return LF_SHANK_v; }
    const Acceleration& getAcceleration_LF_SHANK() const { return LF_SHANK_a; }
    const Force& getForce_LF_SHANK() const { return LF_SHANK_f; }
    const Velocity& getVelocity_LF_ADAPTER() const { return LF_ADAPTER_v; }
    const Acceleration& getAcceleration_LF_ADAPTER() const { return LF_ADAPTER_a; }
    const Force& getForce_LF_ADAPTER() const { return LF_ADAPTER_f; }
    const Velocity& getVelocity_LF_FOOT() const { return LF_FOOT_v; }
    const Acceleration& getAcceleration_LF_FOOT() const { return LF_FOOT_a; }
    const Force& getForce_LF_FOOT() const { return LF_FOOT_f; }
    const Velocity& getVelocity_RF_HIP() const { return RF_HIP_v; }
    const Acceleration& getAcceleration_RF_HIP() const { return RF_HIP_a; }
    const Force& getForce_RF_HIP() const { return RF_HIP_f; }
    const Velocity& getVelocity_RF_THIGH() const { return RF_THIGH_v; }
    const Acceleration& getAcceleration_RF_THIGH() const { return RF_THIGH_a; }
    const Force& getForce_RF_THIGH() const { return RF_THIGH_f; }
    const Velocity& getVelocity_RF_SHANK() const { return RF_SHANK_v; }
    const Acceleration& getAcceleration_RF_SHANK() const { return RF_SHANK_a; }
    const Force& getForce_RF_SHANK() const { return RF_SHANK_f; }
    const Velocity& getVelocity_RF_ADAPTER() const { return RF_ADAPTER_v; }
    const Acceleration& getAcceleration_RF_ADAPTER() const { return RF_ADAPTER_a; }
    const Force& getForce_RF_ADAPTER() const { return RF_ADAPTER_f; }
    const Velocity& getVelocity_RF_FOOT() const { return RF_FOOT_v; }
    const Acceleration& getAcceleration_RF_FOOT() const { return RF_FOOT_a; }
    const Force& getForce_RF_FOOT() const { return RF_FOOT_f; }
    const Velocity& getVelocity_LH_HIP() const { return LH_HIP_v; }
    const Acceleration& getAcceleration_LH_HIP() const { return LH_HIP_a; }
    const Force& getForce_LH_HIP() const { return LH_HIP_f; }
    const Velocity& getVelocity_LH_THIGH() const { return LH_THIGH_v; }
    const Acceleration& getAcceleration_LH_THIGH() const { return LH_THIGH_a; }
    const Force& getForce_LH_THIGH() const { return LH_THIGH_f; }
    const Velocity& getVelocity_LH_SHANK() const { return LH_SHANK_v; }
    const Acceleration& getAcceleration_LH_SHANK() const { return LH_SHANK_a; }
    const Force& getForce_LH_SHANK() const { return LH_SHANK_f; }
    const Velocity& getVelocity_LH_ADAPTER() const { return LH_ADAPTER_v; }
    const Acceleration& getAcceleration_LH_ADAPTER() const { return LH_ADAPTER_a; }
    const Force& getForce_LH_ADAPTER() const { return LH_ADAPTER_f; }
    const Velocity& getVelocity_LH_FOOT() const { return LH_FOOT_v; }
    const Acceleration& getAcceleration_LH_FOOT() const { return LH_FOOT_a; }
    const Force& getForce_LH_FOOT() const { return LH_FOOT_f; }
    const Velocity& getVelocity_RH_HIP() const { return RH_HIP_v; }
    const Acceleration& getAcceleration_RH_HIP() const { return RH_HIP_a; }
    const Force& getForce_RH_HIP() const { return RH_HIP_f; }
    const Velocity& getVelocity_RH_THIGH() const { return RH_THIGH_v; }
    const Acceleration& getAcceleration_RH_THIGH() const { return RH_THIGH_a; }
    const Force& getForce_RH_THIGH() const { return RH_THIGH_f; }
    const Velocity& getVelocity_RH_SHANK() const { return RH_SHANK_v; }
    const Acceleration& getAcceleration_RH_SHANK() const { return RH_SHANK_a; }
    const Force& getForce_RH_SHANK() const { return RH_SHANK_f; }
    const Velocity& getVelocity_RH_ADAPTER() const { return RH_ADAPTER_v; }
    const Acceleration& getAcceleration_RH_ADAPTER() const { return RH_ADAPTER_a; }
    const Force& getForce_RH_ADAPTER() const { return RH_ADAPTER_f; }
    const Velocity& getVelocity_RH_FOOT() const { return RH_FOOT_v; }
    const Acceleration& getAcceleration_RH_FOOT() const { return RH_FOOT_a; }
    const Force& getForce_RH_FOOT() const { return RH_FOOT_f; }
    ///@}
protected:
    void firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext);
    void secondPass(JointState& jForces);

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* xm;
private:
    Matrix66 vcross; // support variable
    // Link 'base_inertia' :
    const InertiaMatrix& base_inertia_I;
    Velocity      base_inertia_v;
    Acceleration  base_inertia_a;
    Force         base_inertia_f;
    // Link 'LF_HIP' :
    const InertiaMatrix& LF_HIP_I;
    Velocity      LF_HIP_v;
    Acceleration  LF_HIP_a;
    Force         LF_HIP_f;
    // Link 'LF_THIGH' :
    const InertiaMatrix& LF_THIGH_I;
    Velocity      LF_THIGH_v;
    Acceleration  LF_THIGH_a;
    Force         LF_THIGH_f;
    // Link 'LF_SHANK' :
    const InertiaMatrix& LF_SHANK_I;
    Velocity      LF_SHANK_v;
    Acceleration  LF_SHANK_a;
    Force         LF_SHANK_f;
    // Link 'LF_ADAPTER' :
    const InertiaMatrix& LF_ADAPTER_I;
    Velocity      LF_ADAPTER_v;
    Acceleration  LF_ADAPTER_a;
    Force         LF_ADAPTER_f;
    // Link 'LF_FOOT' :
    const InertiaMatrix& LF_FOOT_I;
    Velocity      LF_FOOT_v;
    Acceleration  LF_FOOT_a;
    Force         LF_FOOT_f;
    // Link 'RF_HIP' :
    const InertiaMatrix& RF_HIP_I;
    Velocity      RF_HIP_v;
    Acceleration  RF_HIP_a;
    Force         RF_HIP_f;
    // Link 'RF_THIGH' :
    const InertiaMatrix& RF_THIGH_I;
    Velocity      RF_THIGH_v;
    Acceleration  RF_THIGH_a;
    Force         RF_THIGH_f;
    // Link 'RF_SHANK' :
    const InertiaMatrix& RF_SHANK_I;
    Velocity      RF_SHANK_v;
    Acceleration  RF_SHANK_a;
    Force         RF_SHANK_f;
    // Link 'RF_ADAPTER' :
    const InertiaMatrix& RF_ADAPTER_I;
    Velocity      RF_ADAPTER_v;
    Acceleration  RF_ADAPTER_a;
    Force         RF_ADAPTER_f;
    // Link 'RF_FOOT' :
    const InertiaMatrix& RF_FOOT_I;
    Velocity      RF_FOOT_v;
    Acceleration  RF_FOOT_a;
    Force         RF_FOOT_f;
    // Link 'LH_HIP' :
    const InertiaMatrix& LH_HIP_I;
    Velocity      LH_HIP_v;
    Acceleration  LH_HIP_a;
    Force         LH_HIP_f;
    // Link 'LH_THIGH' :
    const InertiaMatrix& LH_THIGH_I;
    Velocity      LH_THIGH_v;
    Acceleration  LH_THIGH_a;
    Force         LH_THIGH_f;
    // Link 'LH_SHANK' :
    const InertiaMatrix& LH_SHANK_I;
    Velocity      LH_SHANK_v;
    Acceleration  LH_SHANK_a;
    Force         LH_SHANK_f;
    // Link 'LH_ADAPTER' :
    const InertiaMatrix& LH_ADAPTER_I;
    Velocity      LH_ADAPTER_v;
    Acceleration  LH_ADAPTER_a;
    Force         LH_ADAPTER_f;
    // Link 'LH_FOOT' :
    const InertiaMatrix& LH_FOOT_I;
    Velocity      LH_FOOT_v;
    Acceleration  LH_FOOT_a;
    Force         LH_FOOT_f;
    // Link 'RH_HIP' :
    const InertiaMatrix& RH_HIP_I;
    Velocity      RH_HIP_v;
    Acceleration  RH_HIP_a;
    Force         RH_HIP_f;
    // Link 'RH_THIGH' :
    const InertiaMatrix& RH_THIGH_I;
    Velocity      RH_THIGH_v;
    Acceleration  RH_THIGH_a;
    Force         RH_THIGH_f;
    // Link 'RH_SHANK' :
    const InertiaMatrix& RH_SHANK_I;
    Velocity      RH_SHANK_v;
    Acceleration  RH_SHANK_a;
    Force         RH_SHANK_f;
    // Link 'RH_ADAPTER' :
    const InertiaMatrix& RH_ADAPTER_I;
    Velocity      RH_ADAPTER_v;
    Acceleration  RH_ADAPTER_a;
    Force         RH_ADAPTER_f;
    // Link 'RH_FOOT' :
    const InertiaMatrix& RH_FOOT_I;
    Velocity      RH_FOOT_v;
    Acceleration  RH_FOOT_a;
    Force         RH_FOOT_f;


private:
    static const ExtForces zeroExtForces;
};

inline void InverseDynamics::setJointStatus(const JointState& q) const
{
    (xm->fr_base_inertia_X_fr_base)(q);
    (xm->fr_LF_HIP_X_fr_base)(q);
    (xm->fr_LF_THIGH_X_fr_LF_HIP)(q);
    (xm->fr_LF_SHANK_X_fr_LF_THIGH)(q);
    (xm->fr_LF_ADAPTER_X_fr_LF_SHANK)(q);
    (xm->fr_LF_FOOT_X_fr_LF_ADAPTER)(q);
    (xm->fr_RF_HIP_X_fr_base)(q);
    (xm->fr_RF_THIGH_X_fr_RF_HIP)(q);
    (xm->fr_RF_SHANK_X_fr_RF_THIGH)(q);
    (xm->fr_RF_ADAPTER_X_fr_RF_SHANK)(q);
    (xm->fr_RF_FOOT_X_fr_RF_ADAPTER)(q);
    (xm->fr_LH_HIP_X_fr_base)(q);
    (xm->fr_LH_THIGH_X_fr_LH_HIP)(q);
    (xm->fr_LH_SHANK_X_fr_LH_THIGH)(q);
    (xm->fr_LH_ADAPTER_X_fr_LH_SHANK)(q);
    (xm->fr_LH_FOOT_X_fr_LH_ADAPTER)(q);
    (xm->fr_RH_HIP_X_fr_base)(q);
    (xm->fr_RH_THIGH_X_fr_RH_HIP)(q);
    (xm->fr_RH_SHANK_X_fr_RH_THIGH)(q);
    (xm->fr_RH_ADAPTER_X_fr_RH_SHANK)(q);
    (xm->fr_RH_FOOT_X_fr_RH_ADAPTER)(q);
}

inline void InverseDynamics::G_terms(JointState& jForces, const JointState& q)
{
    setJointStatus(q);
    G_terms(jForces);
}

inline void InverseDynamics::C_terms(JointState& jForces, const JointState& q, const JointState& qd)
{
    setJointStatus(q);
    C_terms(jForces, qd);
}

inline void InverseDynamics::id(
    JointState& jForces,
    const JointState& q, const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    setJointStatus(q);
    id(jForces, qd, qdd, fext);
}

}
}
}

#endif
