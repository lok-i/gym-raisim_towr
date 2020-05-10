#ifndef IIT_ROBOGEN__ANYMAL_TRAITS_H_
#define IIT_ROBOGEN__ANYMAL_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace iit {
namespace anymal {

struct Traits {
    typedef typename anymal::ScalarTraits ScalarTraits;

    typedef typename anymal::JointState JointState;

    typedef typename anymal::JointIdentifiers JointID;
    typedef typename anymal::LinkIdentifiers  LinkID;

    typedef typename anymal::HomogeneousTransforms HomogeneousTransforms;
    typedef typename anymal::MotionTransforms MotionTransforms;
    typedef typename anymal::ForceTransforms ForceTransforms;

    typedef typename anymal::dyn::InertiaProperties InertiaProperties;
    typedef typename anymal::dyn::ForwardDynamics FwdDynEngine;
    typedef typename anymal::dyn::InverseDynamics InvDynEngine;
    typedef typename anymal::dyn::JSIM JSIM;

    static const int joints_count = anymal::jointsCount;
    static const int links_count  = anymal::linksCount;
    static const bool floating_base = false;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return anymal::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return anymal::orderedLinkIDs;
}

}
}

#endif
