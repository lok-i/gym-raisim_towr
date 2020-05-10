#ifndef IIT_ROBOT_ANYMAL_DECLARATIONS_H_
#define IIT_ROBOT_ANYMAL_DECLARATIONS_H_

#include "rbd_types.h"

namespace iit {
namespace anymal {

static constexpr int JointSpaceDimension = 21;
static constexpr int jointsCount = 21;
/** The total number of rigid bodies of this robot, including the base */
static constexpr int linksCount  = 22;

typedef Matrix<21, 1> Column21d;
typedef Column21d JointState;

enum JointIdentifiers {
    BASE_TO_BASE_INERTIA = 0
    , LF_HAA
    , LF_HFE
    , LF_KFE
    , LF_SHANK_TO_ADAPTER
    , LF_ADAPTER_TO_FOOT
    , RF_HAA
    , RF_HFE
    , RF_KFE
    , RF_SHANK_TO_ADAPTER
    , RF_ADAPTER_TO_FOOT
    , LH_HAA
    , LH_HFE
    , LH_KFE
    , LH_SHANK_TO_ADAPTER
    , LH_ADAPTER_TO_FOOT
    , RH_HAA
    , RH_HFE
    , RH_KFE
    , RH_SHANK_TO_ADAPTER
    , RH_ADAPTER_TO_FOOT
};

enum LinkIdentifiers {
    BASE = 0
    , BASE_INERTIA
    , LF_HIP
    , LF_THIGH
    , LF_SHANK
    , LF_ADAPTER
    , LF_FOOT
    , RF_HIP
    , RF_THIGH
    , RF_SHANK
    , RF_ADAPTER
    , RF_FOOT
    , LH_HIP
    , LH_THIGH
    , LH_SHANK
    , LH_ADAPTER
    , LH_FOOT
    , RH_HIP
    , RH_THIGH
    , RH_SHANK
    , RH_ADAPTER
    , RH_FOOT
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {BASE_TO_BASE_INERTIA,LF_HAA,LF_HFE,LF_KFE,LF_SHANK_TO_ADAPTER,LF_ADAPTER_TO_FOOT,RF_HAA,RF_HFE,RF_KFE,RF_SHANK_TO_ADAPTER,RF_ADAPTER_TO_FOOT,LH_HAA,LH_HFE,LH_KFE,LH_SHANK_TO_ADAPTER,LH_ADAPTER_TO_FOOT,RH_HAA,RH_HFE,RH_KFE,RH_SHANK_TO_ADAPTER,RH_ADAPTER_TO_FOOT};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {BASE,BASE_INERTIA,LF_HIP,LF_THIGH,LF_SHANK,LF_ADAPTER,LF_FOOT,RF_HIP,RF_THIGH,RF_SHANK,RF_ADAPTER,RF_FOOT,LH_HIP,LH_THIGH,LH_SHANK,LH_ADAPTER,LH_FOOT,RH_HIP,RH_THIGH,RH_SHANK,RH_ADAPTER,RH_FOOT};

}
}
#endif
