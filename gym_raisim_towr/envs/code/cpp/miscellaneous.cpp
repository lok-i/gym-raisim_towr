#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace iit::anymal;
using namespace iit::anymal::dyn;

Vector3 iit::anymal::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    Vector3 tmpSum(Vector3::Zero());


    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    HomogeneousTransforms::MatrixType base_X_base_to_base_inertia_chain;
    HomogeneousTransforms::MatrixType base_X_LF_HAA_chain;
    HomogeneousTransforms::MatrixType base_X_RF_HAA_chain;
    HomogeneousTransforms::MatrixType base_X_LH_HAA_chain;
    HomogeneousTransforms::MatrixType base_X_RH_HAA_chain;
    
    
    base_X_base_to_base_inertia_chain = tmpX * ht.fr_base_X_fr_base_inertia;
    tmpSum += inertiaProps.getMass_base_inertia() *
            ( iit::rbd::Utils::transform(base_X_base_to_base_inertia_chain, inertiaProps.getCOM_base_inertia()));
    
    base_X_LF_HAA_chain = tmpX * ht.fr_base_X_fr_LF_HIP;
    tmpSum += inertiaProps.getMass_LF_HIP() *
            ( iit::rbd::Utils::transform(base_X_LF_HAA_chain, inertiaProps.getCOM_LF_HIP()));
    
    base_X_LF_HAA_chain = base_X_LF_HAA_chain * ht.fr_LF_HIP_X_fr_LF_THIGH;
    tmpSum += inertiaProps.getMass_LF_THIGH() *
            ( iit::rbd::Utils::transform(base_X_LF_HAA_chain, inertiaProps.getCOM_LF_THIGH()));
    
    base_X_LF_HAA_chain = base_X_LF_HAA_chain * ht.fr_LF_THIGH_X_fr_LF_SHANK;
    tmpSum += inertiaProps.getMass_LF_SHANK() *
            ( iit::rbd::Utils::transform(base_X_LF_HAA_chain, inertiaProps.getCOM_LF_SHANK()));
    
    base_X_LF_HAA_chain = base_X_LF_HAA_chain * ht.fr_LF_SHANK_X_fr_LF_ADAPTER;
    tmpSum += inertiaProps.getMass_LF_ADAPTER() *
            ( iit::rbd::Utils::transform(base_X_LF_HAA_chain, inertiaProps.getCOM_LF_ADAPTER()));
    
    base_X_LF_HAA_chain = base_X_LF_HAA_chain * ht.fr_LF_ADAPTER_X_fr_LF_FOOT;
    tmpSum += inertiaProps.getMass_LF_FOOT() *
            ( iit::rbd::Utils::transform(base_X_LF_HAA_chain, inertiaProps.getCOM_LF_FOOT()));
    
    base_X_RF_HAA_chain = tmpX * ht.fr_base_X_fr_RF_HIP;
    tmpSum += inertiaProps.getMass_RF_HIP() *
            ( iit::rbd::Utils::transform(base_X_RF_HAA_chain, inertiaProps.getCOM_RF_HIP()));
    
    base_X_RF_HAA_chain = base_X_RF_HAA_chain * ht.fr_RF_HIP_X_fr_RF_THIGH;
    tmpSum += inertiaProps.getMass_RF_THIGH() *
            ( iit::rbd::Utils::transform(base_X_RF_HAA_chain, inertiaProps.getCOM_RF_THIGH()));
    
    base_X_RF_HAA_chain = base_X_RF_HAA_chain * ht.fr_RF_THIGH_X_fr_RF_SHANK;
    tmpSum += inertiaProps.getMass_RF_SHANK() *
            ( iit::rbd::Utils::transform(base_X_RF_HAA_chain, inertiaProps.getCOM_RF_SHANK()));
    
    base_X_RF_HAA_chain = base_X_RF_HAA_chain * ht.fr_RF_SHANK_X_fr_RF_ADAPTER;
    tmpSum += inertiaProps.getMass_RF_ADAPTER() *
            ( iit::rbd::Utils::transform(base_X_RF_HAA_chain, inertiaProps.getCOM_RF_ADAPTER()));
    
    base_X_RF_HAA_chain = base_X_RF_HAA_chain * ht.fr_RF_ADAPTER_X_fr_RF_FOOT;
    tmpSum += inertiaProps.getMass_RF_FOOT() *
            ( iit::rbd::Utils::transform(base_X_RF_HAA_chain, inertiaProps.getCOM_RF_FOOT()));
    
    base_X_LH_HAA_chain = tmpX * ht.fr_base_X_fr_LH_HIP;
    tmpSum += inertiaProps.getMass_LH_HIP() *
            ( iit::rbd::Utils::transform(base_X_LH_HAA_chain, inertiaProps.getCOM_LH_HIP()));
    
    base_X_LH_HAA_chain = base_X_LH_HAA_chain * ht.fr_LH_HIP_X_fr_LH_THIGH;
    tmpSum += inertiaProps.getMass_LH_THIGH() *
            ( iit::rbd::Utils::transform(base_X_LH_HAA_chain, inertiaProps.getCOM_LH_THIGH()));
    
    base_X_LH_HAA_chain = base_X_LH_HAA_chain * ht.fr_LH_THIGH_X_fr_LH_SHANK;
    tmpSum += inertiaProps.getMass_LH_SHANK() *
            ( iit::rbd::Utils::transform(base_X_LH_HAA_chain, inertiaProps.getCOM_LH_SHANK()));
    
    base_X_LH_HAA_chain = base_X_LH_HAA_chain * ht.fr_LH_SHANK_X_fr_LH_ADAPTER;
    tmpSum += inertiaProps.getMass_LH_ADAPTER() *
            ( iit::rbd::Utils::transform(base_X_LH_HAA_chain, inertiaProps.getCOM_LH_ADAPTER()));
    
    base_X_LH_HAA_chain = base_X_LH_HAA_chain * ht.fr_LH_ADAPTER_X_fr_LH_FOOT;
    tmpSum += inertiaProps.getMass_LH_FOOT() *
            ( iit::rbd::Utils::transform(base_X_LH_HAA_chain, inertiaProps.getCOM_LH_FOOT()));
    
    base_X_RH_HAA_chain = tmpX * ht.fr_base_X_fr_RH_HIP;
    tmpSum += inertiaProps.getMass_RH_HIP() *
            ( iit::rbd::Utils::transform(base_X_RH_HAA_chain, inertiaProps.getCOM_RH_HIP()));
    
    base_X_RH_HAA_chain = base_X_RH_HAA_chain * ht.fr_RH_HIP_X_fr_RH_THIGH;
    tmpSum += inertiaProps.getMass_RH_THIGH() *
            ( iit::rbd::Utils::transform(base_X_RH_HAA_chain, inertiaProps.getCOM_RH_THIGH()));
    
    base_X_RH_HAA_chain = base_X_RH_HAA_chain * ht.fr_RH_THIGH_X_fr_RH_SHANK;
    tmpSum += inertiaProps.getMass_RH_SHANK() *
            ( iit::rbd::Utils::transform(base_X_RH_HAA_chain, inertiaProps.getCOM_RH_SHANK()));
    
    base_X_RH_HAA_chain = base_X_RH_HAA_chain * ht.fr_RH_SHANK_X_fr_RH_ADAPTER;
    tmpSum += inertiaProps.getMass_RH_ADAPTER() *
            ( iit::rbd::Utils::transform(base_X_RH_HAA_chain, inertiaProps.getCOM_RH_ADAPTER()));
    
    base_X_RH_HAA_chain = base_X_RH_HAA_chain * ht.fr_RH_ADAPTER_X_fr_RH_FOOT;
    tmpSum += inertiaProps.getMass_RH_FOOT() *
            ( iit::rbd::Utils::transform(base_X_RH_HAA_chain, inertiaProps.getCOM_RH_FOOT()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

Vector3 iit::anymal::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_base_X_fr_base_inertia(q);
    ht.fr_base_X_fr_LF_HIP(q);
    ht.fr_base_X_fr_RF_HIP(q);
    ht.fr_base_X_fr_LH_HIP(q);
    ht.fr_base_X_fr_RH_HIP(q);
    ht.fr_LF_HIP_X_fr_LF_THIGH(q);
    ht.fr_LF_THIGH_X_fr_LF_SHANK(q);
    ht.fr_LF_SHANK_X_fr_LF_ADAPTER(q);
    ht.fr_LF_ADAPTER_X_fr_LF_FOOT(q);
    ht.fr_RF_HIP_X_fr_RF_THIGH(q);
    ht.fr_RF_THIGH_X_fr_RF_SHANK(q);
    ht.fr_RF_SHANK_X_fr_RF_ADAPTER(q);
    ht.fr_RF_ADAPTER_X_fr_RF_FOOT(q);
    ht.fr_LH_HIP_X_fr_LH_THIGH(q);
    ht.fr_LH_THIGH_X_fr_LH_SHANK(q);
    ht.fr_LH_SHANK_X_fr_LH_ADAPTER(q);
    ht.fr_LH_ADAPTER_X_fr_LH_FOOT(q);
    ht.fr_RH_HIP_X_fr_RH_THIGH(q);
    ht.fr_RH_THIGH_X_fr_RH_SHANK(q);
    ht.fr_RH_SHANK_X_fr_RH_ADAPTER(q);
    ht.fr_RH_ADAPTER_X_fr_RH_FOOT(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
