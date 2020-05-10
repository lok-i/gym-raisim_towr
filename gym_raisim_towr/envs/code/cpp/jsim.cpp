#include "transforms.h"
#include "jsim.h"

#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

//Implementation of default constructor
iit::anymal::dyn::JSIM::JSIM(InertiaProperties& inertiaProperties, ForceTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    base_inertia_Ic(linkInertias.getTensor_base_inertia()),
    LF_FOOT_Ic(linkInertias.getTensor_LF_FOOT()),
    RF_FOOT_Ic(linkInertias.getTensor_RF_FOOT()),
    LH_FOOT_Ic(linkInertias.getTensor_LH_FOOT()),
    RH_FOOT_Ic(linkInertias.getTensor_RH_FOOT())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA operator()
const iit::anymal::dyn::JSIM& iit::anymal::dyn::JSIM::update(const JointState& state) {
    Force F;

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_RH_ADAPTER_X_fr_RH_FOOT(state);
    frcTransf -> fr_RH_SHANK_X_fr_RH_ADAPTER(state);
    frcTransf -> fr_RH_THIGH_X_fr_RH_SHANK(state);
    frcTransf -> fr_RH_HIP_X_fr_RH_THIGH(state);
    frcTransf -> fr_LH_ADAPTER_X_fr_LH_FOOT(state);
    frcTransf -> fr_LH_SHANK_X_fr_LH_ADAPTER(state);
    frcTransf -> fr_LH_THIGH_X_fr_LH_SHANK(state);
    frcTransf -> fr_LH_HIP_X_fr_LH_THIGH(state);
    frcTransf -> fr_RF_ADAPTER_X_fr_RF_FOOT(state);
    frcTransf -> fr_RF_SHANK_X_fr_RF_ADAPTER(state);
    frcTransf -> fr_RF_THIGH_X_fr_RF_SHANK(state);
    frcTransf -> fr_RF_HIP_X_fr_RF_THIGH(state);
    frcTransf -> fr_LF_ADAPTER_X_fr_LF_FOOT(state);
    frcTransf -> fr_LF_SHANK_X_fr_LF_ADAPTER(state);
    frcTransf -> fr_LF_THIGH_X_fr_LF_SHANK(state);
    frcTransf -> fr_LF_HIP_X_fr_LF_THIGH(state);

    // Initializes the composite inertia tensors
    LF_HIP_Ic = linkInertias.getTensor_LF_HIP();
    LF_THIGH_Ic = linkInertias.getTensor_LF_THIGH();
    LF_SHANK_Ic = linkInertias.getTensor_LF_SHANK();
    LF_ADAPTER_Ic = linkInertias.getTensor_LF_ADAPTER();
    RF_HIP_Ic = linkInertias.getTensor_RF_HIP();
    RF_THIGH_Ic = linkInertias.getTensor_RF_THIGH();
    RF_SHANK_Ic = linkInertias.getTensor_RF_SHANK();
    RF_ADAPTER_Ic = linkInertias.getTensor_RF_ADAPTER();
    LH_HIP_Ic = linkInertias.getTensor_LH_HIP();
    LH_THIGH_Ic = linkInertias.getTensor_LH_THIGH();
    LH_SHANK_Ic = linkInertias.getTensor_LH_SHANK();
    LH_ADAPTER_Ic = linkInertias.getTensor_LH_ADAPTER();
    RH_HIP_Ic = linkInertias.getTensor_RH_HIP();
    RH_THIGH_Ic = linkInertias.getTensor_RH_THIGH();
    RH_SHANK_Ic = linkInertias.getTensor_RH_SHANK();
    RH_ADAPTER_Ic = linkInertias.getTensor_RH_ADAPTER();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link RH_FOOT:
    iit::rbd::transformInertia<Scalar>(RH_FOOT_Ic, frcTransf -> fr_RH_ADAPTER_X_fr_RH_FOOT, Ic_spare);
    RH_ADAPTER_Ic += Ic_spare;

    F = RH_FOOT_Ic.col(AZ);
    DATA(RH_ADAPTER_TO_FOOT, RH_ADAPTER_TO_FOOT) = F(AZ);

    F = frcTransf -> fr_RH_ADAPTER_X_fr_RH_FOOT * F;
    DATA(RH_ADAPTER_TO_FOOT, RH_SHANK_TO_ADAPTER) = F(AZ);
    DATA(RH_SHANK_TO_ADAPTER, RH_ADAPTER_TO_FOOT) = DATA(RH_ADAPTER_TO_FOOT, RH_SHANK_TO_ADAPTER);
    F = frcTransf -> fr_RH_SHANK_X_fr_RH_ADAPTER * F;
    DATA(RH_ADAPTER_TO_FOOT, RH_KFE) = F(AZ);
    DATA(RH_KFE, RH_ADAPTER_TO_FOOT) = DATA(RH_ADAPTER_TO_FOOT, RH_KFE);
    F = frcTransf -> fr_RH_THIGH_X_fr_RH_SHANK * F;
    DATA(RH_ADAPTER_TO_FOOT, RH_HFE) = F(AZ);
    DATA(RH_HFE, RH_ADAPTER_TO_FOOT) = DATA(RH_ADAPTER_TO_FOOT, RH_HFE);
    F = frcTransf -> fr_RH_HIP_X_fr_RH_THIGH * F;
    DATA(RH_ADAPTER_TO_FOOT, RH_HAA) = F(AZ);
    DATA(RH_HAA, RH_ADAPTER_TO_FOOT) = DATA(RH_ADAPTER_TO_FOOT, RH_HAA);

    // Link RH_ADAPTER:
    iit::rbd::transformInertia<Scalar>(RH_ADAPTER_Ic, frcTransf -> fr_RH_SHANK_X_fr_RH_ADAPTER, Ic_spare);
    RH_SHANK_Ic += Ic_spare;

    F = RH_ADAPTER_Ic.col(AZ);
    DATA(RH_SHANK_TO_ADAPTER, RH_SHANK_TO_ADAPTER) = F(AZ);

    F = frcTransf -> fr_RH_SHANK_X_fr_RH_ADAPTER * F;
    DATA(RH_SHANK_TO_ADAPTER, RH_KFE) = F(AZ);
    DATA(RH_KFE, RH_SHANK_TO_ADAPTER) = DATA(RH_SHANK_TO_ADAPTER, RH_KFE);
    F = frcTransf -> fr_RH_THIGH_X_fr_RH_SHANK * F;
    DATA(RH_SHANK_TO_ADAPTER, RH_HFE) = F(AZ);
    DATA(RH_HFE, RH_SHANK_TO_ADAPTER) = DATA(RH_SHANK_TO_ADAPTER, RH_HFE);
    F = frcTransf -> fr_RH_HIP_X_fr_RH_THIGH * F;
    DATA(RH_SHANK_TO_ADAPTER, RH_HAA) = F(AZ);
    DATA(RH_HAA, RH_SHANK_TO_ADAPTER) = DATA(RH_SHANK_TO_ADAPTER, RH_HAA);

    // Link RH_SHANK:
    iit::rbd::transformInertia<Scalar>(RH_SHANK_Ic, frcTransf -> fr_RH_THIGH_X_fr_RH_SHANK, Ic_spare);
    RH_THIGH_Ic += Ic_spare;

    F = RH_SHANK_Ic.col(AZ);
    DATA(RH_KFE, RH_KFE) = F(AZ);

    F = frcTransf -> fr_RH_THIGH_X_fr_RH_SHANK * F;
    DATA(RH_KFE, RH_HFE) = F(AZ);
    DATA(RH_HFE, RH_KFE) = DATA(RH_KFE, RH_HFE);
    F = frcTransf -> fr_RH_HIP_X_fr_RH_THIGH * F;
    DATA(RH_KFE, RH_HAA) = F(AZ);
    DATA(RH_HAA, RH_KFE) = DATA(RH_KFE, RH_HAA);

    // Link RH_THIGH:
    iit::rbd::transformInertia<Scalar>(RH_THIGH_Ic, frcTransf -> fr_RH_HIP_X_fr_RH_THIGH, Ic_spare);
    RH_HIP_Ic += Ic_spare;

    F = RH_THIGH_Ic.col(AZ);
    DATA(RH_HFE, RH_HFE) = F(AZ);

    F = frcTransf -> fr_RH_HIP_X_fr_RH_THIGH * F;
    DATA(RH_HFE, RH_HAA) = F(AZ);
    DATA(RH_HAA, RH_HFE) = DATA(RH_HFE, RH_HAA);

    // Link RH_HIP:

    F = RH_HIP_Ic.col(AZ);
    DATA(RH_HAA, RH_HAA) = F(AZ);


    // Link LH_FOOT:
    iit::rbd::transformInertia<Scalar>(LH_FOOT_Ic, frcTransf -> fr_LH_ADAPTER_X_fr_LH_FOOT, Ic_spare);
    LH_ADAPTER_Ic += Ic_spare;

    F = LH_FOOT_Ic.col(AZ);
    DATA(LH_ADAPTER_TO_FOOT, LH_ADAPTER_TO_FOOT) = F(AZ);

    F = frcTransf -> fr_LH_ADAPTER_X_fr_LH_FOOT * F;
    DATA(LH_ADAPTER_TO_FOOT, LH_SHANK_TO_ADAPTER) = F(AZ);
    DATA(LH_SHANK_TO_ADAPTER, LH_ADAPTER_TO_FOOT) = DATA(LH_ADAPTER_TO_FOOT, LH_SHANK_TO_ADAPTER);
    F = frcTransf -> fr_LH_SHANK_X_fr_LH_ADAPTER * F;
    DATA(LH_ADAPTER_TO_FOOT, LH_KFE) = F(AZ);
    DATA(LH_KFE, LH_ADAPTER_TO_FOOT) = DATA(LH_ADAPTER_TO_FOOT, LH_KFE);
    F = frcTransf -> fr_LH_THIGH_X_fr_LH_SHANK * F;
    DATA(LH_ADAPTER_TO_FOOT, LH_HFE) = F(AZ);
    DATA(LH_HFE, LH_ADAPTER_TO_FOOT) = DATA(LH_ADAPTER_TO_FOOT, LH_HFE);
    F = frcTransf -> fr_LH_HIP_X_fr_LH_THIGH * F;
    DATA(LH_ADAPTER_TO_FOOT, LH_HAA) = F(AZ);
    DATA(LH_HAA, LH_ADAPTER_TO_FOOT) = DATA(LH_ADAPTER_TO_FOOT, LH_HAA);

    // Link LH_ADAPTER:
    iit::rbd::transformInertia<Scalar>(LH_ADAPTER_Ic, frcTransf -> fr_LH_SHANK_X_fr_LH_ADAPTER, Ic_spare);
    LH_SHANK_Ic += Ic_spare;

    F = LH_ADAPTER_Ic.col(AZ);
    DATA(LH_SHANK_TO_ADAPTER, LH_SHANK_TO_ADAPTER) = F(AZ);

    F = frcTransf -> fr_LH_SHANK_X_fr_LH_ADAPTER * F;
    DATA(LH_SHANK_TO_ADAPTER, LH_KFE) = F(AZ);
    DATA(LH_KFE, LH_SHANK_TO_ADAPTER) = DATA(LH_SHANK_TO_ADAPTER, LH_KFE);
    F = frcTransf -> fr_LH_THIGH_X_fr_LH_SHANK * F;
    DATA(LH_SHANK_TO_ADAPTER, LH_HFE) = F(AZ);
    DATA(LH_HFE, LH_SHANK_TO_ADAPTER) = DATA(LH_SHANK_TO_ADAPTER, LH_HFE);
    F = frcTransf -> fr_LH_HIP_X_fr_LH_THIGH * F;
    DATA(LH_SHANK_TO_ADAPTER, LH_HAA) = F(AZ);
    DATA(LH_HAA, LH_SHANK_TO_ADAPTER) = DATA(LH_SHANK_TO_ADAPTER, LH_HAA);

    // Link LH_SHANK:
    iit::rbd::transformInertia<Scalar>(LH_SHANK_Ic, frcTransf -> fr_LH_THIGH_X_fr_LH_SHANK, Ic_spare);
    LH_THIGH_Ic += Ic_spare;

    F = LH_SHANK_Ic.col(AZ);
    DATA(LH_KFE, LH_KFE) = F(AZ);

    F = frcTransf -> fr_LH_THIGH_X_fr_LH_SHANK * F;
    DATA(LH_KFE, LH_HFE) = F(AZ);
    DATA(LH_HFE, LH_KFE) = DATA(LH_KFE, LH_HFE);
    F = frcTransf -> fr_LH_HIP_X_fr_LH_THIGH * F;
    DATA(LH_KFE, LH_HAA) = F(AZ);
    DATA(LH_HAA, LH_KFE) = DATA(LH_KFE, LH_HAA);

    // Link LH_THIGH:
    iit::rbd::transformInertia<Scalar>(LH_THIGH_Ic, frcTransf -> fr_LH_HIP_X_fr_LH_THIGH, Ic_spare);
    LH_HIP_Ic += Ic_spare;

    F = LH_THIGH_Ic.col(AZ);
    DATA(LH_HFE, LH_HFE) = F(AZ);

    F = frcTransf -> fr_LH_HIP_X_fr_LH_THIGH * F;
    DATA(LH_HFE, LH_HAA) = F(AZ);
    DATA(LH_HAA, LH_HFE) = DATA(LH_HFE, LH_HAA);

    // Link LH_HIP:

    F = LH_HIP_Ic.col(AZ);
    DATA(LH_HAA, LH_HAA) = F(AZ);


    // Link RF_FOOT:
    iit::rbd::transformInertia<Scalar>(RF_FOOT_Ic, frcTransf -> fr_RF_ADAPTER_X_fr_RF_FOOT, Ic_spare);
    RF_ADAPTER_Ic += Ic_spare;

    F = RF_FOOT_Ic.col(AZ);
    DATA(RF_ADAPTER_TO_FOOT, RF_ADAPTER_TO_FOOT) = F(AZ);

    F = frcTransf -> fr_RF_ADAPTER_X_fr_RF_FOOT * F;
    DATA(RF_ADAPTER_TO_FOOT, RF_SHANK_TO_ADAPTER) = F(AZ);
    DATA(RF_SHANK_TO_ADAPTER, RF_ADAPTER_TO_FOOT) = DATA(RF_ADAPTER_TO_FOOT, RF_SHANK_TO_ADAPTER);
    F = frcTransf -> fr_RF_SHANK_X_fr_RF_ADAPTER * F;
    DATA(RF_ADAPTER_TO_FOOT, RF_KFE) = F(AZ);
    DATA(RF_KFE, RF_ADAPTER_TO_FOOT) = DATA(RF_ADAPTER_TO_FOOT, RF_KFE);
    F = frcTransf -> fr_RF_THIGH_X_fr_RF_SHANK * F;
    DATA(RF_ADAPTER_TO_FOOT, RF_HFE) = F(AZ);
    DATA(RF_HFE, RF_ADAPTER_TO_FOOT) = DATA(RF_ADAPTER_TO_FOOT, RF_HFE);
    F = frcTransf -> fr_RF_HIP_X_fr_RF_THIGH * F;
    DATA(RF_ADAPTER_TO_FOOT, RF_HAA) = F(AZ);
    DATA(RF_HAA, RF_ADAPTER_TO_FOOT) = DATA(RF_ADAPTER_TO_FOOT, RF_HAA);

    // Link RF_ADAPTER:
    iit::rbd::transformInertia<Scalar>(RF_ADAPTER_Ic, frcTransf -> fr_RF_SHANK_X_fr_RF_ADAPTER, Ic_spare);
    RF_SHANK_Ic += Ic_spare;

    F = RF_ADAPTER_Ic.col(AZ);
    DATA(RF_SHANK_TO_ADAPTER, RF_SHANK_TO_ADAPTER) = F(AZ);

    F = frcTransf -> fr_RF_SHANK_X_fr_RF_ADAPTER * F;
    DATA(RF_SHANK_TO_ADAPTER, RF_KFE) = F(AZ);
    DATA(RF_KFE, RF_SHANK_TO_ADAPTER) = DATA(RF_SHANK_TO_ADAPTER, RF_KFE);
    F = frcTransf -> fr_RF_THIGH_X_fr_RF_SHANK * F;
    DATA(RF_SHANK_TO_ADAPTER, RF_HFE) = F(AZ);
    DATA(RF_HFE, RF_SHANK_TO_ADAPTER) = DATA(RF_SHANK_TO_ADAPTER, RF_HFE);
    F = frcTransf -> fr_RF_HIP_X_fr_RF_THIGH * F;
    DATA(RF_SHANK_TO_ADAPTER, RF_HAA) = F(AZ);
    DATA(RF_HAA, RF_SHANK_TO_ADAPTER) = DATA(RF_SHANK_TO_ADAPTER, RF_HAA);

    // Link RF_SHANK:
    iit::rbd::transformInertia<Scalar>(RF_SHANK_Ic, frcTransf -> fr_RF_THIGH_X_fr_RF_SHANK, Ic_spare);
    RF_THIGH_Ic += Ic_spare;

    F = RF_SHANK_Ic.col(AZ);
    DATA(RF_KFE, RF_KFE) = F(AZ);

    F = frcTransf -> fr_RF_THIGH_X_fr_RF_SHANK * F;
    DATA(RF_KFE, RF_HFE) = F(AZ);
    DATA(RF_HFE, RF_KFE) = DATA(RF_KFE, RF_HFE);
    F = frcTransf -> fr_RF_HIP_X_fr_RF_THIGH * F;
    DATA(RF_KFE, RF_HAA) = F(AZ);
    DATA(RF_HAA, RF_KFE) = DATA(RF_KFE, RF_HAA);

    // Link RF_THIGH:
    iit::rbd::transformInertia<Scalar>(RF_THIGH_Ic, frcTransf -> fr_RF_HIP_X_fr_RF_THIGH, Ic_spare);
    RF_HIP_Ic += Ic_spare;

    F = RF_THIGH_Ic.col(AZ);
    DATA(RF_HFE, RF_HFE) = F(AZ);

    F = frcTransf -> fr_RF_HIP_X_fr_RF_THIGH * F;
    DATA(RF_HFE, RF_HAA) = F(AZ);
    DATA(RF_HAA, RF_HFE) = DATA(RF_HFE, RF_HAA);

    // Link RF_HIP:

    F = RF_HIP_Ic.col(AZ);
    DATA(RF_HAA, RF_HAA) = F(AZ);


    // Link LF_FOOT:
    iit::rbd::transformInertia<Scalar>(LF_FOOT_Ic, frcTransf -> fr_LF_ADAPTER_X_fr_LF_FOOT, Ic_spare);
    LF_ADAPTER_Ic += Ic_spare;

    F = LF_FOOT_Ic.col(AZ);
    DATA(LF_ADAPTER_TO_FOOT, LF_ADAPTER_TO_FOOT) = F(AZ);

    F = frcTransf -> fr_LF_ADAPTER_X_fr_LF_FOOT * F;
    DATA(LF_ADAPTER_TO_FOOT, LF_SHANK_TO_ADAPTER) = F(AZ);
    DATA(LF_SHANK_TO_ADAPTER, LF_ADAPTER_TO_FOOT) = DATA(LF_ADAPTER_TO_FOOT, LF_SHANK_TO_ADAPTER);
    F = frcTransf -> fr_LF_SHANK_X_fr_LF_ADAPTER * F;
    DATA(LF_ADAPTER_TO_FOOT, LF_KFE) = F(AZ);
    DATA(LF_KFE, LF_ADAPTER_TO_FOOT) = DATA(LF_ADAPTER_TO_FOOT, LF_KFE);
    F = frcTransf -> fr_LF_THIGH_X_fr_LF_SHANK * F;
    DATA(LF_ADAPTER_TO_FOOT, LF_HFE) = F(AZ);
    DATA(LF_HFE, LF_ADAPTER_TO_FOOT) = DATA(LF_ADAPTER_TO_FOOT, LF_HFE);
    F = frcTransf -> fr_LF_HIP_X_fr_LF_THIGH * F;
    DATA(LF_ADAPTER_TO_FOOT, LF_HAA) = F(AZ);
    DATA(LF_HAA, LF_ADAPTER_TO_FOOT) = DATA(LF_ADAPTER_TO_FOOT, LF_HAA);

    // Link LF_ADAPTER:
    iit::rbd::transformInertia<Scalar>(LF_ADAPTER_Ic, frcTransf -> fr_LF_SHANK_X_fr_LF_ADAPTER, Ic_spare);
    LF_SHANK_Ic += Ic_spare;

    F = LF_ADAPTER_Ic.col(AZ);
    DATA(LF_SHANK_TO_ADAPTER, LF_SHANK_TO_ADAPTER) = F(AZ);

    F = frcTransf -> fr_LF_SHANK_X_fr_LF_ADAPTER * F;
    DATA(LF_SHANK_TO_ADAPTER, LF_KFE) = F(AZ);
    DATA(LF_KFE, LF_SHANK_TO_ADAPTER) = DATA(LF_SHANK_TO_ADAPTER, LF_KFE);
    F = frcTransf -> fr_LF_THIGH_X_fr_LF_SHANK * F;
    DATA(LF_SHANK_TO_ADAPTER, LF_HFE) = F(AZ);
    DATA(LF_HFE, LF_SHANK_TO_ADAPTER) = DATA(LF_SHANK_TO_ADAPTER, LF_HFE);
    F = frcTransf -> fr_LF_HIP_X_fr_LF_THIGH * F;
    DATA(LF_SHANK_TO_ADAPTER, LF_HAA) = F(AZ);
    DATA(LF_HAA, LF_SHANK_TO_ADAPTER) = DATA(LF_SHANK_TO_ADAPTER, LF_HAA);

    // Link LF_SHANK:
    iit::rbd::transformInertia<Scalar>(LF_SHANK_Ic, frcTransf -> fr_LF_THIGH_X_fr_LF_SHANK, Ic_spare);
    LF_THIGH_Ic += Ic_spare;

    F = LF_SHANK_Ic.col(AZ);
    DATA(LF_KFE, LF_KFE) = F(AZ);

    F = frcTransf -> fr_LF_THIGH_X_fr_LF_SHANK * F;
    DATA(LF_KFE, LF_HFE) = F(AZ);
    DATA(LF_HFE, LF_KFE) = DATA(LF_KFE, LF_HFE);
    F = frcTransf -> fr_LF_HIP_X_fr_LF_THIGH * F;
    DATA(LF_KFE, LF_HAA) = F(AZ);
    DATA(LF_HAA, LF_KFE) = DATA(LF_KFE, LF_HAA);

    // Link LF_THIGH:
    iit::rbd::transformInertia<Scalar>(LF_THIGH_Ic, frcTransf -> fr_LF_HIP_X_fr_LF_THIGH, Ic_spare);
    LF_HIP_Ic += Ic_spare;

    F = LF_THIGH_Ic.col(AZ);
    DATA(LF_HFE, LF_HFE) = F(AZ);

    F = frcTransf -> fr_LF_HIP_X_fr_LF_THIGH * F;
    DATA(LF_HFE, LF_HAA) = F(AZ);
    DATA(LF_HAA, LF_HFE) = DATA(LF_HFE, LF_HAA);

    // Link LF_HIP:

    F = LF_HIP_Ic.col(AZ);
    DATA(LF_HAA, LF_HAA) = F(AZ);


    // Link base_inertia:

    F = base_inertia_Ic.col(AZ);
    DATA(BASE_TO_BASE_INERTIA, BASE_TO_BASE_INERTIA) = F(AZ);


    return *this;
}

#undef DATA
#undef F

void iit::anymal::dyn::JSIM::computeL() {
    L = this -> triangularView<Eigen::Lower>();
    // Joint RH_ADAPTER_TO_FOOT, index 20 :
    L(20, 20) = ScalarTraits::sqrt(L(20, 20));
    L(20, 19) = L(20, 19) / L(20, 20);
    L(20, 18) = L(20, 18) / L(20, 20);
    L(20, 17) = L(20, 17) / L(20, 20);
    L(20, 16) = L(20, 16) / L(20, 20);
    L(19, 19) = L(19, 19) - L(20, 19) * L(20, 19);
    L(19, 18) = L(19, 18) - L(20, 19) * L(20, 18);
    L(19, 17) = L(19, 17) - L(20, 19) * L(20, 17);
    L(19, 16) = L(19, 16) - L(20, 19) * L(20, 16);
    L(18, 18) = L(18, 18) - L(20, 18) * L(20, 18);
    L(18, 17) = L(18, 17) - L(20, 18) * L(20, 17);
    L(18, 16) = L(18, 16) - L(20, 18) * L(20, 16);
    L(17, 17) = L(17, 17) - L(20, 17) * L(20, 17);
    L(17, 16) = L(17, 16) - L(20, 17) * L(20, 16);
    L(16, 16) = L(16, 16) - L(20, 16) * L(20, 16);
    
    // Joint RH_SHANK_TO_ADAPTER, index 19 :
    L(19, 19) = ScalarTraits::sqrt(L(19, 19));
    L(19, 18) = L(19, 18) / L(19, 19);
    L(19, 17) = L(19, 17) / L(19, 19);
    L(19, 16) = L(19, 16) / L(19, 19);
    L(18, 18) = L(18, 18) - L(19, 18) * L(19, 18);
    L(18, 17) = L(18, 17) - L(19, 18) * L(19, 17);
    L(18, 16) = L(18, 16) - L(19, 18) * L(19, 16);
    L(17, 17) = L(17, 17) - L(19, 17) * L(19, 17);
    L(17, 16) = L(17, 16) - L(19, 17) * L(19, 16);
    L(16, 16) = L(16, 16) - L(19, 16) * L(19, 16);
    
    // Joint RH_KFE, index 18 :
    L(18, 18) = ScalarTraits::sqrt(L(18, 18));
    L(18, 17) = L(18, 17) / L(18, 18);
    L(18, 16) = L(18, 16) / L(18, 18);
    L(17, 17) = L(17, 17) - L(18, 17) * L(18, 17);
    L(17, 16) = L(17, 16) - L(18, 17) * L(18, 16);
    L(16, 16) = L(16, 16) - L(18, 16) * L(18, 16);
    
    // Joint RH_HFE, index 17 :
    L(17, 17) = ScalarTraits::sqrt(L(17, 17));
    L(17, 16) = L(17, 16) / L(17, 17);
    L(16, 16) = L(16, 16) - L(17, 16) * L(17, 16);
    
    // Joint RH_HAA, index 16 :
    L(16, 16) = ScalarTraits::sqrt(L(16, 16));
    
    // Joint LH_ADAPTER_TO_FOOT, index 15 :
    L(15, 15) = ScalarTraits::sqrt(L(15, 15));
    L(15, 14) = L(15, 14) / L(15, 15);
    L(15, 13) = L(15, 13) / L(15, 15);
    L(15, 12) = L(15, 12) / L(15, 15);
    L(15, 11) = L(15, 11) / L(15, 15);
    L(14, 14) = L(14, 14) - L(15, 14) * L(15, 14);
    L(14, 13) = L(14, 13) - L(15, 14) * L(15, 13);
    L(14, 12) = L(14, 12) - L(15, 14) * L(15, 12);
    L(14, 11) = L(14, 11) - L(15, 14) * L(15, 11);
    L(13, 13) = L(13, 13) - L(15, 13) * L(15, 13);
    L(13, 12) = L(13, 12) - L(15, 13) * L(15, 12);
    L(13, 11) = L(13, 11) - L(15, 13) * L(15, 11);
    L(12, 12) = L(12, 12) - L(15, 12) * L(15, 12);
    L(12, 11) = L(12, 11) - L(15, 12) * L(15, 11);
    L(11, 11) = L(11, 11) - L(15, 11) * L(15, 11);
    
    // Joint LH_SHANK_TO_ADAPTER, index 14 :
    L(14, 14) = ScalarTraits::sqrt(L(14, 14));
    L(14, 13) = L(14, 13) / L(14, 14);
    L(14, 12) = L(14, 12) / L(14, 14);
    L(14, 11) = L(14, 11) / L(14, 14);
    L(13, 13) = L(13, 13) - L(14, 13) * L(14, 13);
    L(13, 12) = L(13, 12) - L(14, 13) * L(14, 12);
    L(13, 11) = L(13, 11) - L(14, 13) * L(14, 11);
    L(12, 12) = L(12, 12) - L(14, 12) * L(14, 12);
    L(12, 11) = L(12, 11) - L(14, 12) * L(14, 11);
    L(11, 11) = L(11, 11) - L(14, 11) * L(14, 11);
    
    // Joint LH_KFE, index 13 :
    L(13, 13) = ScalarTraits::sqrt(L(13, 13));
    L(13, 12) = L(13, 12) / L(13, 13);
    L(13, 11) = L(13, 11) / L(13, 13);
    L(12, 12) = L(12, 12) - L(13, 12) * L(13, 12);
    L(12, 11) = L(12, 11) - L(13, 12) * L(13, 11);
    L(11, 11) = L(11, 11) - L(13, 11) * L(13, 11);
    
    // Joint LH_HFE, index 12 :
    L(12, 12) = ScalarTraits::sqrt(L(12, 12));
    L(12, 11) = L(12, 11) / L(12, 12);
    L(11, 11) = L(11, 11) - L(12, 11) * L(12, 11);
    
    // Joint LH_HAA, index 11 :
    L(11, 11) = ScalarTraits::sqrt(L(11, 11));
    
    // Joint RF_ADAPTER_TO_FOOT, index 10 :
    L(10, 10) = ScalarTraits::sqrt(L(10, 10));
    L(10, 9) = L(10, 9) / L(10, 10);
    L(10, 8) = L(10, 8) / L(10, 10);
    L(10, 7) = L(10, 7) / L(10, 10);
    L(10, 6) = L(10, 6) / L(10, 10);
    L(9, 9) = L(9, 9) - L(10, 9) * L(10, 9);
    L(9, 8) = L(9, 8) - L(10, 9) * L(10, 8);
    L(9, 7) = L(9, 7) - L(10, 9) * L(10, 7);
    L(9, 6) = L(9, 6) - L(10, 9) * L(10, 6);
    L(8, 8) = L(8, 8) - L(10, 8) * L(10, 8);
    L(8, 7) = L(8, 7) - L(10, 8) * L(10, 7);
    L(8, 6) = L(8, 6) - L(10, 8) * L(10, 6);
    L(7, 7) = L(7, 7) - L(10, 7) * L(10, 7);
    L(7, 6) = L(7, 6) - L(10, 7) * L(10, 6);
    L(6, 6) = L(6, 6) - L(10, 6) * L(10, 6);
    
    // Joint RF_SHANK_TO_ADAPTER, index 9 :
    L(9, 9) = ScalarTraits::sqrt(L(9, 9));
    L(9, 8) = L(9, 8) / L(9, 9);
    L(9, 7) = L(9, 7) / L(9, 9);
    L(9, 6) = L(9, 6) / L(9, 9);
    L(8, 8) = L(8, 8) - L(9, 8) * L(9, 8);
    L(8, 7) = L(8, 7) - L(9, 8) * L(9, 7);
    L(8, 6) = L(8, 6) - L(9, 8) * L(9, 6);
    L(7, 7) = L(7, 7) - L(9, 7) * L(9, 7);
    L(7, 6) = L(7, 6) - L(9, 7) * L(9, 6);
    L(6, 6) = L(6, 6) - L(9, 6) * L(9, 6);
    
    // Joint RF_KFE, index 8 :
    L(8, 8) = ScalarTraits::sqrt(L(8, 8));
    L(8, 7) = L(8, 7) / L(8, 8);
    L(8, 6) = L(8, 6) / L(8, 8);
    L(7, 7) = L(7, 7) - L(8, 7) * L(8, 7);
    L(7, 6) = L(7, 6) - L(8, 7) * L(8, 6);
    L(6, 6) = L(6, 6) - L(8, 6) * L(8, 6);
    
    // Joint RF_HFE, index 7 :
    L(7, 7) = ScalarTraits::sqrt(L(7, 7));
    L(7, 6) = L(7, 6) / L(7, 7);
    L(6, 6) = L(6, 6) - L(7, 6) * L(7, 6);
    
    // Joint RF_HAA, index 6 :
    L(6, 6) = ScalarTraits::sqrt(L(6, 6));
    
    // Joint LF_ADAPTER_TO_FOOT, index 5 :
    L(5, 5) = ScalarTraits::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(5, 3) = L(5, 3) / L(5, 5);
    L(5, 2) = L(5, 2) / L(5, 5);
    L(5, 1) = L(5, 1) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    L(4, 3) = L(4, 3) - L(5, 4) * L(5, 3);
    L(4, 2) = L(4, 2) - L(5, 4) * L(5, 2);
    L(4, 1) = L(4, 1) - L(5, 4) * L(5, 1);
    L(3, 3) = L(3, 3) - L(5, 3) * L(5, 3);
    L(3, 2) = L(3, 2) - L(5, 3) * L(5, 2);
    L(3, 1) = L(3, 1) - L(5, 3) * L(5, 1);
    L(2, 2) = L(2, 2) - L(5, 2) * L(5, 2);
    L(2, 1) = L(2, 1) - L(5, 2) * L(5, 1);
    L(1, 1) = L(1, 1) - L(5, 1) * L(5, 1);
    
    // Joint LF_SHANK_TO_ADAPTER, index 4 :
    L(4, 4) = ScalarTraits::sqrt(L(4, 4));
    L(4, 3) = L(4, 3) / L(4, 4);
    L(4, 2) = L(4, 2) / L(4, 4);
    L(4, 1) = L(4, 1) / L(4, 4);
    L(3, 3) = L(3, 3) - L(4, 3) * L(4, 3);
    L(3, 2) = L(3, 2) - L(4, 3) * L(4, 2);
    L(3, 1) = L(3, 1) - L(4, 3) * L(4, 1);
    L(2, 2) = L(2, 2) - L(4, 2) * L(4, 2);
    L(2, 1) = L(2, 1) - L(4, 2) * L(4, 1);
    L(1, 1) = L(1, 1) - L(4, 1) * L(4, 1);
    
    // Joint LF_KFE, index 3 :
    L(3, 3) = ScalarTraits::sqrt(L(3, 3));
    L(3, 2) = L(3, 2) / L(3, 3);
    L(3, 1) = L(3, 1) / L(3, 3);
    L(2, 2) = L(2, 2) - L(3, 2) * L(3, 2);
    L(2, 1) = L(2, 1) - L(3, 2) * L(3, 1);
    L(1, 1) = L(1, 1) - L(3, 1) * L(3, 1);
    
    // Joint LF_HFE, index 2 :
    L(2, 2) = ScalarTraits::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    
    // Joint LF_HAA, index 1 :
    L(1, 1) = ScalarTraits::sqrt(L(1, 1));
    
    // Joint base_to_base_inertia, index 0 :
    L(0, 0) = ScalarTraits::sqrt(L(0, 0));
    
}

void iit::anymal::dyn::JSIM::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 1) * Linv(1, 1));
    inverse(2, 2) =  + (Linv(2, 1) * Linv(2, 1)) + (Linv(2, 2) * Linv(2, 2));
    inverse(2, 1) =  + (Linv(2, 1) * Linv(1, 1));
    inverse(1, 2) = inverse(2, 1);
    inverse(3, 3) =  + (Linv(3, 1) * Linv(3, 1)) + (Linv(3, 2) * Linv(3, 2)) + (Linv(3, 3) * Linv(3, 3));
    inverse(3, 2) =  + (Linv(3, 1) * Linv(2, 1)) + (Linv(3, 2) * Linv(2, 2));
    inverse(2, 3) = inverse(3, 2);
    inverse(3, 1) =  + (Linv(3, 1) * Linv(1, 1));
    inverse(1, 3) = inverse(3, 1);
    inverse(4, 4) =  + (Linv(4, 1) * Linv(4, 1)) + (Linv(4, 2) * Linv(4, 2)) + (Linv(4, 3) * Linv(4, 3)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 3) =  + (Linv(4, 1) * Linv(3, 1)) + (Linv(4, 2) * Linv(3, 2)) + (Linv(4, 3) * Linv(3, 3));
    inverse(3, 4) = inverse(4, 3);
    inverse(4, 2) =  + (Linv(4, 1) * Linv(2, 1)) + (Linv(4, 2) * Linv(2, 2));
    inverse(2, 4) = inverse(4, 2);
    inverse(4, 1) =  + (Linv(4, 1) * Linv(1, 1));
    inverse(1, 4) = inverse(4, 1);
    inverse(5, 5) =  + (Linv(5, 1) * Linv(5, 1)) + (Linv(5, 2) * Linv(5, 2)) + (Linv(5, 3) * Linv(5, 3)) + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) =  + (Linv(5, 1) * Linv(4, 1)) + (Linv(5, 2) * Linv(4, 2)) + (Linv(5, 3) * Linv(4, 3)) + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(5, 3) =  + (Linv(5, 1) * Linv(3, 1)) + (Linv(5, 2) * Linv(3, 2)) + (Linv(5, 3) * Linv(3, 3));
    inverse(3, 5) = inverse(5, 3);
    inverse(5, 2) =  + (Linv(5, 1) * Linv(2, 1)) + (Linv(5, 2) * Linv(2, 2));
    inverse(2, 5) = inverse(5, 2);
    inverse(5, 1) =  + (Linv(5, 1) * Linv(1, 1));
    inverse(1, 5) = inverse(5, 1);
    inverse(6, 6) =  + (Linv(6, 6) * Linv(6, 6));
    inverse(7, 7) =  + (Linv(7, 6) * Linv(7, 6)) + (Linv(7, 7) * Linv(7, 7));
    inverse(7, 6) =  + (Linv(7, 6) * Linv(6, 6));
    inverse(6, 7) = inverse(7, 6);
    inverse(8, 8) =  + (Linv(8, 6) * Linv(8, 6)) + (Linv(8, 7) * Linv(8, 7)) + (Linv(8, 8) * Linv(8, 8));
    inverse(8, 7) =  + (Linv(8, 6) * Linv(7, 6)) + (Linv(8, 7) * Linv(7, 7));
    inverse(7, 8) = inverse(8, 7);
    inverse(8, 6) =  + (Linv(8, 6) * Linv(6, 6));
    inverse(6, 8) = inverse(8, 6);
    inverse(9, 9) =  + (Linv(9, 6) * Linv(9, 6)) + (Linv(9, 7) * Linv(9, 7)) + (Linv(9, 8) * Linv(9, 8)) + (Linv(9, 9) * Linv(9, 9));
    inverse(9, 8) =  + (Linv(9, 6) * Linv(8, 6)) + (Linv(9, 7) * Linv(8, 7)) + (Linv(9, 8) * Linv(8, 8));
    inverse(8, 9) = inverse(9, 8);
    inverse(9, 7) =  + (Linv(9, 6) * Linv(7, 6)) + (Linv(9, 7) * Linv(7, 7));
    inverse(7, 9) = inverse(9, 7);
    inverse(9, 6) =  + (Linv(9, 6) * Linv(6, 6));
    inverse(6, 9) = inverse(9, 6);
    inverse(10, 10) =  + (Linv(10, 6) * Linv(10, 6)) + (Linv(10, 7) * Linv(10, 7)) + (Linv(10, 8) * Linv(10, 8)) + (Linv(10, 9) * Linv(10, 9)) + (Linv(10, 10) * Linv(10, 10));
    inverse(10, 9) =  + (Linv(10, 6) * Linv(9, 6)) + (Linv(10, 7) * Linv(9, 7)) + (Linv(10, 8) * Linv(9, 8)) + (Linv(10, 9) * Linv(9, 9));
    inverse(9, 10) = inverse(10, 9);
    inverse(10, 8) =  + (Linv(10, 6) * Linv(8, 6)) + (Linv(10, 7) * Linv(8, 7)) + (Linv(10, 8) * Linv(8, 8));
    inverse(8, 10) = inverse(10, 8);
    inverse(10, 7) =  + (Linv(10, 6) * Linv(7, 6)) + (Linv(10, 7) * Linv(7, 7));
    inverse(7, 10) = inverse(10, 7);
    inverse(10, 6) =  + (Linv(10, 6) * Linv(6, 6));
    inverse(6, 10) = inverse(10, 6);
    inverse(11, 11) =  + (Linv(11, 11) * Linv(11, 11));
    inverse(12, 12) =  + (Linv(12, 11) * Linv(12, 11)) + (Linv(12, 12) * Linv(12, 12));
    inverse(12, 11) =  + (Linv(12, 11) * Linv(11, 11));
    inverse(11, 12) = inverse(12, 11);
    inverse(13, 13) =  + (Linv(13, 11) * Linv(13, 11)) + (Linv(13, 12) * Linv(13, 12)) + (Linv(13, 13) * Linv(13, 13));
    inverse(13, 12) =  + (Linv(13, 11) * Linv(12, 11)) + (Linv(13, 12) * Linv(12, 12));
    inverse(12, 13) = inverse(13, 12);
    inverse(13, 11) =  + (Linv(13, 11) * Linv(11, 11));
    inverse(11, 13) = inverse(13, 11);
    inverse(14, 14) =  + (Linv(14, 11) * Linv(14, 11)) + (Linv(14, 12) * Linv(14, 12)) + (Linv(14, 13) * Linv(14, 13)) + (Linv(14, 14) * Linv(14, 14));
    inverse(14, 13) =  + (Linv(14, 11) * Linv(13, 11)) + (Linv(14, 12) * Linv(13, 12)) + (Linv(14, 13) * Linv(13, 13));
    inverse(13, 14) = inverse(14, 13);
    inverse(14, 12) =  + (Linv(14, 11) * Linv(12, 11)) + (Linv(14, 12) * Linv(12, 12));
    inverse(12, 14) = inverse(14, 12);
    inverse(14, 11) =  + (Linv(14, 11) * Linv(11, 11));
    inverse(11, 14) = inverse(14, 11);
    inverse(15, 15) =  + (Linv(15, 11) * Linv(15, 11)) + (Linv(15, 12) * Linv(15, 12)) + (Linv(15, 13) * Linv(15, 13)) + (Linv(15, 14) * Linv(15, 14)) + (Linv(15, 15) * Linv(15, 15));
    inverse(15, 14) =  + (Linv(15, 11) * Linv(14, 11)) + (Linv(15, 12) * Linv(14, 12)) + (Linv(15, 13) * Linv(14, 13)) + (Linv(15, 14) * Linv(14, 14));
    inverse(14, 15) = inverse(15, 14);
    inverse(15, 13) =  + (Linv(15, 11) * Linv(13, 11)) + (Linv(15, 12) * Linv(13, 12)) + (Linv(15, 13) * Linv(13, 13));
    inverse(13, 15) = inverse(15, 13);
    inverse(15, 12) =  + (Linv(15, 11) * Linv(12, 11)) + (Linv(15, 12) * Linv(12, 12));
    inverse(12, 15) = inverse(15, 12);
    inverse(15, 11) =  + (Linv(15, 11) * Linv(11, 11));
    inverse(11, 15) = inverse(15, 11);
    inverse(16, 16) =  + (Linv(16, 16) * Linv(16, 16));
    inverse(17, 17) =  + (Linv(17, 16) * Linv(17, 16)) + (Linv(17, 17) * Linv(17, 17));
    inverse(17, 16) =  + (Linv(17, 16) * Linv(16, 16));
    inverse(16, 17) = inverse(17, 16);
    inverse(18, 18) =  + (Linv(18, 16) * Linv(18, 16)) + (Linv(18, 17) * Linv(18, 17)) + (Linv(18, 18) * Linv(18, 18));
    inverse(18, 17) =  + (Linv(18, 16) * Linv(17, 16)) + (Linv(18, 17) * Linv(17, 17));
    inverse(17, 18) = inverse(18, 17);
    inverse(18, 16) =  + (Linv(18, 16) * Linv(16, 16));
    inverse(16, 18) = inverse(18, 16);
    inverse(19, 19) =  + (Linv(19, 16) * Linv(19, 16)) + (Linv(19, 17) * Linv(19, 17)) + (Linv(19, 18) * Linv(19, 18)) + (Linv(19, 19) * Linv(19, 19));
    inverse(19, 18) =  + (Linv(19, 16) * Linv(18, 16)) + (Linv(19, 17) * Linv(18, 17)) + (Linv(19, 18) * Linv(18, 18));
    inverse(18, 19) = inverse(19, 18);
    inverse(19, 17) =  + (Linv(19, 16) * Linv(17, 16)) + (Linv(19, 17) * Linv(17, 17));
    inverse(17, 19) = inverse(19, 17);
    inverse(19, 16) =  + (Linv(19, 16) * Linv(16, 16));
    inverse(16, 19) = inverse(19, 16);
    inverse(20, 20) =  + (Linv(20, 16) * Linv(20, 16)) + (Linv(20, 17) * Linv(20, 17)) + (Linv(20, 18) * Linv(20, 18)) + (Linv(20, 19) * Linv(20, 19)) + (Linv(20, 20) * Linv(20, 20));
    inverse(20, 19) =  + (Linv(20, 16) * Linv(19, 16)) + (Linv(20, 17) * Linv(19, 17)) + (Linv(20, 18) * Linv(19, 18)) + (Linv(20, 19) * Linv(19, 19));
    inverse(19, 20) = inverse(20, 19);
    inverse(20, 18) =  + (Linv(20, 16) * Linv(18, 16)) + (Linv(20, 17) * Linv(18, 17)) + (Linv(20, 18) * Linv(18, 18));
    inverse(18, 20) = inverse(20, 18);
    inverse(20, 17) =  + (Linv(20, 16) * Linv(17, 16)) + (Linv(20, 17) * Linv(17, 17));
    inverse(17, 20) = inverse(20, 17);
    inverse(20, 16) =  + (Linv(20, 16) * Linv(16, 16));
    inverse(16, 20) = inverse(20, 16);
}

void iit::anymal::dyn::JSIM::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(6, 6) = 1 / L(6, 6);
    Linv(7, 7) = 1 / L(7, 7);
    Linv(8, 8) = 1 / L(8, 8);
    Linv(9, 9) = 1 / L(9, 9);
    Linv(10, 10) = 1 / L(10, 10);
    Linv(11, 11) = 1 / L(11, 11);
    Linv(12, 12) = 1 / L(12, 12);
    Linv(13, 13) = 1 / L(13, 13);
    Linv(14, 14) = 1 / L(14, 14);
    Linv(15, 15) = 1 / L(15, 15);
    Linv(16, 16) = 1 / L(16, 16);
    Linv(17, 17) = 1 / L(17, 17);
    Linv(18, 18) = 1 / L(18, 18);
    Linv(19, 19) = 1 / L(19, 19);
    Linv(20, 20) = 1 / L(20, 20);
    Linv(2, 1) = - Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(3, 2) = - Linv(2, 2) * ((Linv(3, 3) * L(3, 2)) + 0);
    Linv(3, 1) = - Linv(1, 1) * ((Linv(3, 2) * L(2, 1)) + (Linv(3, 3) * L(3, 1)) + 0);
    Linv(4, 3) = - Linv(3, 3) * ((Linv(4, 4) * L(4, 3)) + 0);
    Linv(4, 2) = - Linv(2, 2) * ((Linv(4, 3) * L(3, 2)) + (Linv(4, 4) * L(4, 2)) + 0);
    Linv(4, 1) = - Linv(1, 1) * ((Linv(4, 2) * L(2, 1)) + (Linv(4, 3) * L(3, 1)) + (Linv(4, 4) * L(4, 1)) + 0);
    Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(5, 3) = - Linv(3, 3) * ((Linv(5, 4) * L(4, 3)) + (Linv(5, 5) * L(5, 3)) + 0);
    Linv(5, 2) = - Linv(2, 2) * ((Linv(5, 3) * L(3, 2)) + (Linv(5, 4) * L(4, 2)) + (Linv(5, 5) * L(5, 2)) + 0);
    Linv(5, 1) = - Linv(1, 1) * ((Linv(5, 2) * L(2, 1)) + (Linv(5, 3) * L(3, 1)) + (Linv(5, 4) * L(4, 1)) + (Linv(5, 5) * L(5, 1)) + 0);
    Linv(7, 6) = - Linv(6, 6) * ((Linv(7, 7) * L(7, 6)) + 0);
    Linv(8, 7) = - Linv(7, 7) * ((Linv(8, 8) * L(8, 7)) + 0);
    Linv(8, 6) = - Linv(6, 6) * ((Linv(8, 7) * L(7, 6)) + (Linv(8, 8) * L(8, 6)) + 0);
    Linv(9, 8) = - Linv(8, 8) * ((Linv(9, 9) * L(9, 8)) + 0);
    Linv(9, 7) = - Linv(7, 7) * ((Linv(9, 8) * L(8, 7)) + (Linv(9, 9) * L(9, 7)) + 0);
    Linv(9, 6) = - Linv(6, 6) * ((Linv(9, 7) * L(7, 6)) + (Linv(9, 8) * L(8, 6)) + (Linv(9, 9) * L(9, 6)) + 0);
    Linv(10, 9) = - Linv(9, 9) * ((Linv(10, 10) * L(10, 9)) + 0);
    Linv(10, 8) = - Linv(8, 8) * ((Linv(10, 9) * L(9, 8)) + (Linv(10, 10) * L(10, 8)) + 0);
    Linv(10, 7) = - Linv(7, 7) * ((Linv(10, 8) * L(8, 7)) + (Linv(10, 9) * L(9, 7)) + (Linv(10, 10) * L(10, 7)) + 0);
    Linv(10, 6) = - Linv(6, 6) * ((Linv(10, 7) * L(7, 6)) + (Linv(10, 8) * L(8, 6)) + (Linv(10, 9) * L(9, 6)) + (Linv(10, 10) * L(10, 6)) + 0);
    Linv(12, 11) = - Linv(11, 11) * ((Linv(12, 12) * L(12, 11)) + 0);
    Linv(13, 12) = - Linv(12, 12) * ((Linv(13, 13) * L(13, 12)) + 0);
    Linv(13, 11) = - Linv(11, 11) * ((Linv(13, 12) * L(12, 11)) + (Linv(13, 13) * L(13, 11)) + 0);
    Linv(14, 13) = - Linv(13, 13) * ((Linv(14, 14) * L(14, 13)) + 0);
    Linv(14, 12) = - Linv(12, 12) * ((Linv(14, 13) * L(13, 12)) + (Linv(14, 14) * L(14, 12)) + 0);
    Linv(14, 11) = - Linv(11, 11) * ((Linv(14, 12) * L(12, 11)) + (Linv(14, 13) * L(13, 11)) + (Linv(14, 14) * L(14, 11)) + 0);
    Linv(15, 14) = - Linv(14, 14) * ((Linv(15, 15) * L(15, 14)) + 0);
    Linv(15, 13) = - Linv(13, 13) * ((Linv(15, 14) * L(14, 13)) + (Linv(15, 15) * L(15, 13)) + 0);
    Linv(15, 12) = - Linv(12, 12) * ((Linv(15, 13) * L(13, 12)) + (Linv(15, 14) * L(14, 12)) + (Linv(15, 15) * L(15, 12)) + 0);
    Linv(15, 11) = - Linv(11, 11) * ((Linv(15, 12) * L(12, 11)) + (Linv(15, 13) * L(13, 11)) + (Linv(15, 14) * L(14, 11)) + (Linv(15, 15) * L(15, 11)) + 0);
    Linv(17, 16) = - Linv(16, 16) * ((Linv(17, 17) * L(17, 16)) + 0);
    Linv(18, 17) = - Linv(17, 17) * ((Linv(18, 18) * L(18, 17)) + 0);
    Linv(18, 16) = - Linv(16, 16) * ((Linv(18, 17) * L(17, 16)) + (Linv(18, 18) * L(18, 16)) + 0);
    Linv(19, 18) = - Linv(18, 18) * ((Linv(19, 19) * L(19, 18)) + 0);
    Linv(19, 17) = - Linv(17, 17) * ((Linv(19, 18) * L(18, 17)) + (Linv(19, 19) * L(19, 17)) + 0);
    Linv(19, 16) = - Linv(16, 16) * ((Linv(19, 17) * L(17, 16)) + (Linv(19, 18) * L(18, 16)) + (Linv(19, 19) * L(19, 16)) + 0);
    Linv(20, 19) = - Linv(19, 19) * ((Linv(20, 20) * L(20, 19)) + 0);
    Linv(20, 18) = - Linv(18, 18) * ((Linv(20, 19) * L(19, 18)) + (Linv(20, 20) * L(20, 18)) + 0);
    Linv(20, 17) = - Linv(17, 17) * ((Linv(20, 18) * L(18, 17)) + (Linv(20, 19) * L(19, 17)) + (Linv(20, 20) * L(20, 17)) + 0);
    Linv(20, 16) = - Linv(16, 16) * ((Linv(20, 17) * L(17, 16)) + (Linv(20, 18) * L(18, 16)) + (Linv(20, 19) * L(19, 16)) + (Linv(20, 20) * L(20, 16)) + 0);
}
