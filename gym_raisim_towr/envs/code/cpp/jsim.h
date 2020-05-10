#ifndef IIT_ANYMAL_JSIM_H_
#define IIT_ANYMAL_JSIM_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/StateDependentMatrix.h>

#include "rbd_types.h"
#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"

namespace iit {
namespace anymal {
namespace dyn {

/**
 * The type of the Joint Space Inertia Matrix (JSIM) of the robot anymal.
 */
class JSIM : public iit::rbd::StateDependentMatrix<iit::anymal::JointState, 21, 21, JSIM>
{
    private:
        typedef iit::rbd::StateDependentMatrix<iit::anymal::JointState, 21, 21, JSIM> Base;
    public:
        typedef Base::Scalar     Scalar;
        typedef Base::Index      Index;
        typedef Base::MatrixType MatrixType;
    public:
        JSIM(InertiaProperties&, ForceTransforms&);
        ~JSIM() {}

        const JSIM& update(const JointState&);


        /**
         * Computes and saves the matrix L of the L^T L factorization of this JSIM.
         */
        void computeL();
        /**
         * Computes and saves the inverse of this JSIM.
         * This function assumes that computeL() has been called already, since it
         * uses L to compute the inverse. The algorithm takes advantage of the branch
         * induced sparsity of the robot, if any.
         */
        void computeInverse();
        /**
         * Returns an unmodifiable reference to the matrix L. See also computeL()
         */
        const MatrixType& getL() const;
        /**
         * Returns an unmodifiable reference to the inverse of this JSIM
         */
        const MatrixType& getInverse() const;

    protected:
        /**
         * Computes and saves the inverse of the matrix L. See also computeL()
         */
        void computeLInverse();
    private:
        InertiaProperties& linkInertias;
        ForceTransforms* frcTransf;

        // The composite-inertia tensor for each link
        const InertiaMatrix& base_inertia_Ic;
        InertiaMatrix LF_HIP_Ic;
        InertiaMatrix LF_THIGH_Ic;
        InertiaMatrix LF_SHANK_Ic;
        InertiaMatrix LF_ADAPTER_Ic;
        const InertiaMatrix& LF_FOOT_Ic;
        InertiaMatrix RF_HIP_Ic;
        InertiaMatrix RF_THIGH_Ic;
        InertiaMatrix RF_SHANK_Ic;
        InertiaMatrix RF_ADAPTER_Ic;
        const InertiaMatrix& RF_FOOT_Ic;
        InertiaMatrix LH_HIP_Ic;
        InertiaMatrix LH_THIGH_Ic;
        InertiaMatrix LH_SHANK_Ic;
        InertiaMatrix LH_ADAPTER_Ic;
        const InertiaMatrix& LH_FOOT_Ic;
        InertiaMatrix RH_HIP_Ic;
        InertiaMatrix RH_THIGH_Ic;
        InertiaMatrix RH_SHANK_Ic;
        InertiaMatrix RH_ADAPTER_Ic;
        const InertiaMatrix& RH_FOOT_Ic;
        InertiaMatrix Ic_spare;

        MatrixType L;
        MatrixType Linv;
        MatrixType inverse;
};


inline const JSIM::MatrixType& JSIM::getL() const {
    return L;
}

inline const JSIM::MatrixType& JSIM::getInverse() const {
    return inverse;
}




}
}
}
#endif
