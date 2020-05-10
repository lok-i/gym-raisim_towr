#ifndef IIT_ROBOT_ANYMAL_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_ANYMAL_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "model_constants.h"
#include "dynamics_parameters.h"

namespace iit {
namespace anymal {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot anymal.
 */
namespace dyn {

class InertiaProperties {
    public:
        InertiaProperties();
        ~InertiaProperties();
        const InertiaMatrix& getTensor_base_inertia() const;
        const InertiaMatrix& getTensor_LF_HIP() const;
        const InertiaMatrix& getTensor_LF_THIGH() const;
        const InertiaMatrix& getTensor_LF_SHANK() const;
        const InertiaMatrix& getTensor_LF_ADAPTER() const;
        const InertiaMatrix& getTensor_LF_FOOT() const;
        const InertiaMatrix& getTensor_RF_HIP() const;
        const InertiaMatrix& getTensor_RF_THIGH() const;
        const InertiaMatrix& getTensor_RF_SHANK() const;
        const InertiaMatrix& getTensor_RF_ADAPTER() const;
        const InertiaMatrix& getTensor_RF_FOOT() const;
        const InertiaMatrix& getTensor_LH_HIP() const;
        const InertiaMatrix& getTensor_LH_THIGH() const;
        const InertiaMatrix& getTensor_LH_SHANK() const;
        const InertiaMatrix& getTensor_LH_ADAPTER() const;
        const InertiaMatrix& getTensor_LH_FOOT() const;
        const InertiaMatrix& getTensor_RH_HIP() const;
        const InertiaMatrix& getTensor_RH_THIGH() const;
        const InertiaMatrix& getTensor_RH_SHANK() const;
        const InertiaMatrix& getTensor_RH_ADAPTER() const;
        const InertiaMatrix& getTensor_RH_FOOT() const;
        Scalar getMass_base_inertia() const;
        Scalar getMass_LF_HIP() const;
        Scalar getMass_LF_THIGH() const;
        Scalar getMass_LF_SHANK() const;
        Scalar getMass_LF_ADAPTER() const;
        Scalar getMass_LF_FOOT() const;
        Scalar getMass_RF_HIP() const;
        Scalar getMass_RF_THIGH() const;
        Scalar getMass_RF_SHANK() const;
        Scalar getMass_RF_ADAPTER() const;
        Scalar getMass_RF_FOOT() const;
        Scalar getMass_LH_HIP() const;
        Scalar getMass_LH_THIGH() const;
        Scalar getMass_LH_SHANK() const;
        Scalar getMass_LH_ADAPTER() const;
        Scalar getMass_LH_FOOT() const;
        Scalar getMass_RH_HIP() const;
        Scalar getMass_RH_THIGH() const;
        Scalar getMass_RH_SHANK() const;
        Scalar getMass_RH_ADAPTER() const;
        Scalar getMass_RH_FOOT() const;
        const Vector3& getCOM_base_inertia() const;
        const Vector3& getCOM_LF_HIP() const;
        const Vector3& getCOM_LF_THIGH() const;
        const Vector3& getCOM_LF_SHANK() const;
        const Vector3& getCOM_LF_ADAPTER() const;
        const Vector3& getCOM_LF_FOOT() const;
        const Vector3& getCOM_RF_HIP() const;
        const Vector3& getCOM_RF_THIGH() const;
        const Vector3& getCOM_RF_SHANK() const;
        const Vector3& getCOM_RF_ADAPTER() const;
        const Vector3& getCOM_RF_FOOT() const;
        const Vector3& getCOM_LH_HIP() const;
        const Vector3& getCOM_LH_THIGH() const;
        const Vector3& getCOM_LH_SHANK() const;
        const Vector3& getCOM_LH_ADAPTER() const;
        const Vector3& getCOM_LH_FOOT() const;
        const Vector3& getCOM_RH_HIP() const;
        const Vector3& getCOM_RH_THIGH() const;
        const Vector3& getCOM_RH_SHANK() const;
        const Vector3& getCOM_RH_ADAPTER() const;
        const Vector3& getCOM_RH_FOOT() const;
        Scalar getTotalMass() const;


        /*!
         * Fresh values for the runtime parameters of the robot anymal,
         * causing the update of the inertia properties modeled by this
         * instance.
         */
        void updateParameters(const RuntimeInertiaParams&);

    private:
        RuntimeInertiaParams params;

        InertiaMatrix tensor_base_inertia;
        InertiaMatrix tensor_LF_HIP;
        InertiaMatrix tensor_LF_THIGH;
        InertiaMatrix tensor_LF_SHANK;
        InertiaMatrix tensor_LF_ADAPTER;
        InertiaMatrix tensor_LF_FOOT;
        InertiaMatrix tensor_RF_HIP;
        InertiaMatrix tensor_RF_THIGH;
        InertiaMatrix tensor_RF_SHANK;
        InertiaMatrix tensor_RF_ADAPTER;
        InertiaMatrix tensor_RF_FOOT;
        InertiaMatrix tensor_LH_HIP;
        InertiaMatrix tensor_LH_THIGH;
        InertiaMatrix tensor_LH_SHANK;
        InertiaMatrix tensor_LH_ADAPTER;
        InertiaMatrix tensor_LH_FOOT;
        InertiaMatrix tensor_RH_HIP;
        InertiaMatrix tensor_RH_THIGH;
        InertiaMatrix tensor_RH_SHANK;
        InertiaMatrix tensor_RH_ADAPTER;
        InertiaMatrix tensor_RH_FOOT;
        Vector3 com_base_inertia;
        Vector3 com_LF_HIP;
        Vector3 com_LF_THIGH;
        Vector3 com_LF_SHANK;
        Vector3 com_LF_ADAPTER;
        Vector3 com_LF_FOOT;
        Vector3 com_RF_HIP;
        Vector3 com_RF_THIGH;
        Vector3 com_RF_SHANK;
        Vector3 com_RF_ADAPTER;
        Vector3 com_RF_FOOT;
        Vector3 com_LH_HIP;
        Vector3 com_LH_THIGH;
        Vector3 com_LH_SHANK;
        Vector3 com_LH_ADAPTER;
        Vector3 com_LH_FOOT;
        Vector3 com_RH_HIP;
        Vector3 com_RH_THIGH;
        Vector3 com_RH_SHANK;
        Vector3 com_RH_ADAPTER;
        Vector3 com_RH_FOOT;
};


inline InertiaProperties::~InertiaProperties() {}

inline const InertiaMatrix& InertiaProperties::getTensor_base_inertia() const {
    return this->tensor_base_inertia;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LF_HIP() const {
    return this->tensor_LF_HIP;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LF_THIGH() const {
    return this->tensor_LF_THIGH;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LF_SHANK() const {
    return this->tensor_LF_SHANK;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LF_ADAPTER() const {
    return this->tensor_LF_ADAPTER;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LF_FOOT() const {
    return this->tensor_LF_FOOT;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RF_HIP() const {
    return this->tensor_RF_HIP;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RF_THIGH() const {
    return this->tensor_RF_THIGH;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RF_SHANK() const {
    return this->tensor_RF_SHANK;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RF_ADAPTER() const {
    return this->tensor_RF_ADAPTER;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RF_FOOT() const {
    return this->tensor_RF_FOOT;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LH_HIP() const {
    return this->tensor_LH_HIP;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LH_THIGH() const {
    return this->tensor_LH_THIGH;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LH_SHANK() const {
    return this->tensor_LH_SHANK;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LH_ADAPTER() const {
    return this->tensor_LH_ADAPTER;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LH_FOOT() const {
    return this->tensor_LH_FOOT;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RH_HIP() const {
    return this->tensor_RH_HIP;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RH_THIGH() const {
    return this->tensor_RH_THIGH;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RH_SHANK() const {
    return this->tensor_RH_SHANK;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RH_ADAPTER() const {
    return this->tensor_RH_ADAPTER;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RH_FOOT() const {
    return this->tensor_RH_FOOT;
}
inline Scalar InertiaProperties::getMass_base_inertia() const {
    return this->tensor_base_inertia.getMass();
}
inline Scalar InertiaProperties::getMass_LF_HIP() const {
    return this->tensor_LF_HIP.getMass();
}
inline Scalar InertiaProperties::getMass_LF_THIGH() const {
    return this->tensor_LF_THIGH.getMass();
}
inline Scalar InertiaProperties::getMass_LF_SHANK() const {
    return this->tensor_LF_SHANK.getMass();
}
inline Scalar InertiaProperties::getMass_LF_ADAPTER() const {
    return this->tensor_LF_ADAPTER.getMass();
}
inline Scalar InertiaProperties::getMass_LF_FOOT() const {
    return this->tensor_LF_FOOT.getMass();
}
inline Scalar InertiaProperties::getMass_RF_HIP() const {
    return this->tensor_RF_HIP.getMass();
}
inline Scalar InertiaProperties::getMass_RF_THIGH() const {
    return this->tensor_RF_THIGH.getMass();
}
inline Scalar InertiaProperties::getMass_RF_SHANK() const {
    return this->tensor_RF_SHANK.getMass();
}
inline Scalar InertiaProperties::getMass_RF_ADAPTER() const {
    return this->tensor_RF_ADAPTER.getMass();
}
inline Scalar InertiaProperties::getMass_RF_FOOT() const {
    return this->tensor_RF_FOOT.getMass();
}
inline Scalar InertiaProperties::getMass_LH_HIP() const {
    return this->tensor_LH_HIP.getMass();
}
inline Scalar InertiaProperties::getMass_LH_THIGH() const {
    return this->tensor_LH_THIGH.getMass();
}
inline Scalar InertiaProperties::getMass_LH_SHANK() const {
    return this->tensor_LH_SHANK.getMass();
}
inline Scalar InertiaProperties::getMass_LH_ADAPTER() const {
    return this->tensor_LH_ADAPTER.getMass();
}
inline Scalar InertiaProperties::getMass_LH_FOOT() const {
    return this->tensor_LH_FOOT.getMass();
}
inline Scalar InertiaProperties::getMass_RH_HIP() const {
    return this->tensor_RH_HIP.getMass();
}
inline Scalar InertiaProperties::getMass_RH_THIGH() const {
    return this->tensor_RH_THIGH.getMass();
}
inline Scalar InertiaProperties::getMass_RH_SHANK() const {
    return this->tensor_RH_SHANK.getMass();
}
inline Scalar InertiaProperties::getMass_RH_ADAPTER() const {
    return this->tensor_RH_ADAPTER.getMass();
}
inline Scalar InertiaProperties::getMass_RH_FOOT() const {
    return this->tensor_RH_FOOT.getMass();
}
inline const Vector3& InertiaProperties::getCOM_base_inertia() const {
    return this->com_base_inertia;
}
inline const Vector3& InertiaProperties::getCOM_LF_HIP() const {
    return this->com_LF_HIP;
}
inline const Vector3& InertiaProperties::getCOM_LF_THIGH() const {
    return this->com_LF_THIGH;
}
inline const Vector3& InertiaProperties::getCOM_LF_SHANK() const {
    return this->com_LF_SHANK;
}
inline const Vector3& InertiaProperties::getCOM_LF_ADAPTER() const {
    return this->com_LF_ADAPTER;
}
inline const Vector3& InertiaProperties::getCOM_LF_FOOT() const {
    return this->com_LF_FOOT;
}
inline const Vector3& InertiaProperties::getCOM_RF_HIP() const {
    return this->com_RF_HIP;
}
inline const Vector3& InertiaProperties::getCOM_RF_THIGH() const {
    return this->com_RF_THIGH;
}
inline const Vector3& InertiaProperties::getCOM_RF_SHANK() const {
    return this->com_RF_SHANK;
}
inline const Vector3& InertiaProperties::getCOM_RF_ADAPTER() const {
    return this->com_RF_ADAPTER;
}
inline const Vector3& InertiaProperties::getCOM_RF_FOOT() const {
    return this->com_RF_FOOT;
}
inline const Vector3& InertiaProperties::getCOM_LH_HIP() const {
    return this->com_LH_HIP;
}
inline const Vector3& InertiaProperties::getCOM_LH_THIGH() const {
    return this->com_LH_THIGH;
}
inline const Vector3& InertiaProperties::getCOM_LH_SHANK() const {
    return this->com_LH_SHANK;
}
inline const Vector3& InertiaProperties::getCOM_LH_ADAPTER() const {
    return this->com_LH_ADAPTER;
}
inline const Vector3& InertiaProperties::getCOM_LH_FOOT() const {
    return this->com_LH_FOOT;
}
inline const Vector3& InertiaProperties::getCOM_RH_HIP() const {
    return this->com_RH_HIP;
}
inline const Vector3& InertiaProperties::getCOM_RH_THIGH() const {
    return this->com_RH_THIGH;
}
inline const Vector3& InertiaProperties::getCOM_RH_SHANK() const {
    return this->com_RH_SHANK;
}
inline const Vector3& InertiaProperties::getCOM_RH_ADAPTER() const {
    return this->com_RH_ADAPTER;
}
inline const Vector3& InertiaProperties::getCOM_RH_FOOT() const {
    return this->com_RH_FOOT;
}

inline Scalar InertiaProperties::getTotalMass() const {
    return m_base_inertia + m_LF_HIP + m_LF_THIGH + m_LF_SHANK + m_LF_ADAPTER + 0.0 + m_RF_HIP + m_RF_THIGH + m_RF_SHANK + m_RF_ADAPTER + 0.0 + m_LH_HIP + m_LH_THIGH + m_LH_SHANK + m_LH_ADAPTER + 0.0 + m_RH_HIP + m_RH_THIGH + m_RH_SHANK + m_RH_ADAPTER + 0.0;
}

}
}
}

#endif
