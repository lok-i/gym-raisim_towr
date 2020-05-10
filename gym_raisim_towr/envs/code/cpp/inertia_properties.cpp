#include "inertia_properties.h"

using namespace std;
using namespace iit::rbd;

iit::anymal::dyn::InertiaProperties::InertiaProperties()
{
    com_base_inertia = Vector3(comx_base_inertia,comy_base_inertia,comz_base_inertia);
    tensor_base_inertia.fill(
        m_base_inertia,
        com_base_inertia,
        Utils::buildInertiaTensor<Scalar>(ix_base_inertia,iy_base_inertia,iz_base_inertia,ixy_base_inertia,ixz_base_inertia,iyz_base_inertia) );

    com_LF_HIP = Vector3(comx_LF_HIP,comy_LF_HIP,comz_LF_HIP);
    tensor_LF_HIP.fill(
        m_LF_HIP,
        com_LF_HIP,
        Utils::buildInertiaTensor<Scalar>(ix_LF_HIP,iy_LF_HIP,iz_LF_HIP,ixy_LF_HIP,ixz_LF_HIP,iyz_LF_HIP) );

    com_LF_THIGH = Vector3(comx_LF_THIGH,comy_LF_THIGH,comz_LF_THIGH);
    tensor_LF_THIGH.fill(
        m_LF_THIGH,
        com_LF_THIGH,
        Utils::buildInertiaTensor<Scalar>(ix_LF_THIGH,iy_LF_THIGH,iz_LF_THIGH,ixy_LF_THIGH,ixz_LF_THIGH,iyz_LF_THIGH) );

    com_LF_SHANK = Vector3(comx_LF_SHANK,comy_LF_SHANK,comz_LF_SHANK);
    tensor_LF_SHANK.fill(
        m_LF_SHANK,
        com_LF_SHANK,
        Utils::buildInertiaTensor<Scalar>(ix_LF_SHANK,iy_LF_SHANK,iz_LF_SHANK,ixy_LF_SHANK,0.0,iyz_LF_SHANK) );

    com_LF_ADAPTER = Vector3(0.0,0.0,comz_LF_ADAPTER);
    tensor_LF_ADAPTER.fill(
        m_LF_ADAPTER,
        com_LF_ADAPTER,
        Utils::buildInertiaTensor<Scalar>(ix_LF_ADAPTER,iy_LF_ADAPTER,iz_LF_ADAPTER,0.0,0.0,0.0) );

    com_LF_FOOT = Vector3(0.0,0.0,0.0);
    tensor_LF_FOOT.fill(
        0.0,
        com_LF_FOOT,
        Utils::buildInertiaTensor<Scalar>(0.0,0.0,0.0,0.0,0.0,0.0) );

    com_RF_HIP = Vector3(comx_RF_HIP,comy_RF_HIP,comz_RF_HIP);
    tensor_RF_HIP.fill(
        m_RF_HIP,
        com_RF_HIP,
        Utils::buildInertiaTensor<Scalar>(ix_RF_HIP,iy_RF_HIP,iz_RF_HIP,ixy_RF_HIP,ixz_RF_HIP,iyz_RF_HIP) );

    com_RF_THIGH = Vector3(comx_RF_THIGH,comy_RF_THIGH,comz_RF_THIGH);
    tensor_RF_THIGH.fill(
        m_RF_THIGH,
        com_RF_THIGH,
        Utils::buildInertiaTensor<Scalar>(ix_RF_THIGH,iy_RF_THIGH,iz_RF_THIGH,ixy_RF_THIGH,ixz_RF_THIGH,iyz_RF_THIGH) );

    com_RF_SHANK = Vector3(comx_RF_SHANK,comy_RF_SHANK,comz_RF_SHANK);
    tensor_RF_SHANK.fill(
        m_RF_SHANK,
        com_RF_SHANK,
        Utils::buildInertiaTensor<Scalar>(ix_RF_SHANK,iy_RF_SHANK,iz_RF_SHANK,ixy_RF_SHANK,0.0,iyz_RF_SHANK) );

    com_RF_ADAPTER = Vector3(0.0,0.0,comz_RF_ADAPTER);
    tensor_RF_ADAPTER.fill(
        m_RF_ADAPTER,
        com_RF_ADAPTER,
        Utils::buildInertiaTensor<Scalar>(ix_RF_ADAPTER,iy_RF_ADAPTER,iz_RF_ADAPTER,0.0,0.0,0.0) );

    com_RF_FOOT = Vector3(0.0,0.0,0.0);
    tensor_RF_FOOT.fill(
        0.0,
        com_RF_FOOT,
        Utils::buildInertiaTensor<Scalar>(0.0,0.0,0.0,0.0,0.0,0.0) );

    com_LH_HIP = Vector3(comx_LH_HIP,comy_LH_HIP,comz_LH_HIP);
    tensor_LH_HIP.fill(
        m_LH_HIP,
        com_LH_HIP,
        Utils::buildInertiaTensor<Scalar>(ix_LH_HIP,iy_LH_HIP,iz_LH_HIP,ixy_LH_HIP,ixz_LH_HIP,iyz_LH_HIP) );

    com_LH_THIGH = Vector3(comx_LH_THIGH,comy_LH_THIGH,comz_LH_THIGH);
    tensor_LH_THIGH.fill(
        m_LH_THIGH,
        com_LH_THIGH,
        Utils::buildInertiaTensor<Scalar>(ix_LH_THIGH,iy_LH_THIGH,iz_LH_THIGH,ixy_LH_THIGH,ixz_LH_THIGH,iyz_LH_THIGH) );

    com_LH_SHANK = Vector3(comx_LH_SHANK,comy_LH_SHANK,comz_LH_SHANK);
    tensor_LH_SHANK.fill(
        m_LH_SHANK,
        com_LH_SHANK,
        Utils::buildInertiaTensor<Scalar>(ix_LH_SHANK,iy_LH_SHANK,iz_LH_SHANK,ixy_LH_SHANK,0.0,iyz_LH_SHANK) );

    com_LH_ADAPTER = Vector3(0.0,0.0,comz_LH_ADAPTER);
    tensor_LH_ADAPTER.fill(
        m_LH_ADAPTER,
        com_LH_ADAPTER,
        Utils::buildInertiaTensor<Scalar>(ix_LH_ADAPTER,iy_LH_ADAPTER,iz_LH_ADAPTER,0.0,0.0,0.0) );

    com_LH_FOOT = Vector3(0.0,0.0,0.0);
    tensor_LH_FOOT.fill(
        0.0,
        com_LH_FOOT,
        Utils::buildInertiaTensor<Scalar>(0.0,0.0,0.0,0.0,0.0,0.0) );

    com_RH_HIP = Vector3(comx_RH_HIP,comy_RH_HIP,comz_RH_HIP);
    tensor_RH_HIP.fill(
        m_RH_HIP,
        com_RH_HIP,
        Utils::buildInertiaTensor<Scalar>(ix_RH_HIP,iy_RH_HIP,iz_RH_HIP,ixy_RH_HIP,ixz_RH_HIP,iyz_RH_HIP) );

    com_RH_THIGH = Vector3(comx_RH_THIGH,comy_RH_THIGH,comz_RH_THIGH);
    tensor_RH_THIGH.fill(
        m_RH_THIGH,
        com_RH_THIGH,
        Utils::buildInertiaTensor<Scalar>(ix_RH_THIGH,iy_RH_THIGH,iz_RH_THIGH,ixy_RH_THIGH,ixz_RH_THIGH,iyz_RH_THIGH) );

    com_RH_SHANK = Vector3(comx_RH_SHANK,comy_RH_SHANK,comz_RH_SHANK);
    tensor_RH_SHANK.fill(
        m_RH_SHANK,
        com_RH_SHANK,
        Utils::buildInertiaTensor<Scalar>(ix_RH_SHANK,iy_RH_SHANK,iz_RH_SHANK,ixy_RH_SHANK,0.0,iyz_RH_SHANK) );

    com_RH_ADAPTER = Vector3(0.0,0.0,comz_RH_ADAPTER);
    tensor_RH_ADAPTER.fill(
        m_RH_ADAPTER,
        com_RH_ADAPTER,
        Utils::buildInertiaTensor<Scalar>(ix_RH_ADAPTER,iy_RH_ADAPTER,iz_RH_ADAPTER,0.0,0.0,0.0) );

    com_RH_FOOT = Vector3(0.0,0.0,0.0);
    tensor_RH_FOOT.fill(
        0.0,
        com_RH_FOOT,
        Utils::buildInertiaTensor<Scalar>(0.0,0.0,0.0,0.0,0.0,0.0) );

}


void iit::anymal::dyn::InertiaProperties::updateParameters(const RuntimeInertiaParams& fresh)
{
    this-> params = fresh;
}
