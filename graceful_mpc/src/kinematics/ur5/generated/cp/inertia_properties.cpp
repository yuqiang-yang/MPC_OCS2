#include "inertia_properties.h"

using namespace std;
using namespace iit::rbd;

ur5e::rcg::InertiaProperties::InertiaProperties()
{
    com_base_link_ur5e = Vector3(0.0,0.0,0.0);
    tensor_base_link_ur5e.fill(
        m_base_link_ur5e,
        com_base_link_ur5e,
        Utils::buildInertiaTensor<Scalar>(ix_base_link_ur5e,iy_base_link_ur5e,iz_base_link_ur5e,0.0,0.0,0.0) );

    com_shoulder_link = Vector3(0.0,0.0,0.0);
    tensor_shoulder_link.fill(
        m_shoulder_link,
        com_shoulder_link,
        Utils::buildInertiaTensor<Scalar>(ix_shoulder_link,iy_shoulder_link,iz_shoulder_link,0.0,0.0,0.0) );

    com_upper_arm_link = Vector3(comx_upper_arm_link,comy_upper_arm_link,comz_upper_arm_link);
    tensor_upper_arm_link.fill(
        m_upper_arm_link,
        com_upper_arm_link,
        Utils::buildInertiaTensor<Scalar>(ix_upper_arm_link,iy_upper_arm_link,iz_upper_arm_link,ixy_upper_arm_link,ixz_upper_arm_link,iyz_upper_arm_link) );

    com_forearm_link = Vector3(comx_forearm_link,comy_forearm_link,comz_forearm_link);
    tensor_forearm_link.fill(
        m_forearm_link,
        com_forearm_link,
        Utils::buildInertiaTensor<Scalar>(ix_forearm_link,iy_forearm_link,iz_forearm_link,ixy_forearm_link,ixz_forearm_link,iyz_forearm_link) );

    com_wrist_1_link = Vector3(0.0,0.0,comz_wrist_1_link);
    tensor_wrist_1_link.fill(
        m_wrist_1_link,
        com_wrist_1_link,
        Utils::buildInertiaTensor<Scalar>(ix_wrist_1_link,iy_wrist_1_link,iz_wrist_1_link,0.0,ixz_wrist_1_link,iyz_wrist_1_link) );

    com_wrist_2_link = Vector3(comx_wrist_2_link,0.0,comz_wrist_2_link);
    tensor_wrist_2_link.fill(
        m_wrist_2_link,
        com_wrist_2_link,
        Utils::buildInertiaTensor<Scalar>(ix_wrist_2_link,iy_wrist_2_link,iz_wrist_2_link,0.0,ixz_wrist_2_link,0.0) );

    com_wrist_3_link = Vector3(comx_wrist_3_link,0.0,comz_wrist_3_link);
    tensor_wrist_3_link.fill(
        m_wrist_3_link,
        com_wrist_3_link,
        Utils::buildInertiaTensor<Scalar>(ix_wrist_3_link,iy_wrist_3_link,iz_wrist_3_link,ixy_wrist_3_link,0.0,iyz_wrist_3_link) );

}


void ur5e::rcg::InertiaProperties::updateParameters(const RuntimeInertiaParams& fresh)
{
    this-> params = fresh;
}
