#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace iit::ur5e;
using namespace iit::ur5e::dyn;

iit::rbd::Vector3d iit::ur5e::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    iit::rbd::Vector3d tmpSum(iit::rbd::Vector3d::Zero());

    tmpSum += inertiaProps.getCOM_base_link_ur5e() * inertiaProps.getMass_base_link_ur5e();

    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    tmpX = tmpX * ht.fr_base_link_ur5e_X_fr_shoulder_link;
    tmpSum += inertiaProps.getMass_shoulder_link() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_shoulder_link()));
    
    tmpX = tmpX * ht.fr_shoulder_link_X_fr_upper_arm_link;
    tmpSum += inertiaProps.getMass_upper_arm_link() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_upper_arm_link()));
    
    tmpX = tmpX * ht.fr_upper_arm_link_X_fr_forearm_link;
    tmpSum += inertiaProps.getMass_forearm_link() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_forearm_link()));
    
    tmpX = tmpX * ht.fr_forearm_link_X_fr_wrist_1_link;
    tmpSum += inertiaProps.getMass_wrist_1_link() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_wrist_1_link()));
    
    tmpX = tmpX * ht.fr_wrist_1_link_X_fr_wrist_2_link;
    tmpSum += inertiaProps.getMass_wrist_2_link() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_wrist_2_link()));
    
    tmpX = tmpX * ht.fr_wrist_2_link_X_fr_wrist_3_link;
    tmpSum += inertiaProps.getMass_wrist_3_link() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_wrist_3_link()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

iit::rbd::Vector3d iit::ur5e::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_base_link_ur5e_X_fr_shoulder_link(q);
    ht.fr_shoulder_link_X_fr_upper_arm_link(q);
    ht.fr_upper_arm_link_X_fr_forearm_link(q);
    ht.fr_forearm_link_X_fr_wrist_1_link(q);
    ht.fr_wrist_1_link_X_fr_wrist_2_link(q);
    ht.fr_wrist_2_link_X_fr_wrist_3_link(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
