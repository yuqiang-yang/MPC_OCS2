#include "transforms.h"

using namespace ur5e::rcg;

// Constructors

MotionTransforms::MotionTransforms()
 :     fr_base_link_ur5e_X_fr_forearm_link(),
    fr_forearm_link_X_fr_base_link_ur5e(),
    fr_base_link_ur5e_X_fr_shoulder_link(),
    fr_shoulder_link_X_fr_base_link_ur5e(),
    fr_base_link_ur5e_X_fr_upper_arm_link(),
    fr_upper_arm_link_X_fr_base_link_ur5e(),
    fr_base_link_ur5e_X_fr_wrist_1_link(),
    fr_wrist_1_link_X_fr_base_link_ur5e(),
    fr_base_link_ur5e_X_fr_wrist_2_link(),
    fr_wrist_2_link_X_fr_base_link_ur5e(),
    fr_base_link_ur5e_X_fr_wrist_3_link(),
    fr_wrist_3_link_X_fr_base_link_ur5e(),
    fr_upper_arm_link_X_fr_shoulder_link(),
    fr_shoulder_link_X_fr_upper_arm_link(),
    fr_forearm_link_X_fr_upper_arm_link(),
    fr_upper_arm_link_X_fr_forearm_link(),
    fr_wrist_1_link_X_fr_forearm_link(),
    fr_forearm_link_X_fr_wrist_1_link(),
    fr_wrist_2_link_X_fr_wrist_1_link(),
    fr_wrist_1_link_X_fr_wrist_2_link(),
    fr_wrist_3_link_X_fr_wrist_2_link(),
    fr_wrist_2_link_X_fr_wrist_3_link()
{}
void MotionTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

ForceTransforms::ForceTransforms()
 :     fr_base_link_ur5e_X_fr_forearm_link(),
    fr_forearm_link_X_fr_base_link_ur5e(),
    fr_base_link_ur5e_X_fr_shoulder_link(),
    fr_shoulder_link_X_fr_base_link_ur5e(),
    fr_base_link_ur5e_X_fr_upper_arm_link(),
    fr_upper_arm_link_X_fr_base_link_ur5e(),
    fr_base_link_ur5e_X_fr_wrist_1_link(),
    fr_wrist_1_link_X_fr_base_link_ur5e(),
    fr_base_link_ur5e_X_fr_wrist_2_link(),
    fr_wrist_2_link_X_fr_base_link_ur5e(),
    fr_base_link_ur5e_X_fr_wrist_3_link(),
    fr_wrist_3_link_X_fr_base_link_ur5e(),
    fr_upper_arm_link_X_fr_shoulder_link(),
    fr_shoulder_link_X_fr_upper_arm_link(),
    fr_forearm_link_X_fr_upper_arm_link(),
    fr_upper_arm_link_X_fr_forearm_link(),
    fr_wrist_1_link_X_fr_forearm_link(),
    fr_forearm_link_X_fr_wrist_1_link(),
    fr_wrist_2_link_X_fr_wrist_1_link(),
    fr_wrist_1_link_X_fr_wrist_2_link(),
    fr_wrist_3_link_X_fr_wrist_2_link(),
    fr_wrist_2_link_X_fr_wrist_3_link()
{}
void ForceTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

HomogeneousTransforms::HomogeneousTransforms()
 :     fr_base_link_ur5e_X_fr_forearm_link(),
    fr_forearm_link_X_fr_base_link_ur5e(),
    fr_base_link_ur5e_X_fr_shoulder_link(),
    fr_shoulder_link_X_fr_base_link_ur5e(),
    fr_base_link_ur5e_X_fr_upper_arm_link(),
    fr_upper_arm_link_X_fr_base_link_ur5e(),
    fr_base_link_ur5e_X_fr_wrist_1_link(),
    fr_wrist_1_link_X_fr_base_link_ur5e(),
    fr_base_link_ur5e_X_fr_wrist_2_link(),
    fr_wrist_2_link_X_fr_base_link_ur5e(),
    fr_base_link_ur5e_X_fr_wrist_3_link(),
    fr_wrist_3_link_X_fr_base_link_ur5e(),
    fr_upper_arm_link_X_fr_shoulder_link(),
    fr_shoulder_link_X_fr_upper_arm_link(),
    fr_forearm_link_X_fr_upper_arm_link(),
    fr_upper_arm_link_X_fr_forearm_link(),
    fr_wrist_1_link_X_fr_forearm_link(),
    fr_forearm_link_X_fr_wrist_1_link(),
    fr_wrist_2_link_X_fr_wrist_1_link(),
    fr_wrist_1_link_X_fr_wrist_2_link(),
    fr_wrist_3_link_X_fr_wrist_2_link(),
    fr_wrist_2_link_X_fr_wrist_3_link()
{}
void HomogeneousTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

MotionTransforms::Type_fr_base_link_ur5e_X_fr_forearm_link::Type_fr_base_link_ur5e_X_fr_forearm_link()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_link_ur5e_X_fr_forearm_link& MotionTransforms::Type_fr_base_link_ur5e_X_fr_forearm_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    (*this)(0,0) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(0,1) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(0,2) = -sin_q_shoulder_pan_joint;
    (*this)(1,0) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(1,1) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(1,2) = cos_q_shoulder_pan_joint;
    (*this)(2,0) = (-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(2,1) = (sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(3,0) = ((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)) * sin_q_shoulder_pan_joint)+((((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint);
    (*this)(3,1) = ((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)) * sin_q_shoulder_pan_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint);
    (*this)(3,2) = (( tx_elbow_joint * sin_q_shoulder_lift_joint)+( ty_elbow_joint * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint;
    (*this)(3,3) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(3,4) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(3,5) = -sin_q_shoulder_pan_joint;
    (*this)(4,0) = ((((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)) * cos_q_shoulder_pan_joint);
    (*this)(4,1) = (((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)) * cos_q_shoulder_pan_joint);
    (*this)(4,2) = (( tx_elbow_joint * sin_q_shoulder_lift_joint)+( ty_elbow_joint * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint;
    (*this)(4,3) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(4,4) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(4,5) = cos_q_shoulder_pan_joint;
    (*this)(5,0) = (( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(5,1) = (( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(5,2) = ( tx_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_shoulder_lift_joint);
    (*this)(5,3) = (-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(5,4) = (sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint);
    return *this;
}
MotionTransforms::Type_fr_forearm_link_X_fr_base_link_ur5e::Type_fr_forearm_link_X_fr_base_link_ur5e()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_forearm_link_X_fr_base_link_ur5e& MotionTransforms::Type_fr_forearm_link_X_fr_base_link_ur5e::update(const state_t& q)
{
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(0,1) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(0,2) = (-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(1,0) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(1,1) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(1,2) = (sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(2,0) = -sin_q_shoulder_pan_joint;
    (*this)(2,1) = cos_q_shoulder_pan_joint;
    (*this)(3,0) = ((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)) * sin_q_shoulder_pan_joint)+((((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint);
    (*this)(3,1) = ((((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)) * cos_q_shoulder_pan_joint);
    (*this)(3,2) = (( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(3,3) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(3,4) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(3,5) = (-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(4,0) = ((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)) * sin_q_shoulder_pan_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint);
    (*this)(4,1) = (((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)) * cos_q_shoulder_pan_joint);
    (*this)(4,2) = (( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(4,3) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(4,4) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(4,5) = (sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(5,0) = (( tx_elbow_joint * sin_q_shoulder_lift_joint)+( ty_elbow_joint * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint;
    (*this)(5,1) = (( tx_elbow_joint * sin_q_shoulder_lift_joint)+( ty_elbow_joint * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint;
    (*this)(5,2) = ( tx_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_shoulder_lift_joint);
    (*this)(5,3) = -sin_q_shoulder_pan_joint;
    (*this)(5,4) = cos_q_shoulder_pan_joint;
    return *this;
}
MotionTransforms::Type_fr_base_link_ur5e_X_fr_shoulder_link::Type_fr_base_link_ur5e_X_fr_shoulder_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_link_ur5e_X_fr_shoulder_link& MotionTransforms::Type_fr_base_link_ur5e_X_fr_shoulder_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = cos_q_shoulder_pan_joint;
    (*this)(0,1) = -sin_q_shoulder_pan_joint;
    (*this)(1,0) = sin_q_shoulder_pan_joint;
    (*this)(1,1) = cos_q_shoulder_pan_joint;
    (*this)(3,0) = - tz_shoulder_pan_joint * sin_q_shoulder_pan_joint;
    (*this)(3,1) = - tz_shoulder_pan_joint * cos_q_shoulder_pan_joint;
    (*this)(3,3) = cos_q_shoulder_pan_joint;
    (*this)(3,4) = -sin_q_shoulder_pan_joint;
    (*this)(4,0) =  tz_shoulder_pan_joint * cos_q_shoulder_pan_joint;
    (*this)(4,1) = - tz_shoulder_pan_joint * sin_q_shoulder_pan_joint;
    (*this)(4,3) = sin_q_shoulder_pan_joint;
    (*this)(4,4) = cos_q_shoulder_pan_joint;
    return *this;
}
MotionTransforms::Type_fr_shoulder_link_X_fr_base_link_ur5e::Type_fr_shoulder_link_X_fr_base_link_ur5e()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_shoulder_link_X_fr_base_link_ur5e& MotionTransforms::Type_fr_shoulder_link_X_fr_base_link_ur5e::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = cos_q_shoulder_pan_joint;
    (*this)(0,1) = sin_q_shoulder_pan_joint;
    (*this)(1,0) = -sin_q_shoulder_pan_joint;
    (*this)(1,1) = cos_q_shoulder_pan_joint;
    (*this)(3,0) = - tz_shoulder_pan_joint * sin_q_shoulder_pan_joint;
    (*this)(3,1) =  tz_shoulder_pan_joint * cos_q_shoulder_pan_joint;
    (*this)(3,3) = cos_q_shoulder_pan_joint;
    (*this)(3,4) = sin_q_shoulder_pan_joint;
    (*this)(4,0) = - tz_shoulder_pan_joint * cos_q_shoulder_pan_joint;
    (*this)(4,1) = - tz_shoulder_pan_joint * sin_q_shoulder_pan_joint;
    (*this)(4,3) = -sin_q_shoulder_pan_joint;
    (*this)(4,4) = cos_q_shoulder_pan_joint;
    return *this;
}
MotionTransforms::Type_fr_base_link_ur5e_X_fr_upper_arm_link::Type_fr_base_link_ur5e_X_fr_upper_arm_link()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_link_ur5e_X_fr_upper_arm_link& MotionTransforms::Type_fr_base_link_ur5e_X_fr_upper_arm_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    (*this)(0,0) = cos_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(0,1) = -sin_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(0,2) = -sin_q_shoulder_pan_joint;
    (*this)(1,0) = cos_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(1,1) = -sin_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(1,2) = cos_q_shoulder_pan_joint;
    (*this)(2,0) = -sin_q_shoulder_lift_joint;
    (*this)(2,1) = -cos_q_shoulder_lift_joint;
    (*this)(3,0) = (- tz_shoulder_pan_joint * cos_q_shoulder_lift_joint * sin_q_shoulder_pan_joint)-( ty_shoulder_lift_joint * sin_q_shoulder_lift_joint * cos_q_shoulder_pan_joint);
    (*this)(3,1) = ( tz_shoulder_pan_joint * sin_q_shoulder_lift_joint * sin_q_shoulder_pan_joint)-( ty_shoulder_lift_joint * cos_q_shoulder_lift_joint * cos_q_shoulder_pan_joint);
    (*this)(3,2) = - tz_shoulder_pan_joint * cos_q_shoulder_pan_joint;
    (*this)(3,3) = cos_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(3,4) = -sin_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(3,5) = -sin_q_shoulder_pan_joint;
    (*this)(4,0) = ( tz_shoulder_pan_joint * cos_q_shoulder_lift_joint * cos_q_shoulder_pan_joint)-( ty_shoulder_lift_joint * sin_q_shoulder_lift_joint * sin_q_shoulder_pan_joint);
    (*this)(4,1) = (- ty_shoulder_lift_joint * cos_q_shoulder_lift_joint * sin_q_shoulder_pan_joint)-( tz_shoulder_pan_joint * sin_q_shoulder_lift_joint * cos_q_shoulder_pan_joint);
    (*this)(4,2) = - tz_shoulder_pan_joint * sin_q_shoulder_pan_joint;
    (*this)(4,3) = cos_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(4,4) = -sin_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(4,5) = cos_q_shoulder_pan_joint;
    (*this)(5,0) = - ty_shoulder_lift_joint * cos_q_shoulder_lift_joint;
    (*this)(5,1) =  ty_shoulder_lift_joint * sin_q_shoulder_lift_joint;
    (*this)(5,3) = -sin_q_shoulder_lift_joint;
    (*this)(5,4) = -cos_q_shoulder_lift_joint;
    return *this;
}
MotionTransforms::Type_fr_upper_arm_link_X_fr_base_link_ur5e::Type_fr_upper_arm_link_X_fr_base_link_ur5e()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_upper_arm_link_X_fr_base_link_ur5e& MotionTransforms::Type_fr_upper_arm_link_X_fr_base_link_ur5e::update(const state_t& q)
{
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = cos_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(0,1) = cos_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(0,2) = -sin_q_shoulder_lift_joint;
    (*this)(1,0) = -sin_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(1,1) = -sin_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(1,2) = -cos_q_shoulder_lift_joint;
    (*this)(2,0) = -sin_q_shoulder_pan_joint;
    (*this)(2,1) = cos_q_shoulder_pan_joint;
    (*this)(3,0) = (- tz_shoulder_pan_joint * cos_q_shoulder_lift_joint * sin_q_shoulder_pan_joint)-( ty_shoulder_lift_joint * sin_q_shoulder_lift_joint * cos_q_shoulder_pan_joint);
    (*this)(3,1) = ( tz_shoulder_pan_joint * cos_q_shoulder_lift_joint * cos_q_shoulder_pan_joint)-( ty_shoulder_lift_joint * sin_q_shoulder_lift_joint * sin_q_shoulder_pan_joint);
    (*this)(3,2) = - ty_shoulder_lift_joint * cos_q_shoulder_lift_joint;
    (*this)(3,3) = cos_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(3,4) = cos_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(3,5) = -sin_q_shoulder_lift_joint;
    (*this)(4,0) = ( tz_shoulder_pan_joint * sin_q_shoulder_lift_joint * sin_q_shoulder_pan_joint)-( ty_shoulder_lift_joint * cos_q_shoulder_lift_joint * cos_q_shoulder_pan_joint);
    (*this)(4,1) = (- ty_shoulder_lift_joint * cos_q_shoulder_lift_joint * sin_q_shoulder_pan_joint)-( tz_shoulder_pan_joint * sin_q_shoulder_lift_joint * cos_q_shoulder_pan_joint);
    (*this)(4,2) =  ty_shoulder_lift_joint * sin_q_shoulder_lift_joint;
    (*this)(4,3) = -sin_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(4,4) = -sin_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(4,5) = -cos_q_shoulder_lift_joint;
    (*this)(5,0) = - tz_shoulder_pan_joint * cos_q_shoulder_pan_joint;
    (*this)(5,1) = - tz_shoulder_pan_joint * sin_q_shoulder_pan_joint;
    (*this)(5,3) = -sin_q_shoulder_pan_joint;
    (*this)(5,4) = cos_q_shoulder_pan_joint;
    return *this;
}
MotionTransforms::Type_fr_base_link_ur5e_X_fr_wrist_1_link::Type_fr_base_link_ur5e_X_fr_wrist_1_link()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_link_ur5e_X_fr_wrist_1_link& MotionTransforms::Type_fr_base_link_ur5e_X_fr_wrist_1_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    (*this)(0,0) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(0,1) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(0,2) = -sin_q_shoulder_pan_joint;
    (*this)(1,0) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(1,1) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(1,2) = cos_q_shoulder_pan_joint;
    (*this)(2,0) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(2,1) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(3,0) = ((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(3,1) = (((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(3,2) = ((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint;
    (*this)(3,3) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(3,4) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(3,5) = -sin_q_shoulder_pan_joint;
    (*this)(4,0) = (((((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(4,1) = (((((( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(4,2) = ((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint;
    (*this)(4,3) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(4,4) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(4,5) = cos_q_shoulder_pan_joint;
    (*this)(5,0) = (((( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(5,1) = ((((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(5,2) = (((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint);
    (*this)(5,3) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(5,4) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    return *this;
}
MotionTransforms::Type_fr_wrist_1_link_X_fr_base_link_ur5e::Type_fr_wrist_1_link_X_fr_base_link_ur5e()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_wrist_1_link_X_fr_base_link_ur5e& MotionTransforms::Type_fr_wrist_1_link_X_fr_base_link_ur5e::update(const state_t& q)
{
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(0,1) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(0,2) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(1,0) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(1,1) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(1,2) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(2,0) = -sin_q_shoulder_pan_joint;
    (*this)(2,1) = cos_q_shoulder_pan_joint;
    (*this)(3,0) = ((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(3,1) = (((((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(3,2) = (((( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(3,3) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(3,4) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(3,5) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(4,0) = (((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(4,1) = (((((( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(4,2) = ((((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(4,3) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(4,4) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(4,5) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(5,0) = ((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint;
    (*this)(5,1) = ((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint;
    (*this)(5,2) = (((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint);
    (*this)(5,3) = -sin_q_shoulder_pan_joint;
    (*this)(5,4) = cos_q_shoulder_pan_joint;
    return *this;
}
MotionTransforms::Type_fr_base_link_ur5e_X_fr_wrist_2_link::Type_fr_base_link_ur5e_X_fr_wrist_2_link()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
}

const MotionTransforms::Type_fr_base_link_ur5e_X_fr_wrist_2_link& MotionTransforms::Type_fr_base_link_ur5e_X_fr_wrist_2_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    Scalar sin_q_wrist_2_joint  = ScalarTraits::sin( q(WRIST_2_JOINT) );
    Scalar cos_q_wrist_2_joint  = ScalarTraits::cos( q(WRIST_2_JOINT) );
    (*this)(0,0) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint);
    (*this)(0,1) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(0,2) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(1,0) = (cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(1,1) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(1,2) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(2,0) = ((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint;
    (*this)(2,1) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    (*this)(2,2) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(3,0) = (((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(3,1) = ((((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(3,2) = (((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(3,3) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint);
    (*this)(3,4) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(3,5) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(4,0) = (((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)+((((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(4,1) = (((((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(4,2) = (((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(4,3) = (cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(4,4) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(4,5) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(5,0) = (((((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * sin_q_wrist_2_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(5,1) = (((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(((((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * cos_q_wrist_2_joint);
    (*this)(5,2) = ((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(5,3) = ((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint;
    (*this)(5,4) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    (*this)(5,5) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    return *this;
}
MotionTransforms::Type_fr_wrist_2_link_X_fr_base_link_ur5e::Type_fr_wrist_2_link_X_fr_base_link_ur5e()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
}

const MotionTransforms::Type_fr_wrist_2_link_X_fr_base_link_ur5e& MotionTransforms::Type_fr_wrist_2_link_X_fr_base_link_ur5e::update(const state_t& q)
{
    Scalar sin_q_wrist_2_joint  = ScalarTraits::sin( q(WRIST_2_JOINT) );
    Scalar cos_q_wrist_2_joint  = ScalarTraits::cos( q(WRIST_2_JOINT) );
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint);
    (*this)(0,1) = (cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(0,2) = ((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint;
    (*this)(1,0) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(1,1) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(1,2) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    (*this)(2,0) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(2,1) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(2,2) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(3,0) = (((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(3,1) = (((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)+((((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(3,2) = (((((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * sin_q_wrist_2_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(3,3) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint);
    (*this)(3,4) = (cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(3,5) = ((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint;
    (*this)(4,0) = ((((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(4,1) = (((((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(4,2) = (((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(((((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * cos_q_wrist_2_joint);
    (*this)(4,3) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(4,4) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(4,5) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    (*this)(5,0) = (((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(5,1) = (((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(5,2) = ((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(5,3) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(5,4) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(5,5) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    return *this;
}
MotionTransforms::Type_fr_base_link_ur5e_X_fr_wrist_3_link::Type_fr_base_link_ur5e_X_fr_wrist_3_link()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
}

const MotionTransforms::Type_fr_base_link_ur5e_X_fr_wrist_3_link& MotionTransforms::Type_fr_base_link_ur5e_X_fr_wrist_3_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    Scalar sin_q_wrist_2_joint  = ScalarTraits::sin( q(WRIST_2_JOINT) );
    Scalar cos_q_wrist_2_joint  = ScalarTraits::cos( q(WRIST_2_JOINT) );
    Scalar sin_q_wrist_3_joint  = ScalarTraits::sin( q(WRIST_3_JOINT) );
    Scalar cos_q_wrist_3_joint  = ScalarTraits::cos( q(WRIST_3_JOINT) );
    (*this)(0,0) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(0,1) = (((((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(0,2) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(1,0) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(1,1) = (((cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(1,2) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(2,0) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * cos_q_wrist_3_joint);
    (*this)(2,1) = (((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * sin_q_wrist_3_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(2,2) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    (*this)(3,0) = ((((((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+( tx_wrist_3_joint * sin_q_shoulder_pan_joint * cos_q_wrist_2_joint)+(((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+((((((( tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)+((((( ty_wrist_1_joint * sin_q_elbow_joint)-( tx_wrist_1_joint * cos_q_elbow_joint)- tx_elbow_joint) * sin_q_shoulder_lift_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * cos_q_shoulder_lift_joint)+ tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+((((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)+( tz_wrist_3_joint * sin_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(3,1) = (((((((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+(((((((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)-( tz_wrist_3_joint * sin_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+((((((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+( tx_wrist_3_joint * sin_q_shoulder_pan_joint * cos_q_wrist_2_joint)+(((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(3,2) = ((((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)+( tz_wrist_3_joint * sin_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+(((((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((- tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(3,3) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(3,4) = (((((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(3,5) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(4,0) = ((((((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-( tx_wrist_3_joint * cos_q_shoulder_pan_joint * cos_q_wrist_2_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+((((((( tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)+((((( ty_wrist_1_joint * sin_q_elbow_joint)-( tx_wrist_1_joint * cos_q_elbow_joint)- tx_elbow_joint) * sin_q_shoulder_lift_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * cos_q_shoulder_lift_joint)+ tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+(((((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)-( tz_wrist_3_joint * cos_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(4,1) = (((((((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+((((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)+( tz_wrist_3_joint * cos_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+((((((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-( tx_wrist_3_joint * cos_q_shoulder_pan_joint * cos_q_wrist_2_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(4,2) = (((((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)-( tz_wrist_3_joint * cos_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+(((((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((- tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(4,3) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(4,4) = (((cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(4,5) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(5,0) = ((((((( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+((((((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * sin_q_shoulder_lift_joint)+((( ty_wrist_1_joint * sin_q_elbow_joint)-( tx_wrist_1_joint * cos_q_elbow_joint)- tx_elbow_joint) * cos_q_shoulder_lift_joint)) * sin_q_wrist_2_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(5,1) = ((((((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * sin_q_wrist_2_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+((((((( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(5,2) = (((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+((((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * cos_q_wrist_2_joint)+((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(5,3) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * cos_q_wrist_3_joint);
    (*this)(5,4) = (((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * sin_q_wrist_3_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(5,5) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    return *this;
}
MotionTransforms::Type_fr_wrist_3_link_X_fr_base_link_ur5e::Type_fr_wrist_3_link_X_fr_base_link_ur5e()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
}

const MotionTransforms::Type_fr_wrist_3_link_X_fr_base_link_ur5e& MotionTransforms::Type_fr_wrist_3_link_X_fr_base_link_ur5e::update(const state_t& q)
{
    Scalar sin_q_wrist_3_joint  = ScalarTraits::sin( q(WRIST_3_JOINT) );
    Scalar cos_q_wrist_3_joint  = ScalarTraits::cos( q(WRIST_3_JOINT) );
    Scalar sin_q_wrist_2_joint  = ScalarTraits::sin( q(WRIST_2_JOINT) );
    Scalar cos_q_wrist_2_joint  = ScalarTraits::cos( q(WRIST_2_JOINT) );
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(0,1) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(0,2) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * cos_q_wrist_3_joint);
    (*this)(1,0) = (((((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(1,1) = (((cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(1,2) = (((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * sin_q_wrist_3_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(2,0) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(2,1) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(2,2) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    (*this)(3,0) = ((((((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+( tx_wrist_3_joint * sin_q_shoulder_pan_joint * cos_q_wrist_2_joint)+(((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+((((((( tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)+((((( ty_wrist_1_joint * sin_q_elbow_joint)-( tx_wrist_1_joint * cos_q_elbow_joint)- tx_elbow_joint) * sin_q_shoulder_lift_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * cos_q_shoulder_lift_joint)+ tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+((((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)+( tz_wrist_3_joint * sin_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(3,1) = ((((((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-( tx_wrist_3_joint * cos_q_shoulder_pan_joint * cos_q_wrist_2_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+((((((( tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)+((((( ty_wrist_1_joint * sin_q_elbow_joint)-( tx_wrist_1_joint * cos_q_elbow_joint)- tx_elbow_joint) * sin_q_shoulder_lift_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * cos_q_shoulder_lift_joint)+ tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+(((((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)-( tz_wrist_3_joint * cos_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(3,2) = ((((((( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+((((((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * sin_q_shoulder_lift_joint)+((( ty_wrist_1_joint * sin_q_elbow_joint)-( tx_wrist_1_joint * cos_q_elbow_joint)- tx_elbow_joint) * cos_q_shoulder_lift_joint)) * sin_q_wrist_2_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(3,3) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(3,4) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(3,5) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * cos_q_wrist_3_joint);
    (*this)(4,0) = (((((((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+(((((((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)-( tz_wrist_3_joint * sin_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+((((((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+( tx_wrist_3_joint * sin_q_shoulder_pan_joint * cos_q_wrist_2_joint)+(((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(4,1) = (((((((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+((((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)+( tz_wrist_3_joint * cos_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+((((((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-( tx_wrist_3_joint * cos_q_shoulder_pan_joint * cos_q_wrist_2_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(4,2) = ((((((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * sin_q_wrist_2_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+((((((( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(4,3) = (((((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(4,4) = (((cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(4,5) = (((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * sin_q_wrist_3_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(5,0) = ((((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)+( tz_wrist_3_joint * sin_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+(((((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((- tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(5,1) = (((((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)-( tz_wrist_3_joint * cos_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+(((((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((- tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(5,2) = (((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+((((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * cos_q_wrist_2_joint)+((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(5,3) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(5,4) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(5,5) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    return *this;
}
MotionTransforms::Type_fr_upper_arm_link_X_fr_shoulder_link::Type_fr_upper_arm_link_X_fr_shoulder_link()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_upper_arm_link_X_fr_shoulder_link& MotionTransforms::Type_fr_upper_arm_link_X_fr_shoulder_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    (*this)(0,0) = cos_q_shoulder_lift_joint;
    (*this)(0,2) = -sin_q_shoulder_lift_joint;
    (*this)(1,0) = -sin_q_shoulder_lift_joint;
    (*this)(1,2) = -cos_q_shoulder_lift_joint;
    (*this)(3,0) = - ty_shoulder_lift_joint * sin_q_shoulder_lift_joint;
    (*this)(3,2) = - ty_shoulder_lift_joint * cos_q_shoulder_lift_joint;
    (*this)(3,3) = cos_q_shoulder_lift_joint;
    (*this)(3,5) = -sin_q_shoulder_lift_joint;
    (*this)(4,0) = - ty_shoulder_lift_joint * cos_q_shoulder_lift_joint;
    (*this)(4,2) =  ty_shoulder_lift_joint * sin_q_shoulder_lift_joint;
    (*this)(4,3) = -sin_q_shoulder_lift_joint;
    (*this)(4,5) = -cos_q_shoulder_lift_joint;
    return *this;
}
MotionTransforms::Type_fr_shoulder_link_X_fr_upper_arm_link::Type_fr_shoulder_link_X_fr_upper_arm_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_shoulder_link_X_fr_upper_arm_link& MotionTransforms::Type_fr_shoulder_link_X_fr_upper_arm_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    (*this)(0,0) = cos_q_shoulder_lift_joint;
    (*this)(0,1) = -sin_q_shoulder_lift_joint;
    (*this)(2,0) = -sin_q_shoulder_lift_joint;
    (*this)(2,1) = -cos_q_shoulder_lift_joint;
    (*this)(3,0) = - ty_shoulder_lift_joint * sin_q_shoulder_lift_joint;
    (*this)(3,1) = - ty_shoulder_lift_joint * cos_q_shoulder_lift_joint;
    (*this)(3,3) = cos_q_shoulder_lift_joint;
    (*this)(3,4) = -sin_q_shoulder_lift_joint;
    (*this)(5,0) = - ty_shoulder_lift_joint * cos_q_shoulder_lift_joint;
    (*this)(5,1) =  ty_shoulder_lift_joint * sin_q_shoulder_lift_joint;
    (*this)(5,3) = -sin_q_shoulder_lift_joint;
    (*this)(5,4) = -cos_q_shoulder_lift_joint;
    return *this;
}
MotionTransforms::Type_fr_forearm_link_X_fr_upper_arm_link::Type_fr_forearm_link_X_fr_upper_arm_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) =  ty_elbow_joint;    // Maxima DSL: _k__ty_elbow_joint
    (*this)(5,1) = - tx_elbow_joint;    // Maxima DSL: -_k__tx_elbow_joint
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_forearm_link_X_fr_upper_arm_link& MotionTransforms::Type_fr_forearm_link_X_fr_upper_arm_link::update(const state_t& q)
{
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    (*this)(0,0) = cos_q_elbow_joint;
    (*this)(0,1) = sin_q_elbow_joint;
    (*this)(1,0) = -sin_q_elbow_joint;
    (*this)(1,1) = cos_q_elbow_joint;
    (*this)(3,0) = - tz_elbow_joint * sin_q_elbow_joint;
    (*this)(3,1) =  tz_elbow_joint * cos_q_elbow_joint;
    (*this)(3,2) = ( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint);
    (*this)(3,3) = cos_q_elbow_joint;
    (*this)(3,4) = sin_q_elbow_joint;
    (*this)(4,0) = - tz_elbow_joint * cos_q_elbow_joint;
    (*this)(4,1) = - tz_elbow_joint * sin_q_elbow_joint;
    (*this)(4,2) = ( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint);
    (*this)(4,3) = -sin_q_elbow_joint;
    (*this)(4,4) = cos_q_elbow_joint;
    return *this;
}
MotionTransforms::Type_fr_upper_arm_link_X_fr_forearm_link::Type_fr_upper_arm_link_X_fr_forearm_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) =  ty_elbow_joint;    // Maxima DSL: _k__ty_elbow_joint
    (*this)(3,5) = 0.0;
    (*this)(4,2) = - tx_elbow_joint;    // Maxima DSL: -_k__tx_elbow_joint
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_upper_arm_link_X_fr_forearm_link& MotionTransforms::Type_fr_upper_arm_link_X_fr_forearm_link::update(const state_t& q)
{
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    (*this)(0,0) = cos_q_elbow_joint;
    (*this)(0,1) = -sin_q_elbow_joint;
    (*this)(1,0) = sin_q_elbow_joint;
    (*this)(1,1) = cos_q_elbow_joint;
    (*this)(3,0) = - tz_elbow_joint * sin_q_elbow_joint;
    (*this)(3,1) = - tz_elbow_joint * cos_q_elbow_joint;
    (*this)(3,3) = cos_q_elbow_joint;
    (*this)(3,4) = -sin_q_elbow_joint;
    (*this)(4,0) =  tz_elbow_joint * cos_q_elbow_joint;
    (*this)(4,1) = - tz_elbow_joint * sin_q_elbow_joint;
    (*this)(4,3) = sin_q_elbow_joint;
    (*this)(4,4) = cos_q_elbow_joint;
    (*this)(5,0) = ( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint);
    (*this)(5,1) = ( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint);
    return *this;
}
MotionTransforms::Type_fr_wrist_1_link_X_fr_forearm_link::Type_fr_wrist_1_link_X_fr_forearm_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) =  ty_wrist_1_joint;    // Maxima DSL: _k__ty_wrist_1_joint
    (*this)(5,1) = - tx_wrist_1_joint;    // Maxima DSL: -_k__tx_wrist_1_joint
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_wrist_1_link_X_fr_forearm_link& MotionTransforms::Type_fr_wrist_1_link_X_fr_forearm_link::update(const state_t& q)
{
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    (*this)(0,0) = cos_q_wrist_1_joint;
    (*this)(0,1) = sin_q_wrist_1_joint;
    (*this)(1,0) = -sin_q_wrist_1_joint;
    (*this)(1,1) = cos_q_wrist_1_joint;
    (*this)(3,2) = ( tx_wrist_1_joint * sin_q_wrist_1_joint)-( ty_wrist_1_joint * cos_q_wrist_1_joint);
    (*this)(3,3) = cos_q_wrist_1_joint;
    (*this)(3,4) = sin_q_wrist_1_joint;
    (*this)(4,2) = ( ty_wrist_1_joint * sin_q_wrist_1_joint)+( tx_wrist_1_joint * cos_q_wrist_1_joint);
    (*this)(4,3) = -sin_q_wrist_1_joint;
    (*this)(4,4) = cos_q_wrist_1_joint;
    return *this;
}
MotionTransforms::Type_fr_forearm_link_X_fr_wrist_1_link::Type_fr_forearm_link_X_fr_wrist_1_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) =  ty_wrist_1_joint;    // Maxima DSL: _k__ty_wrist_1_joint
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = - tx_wrist_1_joint;    // Maxima DSL: -_k__tx_wrist_1_joint
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_forearm_link_X_fr_wrist_1_link& MotionTransforms::Type_fr_forearm_link_X_fr_wrist_1_link::update(const state_t& q)
{
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    (*this)(0,0) = cos_q_wrist_1_joint;
    (*this)(0,1) = -sin_q_wrist_1_joint;
    (*this)(1,0) = sin_q_wrist_1_joint;
    (*this)(1,1) = cos_q_wrist_1_joint;
    (*this)(3,3) = cos_q_wrist_1_joint;
    (*this)(3,4) = -sin_q_wrist_1_joint;
    (*this)(4,3) = sin_q_wrist_1_joint;
    (*this)(4,4) = cos_q_wrist_1_joint;
    (*this)(5,0) = ( tx_wrist_1_joint * sin_q_wrist_1_joint)-( ty_wrist_1_joint * cos_q_wrist_1_joint);
    (*this)(5,1) = ( ty_wrist_1_joint * sin_q_wrist_1_joint)+( tx_wrist_1_joint * cos_q_wrist_1_joint);
    return *this;
}
MotionTransforms::Type_fr_wrist_2_link_X_fr_wrist_1_link::Type_fr_wrist_2_link_X_fr_wrist_1_link()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = - tz_wrist_2_joint;    // Maxima DSL: -_k__tz_wrist_2_joint
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_wrist_2_link_X_fr_wrist_1_link& MotionTransforms::Type_fr_wrist_2_link_X_fr_wrist_1_link::update(const state_t& q)
{
    Scalar sin_q_wrist_2_joint  = ScalarTraits::sin( q(WRIST_2_JOINT) );
    Scalar cos_q_wrist_2_joint  = ScalarTraits::cos( q(WRIST_2_JOINT) );
    (*this)(0,0) = -cos_q_wrist_2_joint;
    (*this)(0,2) = sin_q_wrist_2_joint;
    (*this)(1,0) = sin_q_wrist_2_joint;
    (*this)(1,2) = cos_q_wrist_2_joint;
    (*this)(3,1) = - tz_wrist_2_joint * cos_q_wrist_2_joint;
    (*this)(3,3) = -cos_q_wrist_2_joint;
    (*this)(3,5) = sin_q_wrist_2_joint;
    (*this)(4,1) =  tz_wrist_2_joint * sin_q_wrist_2_joint;
    (*this)(4,3) = sin_q_wrist_2_joint;
    (*this)(4,5) = cos_q_wrist_2_joint;
    return *this;
}
MotionTransforms::Type_fr_wrist_1_link_X_fr_wrist_2_link::Type_fr_wrist_1_link_X_fr_wrist_2_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = - tz_wrist_2_joint;    // Maxima DSL: -_k__tz_wrist_2_joint
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_wrist_1_link_X_fr_wrist_2_link& MotionTransforms::Type_fr_wrist_1_link_X_fr_wrist_2_link::update(const state_t& q)
{
    Scalar sin_q_wrist_2_joint  = ScalarTraits::sin( q(WRIST_2_JOINT) );
    Scalar cos_q_wrist_2_joint  = ScalarTraits::cos( q(WRIST_2_JOINT) );
    (*this)(0,0) = -cos_q_wrist_2_joint;
    (*this)(0,1) = sin_q_wrist_2_joint;
    (*this)(2,0) = sin_q_wrist_2_joint;
    (*this)(2,1) = cos_q_wrist_2_joint;
    (*this)(3,3) = -cos_q_wrist_2_joint;
    (*this)(3,4) = sin_q_wrist_2_joint;
    (*this)(4,0) = - tz_wrist_2_joint * cos_q_wrist_2_joint;
    (*this)(4,1) =  tz_wrist_2_joint * sin_q_wrist_2_joint;
    (*this)(5,3) = sin_q_wrist_2_joint;
    (*this)(5,4) = cos_q_wrist_2_joint;
    return *this;
}
MotionTransforms::Type_fr_wrist_3_link_X_fr_wrist_2_link::Type_fr_wrist_3_link_X_fr_wrist_2_link()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = - tz_wrist_3_joint;    // Maxima DSL: -_k__tz_wrist_3_joint
    (*this)(5,1) = 0.0;
    (*this)(5,2) =  tx_wrist_3_joint;    // Maxima DSL: _k__tx_wrist_3_joint
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_wrist_3_link_X_fr_wrist_2_link& MotionTransforms::Type_fr_wrist_3_link_X_fr_wrist_2_link::update(const state_t& q)
{
    Scalar sin_q_wrist_3_joint  = ScalarTraits::sin( q(WRIST_3_JOINT) );
    Scalar cos_q_wrist_3_joint  = ScalarTraits::cos( q(WRIST_3_JOINT) );
    (*this)(0,0) = -cos_q_wrist_3_joint;
    (*this)(0,2) = sin_q_wrist_3_joint;
    (*this)(1,0) = sin_q_wrist_3_joint;
    (*this)(1,2) = cos_q_wrist_3_joint;
    (*this)(3,1) = (- tx_wrist_3_joint * sin_q_wrist_3_joint)-( tz_wrist_3_joint * cos_q_wrist_3_joint);
    (*this)(3,3) = -cos_q_wrist_3_joint;
    (*this)(3,5) = sin_q_wrist_3_joint;
    (*this)(4,1) = ( tz_wrist_3_joint * sin_q_wrist_3_joint)-( tx_wrist_3_joint * cos_q_wrist_3_joint);
    (*this)(4,3) = sin_q_wrist_3_joint;
    (*this)(4,5) = cos_q_wrist_3_joint;
    return *this;
}
MotionTransforms::Type_fr_wrist_2_link_X_fr_wrist_3_link::Type_fr_wrist_2_link_X_fr_wrist_3_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = - tz_wrist_3_joint;    // Maxima DSL: -_k__tz_wrist_3_joint
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) =  tx_wrist_3_joint;    // Maxima DSL: _k__tx_wrist_3_joint
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_wrist_2_link_X_fr_wrist_3_link& MotionTransforms::Type_fr_wrist_2_link_X_fr_wrist_3_link::update(const state_t& q)
{
    Scalar sin_q_wrist_3_joint  = ScalarTraits::sin( q(WRIST_3_JOINT) );
    Scalar cos_q_wrist_3_joint  = ScalarTraits::cos( q(WRIST_3_JOINT) );
    (*this)(0,0) = -cos_q_wrist_3_joint;
    (*this)(0,1) = sin_q_wrist_3_joint;
    (*this)(2,0) = sin_q_wrist_3_joint;
    (*this)(2,1) = cos_q_wrist_3_joint;
    (*this)(3,3) = -cos_q_wrist_3_joint;
    (*this)(3,4) = sin_q_wrist_3_joint;
    (*this)(4,0) = (- tx_wrist_3_joint * sin_q_wrist_3_joint)-( tz_wrist_3_joint * cos_q_wrist_3_joint);
    (*this)(4,1) = ( tz_wrist_3_joint * sin_q_wrist_3_joint)-( tx_wrist_3_joint * cos_q_wrist_3_joint);
    (*this)(5,3) = sin_q_wrist_3_joint;
    (*this)(5,4) = cos_q_wrist_3_joint;
    return *this;
}

ForceTransforms::Type_fr_base_link_ur5e_X_fr_forearm_link::Type_fr_base_link_ur5e_X_fr_forearm_link()
{
    (*this)(2,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_link_ur5e_X_fr_forearm_link& ForceTransforms::Type_fr_base_link_ur5e_X_fr_forearm_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    (*this)(0,0) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(0,1) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(0,2) = -sin_q_shoulder_pan_joint;
    (*this)(0,3) = ((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)) * sin_q_shoulder_pan_joint)+((((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint);
    (*this)(0,4) = ((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)) * sin_q_shoulder_pan_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint);
    (*this)(0,5) = (( tx_elbow_joint * sin_q_shoulder_lift_joint)+( ty_elbow_joint * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint;
    (*this)(1,0) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(1,1) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(1,2) = cos_q_shoulder_pan_joint;
    (*this)(1,3) = ((((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)) * cos_q_shoulder_pan_joint);
    (*this)(1,4) = (((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)) * cos_q_shoulder_pan_joint);
    (*this)(1,5) = (( tx_elbow_joint * sin_q_shoulder_lift_joint)+( ty_elbow_joint * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint;
    (*this)(2,0) = (-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(2,1) = (sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(2,3) = (( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(2,4) = (( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(2,5) = ( tx_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_shoulder_lift_joint);
    (*this)(3,3) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(3,4) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(3,5) = -sin_q_shoulder_pan_joint;
    (*this)(4,3) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(4,4) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(4,5) = cos_q_shoulder_pan_joint;
    (*this)(5,3) = (-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(5,4) = (sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint);
    return *this;
}
ForceTransforms::Type_fr_forearm_link_X_fr_base_link_ur5e::Type_fr_forearm_link_X_fr_base_link_ur5e()
{
    (*this)(2,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_forearm_link_X_fr_base_link_ur5e& ForceTransforms::Type_fr_forearm_link_X_fr_base_link_ur5e::update(const state_t& q)
{
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(0,1) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(0,2) = (-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(0,3) = ((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)) * sin_q_shoulder_pan_joint)+((((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint);
    (*this)(0,4) = ((((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)) * cos_q_shoulder_pan_joint);
    (*this)(0,5) = (( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(1,0) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(1,1) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(1,2) = (sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(1,3) = ((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)) * sin_q_shoulder_pan_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint);
    (*this)(1,4) = (((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)) * cos_q_shoulder_pan_joint);
    (*this)(1,5) = (( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(2,0) = -sin_q_shoulder_pan_joint;
    (*this)(2,1) = cos_q_shoulder_pan_joint;
    (*this)(2,3) = (( tx_elbow_joint * sin_q_shoulder_lift_joint)+( ty_elbow_joint * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint;
    (*this)(2,4) = (( tx_elbow_joint * sin_q_shoulder_lift_joint)+( ty_elbow_joint * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint;
    (*this)(2,5) = ( tx_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_shoulder_lift_joint);
    (*this)(3,3) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(3,4) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(3,5) = (-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(4,3) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(4,4) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(4,5) = (sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(5,3) = -sin_q_shoulder_pan_joint;
    (*this)(5,4) = cos_q_shoulder_pan_joint;
    return *this;
}
ForceTransforms::Type_fr_base_link_ur5e_X_fr_shoulder_link::Type_fr_base_link_ur5e_X_fr_shoulder_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_link_ur5e_X_fr_shoulder_link& ForceTransforms::Type_fr_base_link_ur5e_X_fr_shoulder_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = cos_q_shoulder_pan_joint;
    (*this)(0,1) = -sin_q_shoulder_pan_joint;
    (*this)(0,3) = - tz_shoulder_pan_joint * sin_q_shoulder_pan_joint;
    (*this)(0,4) = - tz_shoulder_pan_joint * cos_q_shoulder_pan_joint;
    (*this)(1,0) = sin_q_shoulder_pan_joint;
    (*this)(1,1) = cos_q_shoulder_pan_joint;
    (*this)(1,3) =  tz_shoulder_pan_joint * cos_q_shoulder_pan_joint;
    (*this)(1,4) = - tz_shoulder_pan_joint * sin_q_shoulder_pan_joint;
    (*this)(3,3) = cos_q_shoulder_pan_joint;
    (*this)(3,4) = -sin_q_shoulder_pan_joint;
    (*this)(4,3) = sin_q_shoulder_pan_joint;
    (*this)(4,4) = cos_q_shoulder_pan_joint;
    return *this;
}
ForceTransforms::Type_fr_shoulder_link_X_fr_base_link_ur5e::Type_fr_shoulder_link_X_fr_base_link_ur5e()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_shoulder_link_X_fr_base_link_ur5e& ForceTransforms::Type_fr_shoulder_link_X_fr_base_link_ur5e::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = cos_q_shoulder_pan_joint;
    (*this)(0,1) = sin_q_shoulder_pan_joint;
    (*this)(0,3) = - tz_shoulder_pan_joint * sin_q_shoulder_pan_joint;
    (*this)(0,4) =  tz_shoulder_pan_joint * cos_q_shoulder_pan_joint;
    (*this)(1,0) = -sin_q_shoulder_pan_joint;
    (*this)(1,1) = cos_q_shoulder_pan_joint;
    (*this)(1,3) = - tz_shoulder_pan_joint * cos_q_shoulder_pan_joint;
    (*this)(1,4) = - tz_shoulder_pan_joint * sin_q_shoulder_pan_joint;
    (*this)(3,3) = cos_q_shoulder_pan_joint;
    (*this)(3,4) = sin_q_shoulder_pan_joint;
    (*this)(4,3) = -sin_q_shoulder_pan_joint;
    (*this)(4,4) = cos_q_shoulder_pan_joint;
    return *this;
}
ForceTransforms::Type_fr_base_link_ur5e_X_fr_upper_arm_link::Type_fr_base_link_ur5e_X_fr_upper_arm_link()
{
    (*this)(2,2) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_link_ur5e_X_fr_upper_arm_link& ForceTransforms::Type_fr_base_link_ur5e_X_fr_upper_arm_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    (*this)(0,0) = cos_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(0,1) = -sin_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(0,2) = -sin_q_shoulder_pan_joint;
    (*this)(0,3) = (- tz_shoulder_pan_joint * cos_q_shoulder_lift_joint * sin_q_shoulder_pan_joint)-( ty_shoulder_lift_joint * sin_q_shoulder_lift_joint * cos_q_shoulder_pan_joint);
    (*this)(0,4) = ( tz_shoulder_pan_joint * sin_q_shoulder_lift_joint * sin_q_shoulder_pan_joint)-( ty_shoulder_lift_joint * cos_q_shoulder_lift_joint * cos_q_shoulder_pan_joint);
    (*this)(0,5) = - tz_shoulder_pan_joint * cos_q_shoulder_pan_joint;
    (*this)(1,0) = cos_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(1,1) = -sin_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(1,2) = cos_q_shoulder_pan_joint;
    (*this)(1,3) = ( tz_shoulder_pan_joint * cos_q_shoulder_lift_joint * cos_q_shoulder_pan_joint)-( ty_shoulder_lift_joint * sin_q_shoulder_lift_joint * sin_q_shoulder_pan_joint);
    (*this)(1,4) = (- ty_shoulder_lift_joint * cos_q_shoulder_lift_joint * sin_q_shoulder_pan_joint)-( tz_shoulder_pan_joint * sin_q_shoulder_lift_joint * cos_q_shoulder_pan_joint);
    (*this)(1,5) = - tz_shoulder_pan_joint * sin_q_shoulder_pan_joint;
    (*this)(2,0) = -sin_q_shoulder_lift_joint;
    (*this)(2,1) = -cos_q_shoulder_lift_joint;
    (*this)(2,3) = - ty_shoulder_lift_joint * cos_q_shoulder_lift_joint;
    (*this)(2,4) =  ty_shoulder_lift_joint * sin_q_shoulder_lift_joint;
    (*this)(3,3) = cos_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(3,4) = -sin_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(3,5) = -sin_q_shoulder_pan_joint;
    (*this)(4,3) = cos_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(4,4) = -sin_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(4,5) = cos_q_shoulder_pan_joint;
    (*this)(5,3) = -sin_q_shoulder_lift_joint;
    (*this)(5,4) = -cos_q_shoulder_lift_joint;
    return *this;
}
ForceTransforms::Type_fr_upper_arm_link_X_fr_base_link_ur5e::Type_fr_upper_arm_link_X_fr_base_link_ur5e()
{
    (*this)(2,2) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_upper_arm_link_X_fr_base_link_ur5e& ForceTransforms::Type_fr_upper_arm_link_X_fr_base_link_ur5e::update(const state_t& q)
{
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = cos_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(0,1) = cos_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(0,2) = -sin_q_shoulder_lift_joint;
    (*this)(0,3) = (- tz_shoulder_pan_joint * cos_q_shoulder_lift_joint * sin_q_shoulder_pan_joint)-( ty_shoulder_lift_joint * sin_q_shoulder_lift_joint * cos_q_shoulder_pan_joint);
    (*this)(0,4) = ( tz_shoulder_pan_joint * cos_q_shoulder_lift_joint * cos_q_shoulder_pan_joint)-( ty_shoulder_lift_joint * sin_q_shoulder_lift_joint * sin_q_shoulder_pan_joint);
    (*this)(0,5) = - ty_shoulder_lift_joint * cos_q_shoulder_lift_joint;
    (*this)(1,0) = -sin_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(1,1) = -sin_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(1,2) = -cos_q_shoulder_lift_joint;
    (*this)(1,3) = ( tz_shoulder_pan_joint * sin_q_shoulder_lift_joint * sin_q_shoulder_pan_joint)-( ty_shoulder_lift_joint * cos_q_shoulder_lift_joint * cos_q_shoulder_pan_joint);
    (*this)(1,4) = (- ty_shoulder_lift_joint * cos_q_shoulder_lift_joint * sin_q_shoulder_pan_joint)-( tz_shoulder_pan_joint * sin_q_shoulder_lift_joint * cos_q_shoulder_pan_joint);
    (*this)(1,5) =  ty_shoulder_lift_joint * sin_q_shoulder_lift_joint;
    (*this)(2,0) = -sin_q_shoulder_pan_joint;
    (*this)(2,1) = cos_q_shoulder_pan_joint;
    (*this)(2,3) = - tz_shoulder_pan_joint * cos_q_shoulder_pan_joint;
    (*this)(2,4) = - tz_shoulder_pan_joint * sin_q_shoulder_pan_joint;
    (*this)(3,3) = cos_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(3,4) = cos_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(3,5) = -sin_q_shoulder_lift_joint;
    (*this)(4,3) = -sin_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(4,4) = -sin_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(4,5) = -cos_q_shoulder_lift_joint;
    (*this)(5,3) = -sin_q_shoulder_pan_joint;
    (*this)(5,4) = cos_q_shoulder_pan_joint;
    return *this;
}
ForceTransforms::Type_fr_base_link_ur5e_X_fr_wrist_1_link::Type_fr_base_link_ur5e_X_fr_wrist_1_link()
{
    (*this)(2,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_link_ur5e_X_fr_wrist_1_link& ForceTransforms::Type_fr_base_link_ur5e_X_fr_wrist_1_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    (*this)(0,0) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(0,1) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(0,2) = -sin_q_shoulder_pan_joint;
    (*this)(0,3) = ((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(0,4) = (((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(0,5) = ((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint;
    (*this)(1,0) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(1,1) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(1,2) = cos_q_shoulder_pan_joint;
    (*this)(1,3) = (((((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(1,4) = (((((( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(1,5) = ((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint;
    (*this)(2,0) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(2,1) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(2,3) = (((( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(2,4) = ((((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(2,5) = (((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint);
    (*this)(3,3) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(3,4) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(3,5) = -sin_q_shoulder_pan_joint;
    (*this)(4,3) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(4,4) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(4,5) = cos_q_shoulder_pan_joint;
    (*this)(5,3) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(5,4) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    return *this;
}
ForceTransforms::Type_fr_wrist_1_link_X_fr_base_link_ur5e::Type_fr_wrist_1_link_X_fr_base_link_ur5e()
{
    (*this)(2,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_wrist_1_link_X_fr_base_link_ur5e& ForceTransforms::Type_fr_wrist_1_link_X_fr_base_link_ur5e::update(const state_t& q)
{
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(0,1) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(0,2) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(0,3) = ((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(0,4) = (((((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(0,5) = (((( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(1,0) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(1,1) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(1,2) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(1,3) = (((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(1,4) = (((((( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(1,5) = ((((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(2,0) = -sin_q_shoulder_pan_joint;
    (*this)(2,1) = cos_q_shoulder_pan_joint;
    (*this)(2,3) = ((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint;
    (*this)(2,4) = ((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint;
    (*this)(2,5) = (((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint);
    (*this)(3,3) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(3,4) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(3,5) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(4,3) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(4,4) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(4,5) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(5,3) = -sin_q_shoulder_pan_joint;
    (*this)(5,4) = cos_q_shoulder_pan_joint;
    return *this;
}
ForceTransforms::Type_fr_base_link_ur5e_X_fr_wrist_2_link::Type_fr_base_link_ur5e_X_fr_wrist_2_link()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_link_ur5e_X_fr_wrist_2_link& ForceTransforms::Type_fr_base_link_ur5e_X_fr_wrist_2_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    Scalar sin_q_wrist_2_joint  = ScalarTraits::sin( q(WRIST_2_JOINT) );
    Scalar cos_q_wrist_2_joint  = ScalarTraits::cos( q(WRIST_2_JOINT) );
    (*this)(0,0) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint);
    (*this)(0,1) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(0,2) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(0,3) = (((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(0,4) = ((((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(0,5) = (((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(1,0) = (cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(1,1) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(1,2) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(1,3) = (((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)+((((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(1,4) = (((((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(1,5) = (((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(2,0) = ((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint;
    (*this)(2,1) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    (*this)(2,2) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(2,3) = (((((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * sin_q_wrist_2_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(2,4) = (((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(((((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * cos_q_wrist_2_joint);
    (*this)(2,5) = ((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(3,3) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint);
    (*this)(3,4) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(3,5) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(4,3) = (cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(4,4) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(4,5) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(5,3) = ((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint;
    (*this)(5,4) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    (*this)(5,5) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    return *this;
}
ForceTransforms::Type_fr_wrist_2_link_X_fr_base_link_ur5e::Type_fr_wrist_2_link_X_fr_base_link_ur5e()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_wrist_2_link_X_fr_base_link_ur5e& ForceTransforms::Type_fr_wrist_2_link_X_fr_base_link_ur5e::update(const state_t& q)
{
    Scalar sin_q_wrist_2_joint  = ScalarTraits::sin( q(WRIST_2_JOINT) );
    Scalar cos_q_wrist_2_joint  = ScalarTraits::cos( q(WRIST_2_JOINT) );
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint);
    (*this)(0,1) = (cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(0,2) = ((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint;
    (*this)(0,3) = (((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(0,4) = (((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)+((((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(0,5) = (((((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * sin_q_wrist_2_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(1,0) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(1,1) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(1,2) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    (*this)(1,3) = ((((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(1,4) = (((((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(1,5) = (((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(((((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * cos_q_wrist_2_joint);
    (*this)(2,0) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(2,1) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(2,2) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(2,3) = (((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(2,4) = (((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint);
    (*this)(2,5) = ((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(3,3) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint);
    (*this)(3,4) = (cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(3,5) = ((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint;
    (*this)(4,3) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(4,4) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(4,5) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    (*this)(5,3) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(5,4) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(5,5) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    return *this;
}
ForceTransforms::Type_fr_base_link_ur5e_X_fr_wrist_3_link::Type_fr_base_link_ur5e_X_fr_wrist_3_link()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_base_link_ur5e_X_fr_wrist_3_link& ForceTransforms::Type_fr_base_link_ur5e_X_fr_wrist_3_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    Scalar sin_q_wrist_2_joint  = ScalarTraits::sin( q(WRIST_2_JOINT) );
    Scalar cos_q_wrist_2_joint  = ScalarTraits::cos( q(WRIST_2_JOINT) );
    Scalar sin_q_wrist_3_joint  = ScalarTraits::sin( q(WRIST_3_JOINT) );
    Scalar cos_q_wrist_3_joint  = ScalarTraits::cos( q(WRIST_3_JOINT) );
    (*this)(0,0) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(0,1) = (((((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(0,2) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(0,3) = ((((((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+( tx_wrist_3_joint * sin_q_shoulder_pan_joint * cos_q_wrist_2_joint)+(((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+((((((( tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)+((((( ty_wrist_1_joint * sin_q_elbow_joint)-( tx_wrist_1_joint * cos_q_elbow_joint)- tx_elbow_joint) * sin_q_shoulder_lift_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * cos_q_shoulder_lift_joint)+ tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+((((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)+( tz_wrist_3_joint * sin_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(0,4) = (((((((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+(((((((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)-( tz_wrist_3_joint * sin_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+((((((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+( tx_wrist_3_joint * sin_q_shoulder_pan_joint * cos_q_wrist_2_joint)+(((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(0,5) = ((((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)+( tz_wrist_3_joint * sin_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+(((((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((- tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(1,0) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(1,1) = (((cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(1,2) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(1,3) = ((((((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-( tx_wrist_3_joint * cos_q_shoulder_pan_joint * cos_q_wrist_2_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+((((((( tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)+((((( ty_wrist_1_joint * sin_q_elbow_joint)-( tx_wrist_1_joint * cos_q_elbow_joint)- tx_elbow_joint) * sin_q_shoulder_lift_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * cos_q_shoulder_lift_joint)+ tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+(((((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)-( tz_wrist_3_joint * cos_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(1,4) = (((((((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+((((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)+( tz_wrist_3_joint * cos_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+((((((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-( tx_wrist_3_joint * cos_q_shoulder_pan_joint * cos_q_wrist_2_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(1,5) = (((((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)-( tz_wrist_3_joint * cos_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+(((((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((- tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(2,0) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * cos_q_wrist_3_joint);
    (*this)(2,1) = (((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * sin_q_wrist_3_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(2,2) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    (*this)(2,3) = ((((((( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+((((((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * sin_q_shoulder_lift_joint)+((( ty_wrist_1_joint * sin_q_elbow_joint)-( tx_wrist_1_joint * cos_q_elbow_joint)- tx_elbow_joint) * cos_q_shoulder_lift_joint)) * sin_q_wrist_2_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(2,4) = ((((((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * sin_q_wrist_2_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+((((((( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(2,5) = (((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+((((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * cos_q_wrist_2_joint)+((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(3,3) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(3,4) = (((((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(3,5) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(4,3) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(4,4) = (((cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(4,5) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(5,3) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * cos_q_wrist_3_joint);
    (*this)(5,4) = (((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * sin_q_wrist_3_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(5,5) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    return *this;
}
ForceTransforms::Type_fr_wrist_3_link_X_fr_base_link_ur5e::Type_fr_wrist_3_link_X_fr_base_link_ur5e()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
}

const ForceTransforms::Type_fr_wrist_3_link_X_fr_base_link_ur5e& ForceTransforms::Type_fr_wrist_3_link_X_fr_base_link_ur5e::update(const state_t& q)
{
    Scalar sin_q_wrist_3_joint  = ScalarTraits::sin( q(WRIST_3_JOINT) );
    Scalar cos_q_wrist_3_joint  = ScalarTraits::cos( q(WRIST_3_JOINT) );
    Scalar sin_q_wrist_2_joint  = ScalarTraits::sin( q(WRIST_2_JOINT) );
    Scalar cos_q_wrist_2_joint  = ScalarTraits::cos( q(WRIST_2_JOINT) );
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(0,1) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(0,2) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * cos_q_wrist_3_joint);
    (*this)(0,3) = ((((((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+( tx_wrist_3_joint * sin_q_shoulder_pan_joint * cos_q_wrist_2_joint)+(((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+((((((( tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)+((((( ty_wrist_1_joint * sin_q_elbow_joint)-( tx_wrist_1_joint * cos_q_elbow_joint)- tx_elbow_joint) * sin_q_shoulder_lift_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * cos_q_shoulder_lift_joint)+ tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+((((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)+( tz_wrist_3_joint * sin_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(0,4) = ((((((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-( tx_wrist_3_joint * cos_q_shoulder_pan_joint * cos_q_wrist_2_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+((((((( tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)+((((( ty_wrist_1_joint * sin_q_elbow_joint)-( tx_wrist_1_joint * cos_q_elbow_joint)- tx_elbow_joint) * sin_q_shoulder_lift_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * cos_q_shoulder_lift_joint)+ tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+(((((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)-( tz_wrist_3_joint * cos_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(0,5) = ((((((( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+((((((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * sin_q_shoulder_lift_joint)+((( ty_wrist_1_joint * sin_q_elbow_joint)-( tx_wrist_1_joint * cos_q_elbow_joint)- tx_elbow_joint) * cos_q_shoulder_lift_joint)) * sin_q_wrist_2_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(1,0) = (((((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(1,1) = (((cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(1,2) = (((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * sin_q_wrist_3_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(1,3) = (((((((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+(((((((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)-( tz_wrist_3_joint * sin_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+((((((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+( tx_wrist_3_joint * sin_q_shoulder_pan_joint * cos_q_wrist_2_joint)+(((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(1,4) = (((((((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+((((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)+( tz_wrist_3_joint * cos_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+((((((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-( tx_wrist_3_joint * cos_q_shoulder_pan_joint * cos_q_wrist_2_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+(((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(1,5) = ((((((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * sin_q_wrist_2_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+((((((( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(2,0) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(2,1) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(2,2) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    (*this)(2,3) = ((((((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * sin_q_shoulder_pan_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * sin_q_shoulder_pan_joint)+((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)+( tz_wrist_3_joint * sin_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+(((((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((- tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(2,4) = (((((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_shoulder_pan_joint)) * sin_q_wrist_1_joint)+((((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_shoulder_pan_joint)) * cos_q_wrist_1_joint)-( tz_wrist_3_joint * cos_q_shoulder_pan_joint)) * sin_q_wrist_2_joint)+(((((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)+(((((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * sin_q_shoulder_lift_joint)+((( tx_wrist_1_joint * sin_q_elbow_joint)+( ty_wrist_1_joint * cos_q_elbow_joint)+ ty_elbow_joint) * cos_q_shoulder_lift_joint)- tz_shoulder_pan_joint) * sin_q_shoulder_pan_joint)) * cos_q_wrist_2_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((- tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(2,5) = (((((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+((((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * cos_q_wrist_2_joint)+((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(3,3) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(3,4) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(3,5) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * cos_q_wrist_3_joint);
    (*this)(4,3) = (((((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(4,4) = (((cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(4,5) = (((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * sin_q_wrist_3_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(5,3) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(5,4) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(5,5) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    return *this;
}
ForceTransforms::Type_fr_upper_arm_link_X_fr_shoulder_link::Type_fr_upper_arm_link_X_fr_shoulder_link()
{
    (*this)(0,1) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_upper_arm_link_X_fr_shoulder_link& ForceTransforms::Type_fr_upper_arm_link_X_fr_shoulder_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    (*this)(0,0) = cos_q_shoulder_lift_joint;
    (*this)(0,2) = -sin_q_shoulder_lift_joint;
    (*this)(0,3) = - ty_shoulder_lift_joint * sin_q_shoulder_lift_joint;
    (*this)(0,5) = - ty_shoulder_lift_joint * cos_q_shoulder_lift_joint;
    (*this)(1,0) = -sin_q_shoulder_lift_joint;
    (*this)(1,2) = -cos_q_shoulder_lift_joint;
    (*this)(1,3) = - ty_shoulder_lift_joint * cos_q_shoulder_lift_joint;
    (*this)(1,5) =  ty_shoulder_lift_joint * sin_q_shoulder_lift_joint;
    (*this)(3,3) = cos_q_shoulder_lift_joint;
    (*this)(3,5) = -sin_q_shoulder_lift_joint;
    (*this)(4,3) = -sin_q_shoulder_lift_joint;
    (*this)(4,5) = -cos_q_shoulder_lift_joint;
    return *this;
}
ForceTransforms::Type_fr_shoulder_link_X_fr_upper_arm_link::Type_fr_shoulder_link_X_fr_upper_arm_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_shoulder_link_X_fr_upper_arm_link& ForceTransforms::Type_fr_shoulder_link_X_fr_upper_arm_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    (*this)(0,0) = cos_q_shoulder_lift_joint;
    (*this)(0,1) = -sin_q_shoulder_lift_joint;
    (*this)(0,3) = - ty_shoulder_lift_joint * sin_q_shoulder_lift_joint;
    (*this)(0,4) = - ty_shoulder_lift_joint * cos_q_shoulder_lift_joint;
    (*this)(2,0) = -sin_q_shoulder_lift_joint;
    (*this)(2,1) = -cos_q_shoulder_lift_joint;
    (*this)(2,3) = - ty_shoulder_lift_joint * cos_q_shoulder_lift_joint;
    (*this)(2,4) =  ty_shoulder_lift_joint * sin_q_shoulder_lift_joint;
    (*this)(3,3) = cos_q_shoulder_lift_joint;
    (*this)(3,4) = -sin_q_shoulder_lift_joint;
    (*this)(5,3) = -sin_q_shoulder_lift_joint;
    (*this)(5,4) = -cos_q_shoulder_lift_joint;
    return *this;
}
ForceTransforms::Type_fr_forearm_link_X_fr_upper_arm_link::Type_fr_forearm_link_X_fr_upper_arm_link()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  ty_elbow_joint;    // Maxima DSL: _k__ty_elbow_joint
    (*this)(2,4) = - tx_elbow_joint;    // Maxima DSL: -_k__tx_elbow_joint
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_forearm_link_X_fr_upper_arm_link& ForceTransforms::Type_fr_forearm_link_X_fr_upper_arm_link::update(const state_t& q)
{
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    (*this)(0,0) = cos_q_elbow_joint;
    (*this)(0,1) = sin_q_elbow_joint;
    (*this)(0,3) = - tz_elbow_joint * sin_q_elbow_joint;
    (*this)(0,4) =  tz_elbow_joint * cos_q_elbow_joint;
    (*this)(0,5) = ( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint);
    (*this)(1,0) = -sin_q_elbow_joint;
    (*this)(1,1) = cos_q_elbow_joint;
    (*this)(1,3) = - tz_elbow_joint * cos_q_elbow_joint;
    (*this)(1,4) = - tz_elbow_joint * sin_q_elbow_joint;
    (*this)(1,5) = ( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint);
    (*this)(3,3) = cos_q_elbow_joint;
    (*this)(3,4) = sin_q_elbow_joint;
    (*this)(4,3) = -sin_q_elbow_joint;
    (*this)(4,4) = cos_q_elbow_joint;
    return *this;
}
ForceTransforms::Type_fr_upper_arm_link_X_fr_forearm_link::Type_fr_upper_arm_link_X_fr_forearm_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) =  ty_elbow_joint;    // Maxima DSL: _k__ty_elbow_joint
    (*this)(1,2) = 0.0;
    (*this)(1,5) = - tx_elbow_joint;    // Maxima DSL: -_k__tx_elbow_joint
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_upper_arm_link_X_fr_forearm_link& ForceTransforms::Type_fr_upper_arm_link_X_fr_forearm_link::update(const state_t& q)
{
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    (*this)(0,0) = cos_q_elbow_joint;
    (*this)(0,1) = -sin_q_elbow_joint;
    (*this)(0,3) = - tz_elbow_joint * sin_q_elbow_joint;
    (*this)(0,4) = - tz_elbow_joint * cos_q_elbow_joint;
    (*this)(1,0) = sin_q_elbow_joint;
    (*this)(1,1) = cos_q_elbow_joint;
    (*this)(1,3) =  tz_elbow_joint * cos_q_elbow_joint;
    (*this)(1,4) = - tz_elbow_joint * sin_q_elbow_joint;
    (*this)(2,3) = ( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint);
    (*this)(2,4) = ( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint);
    (*this)(3,3) = cos_q_elbow_joint;
    (*this)(3,4) = -sin_q_elbow_joint;
    (*this)(4,3) = sin_q_elbow_joint;
    (*this)(4,4) = cos_q_elbow_joint;
    return *this;
}
ForceTransforms::Type_fr_wrist_1_link_X_fr_forearm_link::Type_fr_wrist_1_link_X_fr_forearm_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  ty_wrist_1_joint;    // Maxima DSL: _k__ty_wrist_1_joint
    (*this)(2,4) = - tx_wrist_1_joint;    // Maxima DSL: -_k__tx_wrist_1_joint
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_wrist_1_link_X_fr_forearm_link& ForceTransforms::Type_fr_wrist_1_link_X_fr_forearm_link::update(const state_t& q)
{
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    (*this)(0,0) = cos_q_wrist_1_joint;
    (*this)(0,1) = sin_q_wrist_1_joint;
    (*this)(0,5) = ( tx_wrist_1_joint * sin_q_wrist_1_joint)-( ty_wrist_1_joint * cos_q_wrist_1_joint);
    (*this)(1,0) = -sin_q_wrist_1_joint;
    (*this)(1,1) = cos_q_wrist_1_joint;
    (*this)(1,5) = ( ty_wrist_1_joint * sin_q_wrist_1_joint)+( tx_wrist_1_joint * cos_q_wrist_1_joint);
    (*this)(3,3) = cos_q_wrist_1_joint;
    (*this)(3,4) = sin_q_wrist_1_joint;
    (*this)(4,3) = -sin_q_wrist_1_joint;
    (*this)(4,4) = cos_q_wrist_1_joint;
    return *this;
}
ForceTransforms::Type_fr_forearm_link_X_fr_wrist_1_link::Type_fr_forearm_link_X_fr_wrist_1_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) =  ty_wrist_1_joint;    // Maxima DSL: _k__ty_wrist_1_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = - tx_wrist_1_joint;    // Maxima DSL: -_k__tx_wrist_1_joint
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_forearm_link_X_fr_wrist_1_link& ForceTransforms::Type_fr_forearm_link_X_fr_wrist_1_link::update(const state_t& q)
{
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    (*this)(0,0) = cos_q_wrist_1_joint;
    (*this)(0,1) = -sin_q_wrist_1_joint;
    (*this)(1,0) = sin_q_wrist_1_joint;
    (*this)(1,1) = cos_q_wrist_1_joint;
    (*this)(2,3) = ( tx_wrist_1_joint * sin_q_wrist_1_joint)-( ty_wrist_1_joint * cos_q_wrist_1_joint);
    (*this)(2,4) = ( ty_wrist_1_joint * sin_q_wrist_1_joint)+( tx_wrist_1_joint * cos_q_wrist_1_joint);
    (*this)(3,3) = cos_q_wrist_1_joint;
    (*this)(3,4) = -sin_q_wrist_1_joint;
    (*this)(4,3) = sin_q_wrist_1_joint;
    (*this)(4,4) = cos_q_wrist_1_joint;
    return *this;
}
ForceTransforms::Type_fr_wrist_2_link_X_fr_wrist_1_link::Type_fr_wrist_2_link_X_fr_wrist_1_link()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tz_wrist_2_joint;    // Maxima DSL: -_k__tz_wrist_2_joint
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_wrist_2_link_X_fr_wrist_1_link& ForceTransforms::Type_fr_wrist_2_link_X_fr_wrist_1_link::update(const state_t& q)
{
    Scalar sin_q_wrist_2_joint  = ScalarTraits::sin( q(WRIST_2_JOINT) );
    Scalar cos_q_wrist_2_joint  = ScalarTraits::cos( q(WRIST_2_JOINT) );
    (*this)(0,0) = -cos_q_wrist_2_joint;
    (*this)(0,2) = sin_q_wrist_2_joint;
    (*this)(0,4) = - tz_wrist_2_joint * cos_q_wrist_2_joint;
    (*this)(1,0) = sin_q_wrist_2_joint;
    (*this)(1,2) = cos_q_wrist_2_joint;
    (*this)(1,4) =  tz_wrist_2_joint * sin_q_wrist_2_joint;
    (*this)(3,3) = -cos_q_wrist_2_joint;
    (*this)(3,5) = sin_q_wrist_2_joint;
    (*this)(4,3) = sin_q_wrist_2_joint;
    (*this)(4,5) = cos_q_wrist_2_joint;
    return *this;
}
ForceTransforms::Type_fr_wrist_1_link_X_fr_wrist_2_link::Type_fr_wrist_1_link_X_fr_wrist_2_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = - tz_wrist_2_joint;    // Maxima DSL: -_k__tz_wrist_2_joint
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_wrist_1_link_X_fr_wrist_2_link& ForceTransforms::Type_fr_wrist_1_link_X_fr_wrist_2_link::update(const state_t& q)
{
    Scalar sin_q_wrist_2_joint  = ScalarTraits::sin( q(WRIST_2_JOINT) );
    Scalar cos_q_wrist_2_joint  = ScalarTraits::cos( q(WRIST_2_JOINT) );
    (*this)(0,0) = -cos_q_wrist_2_joint;
    (*this)(0,1) = sin_q_wrist_2_joint;
    (*this)(1,3) = - tz_wrist_2_joint * cos_q_wrist_2_joint;
    (*this)(1,4) =  tz_wrist_2_joint * sin_q_wrist_2_joint;
    (*this)(2,0) = sin_q_wrist_2_joint;
    (*this)(2,1) = cos_q_wrist_2_joint;
    (*this)(3,3) = -cos_q_wrist_2_joint;
    (*this)(3,4) = sin_q_wrist_2_joint;
    (*this)(5,3) = sin_q_wrist_2_joint;
    (*this)(5,4) = cos_q_wrist_2_joint;
    return *this;
}
ForceTransforms::Type_fr_wrist_3_link_X_fr_wrist_2_link::Type_fr_wrist_3_link_X_fr_wrist_2_link()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tz_wrist_3_joint;    // Maxima DSL: -_k__tz_wrist_3_joint
    (*this)(2,4) = 0.0;
    (*this)(2,5) =  tx_wrist_3_joint;    // Maxima DSL: _k__tx_wrist_3_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_wrist_3_link_X_fr_wrist_2_link& ForceTransforms::Type_fr_wrist_3_link_X_fr_wrist_2_link::update(const state_t& q)
{
    Scalar sin_q_wrist_3_joint  = ScalarTraits::sin( q(WRIST_3_JOINT) );
    Scalar cos_q_wrist_3_joint  = ScalarTraits::cos( q(WRIST_3_JOINT) );
    (*this)(0,0) = -cos_q_wrist_3_joint;
    (*this)(0,2) = sin_q_wrist_3_joint;
    (*this)(0,4) = (- tx_wrist_3_joint * sin_q_wrist_3_joint)-( tz_wrist_3_joint * cos_q_wrist_3_joint);
    (*this)(1,0) = sin_q_wrist_3_joint;
    (*this)(1,2) = cos_q_wrist_3_joint;
    (*this)(1,4) = ( tz_wrist_3_joint * sin_q_wrist_3_joint)-( tx_wrist_3_joint * cos_q_wrist_3_joint);
    (*this)(3,3) = -cos_q_wrist_3_joint;
    (*this)(3,5) = sin_q_wrist_3_joint;
    (*this)(4,3) = sin_q_wrist_3_joint;
    (*this)(4,5) = cos_q_wrist_3_joint;
    return *this;
}
ForceTransforms::Type_fr_wrist_2_link_X_fr_wrist_3_link::Type_fr_wrist_2_link_X_fr_wrist_3_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = - tz_wrist_3_joint;    // Maxima DSL: -_k__tz_wrist_3_joint
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) =  tx_wrist_3_joint;    // Maxima DSL: _k__tx_wrist_3_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_wrist_2_link_X_fr_wrist_3_link& ForceTransforms::Type_fr_wrist_2_link_X_fr_wrist_3_link::update(const state_t& q)
{
    Scalar sin_q_wrist_3_joint  = ScalarTraits::sin( q(WRIST_3_JOINT) );
    Scalar cos_q_wrist_3_joint  = ScalarTraits::cos( q(WRIST_3_JOINT) );
    (*this)(0,0) = -cos_q_wrist_3_joint;
    (*this)(0,1) = sin_q_wrist_3_joint;
    (*this)(1,3) = (- tx_wrist_3_joint * sin_q_wrist_3_joint)-( tz_wrist_3_joint * cos_q_wrist_3_joint);
    (*this)(1,4) = ( tz_wrist_3_joint * sin_q_wrist_3_joint)-( tx_wrist_3_joint * cos_q_wrist_3_joint);
    (*this)(2,0) = sin_q_wrist_3_joint;
    (*this)(2,1) = cos_q_wrist_3_joint;
    (*this)(3,3) = -cos_q_wrist_3_joint;
    (*this)(3,4) = sin_q_wrist_3_joint;
    (*this)(5,3) = sin_q_wrist_3_joint;
    (*this)(5,4) = cos_q_wrist_3_joint;
    return *this;
}

HomogeneousTransforms::Type_fr_base_link_ur5e_X_fr_forearm_link::Type_fr_base_link_ur5e_X_fr_forearm_link()
{
    (*this)(2,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_ur5e_X_fr_forearm_link& HomogeneousTransforms::Type_fr_base_link_ur5e_X_fr_forearm_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    (*this)(0,0) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(0,1) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(0,2) = -sin_q_shoulder_pan_joint;
    (*this)(0,3) = ((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_shoulder_pan_joint)+((( tx_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint);
    (*this)(1,0) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(1,1) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(1,2) = cos_q_shoulder_pan_joint;
    (*this)(1,3) = ((( tx_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_shoulder_pan_joint);
    (*this)(2,0) = (-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(2,1) = (sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(2,3) = (- tx_elbow_joint * sin_q_shoulder_lift_joint)-( ty_elbow_joint * cos_q_shoulder_lift_joint)+ tz_shoulder_pan_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_forearm_link_X_fr_base_link_ur5e::Type_fr_forearm_link_X_fr_base_link_ur5e()
{
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tz_elbow_joint- ty_shoulder_lift_joint;    // Maxima DSL: (-_k__tz_elbow_joint)-_k__ty_shoulder_lift_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_forearm_link_X_fr_base_link_ur5e& HomogeneousTransforms::Type_fr_forearm_link_X_fr_base_link_ur5e::update(const state_t& q)
{
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(0,1) = ((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(0,2) = (-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(0,3) = ( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint);
    (*this)(1,0) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint;
    (*this)(1,1) = ((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint;
    (*this)(1,2) = (sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint);
    (*this)(1,3) = (- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint);
    (*this)(2,0) = -sin_q_shoulder_pan_joint;
    (*this)(2,1) = cos_q_shoulder_pan_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_ur5e_X_fr_shoulder_link::Type_fr_base_link_ur5e_X_fr_shoulder_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_shoulder_pan_joint;    // Maxima DSL: _k__tz_shoulder_pan_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_ur5e_X_fr_shoulder_link& HomogeneousTransforms::Type_fr_base_link_ur5e_X_fr_shoulder_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = cos_q_shoulder_pan_joint;
    (*this)(0,1) = -sin_q_shoulder_pan_joint;
    (*this)(1,0) = sin_q_shoulder_pan_joint;
    (*this)(1,1) = cos_q_shoulder_pan_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_shoulder_link_X_fr_base_link_ur5e::Type_fr_shoulder_link_X_fr_base_link_ur5e()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_shoulder_pan_joint;    // Maxima DSL: -_k__tz_shoulder_pan_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_shoulder_link_X_fr_base_link_ur5e& HomogeneousTransforms::Type_fr_shoulder_link_X_fr_base_link_ur5e::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = cos_q_shoulder_pan_joint;
    (*this)(0,1) = sin_q_shoulder_pan_joint;
    (*this)(1,0) = -sin_q_shoulder_pan_joint;
    (*this)(1,1) = cos_q_shoulder_pan_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_ur5e_X_fr_upper_arm_link::Type_fr_base_link_ur5e_X_fr_upper_arm_link()
{
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_shoulder_pan_joint;    // Maxima DSL: _k__tz_shoulder_pan_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_ur5e_X_fr_upper_arm_link& HomogeneousTransforms::Type_fr_base_link_ur5e_X_fr_upper_arm_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    (*this)(0,0) = cos_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(0,1) = -sin_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(0,2) = -sin_q_shoulder_pan_joint;
    (*this)(0,3) = - ty_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(1,0) = cos_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(1,1) = -sin_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(1,2) = cos_q_shoulder_pan_joint;
    (*this)(1,3) =  ty_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(2,0) = -sin_q_shoulder_lift_joint;
    (*this)(2,1) = -cos_q_shoulder_lift_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_upper_arm_link_X_fr_base_link_ur5e::Type_fr_upper_arm_link_X_fr_base_link_ur5e()
{
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - ty_shoulder_lift_joint;    // Maxima DSL: -_k__ty_shoulder_lift_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_upper_arm_link_X_fr_base_link_ur5e& HomogeneousTransforms::Type_fr_upper_arm_link_X_fr_base_link_ur5e::update(const state_t& q)
{
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = cos_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(0,1) = cos_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(0,2) = -sin_q_shoulder_lift_joint;
    (*this)(0,3) =  tz_shoulder_pan_joint * sin_q_shoulder_lift_joint;
    (*this)(1,0) = -sin_q_shoulder_lift_joint * cos_q_shoulder_pan_joint;
    (*this)(1,1) = -sin_q_shoulder_lift_joint * sin_q_shoulder_pan_joint;
    (*this)(1,2) = -cos_q_shoulder_lift_joint;
    (*this)(1,3) =  tz_shoulder_pan_joint * cos_q_shoulder_lift_joint;
    (*this)(2,0) = -sin_q_shoulder_pan_joint;
    (*this)(2,1) = cos_q_shoulder_pan_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_ur5e_X_fr_wrist_1_link::Type_fr_base_link_ur5e_X_fr_wrist_1_link()
{
    (*this)(2,2) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_ur5e_X_fr_wrist_1_link& HomogeneousTransforms::Type_fr_base_link_ur5e_X_fr_wrist_1_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    (*this)(0,0) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(0,1) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(0,2) = -sin_q_shoulder_pan_joint;
    (*this)(0,3) = ((- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_shoulder_pan_joint)+(((((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint);
    (*this)(1,0) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(1,1) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(1,2) = cos_q_shoulder_pan_joint;
    (*this)(1,3) = (((((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(( tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_shoulder_pan_joint);
    (*this)(2,0) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(2,1) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(2,3) = ((( ty_wrist_1_joint * sin_q_elbow_joint)-( tx_wrist_1_joint * cos_q_elbow_joint)- tx_elbow_joint) * sin_q_shoulder_lift_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * cos_q_shoulder_lift_joint)+ tz_shoulder_pan_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_wrist_1_link_X_fr_base_link_ur5e::Type_fr_wrist_1_link_X_fr_base_link_ur5e()
{
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tz_elbow_joint- ty_shoulder_lift_joint;    // Maxima DSL: (-_k__tz_elbow_joint)-_k__ty_shoulder_lift_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_wrist_1_link_X_fr_base_link_ur5e& HomogeneousTransforms::Type_fr_wrist_1_link_X_fr_base_link_ur5e::update(const state_t& q)
{
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(0,1) = (((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(0,2) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(0,3) = (((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_wrist_1_joint)+((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * cos_q_wrist_1_joint);
    (*this)(1,0) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(1,1) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(1,2) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(1,3) = (((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * sin_q_wrist_1_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_wrist_1_joint);
    (*this)(2,0) = -sin_q_shoulder_pan_joint;
    (*this)(2,1) = cos_q_shoulder_pan_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_ur5e_X_fr_wrist_2_link::Type_fr_base_link_ur5e_X_fr_wrist_2_link()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_ur5e_X_fr_wrist_2_link& HomogeneousTransforms::Type_fr_base_link_ur5e_X_fr_wrist_2_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    Scalar sin_q_wrist_2_joint  = ScalarTraits::sin( q(WRIST_2_JOINT) );
    Scalar cos_q_wrist_2_joint  = ScalarTraits::cos( q(WRIST_2_JOINT) );
    (*this)(0,0) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint);
    (*this)(0,1) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(0,2) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(0,3) = ((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_shoulder_pan_joint)+(((((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint);
    (*this)(1,0) = (cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(1,1) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(1,2) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(1,3) = (((((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_shoulder_pan_joint);
    (*this)(2,0) = ((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint;
    (*this)(2,1) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    (*this)(2,2) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(2,3) = ((( ty_wrist_1_joint * sin_q_elbow_joint)-( tx_wrist_1_joint * cos_q_elbow_joint)- tx_elbow_joint) * sin_q_shoulder_lift_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * cos_q_shoulder_lift_joint)+ tz_shoulder_pan_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_wrist_2_link_X_fr_base_link_ur5e::Type_fr_wrist_2_link_X_fr_base_link_ur5e()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_wrist_2_link_X_fr_base_link_ur5e& HomogeneousTransforms::Type_fr_wrist_2_link_X_fr_base_link_ur5e::update(const state_t& q)
{
    Scalar sin_q_wrist_2_joint  = ScalarTraits::sin( q(WRIST_2_JOINT) );
    Scalar cos_q_wrist_2_joint  = ScalarTraits::cos( q(WRIST_2_JOINT) );
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint);
    (*this)(0,1) = (cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(0,2) = ((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint;
    (*this)(0,3) = ((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_wrist_2_joint)+((((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * sin_q_wrist_1_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint);
    (*this)(1,0) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(1,1) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(1,2) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    (*this)(1,3) = (((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_wrist_1_joint)+((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_wrist_2_joint);
    (*this)(2,0) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(2,1) = (((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint);
    (*this)(2,2) = (((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint);
    (*this)(2,3) = (((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * sin_q_wrist_1_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_wrist_1_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_ur5e_X_fr_wrist_3_link::Type_fr_base_link_ur5e_X_fr_wrist_3_link()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_ur5e_X_fr_wrist_3_link& HomogeneousTransforms::Type_fr_base_link_ur5e_X_fr_wrist_3_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    Scalar sin_q_wrist_2_joint  = ScalarTraits::sin( q(WRIST_2_JOINT) );
    Scalar cos_q_wrist_2_joint  = ScalarTraits::cos( q(WRIST_2_JOINT) );
    Scalar sin_q_wrist_3_joint  = ScalarTraits::sin( q(WRIST_3_JOINT) );
    Scalar cos_q_wrist_3_joint  = ScalarTraits::cos( q(WRIST_3_JOINT) );
    (*this)(0,0) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(0,1) = (((((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(0,2) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(0,3) = (- tx_wrist_3_joint * sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)+((((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)+((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_shoulder_pan_joint)+(((((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint);
    (*this)(1,0) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(1,1) = (((cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(1,2) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(1,3) = ( tx_wrist_3_joint * cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+((((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)+((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((- tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)+(((((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * sin_q_shoulder_lift_joint)+(((- ty_wrist_1_joint * sin_q_elbow_joint)+( tx_wrist_1_joint * cos_q_elbow_joint)+ tx_elbow_joint) * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint)+(( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * cos_q_shoulder_pan_joint);
    (*this)(2,0) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * cos_q_wrist_3_joint);
    (*this)(2,1) = (((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * sin_q_wrist_3_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(2,2) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    (*this)(2,3) = ((((( tx_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((( tx_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tx_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)+((( tz_wrist_3_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_wrist_3_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+((( tz_wrist_3_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_wrist_3_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)+((( ty_wrist_1_joint * sin_q_elbow_joint)-( tx_wrist_1_joint * cos_q_elbow_joint)- tx_elbow_joint) * sin_q_shoulder_lift_joint)+(((- tx_wrist_1_joint * sin_q_elbow_joint)-( ty_wrist_1_joint * cos_q_elbow_joint)- ty_elbow_joint) * cos_q_shoulder_lift_joint)+ tz_shoulder_pan_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_wrist_3_link_X_fr_base_link_ur5e::Type_fr_wrist_3_link_X_fr_base_link_ur5e()
{
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_wrist_3_link_X_fr_base_link_ur5e& HomogeneousTransforms::Type_fr_wrist_3_link_X_fr_base_link_ur5e::update(const state_t& q)
{
    Scalar sin_q_wrist_3_joint  = ScalarTraits::sin( q(WRIST_3_JOINT) );
    Scalar cos_q_wrist_3_joint  = ScalarTraits::cos( q(WRIST_3_JOINT) );
    Scalar sin_q_wrist_2_joint  = ScalarTraits::sin( q(WRIST_2_JOINT) );
    Scalar cos_q_wrist_2_joint  = ScalarTraits::cos( q(WRIST_2_JOINT) );
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    Scalar sin_q_shoulder_pan_joint  = ScalarTraits::sin( q(SHOULDER_PAN_JOINT) );
    Scalar cos_q_shoulder_pan_joint  = ScalarTraits::cos( q(SHOULDER_PAN_JOINT) );
    (*this)(0,0) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(0,1) = (((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * cos_q_wrist_3_joint);
    (*this)(0,2) = (((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * cos_q_wrist_3_joint);
    (*this)(0,3) = (((((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * sin_q_wrist_1_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_wrist_1_joint)- tz_wrist_3_joint) * sin_q_wrist_3_joint)+(((( tz_wrist_2_joint+ tz_elbow_joint+ ty_shoulder_lift_joint) * sin_q_wrist_2_joint)+(((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_wrist_1_joint)+((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)+ tx_wrist_3_joint) * cos_q_wrist_3_joint);
    (*this)(1,0) = (((((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * sin_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(1,1) = (((cos_q_shoulder_pan_joint * sin_q_wrist_2_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)) * sin_q_wrist_3_joint)+(((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(1,2) = (((((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint * sin_q_wrist_3_joint)+(((((cos_q_elbow_joint * sin_q_shoulder_lift_joint)+(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * cos_q_wrist_3_joint);
    (*this)(1,3) = ((((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * sin_q_wrist_2_joint)+((((( tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)-( tx_elbow_joint * sin_q_elbow_joint)+( ty_elbow_joint * cos_q_elbow_joint)+ ty_wrist_1_joint) * sin_q_wrist_1_joint)+(((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * cos_q_wrist_1_joint)) * cos_q_wrist_2_joint)- tx_wrist_3_joint) * sin_q_wrist_3_joint)+(((((- tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)-( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)+( ty_elbow_joint * sin_q_elbow_joint)+( tx_elbow_joint * cos_q_elbow_joint)+ tx_wrist_1_joint) * sin_q_wrist_1_joint)+(((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * cos_q_wrist_1_joint)- tz_wrist_3_joint) * cos_q_wrist_3_joint);
    (*this)(2,0) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * cos_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)-(sin_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(2,1) = (((((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * sin_q_wrist_1_joint)+(((cos_q_elbow_joint * cos_q_shoulder_lift_joint)-(sin_q_elbow_joint * sin_q_shoulder_lift_joint)) * sin_q_shoulder_pan_joint * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+(cos_q_shoulder_pan_joint * cos_q_wrist_2_joint);
    (*this)(2,2) = ((((sin_q_elbow_joint * sin_q_shoulder_lift_joint)-(cos_q_elbow_joint * cos_q_shoulder_lift_joint)) * sin_q_wrist_1_joint)+(((-cos_q_elbow_joint * sin_q_shoulder_lift_joint)-(sin_q_elbow_joint * cos_q_shoulder_lift_joint)) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint;
    (*this)(2,3) = (((((- tz_shoulder_pan_joint * sin_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * cos_q_elbow_joint * cos_q_shoulder_lift_joint)+( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint)- ty_wrist_1_joint) * sin_q_wrist_1_joint)+((( tz_shoulder_pan_joint * cos_q_elbow_joint * sin_q_shoulder_lift_joint)+( tz_shoulder_pan_joint * sin_q_elbow_joint * cos_q_shoulder_lift_joint)-( ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint)- tx_wrist_1_joint) * cos_q_wrist_1_joint)) * sin_q_wrist_2_joint)+((- tz_wrist_2_joint- tz_elbow_joint- ty_shoulder_lift_joint) * cos_q_wrist_2_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_upper_arm_link_X_fr_shoulder_link::Type_fr_upper_arm_link_X_fr_shoulder_link()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - ty_shoulder_lift_joint;    // Maxima DSL: -_k__ty_shoulder_lift_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_upper_arm_link_X_fr_shoulder_link& HomogeneousTransforms::Type_fr_upper_arm_link_X_fr_shoulder_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    (*this)(0,0) = cos_q_shoulder_lift_joint;
    (*this)(0,2) = -sin_q_shoulder_lift_joint;
    (*this)(1,0) = -sin_q_shoulder_lift_joint;
    (*this)(1,2) = -cos_q_shoulder_lift_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_shoulder_link_X_fr_upper_arm_link::Type_fr_shoulder_link_X_fr_upper_arm_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) =  ty_shoulder_lift_joint;    // Maxima DSL: _k__ty_shoulder_lift_joint
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_shoulder_link_X_fr_upper_arm_link& HomogeneousTransforms::Type_fr_shoulder_link_X_fr_upper_arm_link::update(const state_t& q)
{
    Scalar sin_q_shoulder_lift_joint  = ScalarTraits::sin( q(SHOULDER_LIFT_JOINT) );
    Scalar cos_q_shoulder_lift_joint  = ScalarTraits::cos( q(SHOULDER_LIFT_JOINT) );
    (*this)(0,0) = cos_q_shoulder_lift_joint;
    (*this)(0,1) = -sin_q_shoulder_lift_joint;
    (*this)(2,0) = -sin_q_shoulder_lift_joint;
    (*this)(2,1) = -cos_q_shoulder_lift_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_forearm_link_X_fr_upper_arm_link::Type_fr_forearm_link_X_fr_upper_arm_link()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_elbow_joint;    // Maxima DSL: -_k__tz_elbow_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_forearm_link_X_fr_upper_arm_link& HomogeneousTransforms::Type_fr_forearm_link_X_fr_upper_arm_link::update(const state_t& q)
{
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    (*this)(0,0) = cos_q_elbow_joint;
    (*this)(0,1) = sin_q_elbow_joint;
    (*this)(0,3) = (- ty_elbow_joint * sin_q_elbow_joint)-( tx_elbow_joint * cos_q_elbow_joint);
    (*this)(1,0) = -sin_q_elbow_joint;
    (*this)(1,1) = cos_q_elbow_joint;
    (*this)(1,3) = ( tx_elbow_joint * sin_q_elbow_joint)-( ty_elbow_joint * cos_q_elbow_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_upper_arm_link_X_fr_forearm_link::Type_fr_upper_arm_link_X_fr_forearm_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_elbow_joint;    // Maxima DSL: _k__tx_elbow_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_elbow_joint;    // Maxima DSL: _k__ty_elbow_joint
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_elbow_joint;    // Maxima DSL: _k__tz_elbow_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_upper_arm_link_X_fr_forearm_link& HomogeneousTransforms::Type_fr_upper_arm_link_X_fr_forearm_link::update(const state_t& q)
{
    Scalar sin_q_elbow_joint  = ScalarTraits::sin( q(ELBOW_JOINT) );
    Scalar cos_q_elbow_joint  = ScalarTraits::cos( q(ELBOW_JOINT) );
    (*this)(0,0) = cos_q_elbow_joint;
    (*this)(0,1) = -sin_q_elbow_joint;
    (*this)(1,0) = sin_q_elbow_joint;
    (*this)(1,1) = cos_q_elbow_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_wrist_1_link_X_fr_forearm_link::Type_fr_wrist_1_link_X_fr_forearm_link()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_wrist_1_link_X_fr_forearm_link& HomogeneousTransforms::Type_fr_wrist_1_link_X_fr_forearm_link::update(const state_t& q)
{
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    (*this)(0,0) = cos_q_wrist_1_joint;
    (*this)(0,1) = sin_q_wrist_1_joint;
    (*this)(0,3) = (- ty_wrist_1_joint * sin_q_wrist_1_joint)-( tx_wrist_1_joint * cos_q_wrist_1_joint);
    (*this)(1,0) = -sin_q_wrist_1_joint;
    (*this)(1,1) = cos_q_wrist_1_joint;
    (*this)(1,3) = ( tx_wrist_1_joint * sin_q_wrist_1_joint)-( ty_wrist_1_joint * cos_q_wrist_1_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_forearm_link_X_fr_wrist_1_link::Type_fr_forearm_link_X_fr_wrist_1_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_wrist_1_joint;    // Maxima DSL: _k__tx_wrist_1_joint
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_wrist_1_joint;    // Maxima DSL: _k__ty_wrist_1_joint
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_forearm_link_X_fr_wrist_1_link& HomogeneousTransforms::Type_fr_forearm_link_X_fr_wrist_1_link::update(const state_t& q)
{
    Scalar sin_q_wrist_1_joint  = ScalarTraits::sin( q(WRIST_1_JOINT) );
    Scalar cos_q_wrist_1_joint  = ScalarTraits::cos( q(WRIST_1_JOINT) );
    (*this)(0,0) = cos_q_wrist_1_joint;
    (*this)(0,1) = -sin_q_wrist_1_joint;
    (*this)(1,0) = sin_q_wrist_1_joint;
    (*this)(1,1) = cos_q_wrist_1_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_wrist_2_link_X_fr_wrist_1_link::Type_fr_wrist_2_link_X_fr_wrist_1_link()
{
    (*this)(0,1) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_wrist_2_link_X_fr_wrist_1_link& HomogeneousTransforms::Type_fr_wrist_2_link_X_fr_wrist_1_link::update(const state_t& q)
{
    Scalar sin_q_wrist_2_joint  = ScalarTraits::sin( q(WRIST_2_JOINT) );
    Scalar cos_q_wrist_2_joint  = ScalarTraits::cos( q(WRIST_2_JOINT) );
    (*this)(0,0) = -cos_q_wrist_2_joint;
    (*this)(0,2) = sin_q_wrist_2_joint;
    (*this)(0,3) = - tz_wrist_2_joint * sin_q_wrist_2_joint;
    (*this)(1,0) = sin_q_wrist_2_joint;
    (*this)(1,2) = cos_q_wrist_2_joint;
    (*this)(1,3) = - tz_wrist_2_joint * cos_q_wrist_2_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_wrist_1_link_X_fr_wrist_2_link::Type_fr_wrist_1_link_X_fr_wrist_2_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_wrist_2_joint;    // Maxima DSL: _k__tz_wrist_2_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_wrist_1_link_X_fr_wrist_2_link& HomogeneousTransforms::Type_fr_wrist_1_link_X_fr_wrist_2_link::update(const state_t& q)
{
    Scalar sin_q_wrist_2_joint  = ScalarTraits::sin( q(WRIST_2_JOINT) );
    Scalar cos_q_wrist_2_joint  = ScalarTraits::cos( q(WRIST_2_JOINT) );
    (*this)(0,0) = -cos_q_wrist_2_joint;
    (*this)(0,1) = sin_q_wrist_2_joint;
    (*this)(2,0) = sin_q_wrist_2_joint;
    (*this)(2,1) = cos_q_wrist_2_joint;
    return *this;
}
HomogeneousTransforms::Type_fr_wrist_3_link_X_fr_wrist_2_link::Type_fr_wrist_3_link_X_fr_wrist_2_link()
{
    (*this)(0,1) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_wrist_3_link_X_fr_wrist_2_link& HomogeneousTransforms::Type_fr_wrist_3_link_X_fr_wrist_2_link::update(const state_t& q)
{
    Scalar sin_q_wrist_3_joint  = ScalarTraits::sin( q(WRIST_3_JOINT) );
    Scalar cos_q_wrist_3_joint  = ScalarTraits::cos( q(WRIST_3_JOINT) );
    (*this)(0,0) = -cos_q_wrist_3_joint;
    (*this)(0,2) = sin_q_wrist_3_joint;
    (*this)(0,3) = ( tx_wrist_3_joint * cos_q_wrist_3_joint)-( tz_wrist_3_joint * sin_q_wrist_3_joint);
    (*this)(1,0) = sin_q_wrist_3_joint;
    (*this)(1,2) = cos_q_wrist_3_joint;
    (*this)(1,3) = (- tx_wrist_3_joint * sin_q_wrist_3_joint)-( tz_wrist_3_joint * cos_q_wrist_3_joint);
    return *this;
}
HomogeneousTransforms::Type_fr_wrist_2_link_X_fr_wrist_3_link::Type_fr_wrist_2_link_X_fr_wrist_3_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_wrist_3_joint;    // Maxima DSL: _k__tx_wrist_3_joint
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_wrist_3_joint;    // Maxima DSL: _k__tz_wrist_3_joint
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_wrist_2_link_X_fr_wrist_3_link& HomogeneousTransforms::Type_fr_wrist_2_link_X_fr_wrist_3_link::update(const state_t& q)
{
    Scalar sin_q_wrist_3_joint  = ScalarTraits::sin( q(WRIST_3_JOINT) );
    Scalar cos_q_wrist_3_joint  = ScalarTraits::cos( q(WRIST_3_JOINT) );
    (*this)(0,0) = -cos_q_wrist_3_joint;
    (*this)(0,1) = sin_q_wrist_3_joint;
    (*this)(2,0) = sin_q_wrist_3_joint;
    (*this)(2,1) = cos_q_wrist_3_joint;
    return *this;
}

