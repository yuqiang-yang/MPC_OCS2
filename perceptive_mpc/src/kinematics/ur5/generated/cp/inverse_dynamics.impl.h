// Initialization of static-const data
template <typename TRAIT>
const typename iit::ur5e::dyn::tpl::InverseDynamics<TRAIT>::ExtForces
iit::ur5e::dyn::tpl::InverseDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::ur5e::dyn::tpl::InverseDynamics<TRAIT>::InverseDynamics(IProperties& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    shoulder_link_I(inertiaProps->getTensor_shoulder_link() ),
    upper_arm_link_I(inertiaProps->getTensor_upper_arm_link() ),
    forearm_link_I(inertiaProps->getTensor_forearm_link() ),
    wrist_1_link_I(inertiaProps->getTensor_wrist_1_link() ),
    wrist_2_link_I(inertiaProps->getTensor_wrist_2_link() ),
    wrist_3_link_I(inertiaProps->getTensor_wrist_3_link() )
    ,
        base_link_ur5e_I( inertiaProps->getTensor_base_link_ur5e() ),
        wrist_3_link_Ic(wrist_3_link_I)
{
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot ur5e, InverseDynamics<TRAIT>::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    shoulder_link_v.setZero();
    upper_arm_link_v.setZero();
    forearm_link_v.setZero();
    wrist_1_link_v.setZero();
    wrist_2_link_v.setZero();
    wrist_3_link_v.setZero();

    vcross.setZero();
}

template <typename TRAIT>
void iit::ur5e::dyn::tpl::InverseDynamics<TRAIT>::id(
    JointState& jForces, Acceleration& base_link_ur5e_a,
    const Acceleration& g, const Velocity& base_link_ur5e_v,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    base_link_ur5e_Ic = base_link_ur5e_I;
    shoulder_link_Ic = shoulder_link_I;
    upper_arm_link_Ic = upper_arm_link_I;
    forearm_link_Ic = forearm_link_I;
    wrist_1_link_Ic = wrist_1_link_I;
    wrist_2_link_Ic = wrist_2_link_I;

    // First pass, link 'shoulder_link'
    shoulder_link_v = ((xm->fr_shoulder_link_X_fr_base_link_ur5e) * base_link_ur5e_v);
    shoulder_link_v(iit::rbd::AZ) += qd(SHOULDER_PAN_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(shoulder_link_v, vcross);
    
    shoulder_link_a = (vcross.col(iit::rbd::AZ) * qd(SHOULDER_PAN_JOINT));
    shoulder_link_a(iit::rbd::AZ) += qdd(SHOULDER_PAN_JOINT);
    
    shoulder_link_f = shoulder_link_I * shoulder_link_a + iit::rbd::vxIv(shoulder_link_v, shoulder_link_I);
    
    // First pass, link 'upper_arm_link'
    upper_arm_link_v = ((xm->fr_upper_arm_link_X_fr_shoulder_link) * shoulder_link_v);
    upper_arm_link_v(iit::rbd::AZ) += qd(SHOULDER_LIFT_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(upper_arm_link_v, vcross);
    
    upper_arm_link_a = (xm->fr_upper_arm_link_X_fr_shoulder_link) * shoulder_link_a + vcross.col(iit::rbd::AZ) * qd(SHOULDER_LIFT_JOINT);
    upper_arm_link_a(iit::rbd::AZ) += qdd(SHOULDER_LIFT_JOINT);
    
    upper_arm_link_f = upper_arm_link_I * upper_arm_link_a + iit::rbd::vxIv(upper_arm_link_v, upper_arm_link_I);
    
    // First pass, link 'forearm_link'
    forearm_link_v = ((xm->fr_forearm_link_X_fr_upper_arm_link) * upper_arm_link_v);
    forearm_link_v(iit::rbd::AZ) += qd(ELBOW_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(forearm_link_v, vcross);
    
    forearm_link_a = (xm->fr_forearm_link_X_fr_upper_arm_link) * upper_arm_link_a + vcross.col(iit::rbd::AZ) * qd(ELBOW_JOINT);
    forearm_link_a(iit::rbd::AZ) += qdd(ELBOW_JOINT);
    
    forearm_link_f = forearm_link_I * forearm_link_a + iit::rbd::vxIv(forearm_link_v, forearm_link_I);
    
    // First pass, link 'wrist_1_link'
    wrist_1_link_v = ((xm->fr_wrist_1_link_X_fr_forearm_link) * forearm_link_v);
    wrist_1_link_v(iit::rbd::AZ) += qd(WRIST_1_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(wrist_1_link_v, vcross);
    
    wrist_1_link_a = (xm->fr_wrist_1_link_X_fr_forearm_link) * forearm_link_a + vcross.col(iit::rbd::AZ) * qd(WRIST_1_JOINT);
    wrist_1_link_a(iit::rbd::AZ) += qdd(WRIST_1_JOINT);
    
    wrist_1_link_f = wrist_1_link_I * wrist_1_link_a + iit::rbd::vxIv(wrist_1_link_v, wrist_1_link_I);
    
    // First pass, link 'wrist_2_link'
    wrist_2_link_v = ((xm->fr_wrist_2_link_X_fr_wrist_1_link) * wrist_1_link_v);
    wrist_2_link_v(iit::rbd::AZ) += qd(WRIST_2_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(wrist_2_link_v, vcross);
    
    wrist_2_link_a = (xm->fr_wrist_2_link_X_fr_wrist_1_link) * wrist_1_link_a + vcross.col(iit::rbd::AZ) * qd(WRIST_2_JOINT);
    wrist_2_link_a(iit::rbd::AZ) += qdd(WRIST_2_JOINT);
    
    wrist_2_link_f = wrist_2_link_I * wrist_2_link_a + iit::rbd::vxIv(wrist_2_link_v, wrist_2_link_I);
    
    // First pass, link 'wrist_3_link'
    wrist_3_link_v = ((xm->fr_wrist_3_link_X_fr_wrist_2_link) * wrist_2_link_v);
    wrist_3_link_v(iit::rbd::AZ) += qd(WRIST_3_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(wrist_3_link_v, vcross);
    
    wrist_3_link_a = (xm->fr_wrist_3_link_X_fr_wrist_2_link) * wrist_2_link_a + vcross.col(iit::rbd::AZ) * qd(WRIST_3_JOINT);
    wrist_3_link_a(iit::rbd::AZ) += qdd(WRIST_3_JOINT);
    
    wrist_3_link_f = wrist_3_link_I * wrist_3_link_a + iit::rbd::vxIv(wrist_3_link_v, wrist_3_link_I);
    
    // The force exerted on the floating base by the links
    base_link_ur5e_f = iit::rbd::vxIv(base_link_ur5e_v, base_link_ur5e_I);
    

    // Add the external forces:
    base_link_ur5e_f -= fext[BASE_LINK_UR5E];
    shoulder_link_f -= fext[SHOULDER_LINK];
    upper_arm_link_f -= fext[UPPER_ARM_LINK];
    forearm_link_f -= fext[FOREARM_LINK];
    wrist_1_link_f -= fext[WRIST_1_LINK];
    wrist_2_link_f -= fext[WRIST_2_LINK];
    wrist_3_link_f -= fext[WRIST_3_LINK];

    wrist_2_link_Ic = wrist_2_link_Ic + (xm->fr_wrist_3_link_X_fr_wrist_2_link).transpose() * wrist_3_link_Ic * (xm->fr_wrist_3_link_X_fr_wrist_2_link);
    wrist_2_link_f = wrist_2_link_f + (xm->fr_wrist_3_link_X_fr_wrist_2_link).transpose() * wrist_3_link_f;
    
    wrist_1_link_Ic = wrist_1_link_Ic + (xm->fr_wrist_2_link_X_fr_wrist_1_link).transpose() * wrist_2_link_Ic * (xm->fr_wrist_2_link_X_fr_wrist_1_link);
    wrist_1_link_f = wrist_1_link_f + (xm->fr_wrist_2_link_X_fr_wrist_1_link).transpose() * wrist_2_link_f;
    
    forearm_link_Ic = forearm_link_Ic + (xm->fr_wrist_1_link_X_fr_forearm_link).transpose() * wrist_1_link_Ic * (xm->fr_wrist_1_link_X_fr_forearm_link);
    forearm_link_f = forearm_link_f + (xm->fr_wrist_1_link_X_fr_forearm_link).transpose() * wrist_1_link_f;
    
    upper_arm_link_Ic = upper_arm_link_Ic + (xm->fr_forearm_link_X_fr_upper_arm_link).transpose() * forearm_link_Ic * (xm->fr_forearm_link_X_fr_upper_arm_link);
    upper_arm_link_f = upper_arm_link_f + (xm->fr_forearm_link_X_fr_upper_arm_link).transpose() * forearm_link_f;
    
    shoulder_link_Ic = shoulder_link_Ic + (xm->fr_upper_arm_link_X_fr_shoulder_link).transpose() * upper_arm_link_Ic * (xm->fr_upper_arm_link_X_fr_shoulder_link);
    shoulder_link_f = shoulder_link_f + (xm->fr_upper_arm_link_X_fr_shoulder_link).transpose() * upper_arm_link_f;
    
    base_link_ur5e_Ic = base_link_ur5e_Ic + (xm->fr_shoulder_link_X_fr_base_link_ur5e).transpose() * shoulder_link_Ic * (xm->fr_shoulder_link_X_fr_base_link_ur5e);
    base_link_ur5e_f = base_link_ur5e_f + (xm->fr_shoulder_link_X_fr_base_link_ur5e).transpose() * shoulder_link_f;
    

    // The base acceleration due to the force due to the movement of the links
    base_link_ur5e_a = - base_link_ur5e_Ic.inverse() * base_link_ur5e_f;
    
    shoulder_link_a = xm->fr_shoulder_link_X_fr_base_link_ur5e * base_link_ur5e_a;
    jForces(SHOULDER_PAN_JOINT) = (shoulder_link_Ic.row(iit::rbd::AZ) * shoulder_link_a + shoulder_link_f(iit::rbd::AZ));
    
    upper_arm_link_a = xm->fr_upper_arm_link_X_fr_shoulder_link * shoulder_link_a;
    jForces(SHOULDER_LIFT_JOINT) = (upper_arm_link_Ic.row(iit::rbd::AZ) * upper_arm_link_a + upper_arm_link_f(iit::rbd::AZ));
    
    forearm_link_a = xm->fr_forearm_link_X_fr_upper_arm_link * upper_arm_link_a;
    jForces(ELBOW_JOINT) = (forearm_link_Ic.row(iit::rbd::AZ) * forearm_link_a + forearm_link_f(iit::rbd::AZ));
    
    wrist_1_link_a = xm->fr_wrist_1_link_X_fr_forearm_link * forearm_link_a;
    jForces(WRIST_1_JOINT) = (wrist_1_link_Ic.row(iit::rbd::AZ) * wrist_1_link_a + wrist_1_link_f(iit::rbd::AZ));
    
    wrist_2_link_a = xm->fr_wrist_2_link_X_fr_wrist_1_link * wrist_1_link_a;
    jForces(WRIST_2_JOINT) = (wrist_2_link_Ic.row(iit::rbd::AZ) * wrist_2_link_a + wrist_2_link_f(iit::rbd::AZ));
    
    wrist_3_link_a = xm->fr_wrist_3_link_X_fr_wrist_2_link * wrist_2_link_a;
    jForces(WRIST_3_JOINT) = (wrist_3_link_Ic.row(iit::rbd::AZ) * wrist_3_link_a + wrist_3_link_f(iit::rbd::AZ));
    

    base_link_ur5e_a += g;
}

template <typename TRAIT>
void iit::ur5e::dyn::tpl::InverseDynamics<TRAIT>::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g)
{
    const Acceleration& base_link_ur5e_a = -g;

    // Link 'shoulder_link'
    shoulder_link_a = (xm->fr_shoulder_link_X_fr_base_link_ur5e) * base_link_ur5e_a;
    shoulder_link_f = shoulder_link_I * shoulder_link_a;
    // Link 'upper_arm_link'
    upper_arm_link_a = (xm->fr_upper_arm_link_X_fr_shoulder_link) * shoulder_link_a;
    upper_arm_link_f = upper_arm_link_I * upper_arm_link_a;
    // Link 'forearm_link'
    forearm_link_a = (xm->fr_forearm_link_X_fr_upper_arm_link) * upper_arm_link_a;
    forearm_link_f = forearm_link_I * forearm_link_a;
    // Link 'wrist_1_link'
    wrist_1_link_a = (xm->fr_wrist_1_link_X_fr_forearm_link) * forearm_link_a;
    wrist_1_link_f = wrist_1_link_I * wrist_1_link_a;
    // Link 'wrist_2_link'
    wrist_2_link_a = (xm->fr_wrist_2_link_X_fr_wrist_1_link) * wrist_1_link_a;
    wrist_2_link_f = wrist_2_link_I * wrist_2_link_a;
    // Link 'wrist_3_link'
    wrist_3_link_a = (xm->fr_wrist_3_link_X_fr_wrist_2_link) * wrist_2_link_a;
    wrist_3_link_f = wrist_3_link_I * wrist_3_link_a;

    base_link_ur5e_f = base_link_ur5e_I * base_link_ur5e_a;

    secondPass_fullyActuated(jForces);

    baseWrench = base_link_ur5e_f;
}

template <typename TRAIT>
void iit::ur5e::dyn::tpl::InverseDynamics<TRAIT>::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& base_link_ur5e_v, const JointState& qd)
{
    // Link 'shoulder_link'
    shoulder_link_v = ((xm->fr_shoulder_link_X_fr_base_link_ur5e) * base_link_ur5e_v);
    shoulder_link_v(iit::rbd::AZ) += qd(SHOULDER_PAN_JOINT);
    iit::rbd::motionCrossProductMx<Scalar>(shoulder_link_v, vcross);
    shoulder_link_a = (vcross.col(iit::rbd::AZ) * qd(SHOULDER_PAN_JOINT));
    shoulder_link_f = shoulder_link_I * shoulder_link_a + iit::rbd::vxIv(shoulder_link_v, shoulder_link_I);
    
    // Link 'upper_arm_link'
    upper_arm_link_v = ((xm->fr_upper_arm_link_X_fr_shoulder_link) * shoulder_link_v);
    upper_arm_link_v(iit::rbd::AZ) += qd(SHOULDER_LIFT_JOINT);
    iit::rbd::motionCrossProductMx<Scalar>(upper_arm_link_v, vcross);
    upper_arm_link_a = (xm->fr_upper_arm_link_X_fr_shoulder_link) * shoulder_link_a + vcross.col(iit::rbd::AZ) * qd(SHOULDER_LIFT_JOINT);
    upper_arm_link_f = upper_arm_link_I * upper_arm_link_a + iit::rbd::vxIv(upper_arm_link_v, upper_arm_link_I);
    
    // Link 'forearm_link'
    forearm_link_v = ((xm->fr_forearm_link_X_fr_upper_arm_link) * upper_arm_link_v);
    forearm_link_v(iit::rbd::AZ) += qd(ELBOW_JOINT);
    iit::rbd::motionCrossProductMx<Scalar>(forearm_link_v, vcross);
    forearm_link_a = (xm->fr_forearm_link_X_fr_upper_arm_link) * upper_arm_link_a + vcross.col(iit::rbd::AZ) * qd(ELBOW_JOINT);
    forearm_link_f = forearm_link_I * forearm_link_a + iit::rbd::vxIv(forearm_link_v, forearm_link_I);
    
    // Link 'wrist_1_link'
    wrist_1_link_v = ((xm->fr_wrist_1_link_X_fr_forearm_link) * forearm_link_v);
    wrist_1_link_v(iit::rbd::AZ) += qd(WRIST_1_JOINT);
    iit::rbd::motionCrossProductMx<Scalar>(wrist_1_link_v, vcross);
    wrist_1_link_a = (xm->fr_wrist_1_link_X_fr_forearm_link) * forearm_link_a + vcross.col(iit::rbd::AZ) * qd(WRIST_1_JOINT);
    wrist_1_link_f = wrist_1_link_I * wrist_1_link_a + iit::rbd::vxIv(wrist_1_link_v, wrist_1_link_I);
    
    // Link 'wrist_2_link'
    wrist_2_link_v = ((xm->fr_wrist_2_link_X_fr_wrist_1_link) * wrist_1_link_v);
    wrist_2_link_v(iit::rbd::AZ) += qd(WRIST_2_JOINT);
    iit::rbd::motionCrossProductMx<Scalar>(wrist_2_link_v, vcross);
    wrist_2_link_a = (xm->fr_wrist_2_link_X_fr_wrist_1_link) * wrist_1_link_a + vcross.col(iit::rbd::AZ) * qd(WRIST_2_JOINT);
    wrist_2_link_f = wrist_2_link_I * wrist_2_link_a + iit::rbd::vxIv(wrist_2_link_v, wrist_2_link_I);
    
    // Link 'wrist_3_link'
    wrist_3_link_v = ((xm->fr_wrist_3_link_X_fr_wrist_2_link) * wrist_2_link_v);
    wrist_3_link_v(iit::rbd::AZ) += qd(WRIST_3_JOINT);
    iit::rbd::motionCrossProductMx<Scalar>(wrist_3_link_v, vcross);
    wrist_3_link_a = (xm->fr_wrist_3_link_X_fr_wrist_2_link) * wrist_2_link_a + vcross.col(iit::rbd::AZ) * qd(WRIST_3_JOINT);
    wrist_3_link_f = wrist_3_link_I * wrist_3_link_a + iit::rbd::vxIv(wrist_3_link_v, wrist_3_link_I);
    

    base_link_ur5e_f = iit::rbd::vxIv(base_link_ur5e_v, base_link_ur5e_I);

    secondPass_fullyActuated(jForces);

    baseWrench = base_link_ur5e_f;
}

template <typename TRAIT>
void iit::ur5e::dyn::tpl::InverseDynamics<TRAIT>::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_link_ur5e_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    Acceleration base_link_ur5e_a = baseAccel -g;

    // First pass, link 'shoulder_link'
    shoulder_link_v = ((xm->fr_shoulder_link_X_fr_base_link_ur5e) * base_link_ur5e_v);
    shoulder_link_v(iit::rbd::AZ) += qd(SHOULDER_PAN_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(shoulder_link_v, vcross);
    
    shoulder_link_a = (xm->fr_shoulder_link_X_fr_base_link_ur5e) * base_link_ur5e_a + vcross.col(iit::rbd::AZ) * qd(SHOULDER_PAN_JOINT);
    shoulder_link_a(iit::rbd::AZ) += qdd(SHOULDER_PAN_JOINT);
    
    shoulder_link_f = shoulder_link_I * shoulder_link_a + iit::rbd::vxIv(shoulder_link_v, shoulder_link_I) - fext[SHOULDER_LINK];
    
    // First pass, link 'upper_arm_link'
    upper_arm_link_v = ((xm->fr_upper_arm_link_X_fr_shoulder_link) * shoulder_link_v);
    upper_arm_link_v(iit::rbd::AZ) += qd(SHOULDER_LIFT_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(upper_arm_link_v, vcross);
    
    upper_arm_link_a = (xm->fr_upper_arm_link_X_fr_shoulder_link) * shoulder_link_a + vcross.col(iit::rbd::AZ) * qd(SHOULDER_LIFT_JOINT);
    upper_arm_link_a(iit::rbd::AZ) += qdd(SHOULDER_LIFT_JOINT);
    
    upper_arm_link_f = upper_arm_link_I * upper_arm_link_a + iit::rbd::vxIv(upper_arm_link_v, upper_arm_link_I) - fext[UPPER_ARM_LINK];
    
    // First pass, link 'forearm_link'
    forearm_link_v = ((xm->fr_forearm_link_X_fr_upper_arm_link) * upper_arm_link_v);
    forearm_link_v(iit::rbd::AZ) += qd(ELBOW_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(forearm_link_v, vcross);
    
    forearm_link_a = (xm->fr_forearm_link_X_fr_upper_arm_link) * upper_arm_link_a + vcross.col(iit::rbd::AZ) * qd(ELBOW_JOINT);
    forearm_link_a(iit::rbd::AZ) += qdd(ELBOW_JOINT);
    
    forearm_link_f = forearm_link_I * forearm_link_a + iit::rbd::vxIv(forearm_link_v, forearm_link_I) - fext[FOREARM_LINK];
    
    // First pass, link 'wrist_1_link'
    wrist_1_link_v = ((xm->fr_wrist_1_link_X_fr_forearm_link) * forearm_link_v);
    wrist_1_link_v(iit::rbd::AZ) += qd(WRIST_1_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(wrist_1_link_v, vcross);
    
    wrist_1_link_a = (xm->fr_wrist_1_link_X_fr_forearm_link) * forearm_link_a + vcross.col(iit::rbd::AZ) * qd(WRIST_1_JOINT);
    wrist_1_link_a(iit::rbd::AZ) += qdd(WRIST_1_JOINT);
    
    wrist_1_link_f = wrist_1_link_I * wrist_1_link_a + iit::rbd::vxIv(wrist_1_link_v, wrist_1_link_I) - fext[WRIST_1_LINK];
    
    // First pass, link 'wrist_2_link'
    wrist_2_link_v = ((xm->fr_wrist_2_link_X_fr_wrist_1_link) * wrist_1_link_v);
    wrist_2_link_v(iit::rbd::AZ) += qd(WRIST_2_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(wrist_2_link_v, vcross);
    
    wrist_2_link_a = (xm->fr_wrist_2_link_X_fr_wrist_1_link) * wrist_1_link_a + vcross.col(iit::rbd::AZ) * qd(WRIST_2_JOINT);
    wrist_2_link_a(iit::rbd::AZ) += qdd(WRIST_2_JOINT);
    
    wrist_2_link_f = wrist_2_link_I * wrist_2_link_a + iit::rbd::vxIv(wrist_2_link_v, wrist_2_link_I) - fext[WRIST_2_LINK];
    
    // First pass, link 'wrist_3_link'
    wrist_3_link_v = ((xm->fr_wrist_3_link_X_fr_wrist_2_link) * wrist_2_link_v);
    wrist_3_link_v(iit::rbd::AZ) += qd(WRIST_3_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(wrist_3_link_v, vcross);
    
    wrist_3_link_a = (xm->fr_wrist_3_link_X_fr_wrist_2_link) * wrist_2_link_a + vcross.col(iit::rbd::AZ) * qd(WRIST_3_JOINT);
    wrist_3_link_a(iit::rbd::AZ) += qdd(WRIST_3_JOINT);
    
    wrist_3_link_f = wrist_3_link_I * wrist_3_link_a + iit::rbd::vxIv(wrist_3_link_v, wrist_3_link_I) - fext[WRIST_3_LINK];
    

    // The base
    base_link_ur5e_f = base_link_ur5e_I * base_link_ur5e_a + iit::rbd::vxIv(base_link_ur5e_v, base_link_ur5e_I) - fext[BASE_LINK_UR5E];

    secondPass_fullyActuated(jForces);

    baseWrench = base_link_ur5e_f;
}

template <typename TRAIT>
void iit::ur5e::dyn::tpl::InverseDynamics<TRAIT>::secondPass_fullyActuated(JointState& jForces)
{
    // Link 'wrist_3_link'
    jForces(WRIST_3_JOINT) = wrist_3_link_f(iit::rbd::AZ);
    wrist_2_link_f += xm->fr_wrist_3_link_X_fr_wrist_2_link.transpose() * wrist_3_link_f;
    // Link 'wrist_2_link'
    jForces(WRIST_2_JOINT) = wrist_2_link_f(iit::rbd::AZ);
    wrist_1_link_f += xm->fr_wrist_2_link_X_fr_wrist_1_link.transpose() * wrist_2_link_f;
    // Link 'wrist_1_link'
    jForces(WRIST_1_JOINT) = wrist_1_link_f(iit::rbd::AZ);
    forearm_link_f += xm->fr_wrist_1_link_X_fr_forearm_link.transpose() * wrist_1_link_f;
    // Link 'forearm_link'
    jForces(ELBOW_JOINT) = forearm_link_f(iit::rbd::AZ);
    upper_arm_link_f += xm->fr_forearm_link_X_fr_upper_arm_link.transpose() * forearm_link_f;
    // Link 'upper_arm_link'
    jForces(SHOULDER_LIFT_JOINT) = upper_arm_link_f(iit::rbd::AZ);
    shoulder_link_f += xm->fr_upper_arm_link_X_fr_shoulder_link.transpose() * upper_arm_link_f;
    // Link 'shoulder_link'
    jForces(SHOULDER_PAN_JOINT) = shoulder_link_f(iit::rbd::AZ);
    base_link_ur5e_f += xm->fr_shoulder_link_X_fr_base_link_ur5e.transpose() * shoulder_link_f;
}

