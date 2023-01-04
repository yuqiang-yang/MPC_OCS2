
// Initialization of static-const data
template <typename TRAIT>
const typename iit::ur5e::dyn::tpl::ForwardDynamics<TRAIT>::ExtForces
    iit::ur5e::dyn::tpl::ForwardDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::ur5e::dyn::tpl::ForwardDynamics<TRAIT>::ForwardDynamics(iit::ur5e::dyn::tpl::InertiaProperties<TRAIT>& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    shoulder_link_v.setZero();
    shoulder_link_c.setZero();
    upper_arm_link_v.setZero();
    upper_arm_link_c.setZero();
    forearm_link_v.setZero();
    forearm_link_c.setZero();
    wrist_1_link_v.setZero();
    wrist_1_link_c.setZero();
    wrist_2_link_v.setZero();
    wrist_2_link_c.setZero();
    wrist_3_link_v.setZero();
    wrist_3_link_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

template <typename TRAIT>
void iit::ur5e::dyn::tpl::ForwardDynamics<TRAIT>::fd(
    JointState& qdd,
    Acceleration& base_link_ur5e_a,
    const Velocity& base_link_ur5e_v,
    const Acceleration& g,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    base_link_ur5e_AI = inertiaProps->getTensor_base_link_ur5e();
    base_link_ur5e_p = - fext[BASE_LINK_UR5E];
    shoulder_link_AI = inertiaProps->getTensor_shoulder_link();
    shoulder_link_p = - fext[SHOULDER_LINK];
    upper_arm_link_AI = inertiaProps->getTensor_upper_arm_link();
    upper_arm_link_p = - fext[UPPER_ARM_LINK];
    forearm_link_AI = inertiaProps->getTensor_forearm_link();
    forearm_link_p = - fext[FOREARM_LINK];
    wrist_1_link_AI = inertiaProps->getTensor_wrist_1_link();
    wrist_1_link_p = - fext[WRIST_1_LINK];
    wrist_2_link_AI = inertiaProps->getTensor_wrist_2_link();
    wrist_2_link_p = - fext[WRIST_2_LINK];
    wrist_3_link_AI = inertiaProps->getTensor_wrist_3_link();
    wrist_3_link_p = - fext[WRIST_3_LINK];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link shoulder_link
    //  - The spatial velocity:
    shoulder_link_v = (motionTransforms-> fr_shoulder_link_X_fr_base_link_ur5e) * base_link_ur5e_v;
    shoulder_link_v(iit::rbd::AZ) += qd(SHOULDER_PAN_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(shoulder_link_v, vcross);
    shoulder_link_c = vcross.col(iit::rbd::AZ) * qd(SHOULDER_PAN_JOINT);
    
    //  - The bias force term:
    shoulder_link_p += iit::rbd::vxIv(shoulder_link_v, shoulder_link_AI);
    
    // + Link upper_arm_link
    //  - The spatial velocity:
    upper_arm_link_v = (motionTransforms-> fr_upper_arm_link_X_fr_shoulder_link) * shoulder_link_v;
    upper_arm_link_v(iit::rbd::AZ) += qd(SHOULDER_LIFT_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(upper_arm_link_v, vcross);
    upper_arm_link_c = vcross.col(iit::rbd::AZ) * qd(SHOULDER_LIFT_JOINT);
    
    //  - The bias force term:
    upper_arm_link_p += iit::rbd::vxIv(upper_arm_link_v, upper_arm_link_AI);
    
    // + Link forearm_link
    //  - The spatial velocity:
    forearm_link_v = (motionTransforms-> fr_forearm_link_X_fr_upper_arm_link) * upper_arm_link_v;
    forearm_link_v(iit::rbd::AZ) += qd(ELBOW_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(forearm_link_v, vcross);
    forearm_link_c = vcross.col(iit::rbd::AZ) * qd(ELBOW_JOINT);
    
    //  - The bias force term:
    forearm_link_p += iit::rbd::vxIv(forearm_link_v, forearm_link_AI);
    
    // + Link wrist_1_link
    //  - The spatial velocity:
    wrist_1_link_v = (motionTransforms-> fr_wrist_1_link_X_fr_forearm_link) * forearm_link_v;
    wrist_1_link_v(iit::rbd::AZ) += qd(WRIST_1_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(wrist_1_link_v, vcross);
    wrist_1_link_c = vcross.col(iit::rbd::AZ) * qd(WRIST_1_JOINT);
    
    //  - The bias force term:
    wrist_1_link_p += iit::rbd::vxIv(wrist_1_link_v, wrist_1_link_AI);
    
    // + Link wrist_2_link
    //  - The spatial velocity:
    wrist_2_link_v = (motionTransforms-> fr_wrist_2_link_X_fr_wrist_1_link) * wrist_1_link_v;
    wrist_2_link_v(iit::rbd::AZ) += qd(WRIST_2_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(wrist_2_link_v, vcross);
    wrist_2_link_c = vcross.col(iit::rbd::AZ) * qd(WRIST_2_JOINT);
    
    //  - The bias force term:
    wrist_2_link_p += iit::rbd::vxIv(wrist_2_link_v, wrist_2_link_AI);
    
    // + Link wrist_3_link
    //  - The spatial velocity:
    wrist_3_link_v = (motionTransforms-> fr_wrist_3_link_X_fr_wrist_2_link) * wrist_2_link_v;
    wrist_3_link_v(iit::rbd::AZ) += qd(WRIST_3_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(wrist_3_link_v, vcross);
    wrist_3_link_c = vcross.col(iit::rbd::AZ) * qd(WRIST_3_JOINT);
    
    //  - The bias force term:
    wrist_3_link_p += iit::rbd::vxIv(wrist_3_link_v, wrist_3_link_AI);
    
    // + The floating base body
    base_link_ur5e_p += iit::rbd::vxIv(base_link_ur5e_v, base_link_ur5e_AI);
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66S IaB;
    Force pa;
    
    // + Link wrist_3_link
    wrist_3_link_u = tau(WRIST_3_JOINT) - wrist_3_link_p(iit::rbd::AZ);
    wrist_3_link_U = wrist_3_link_AI.col(iit::rbd::AZ);
    wrist_3_link_D = wrist_3_link_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(wrist_3_link_AI, wrist_3_link_U, wrist_3_link_D, Ia_r);  // same as: Ia_r = wrist_3_link_AI - wrist_3_link_U/wrist_3_link_D * wrist_3_link_U.transpose();
    pa = wrist_3_link_p + Ia_r * wrist_3_link_c + wrist_3_link_U * wrist_3_link_u/wrist_3_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_wrist_3_link_X_fr_wrist_2_link, IaB);
    wrist_2_link_AI += IaB;
    wrist_2_link_p += (motionTransforms-> fr_wrist_3_link_X_fr_wrist_2_link).transpose() * pa;
    
    // + Link wrist_2_link
    wrist_2_link_u = tau(WRIST_2_JOINT) - wrist_2_link_p(iit::rbd::AZ);
    wrist_2_link_U = wrist_2_link_AI.col(iit::rbd::AZ);
    wrist_2_link_D = wrist_2_link_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(wrist_2_link_AI, wrist_2_link_U, wrist_2_link_D, Ia_r);  // same as: Ia_r = wrist_2_link_AI - wrist_2_link_U/wrist_2_link_D * wrist_2_link_U.transpose();
    pa = wrist_2_link_p + Ia_r * wrist_2_link_c + wrist_2_link_U * wrist_2_link_u/wrist_2_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_wrist_2_link_X_fr_wrist_1_link, IaB);
    wrist_1_link_AI += IaB;
    wrist_1_link_p += (motionTransforms-> fr_wrist_2_link_X_fr_wrist_1_link).transpose() * pa;
    
    // + Link wrist_1_link
    wrist_1_link_u = tau(WRIST_1_JOINT) - wrist_1_link_p(iit::rbd::AZ);
    wrist_1_link_U = wrist_1_link_AI.col(iit::rbd::AZ);
    wrist_1_link_D = wrist_1_link_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(wrist_1_link_AI, wrist_1_link_U, wrist_1_link_D, Ia_r);  // same as: Ia_r = wrist_1_link_AI - wrist_1_link_U/wrist_1_link_D * wrist_1_link_U.transpose();
    pa = wrist_1_link_p + Ia_r * wrist_1_link_c + wrist_1_link_U * wrist_1_link_u/wrist_1_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_wrist_1_link_X_fr_forearm_link, IaB);
    forearm_link_AI += IaB;
    forearm_link_p += (motionTransforms-> fr_wrist_1_link_X_fr_forearm_link).transpose() * pa;
    
    // + Link forearm_link
    forearm_link_u = tau(ELBOW_JOINT) - forearm_link_p(iit::rbd::AZ);
    forearm_link_U = forearm_link_AI.col(iit::rbd::AZ);
    forearm_link_D = forearm_link_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(forearm_link_AI, forearm_link_U, forearm_link_D, Ia_r);  // same as: Ia_r = forearm_link_AI - forearm_link_U/forearm_link_D * forearm_link_U.transpose();
    pa = forearm_link_p + Ia_r * forearm_link_c + forearm_link_U * forearm_link_u/forearm_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_forearm_link_X_fr_upper_arm_link, IaB);
    upper_arm_link_AI += IaB;
    upper_arm_link_p += (motionTransforms-> fr_forearm_link_X_fr_upper_arm_link).transpose() * pa;
    
    // + Link upper_arm_link
    upper_arm_link_u = tau(SHOULDER_LIFT_JOINT) - upper_arm_link_p(iit::rbd::AZ);
    upper_arm_link_U = upper_arm_link_AI.col(iit::rbd::AZ);
    upper_arm_link_D = upper_arm_link_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(upper_arm_link_AI, upper_arm_link_U, upper_arm_link_D, Ia_r);  // same as: Ia_r = upper_arm_link_AI - upper_arm_link_U/upper_arm_link_D * upper_arm_link_U.transpose();
    pa = upper_arm_link_p + Ia_r * upper_arm_link_c + upper_arm_link_U * upper_arm_link_u/upper_arm_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_upper_arm_link_X_fr_shoulder_link, IaB);
    shoulder_link_AI += IaB;
    shoulder_link_p += (motionTransforms-> fr_upper_arm_link_X_fr_shoulder_link).transpose() * pa;
    
    // + Link shoulder_link
    shoulder_link_u = tau(SHOULDER_PAN_JOINT) - shoulder_link_p(iit::rbd::AZ);
    shoulder_link_U = shoulder_link_AI.col(iit::rbd::AZ);
    shoulder_link_D = shoulder_link_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(shoulder_link_AI, shoulder_link_U, shoulder_link_D, Ia_r);  // same as: Ia_r = shoulder_link_AI - shoulder_link_U/shoulder_link_D * shoulder_link_U.transpose();
    pa = shoulder_link_p + Ia_r * shoulder_link_c + shoulder_link_U * shoulder_link_u/shoulder_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_shoulder_link_X_fr_base_link_ur5e, IaB);
    base_link_ur5e_AI += IaB;
    base_link_ur5e_p += (motionTransforms-> fr_shoulder_link_X_fr_base_link_ur5e).transpose() * pa;
    
    // + The acceleration of the floating base base_link_ur5e, without gravity
    base_link_ur5e_a = - TRAIT::solve(base_link_ur5e_AI, base_link_ur5e_p);  // base_link_ur5e_a = - IA^-1 * base_link_ur5e_p
    
    // ---------------------- THIRD PASS ---------------------- //
    shoulder_link_a = (motionTransforms-> fr_shoulder_link_X_fr_base_link_ur5e) * base_link_ur5e_a + shoulder_link_c;
    qdd(SHOULDER_PAN_JOINT) = (shoulder_link_u - shoulder_link_U.dot(shoulder_link_a)) / shoulder_link_D;
    shoulder_link_a(iit::rbd::AZ) += qdd(SHOULDER_PAN_JOINT);
    
    upper_arm_link_a = (motionTransforms-> fr_upper_arm_link_X_fr_shoulder_link) * shoulder_link_a + upper_arm_link_c;
    qdd(SHOULDER_LIFT_JOINT) = (upper_arm_link_u - upper_arm_link_U.dot(upper_arm_link_a)) / upper_arm_link_D;
    upper_arm_link_a(iit::rbd::AZ) += qdd(SHOULDER_LIFT_JOINT);
    
    forearm_link_a = (motionTransforms-> fr_forearm_link_X_fr_upper_arm_link) * upper_arm_link_a + forearm_link_c;
    qdd(ELBOW_JOINT) = (forearm_link_u - forearm_link_U.dot(forearm_link_a)) / forearm_link_D;
    forearm_link_a(iit::rbd::AZ) += qdd(ELBOW_JOINT);
    
    wrist_1_link_a = (motionTransforms-> fr_wrist_1_link_X_fr_forearm_link) * forearm_link_a + wrist_1_link_c;
    qdd(WRIST_1_JOINT) = (wrist_1_link_u - wrist_1_link_U.dot(wrist_1_link_a)) / wrist_1_link_D;
    wrist_1_link_a(iit::rbd::AZ) += qdd(WRIST_1_JOINT);
    
    wrist_2_link_a = (motionTransforms-> fr_wrist_2_link_X_fr_wrist_1_link) * wrist_1_link_a + wrist_2_link_c;
    qdd(WRIST_2_JOINT) = (wrist_2_link_u - wrist_2_link_U.dot(wrist_2_link_a)) / wrist_2_link_D;
    wrist_2_link_a(iit::rbd::AZ) += qdd(WRIST_2_JOINT);
    
    wrist_3_link_a = (motionTransforms-> fr_wrist_3_link_X_fr_wrist_2_link) * wrist_2_link_a + wrist_3_link_c;
    qdd(WRIST_3_JOINT) = (wrist_3_link_u - wrist_3_link_U.dot(wrist_3_link_a)) / wrist_3_link_D;
    wrist_3_link_a(iit::rbd::AZ) += qdd(WRIST_3_JOINT);
    
    
    // + Add gravity to the acceleration of the floating base
    base_link_ur5e_a += g;
}
