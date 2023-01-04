

//Implementation of default constructor
template <typename TRAIT>
iit::ur5e::dyn::tpl::JSIM<TRAIT>::JSIM(IProperties& inertiaProperties, FTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    wrist_3_link_Ic(linkInertias.getTensor_wrist_3_link())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA tpl::JSIM<TRAIT>::operator()
#define Fcol(j) (tpl::JSIM<TRAIT>:: template block<6,1>(0,(j)+6))
#define F(i,j) DATA((i),(j)+6)

template <typename TRAIT>
const typename iit::ur5e::dyn::tpl::JSIM<TRAIT>& iit::ur5e::dyn::tpl::JSIM<TRAIT>::update(const JointState& state) {

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_wrist_2_link_X_fr_wrist_3_link(state);
    frcTransf -> fr_wrist_1_link_X_fr_wrist_2_link(state);
    frcTransf -> fr_forearm_link_X_fr_wrist_1_link(state);
    frcTransf -> fr_upper_arm_link_X_fr_forearm_link(state);
    frcTransf -> fr_shoulder_link_X_fr_upper_arm_link(state);
    frcTransf -> fr_base_link_ur5e_X_fr_shoulder_link(state);

    // Initializes the composite inertia tensors
    base_link_ur5e_Ic = linkInertias.getTensor_base_link_ur5e();
    shoulder_link_Ic = linkInertias.getTensor_shoulder_link();
    upper_arm_link_Ic = linkInertias.getTensor_upper_arm_link();
    forearm_link_Ic = linkInertias.getTensor_forearm_link();
    wrist_1_link_Ic = linkInertias.getTensor_wrist_1_link();
    wrist_2_link_Ic = linkInertias.getTensor_wrist_2_link();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link wrist_3_link:
    iit::rbd::transformInertia<Scalar>(wrist_3_link_Ic, frcTransf -> fr_wrist_2_link_X_fr_wrist_3_link, Ic_spare);
    wrist_2_link_Ic += Ic_spare;

    Fcol(WRIST_3_JOINT) = wrist_3_link_Ic.col(iit::rbd::AZ);
    DATA(WRIST_3_JOINT+6, WRIST_3_JOINT+6) = Fcol(WRIST_3_JOINT)(iit::rbd::AZ);

    Fcol(WRIST_3_JOINT) = frcTransf -> fr_wrist_2_link_X_fr_wrist_3_link * Fcol(WRIST_3_JOINT);
    DATA(WRIST_3_JOINT+6, WRIST_2_JOINT+6) = F(iit::rbd::AZ,WRIST_3_JOINT);
    DATA(WRIST_2_JOINT+6, WRIST_3_JOINT+6) = DATA(WRIST_3_JOINT+6, WRIST_2_JOINT+6);
    Fcol(WRIST_3_JOINT) = frcTransf -> fr_wrist_1_link_X_fr_wrist_2_link * Fcol(WRIST_3_JOINT);
    DATA(WRIST_3_JOINT+6, WRIST_1_JOINT+6) = F(iit::rbd::AZ,WRIST_3_JOINT);
    DATA(WRIST_1_JOINT+6, WRIST_3_JOINT+6) = DATA(WRIST_3_JOINT+6, WRIST_1_JOINT+6);
    Fcol(WRIST_3_JOINT) = frcTransf -> fr_forearm_link_X_fr_wrist_1_link * Fcol(WRIST_3_JOINT);
    DATA(WRIST_3_JOINT+6, ELBOW_JOINT+6) = F(iit::rbd::AZ,WRIST_3_JOINT);
    DATA(ELBOW_JOINT+6, WRIST_3_JOINT+6) = DATA(WRIST_3_JOINT+6, ELBOW_JOINT+6);
    Fcol(WRIST_3_JOINT) = frcTransf -> fr_upper_arm_link_X_fr_forearm_link * Fcol(WRIST_3_JOINT);
    DATA(WRIST_3_JOINT+6, SHOULDER_LIFT_JOINT+6) = F(iit::rbd::AZ,WRIST_3_JOINT);
    DATA(SHOULDER_LIFT_JOINT+6, WRIST_3_JOINT+6) = DATA(WRIST_3_JOINT+6, SHOULDER_LIFT_JOINT+6);
    Fcol(WRIST_3_JOINT) = frcTransf -> fr_shoulder_link_X_fr_upper_arm_link * Fcol(WRIST_3_JOINT);
    DATA(WRIST_3_JOINT+6, SHOULDER_PAN_JOINT+6) = F(iit::rbd::AZ,WRIST_3_JOINT);
    DATA(SHOULDER_PAN_JOINT+6, WRIST_3_JOINT+6) = DATA(WRIST_3_JOINT+6, SHOULDER_PAN_JOINT+6);
    Fcol(WRIST_3_JOINT) = frcTransf -> fr_base_link_ur5e_X_fr_shoulder_link * Fcol(WRIST_3_JOINT);

    // Link wrist_2_link:
    iit::rbd::transformInertia<Scalar>(wrist_2_link_Ic, frcTransf -> fr_wrist_1_link_X_fr_wrist_2_link, Ic_spare);
    wrist_1_link_Ic += Ic_spare;

    Fcol(WRIST_2_JOINT) = wrist_2_link_Ic.col(iit::rbd::AZ);
    DATA(WRIST_2_JOINT+6, WRIST_2_JOINT+6) = Fcol(WRIST_2_JOINT)(iit::rbd::AZ);

    Fcol(WRIST_2_JOINT) = frcTransf -> fr_wrist_1_link_X_fr_wrist_2_link * Fcol(WRIST_2_JOINT);
    DATA(WRIST_2_JOINT+6, WRIST_1_JOINT+6) = F(iit::rbd::AZ,WRIST_2_JOINT);
    DATA(WRIST_1_JOINT+6, WRIST_2_JOINT+6) = DATA(WRIST_2_JOINT+6, WRIST_1_JOINT+6);
    Fcol(WRIST_2_JOINT) = frcTransf -> fr_forearm_link_X_fr_wrist_1_link * Fcol(WRIST_2_JOINT);
    DATA(WRIST_2_JOINT+6, ELBOW_JOINT+6) = F(iit::rbd::AZ,WRIST_2_JOINT);
    DATA(ELBOW_JOINT+6, WRIST_2_JOINT+6) = DATA(WRIST_2_JOINT+6, ELBOW_JOINT+6);
    Fcol(WRIST_2_JOINT) = frcTransf -> fr_upper_arm_link_X_fr_forearm_link * Fcol(WRIST_2_JOINT);
    DATA(WRIST_2_JOINT+6, SHOULDER_LIFT_JOINT+6) = F(iit::rbd::AZ,WRIST_2_JOINT);
    DATA(SHOULDER_LIFT_JOINT+6, WRIST_2_JOINT+6) = DATA(WRIST_2_JOINT+6, SHOULDER_LIFT_JOINT+6);
    Fcol(WRIST_2_JOINT) = frcTransf -> fr_shoulder_link_X_fr_upper_arm_link * Fcol(WRIST_2_JOINT);
    DATA(WRIST_2_JOINT+6, SHOULDER_PAN_JOINT+6) = F(iit::rbd::AZ,WRIST_2_JOINT);
    DATA(SHOULDER_PAN_JOINT+6, WRIST_2_JOINT+6) = DATA(WRIST_2_JOINT+6, SHOULDER_PAN_JOINT+6);
    Fcol(WRIST_2_JOINT) = frcTransf -> fr_base_link_ur5e_X_fr_shoulder_link * Fcol(WRIST_2_JOINT);

    // Link wrist_1_link:
    iit::rbd::transformInertia<Scalar>(wrist_1_link_Ic, frcTransf -> fr_forearm_link_X_fr_wrist_1_link, Ic_spare);
    forearm_link_Ic += Ic_spare;

    Fcol(WRIST_1_JOINT) = wrist_1_link_Ic.col(iit::rbd::AZ);
    DATA(WRIST_1_JOINT+6, WRIST_1_JOINT+6) = Fcol(WRIST_1_JOINT)(iit::rbd::AZ);

    Fcol(WRIST_1_JOINT) = frcTransf -> fr_forearm_link_X_fr_wrist_1_link * Fcol(WRIST_1_JOINT);
    DATA(WRIST_1_JOINT+6, ELBOW_JOINT+6) = F(iit::rbd::AZ,WRIST_1_JOINT);
    DATA(ELBOW_JOINT+6, WRIST_1_JOINT+6) = DATA(WRIST_1_JOINT+6, ELBOW_JOINT+6);
    Fcol(WRIST_1_JOINT) = frcTransf -> fr_upper_arm_link_X_fr_forearm_link * Fcol(WRIST_1_JOINT);
    DATA(WRIST_1_JOINT+6, SHOULDER_LIFT_JOINT+6) = F(iit::rbd::AZ,WRIST_1_JOINT);
    DATA(SHOULDER_LIFT_JOINT+6, WRIST_1_JOINT+6) = DATA(WRIST_1_JOINT+6, SHOULDER_LIFT_JOINT+6);
    Fcol(WRIST_1_JOINT) = frcTransf -> fr_shoulder_link_X_fr_upper_arm_link * Fcol(WRIST_1_JOINT);
    DATA(WRIST_1_JOINT+6, SHOULDER_PAN_JOINT+6) = F(iit::rbd::AZ,WRIST_1_JOINT);
    DATA(SHOULDER_PAN_JOINT+6, WRIST_1_JOINT+6) = DATA(WRIST_1_JOINT+6, SHOULDER_PAN_JOINT+6);
    Fcol(WRIST_1_JOINT) = frcTransf -> fr_base_link_ur5e_X_fr_shoulder_link * Fcol(WRIST_1_JOINT);

    // Link forearm_link:
    iit::rbd::transformInertia<Scalar>(forearm_link_Ic, frcTransf -> fr_upper_arm_link_X_fr_forearm_link, Ic_spare);
    upper_arm_link_Ic += Ic_spare;

    Fcol(ELBOW_JOINT) = forearm_link_Ic.col(iit::rbd::AZ);
    DATA(ELBOW_JOINT+6, ELBOW_JOINT+6) = Fcol(ELBOW_JOINT)(iit::rbd::AZ);

    Fcol(ELBOW_JOINT) = frcTransf -> fr_upper_arm_link_X_fr_forearm_link * Fcol(ELBOW_JOINT);
    DATA(ELBOW_JOINT+6, SHOULDER_LIFT_JOINT+6) = F(iit::rbd::AZ,ELBOW_JOINT);
    DATA(SHOULDER_LIFT_JOINT+6, ELBOW_JOINT+6) = DATA(ELBOW_JOINT+6, SHOULDER_LIFT_JOINT+6);
    Fcol(ELBOW_JOINT) = frcTransf -> fr_shoulder_link_X_fr_upper_arm_link * Fcol(ELBOW_JOINT);
    DATA(ELBOW_JOINT+6, SHOULDER_PAN_JOINT+6) = F(iit::rbd::AZ,ELBOW_JOINT);
    DATA(SHOULDER_PAN_JOINT+6, ELBOW_JOINT+6) = DATA(ELBOW_JOINT+6, SHOULDER_PAN_JOINT+6);
    Fcol(ELBOW_JOINT) = frcTransf -> fr_base_link_ur5e_X_fr_shoulder_link * Fcol(ELBOW_JOINT);

    // Link upper_arm_link:
    iit::rbd::transformInertia<Scalar>(upper_arm_link_Ic, frcTransf -> fr_shoulder_link_X_fr_upper_arm_link, Ic_spare);
    shoulder_link_Ic += Ic_spare;

    Fcol(SHOULDER_LIFT_JOINT) = upper_arm_link_Ic.col(iit::rbd::AZ);
    DATA(SHOULDER_LIFT_JOINT+6, SHOULDER_LIFT_JOINT+6) = Fcol(SHOULDER_LIFT_JOINT)(iit::rbd::AZ);

    Fcol(SHOULDER_LIFT_JOINT) = frcTransf -> fr_shoulder_link_X_fr_upper_arm_link * Fcol(SHOULDER_LIFT_JOINT);
    DATA(SHOULDER_LIFT_JOINT+6, SHOULDER_PAN_JOINT+6) = F(iit::rbd::AZ,SHOULDER_LIFT_JOINT);
    DATA(SHOULDER_PAN_JOINT+6, SHOULDER_LIFT_JOINT+6) = DATA(SHOULDER_LIFT_JOINT+6, SHOULDER_PAN_JOINT+6);
    Fcol(SHOULDER_LIFT_JOINT) = frcTransf -> fr_base_link_ur5e_X_fr_shoulder_link * Fcol(SHOULDER_LIFT_JOINT);

    // Link shoulder_link:
    iit::rbd::transformInertia<Scalar>(shoulder_link_Ic, frcTransf -> fr_base_link_ur5e_X_fr_shoulder_link, Ic_spare);
    base_link_ur5e_Ic += Ic_spare;

    Fcol(SHOULDER_PAN_JOINT) = shoulder_link_Ic.col(iit::rbd::AZ);
    DATA(SHOULDER_PAN_JOINT+6, SHOULDER_PAN_JOINT+6) = Fcol(SHOULDER_PAN_JOINT)(iit::rbd::AZ);

    Fcol(SHOULDER_PAN_JOINT) = frcTransf -> fr_base_link_ur5e_X_fr_shoulder_link * Fcol(SHOULDER_PAN_JOINT);

    // Copies the upper-right block into the lower-left block, after transposing
    JSIM<TRAIT>:: template block<6, 6>(6,0) = (JSIM<TRAIT>:: template block<6, 6>(0,6)).transpose();
    // The composite-inertia of the whole robot is the upper-left quadrant of the JSIM
    JSIM<TRAIT>:: template block<6,6>(0,0) = base_link_ur5e_Ic;
    return *this;
}

#undef DATA
#undef F

template <typename TRAIT>
void iit::ur5e::dyn::tpl::JSIM<TRAIT>::computeL() {
    L = this -> template triangularView<Eigen::Lower>();
    // Joint wrist_3_joint, index 5 :
    L(5, 5) = std::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(5, 3) = L(5, 3) / L(5, 5);
    L(5, 2) = L(5, 2) / L(5, 5);
    L(5, 1) = L(5, 1) / L(5, 5);
    L(5, 0) = L(5, 0) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    L(4, 3) = L(4, 3) - L(5, 4) * L(5, 3);
    L(4, 2) = L(4, 2) - L(5, 4) * L(5, 2);
    L(4, 1) = L(4, 1) - L(5, 4) * L(5, 1);
    L(4, 0) = L(4, 0) - L(5, 4) * L(5, 0);
    L(3, 3) = L(3, 3) - L(5, 3) * L(5, 3);
    L(3, 2) = L(3, 2) - L(5, 3) * L(5, 2);
    L(3, 1) = L(3, 1) - L(5, 3) * L(5, 1);
    L(3, 0) = L(3, 0) - L(5, 3) * L(5, 0);
    L(2, 2) = L(2, 2) - L(5, 2) * L(5, 2);
    L(2, 1) = L(2, 1) - L(5, 2) * L(5, 1);
    L(2, 0) = L(2, 0) - L(5, 2) * L(5, 0);
    L(1, 1) = L(1, 1) - L(5, 1) * L(5, 1);
    L(1, 0) = L(1, 0) - L(5, 1) * L(5, 0);
    L(0, 0) = L(0, 0) - L(5, 0) * L(5, 0);
    
    // Joint wrist_2_joint, index 4 :
    L(4, 4) = std::sqrt(L(4, 4));
    L(4, 3) = L(4, 3) / L(4, 4);
    L(4, 2) = L(4, 2) / L(4, 4);
    L(4, 1) = L(4, 1) / L(4, 4);
    L(4, 0) = L(4, 0) / L(4, 4);
    L(3, 3) = L(3, 3) - L(4, 3) * L(4, 3);
    L(3, 2) = L(3, 2) - L(4, 3) * L(4, 2);
    L(3, 1) = L(3, 1) - L(4, 3) * L(4, 1);
    L(3, 0) = L(3, 0) - L(4, 3) * L(4, 0);
    L(2, 2) = L(2, 2) - L(4, 2) * L(4, 2);
    L(2, 1) = L(2, 1) - L(4, 2) * L(4, 1);
    L(2, 0) = L(2, 0) - L(4, 2) * L(4, 0);
    L(1, 1) = L(1, 1) - L(4, 1) * L(4, 1);
    L(1, 0) = L(1, 0) - L(4, 1) * L(4, 0);
    L(0, 0) = L(0, 0) - L(4, 0) * L(4, 0);
    
    // Joint wrist_1_joint, index 3 :
    L(3, 3) = std::sqrt(L(3, 3));
    L(3, 2) = L(3, 2) / L(3, 3);
    L(3, 1) = L(3, 1) / L(3, 3);
    L(3, 0) = L(3, 0) / L(3, 3);
    L(2, 2) = L(2, 2) - L(3, 2) * L(3, 2);
    L(2, 1) = L(2, 1) - L(3, 2) * L(3, 1);
    L(2, 0) = L(2, 0) - L(3, 2) * L(3, 0);
    L(1, 1) = L(1, 1) - L(3, 1) * L(3, 1);
    L(1, 0) = L(1, 0) - L(3, 1) * L(3, 0);
    L(0, 0) = L(0, 0) - L(3, 0) * L(3, 0);
    
    // Joint elbow_joint, index 2 :
    L(2, 2) = std::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(2, 0) = L(2, 0) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    L(1, 0) = L(1, 0) - L(2, 1) * L(2, 0);
    L(0, 0) = L(0, 0) - L(2, 0) * L(2, 0);
    
    // Joint shoulder_lift_joint, index 1 :
    L(1, 1) = std::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
    
    // Joint shoulder_pan_joint, index 0 :
    L(0, 0) = std::sqrt(L(0, 0));
    
}

template <typename TRAIT>
void iit::ur5e::dyn::tpl::JSIM<TRAIT>::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
    inverse(1, 0) =  + (Linv(1, 0) * Linv(0, 0));
    inverse(0, 1) = inverse(1, 0);
    inverse(2, 2) =  + (Linv(2, 0) * Linv(2, 0)) + (Linv(2, 1) * Linv(2, 1)) + (Linv(2, 2) * Linv(2, 2));
    inverse(2, 1) =  + (Linv(2, 0) * Linv(1, 0)) + (Linv(2, 1) * Linv(1, 1));
    inverse(1, 2) = inverse(2, 1);
    inverse(2, 0) =  + (Linv(2, 0) * Linv(0, 0));
    inverse(0, 2) = inverse(2, 0);
    inverse(3, 3) =  + (Linv(3, 0) * Linv(3, 0)) + (Linv(3, 1) * Linv(3, 1)) + (Linv(3, 2) * Linv(3, 2)) + (Linv(3, 3) * Linv(3, 3));
    inverse(3, 2) =  + (Linv(3, 0) * Linv(2, 0)) + (Linv(3, 1) * Linv(2, 1)) + (Linv(3, 2) * Linv(2, 2));
    inverse(2, 3) = inverse(3, 2);
    inverse(3, 1) =  + (Linv(3, 0) * Linv(1, 0)) + (Linv(3, 1) * Linv(1, 1));
    inverse(1, 3) = inverse(3, 1);
    inverse(3, 0) =  + (Linv(3, 0) * Linv(0, 0));
    inverse(0, 3) = inverse(3, 0);
    inverse(4, 4) =  + (Linv(4, 0) * Linv(4, 0)) + (Linv(4, 1) * Linv(4, 1)) + (Linv(4, 2) * Linv(4, 2)) + (Linv(4, 3) * Linv(4, 3)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 3) =  + (Linv(4, 0) * Linv(3, 0)) + (Linv(4, 1) * Linv(3, 1)) + (Linv(4, 2) * Linv(3, 2)) + (Linv(4, 3) * Linv(3, 3));
    inverse(3, 4) = inverse(4, 3);
    inverse(4, 2) =  + (Linv(4, 0) * Linv(2, 0)) + (Linv(4, 1) * Linv(2, 1)) + (Linv(4, 2) * Linv(2, 2));
    inverse(2, 4) = inverse(4, 2);
    inverse(4, 1) =  + (Linv(4, 0) * Linv(1, 0)) + (Linv(4, 1) * Linv(1, 1));
    inverse(1, 4) = inverse(4, 1);
    inverse(4, 0) =  + (Linv(4, 0) * Linv(0, 0));
    inverse(0, 4) = inverse(4, 0);
    inverse(5, 5) =  + (Linv(5, 0) * Linv(5, 0)) + (Linv(5, 1) * Linv(5, 1)) + (Linv(5, 2) * Linv(5, 2)) + (Linv(5, 3) * Linv(5, 3)) + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) =  + (Linv(5, 0) * Linv(4, 0)) + (Linv(5, 1) * Linv(4, 1)) + (Linv(5, 2) * Linv(4, 2)) + (Linv(5, 3) * Linv(4, 3)) + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(5, 3) =  + (Linv(5, 0) * Linv(3, 0)) + (Linv(5, 1) * Linv(3, 1)) + (Linv(5, 2) * Linv(3, 2)) + (Linv(5, 3) * Linv(3, 3));
    inverse(3, 5) = inverse(5, 3);
    inverse(5, 2) =  + (Linv(5, 0) * Linv(2, 0)) + (Linv(5, 1) * Linv(2, 1)) + (Linv(5, 2) * Linv(2, 2));
    inverse(2, 5) = inverse(5, 2);
    inverse(5, 1) =  + (Linv(5, 0) * Linv(1, 0)) + (Linv(5, 1) * Linv(1, 1));
    inverse(1, 5) = inverse(5, 1);
    inverse(5, 0) =  + (Linv(5, 0) * Linv(0, 0));
    inverse(0, 5) = inverse(5, 0);
}

template <typename TRAIT>
void iit::ur5e::dyn::tpl::JSIM<TRAIT>::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
    Linv(2, 1) = - Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(2, 0) = - Linv(0, 0) * ((Linv(2, 1) * L(1, 0)) + (Linv(2, 2) * L(2, 0)) + 0);
    Linv(3, 2) = - Linv(2, 2) * ((Linv(3, 3) * L(3, 2)) + 0);
    Linv(3, 1) = - Linv(1, 1) * ((Linv(3, 2) * L(2, 1)) + (Linv(3, 3) * L(3, 1)) + 0);
    Linv(3, 0) = - Linv(0, 0) * ((Linv(3, 1) * L(1, 0)) + (Linv(3, 2) * L(2, 0)) + (Linv(3, 3) * L(3, 0)) + 0);
    Linv(4, 3) = - Linv(3, 3) * ((Linv(4, 4) * L(4, 3)) + 0);
    Linv(4, 2) = - Linv(2, 2) * ((Linv(4, 3) * L(3, 2)) + (Linv(4, 4) * L(4, 2)) + 0);
    Linv(4, 1) = - Linv(1, 1) * ((Linv(4, 2) * L(2, 1)) + (Linv(4, 3) * L(3, 1)) + (Linv(4, 4) * L(4, 1)) + 0);
    Linv(4, 0) = - Linv(0, 0) * ((Linv(4, 1) * L(1, 0)) + (Linv(4, 2) * L(2, 0)) + (Linv(4, 3) * L(3, 0)) + (Linv(4, 4) * L(4, 0)) + 0);
    Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(5, 3) = - Linv(3, 3) * ((Linv(5, 4) * L(4, 3)) + (Linv(5, 5) * L(5, 3)) + 0);
    Linv(5, 2) = - Linv(2, 2) * ((Linv(5, 3) * L(3, 2)) + (Linv(5, 4) * L(4, 2)) + (Linv(5, 5) * L(5, 2)) + 0);
    Linv(5, 1) = - Linv(1, 1) * ((Linv(5, 2) * L(2, 1)) + (Linv(5, 3) * L(3, 1)) + (Linv(5, 4) * L(4, 1)) + (Linv(5, 5) * L(5, 1)) + 0);
    Linv(5, 0) = - Linv(0, 0) * ((Linv(5, 1) * L(1, 0)) + (Linv(5, 2) * L(2, 0)) + (Linv(5, 3) * L(3, 0)) + (Linv(5, 4) * L(4, 0)) + (Linv(5, 5) * L(5, 0)) + 0);
}

