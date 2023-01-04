
template <typename TRAIT>
iit::ur5e::tpl::Jacobians<TRAIT>::Jacobians
    ()
     : 
    fr_base_link_ur5e_J_fr_forearm_link(), 
    fr_base_link_ur5e_J_fr_shoulder_link(), 
    fr_base_link_ur5e_J_fr_upper_arm_link(), 
    fr_base_link_ur5e_J_fr_wrist_1_link(), 
    fr_base_link_ur5e_J_fr_wrist_2_link(), 
    fr_base_link_ur5e_J_fr_wrist_3_link()
{
    updateParameters();
}

template <typename TRAIT>
void iit::ur5e::tpl::Jacobians<TRAIT>::updateParameters() {
}


template <typename TRAIT>
iit::ur5e::tpl::Jacobians<TRAIT>::Type_fr_base_link_ur5e_J_fr_forearm_link::Type_fr_base_link_ur5e_J_fr_forearm_link()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(3,2) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,2) = 0;
}

template <typename TRAIT>
const typename iit::ur5e::tpl::Jacobians<TRAIT>::Type_fr_base_link_ur5e_J_fr_forearm_link& iit::ur5e::tpl::Jacobians<TRAIT>::Type_fr_base_link_ur5e_J_fr_forearm_link::update(const JState& jState) {
    Scalar s_q_shoulder_pan_joint_;
    Scalar s_q_shoulder_lift_joint_;
    Scalar c_q_shoulder_pan_joint_;
    Scalar c_q_shoulder_lift_joint_;
    
    s_q_shoulder_pan_joint_ = TRAIT::sin( jState(SHOULDER_PAN_JOINT));
    s_q_shoulder_lift_joint_ = TRAIT::sin( jState(SHOULDER_LIFT_JOINT));
    c_q_shoulder_pan_joint_ = TRAIT::cos( jState(SHOULDER_PAN_JOINT));
    c_q_shoulder_lift_joint_ = TRAIT::cos( jState(SHOULDER_LIFT_JOINT));
    
    (*this)(0,1) = - s_q_shoulder_pan_joint_;
    (*this)(0,2) = - s_q_shoulder_pan_joint_;
    (*this)(1,1) =  c_q_shoulder_pan_joint_;
    (*this)(1,2) =  c_q_shoulder_pan_joint_;
    (*this)(3,0) = (((- 0.425 *  c_q_shoulder_lift_joint_) *  s_q_shoulder_pan_joint_) - ( 0.007 *  c_q_shoulder_pan_joint_));
    (*this)(3,1) = ((- 0.425 *  s_q_shoulder_lift_joint_) *  c_q_shoulder_pan_joint_);
    (*this)(4,0) = ((( 0.425 *  c_q_shoulder_lift_joint_) *  c_q_shoulder_pan_joint_) - ( 0.007 *  s_q_shoulder_pan_joint_));
    (*this)(4,1) = ((- 0.425 *  s_q_shoulder_lift_joint_) *  s_q_shoulder_pan_joint_);
    (*this)(5,1) = (- 0.425 *  c_q_shoulder_lift_joint_);
    return *this;
}
template <typename TRAIT>
iit::ur5e::tpl::Jacobians<TRAIT>::Type_fr_base_link_ur5e_J_fr_shoulder_link::Type_fr_base_link_ur5e_J_fr_shoulder_link()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(3,0) = 0;
    (*this)(4,0) = 0;
    (*this)(5,0) = 0;
}

template <typename TRAIT>
const typename iit::ur5e::tpl::Jacobians<TRAIT>::Type_fr_base_link_ur5e_J_fr_shoulder_link& iit::ur5e::tpl::Jacobians<TRAIT>::Type_fr_base_link_ur5e_J_fr_shoulder_link::update(const JState& jState) {
    return *this;
}
template <typename TRAIT>
iit::ur5e::tpl::Jacobians<TRAIT>::Type_fr_base_link_ur5e_J_fr_upper_arm_link::Type_fr_base_link_ur5e_J_fr_upper_arm_link()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(3,1) = 0;
    (*this)(4,1) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
}

template <typename TRAIT>
const typename iit::ur5e::tpl::Jacobians<TRAIT>::Type_fr_base_link_ur5e_J_fr_upper_arm_link& iit::ur5e::tpl::Jacobians<TRAIT>::Type_fr_base_link_ur5e_J_fr_upper_arm_link::update(const JState& jState) {
    Scalar s_q_shoulder_pan_joint_;
    Scalar c_q_shoulder_pan_joint_;
    
    s_q_shoulder_pan_joint_ = TRAIT::sin( jState(SHOULDER_PAN_JOINT));
    c_q_shoulder_pan_joint_ = TRAIT::cos( jState(SHOULDER_PAN_JOINT));
    
    (*this)(0,1) = - s_q_shoulder_pan_joint_;
    (*this)(1,1) =  c_q_shoulder_pan_joint_;
    (*this)(3,0) = (- 0.138 *  c_q_shoulder_pan_joint_);
    (*this)(4,0) = (- 0.138 *  s_q_shoulder_pan_joint_);
    return *this;
}
template <typename TRAIT>
iit::ur5e::tpl::Jacobians<TRAIT>::Type_fr_base_link_ur5e_J_fr_wrist_1_link::Type_fr_base_link_ur5e_J_fr_wrist_1_link()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,3) = 0;
    (*this)(4,3) = 0;
    (*this)(5,0) = 0;
    (*this)(5,3) = 0;
}

template <typename TRAIT>
const typename iit::ur5e::tpl::Jacobians<TRAIT>::Type_fr_base_link_ur5e_J_fr_wrist_1_link& iit::ur5e::tpl::Jacobians<TRAIT>::Type_fr_base_link_ur5e_J_fr_wrist_1_link::update(const JState& jState) {
    Scalar s_q_shoulder_pan_joint_;
    Scalar s_q_elbow_joint_;
    Scalar s_q_shoulder_lift_joint_;
    Scalar c_q_shoulder_pan_joint_;
    Scalar c_q_elbow_joint_;
    Scalar c_q_shoulder_lift_joint_;
    
    s_q_shoulder_pan_joint_ = TRAIT::sin( jState(SHOULDER_PAN_JOINT));
    s_q_elbow_joint_ = TRAIT::sin( jState(ELBOW_JOINT));
    s_q_shoulder_lift_joint_ = TRAIT::sin( jState(SHOULDER_LIFT_JOINT));
    c_q_shoulder_pan_joint_ = TRAIT::cos( jState(SHOULDER_PAN_JOINT));
    c_q_elbow_joint_ = TRAIT::cos( jState(ELBOW_JOINT));
    c_q_shoulder_lift_joint_ = TRAIT::cos( jState(SHOULDER_LIFT_JOINT));
    
    (*this)(0,1) = - s_q_shoulder_pan_joint_;
    (*this)(0,2) = - s_q_shoulder_pan_joint_;
    (*this)(0,3) = - s_q_shoulder_pan_joint_;
    (*this)(1,1) =  c_q_shoulder_pan_joint_;
    (*this)(1,2) =  c_q_shoulder_pan_joint_;
    (*this)(1,3) =  c_q_shoulder_pan_joint_;
    (*this)(3,0) = ((((( 0.392 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_) + (((- 0.392 *  c_q_elbow_joint_) -  0.425) *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_) - ( 0.007 *  c_q_shoulder_pan_joint_));
    (*this)(3,1) = (((((- 0.392 *  c_q_elbow_joint_) -  0.425) *  s_q_shoulder_lift_joint_) - (( 0.392 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_);
    (*this)(3,2) = ((((- 0.392 *  c_q_elbow_joint_) *  s_q_shoulder_lift_joint_) - (( 0.392 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_);
    (*this)(4,0) = (((((( 0.392 *  c_q_elbow_joint_) +  0.425) *  c_q_shoulder_lift_joint_) - (( 0.392 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_) - ( 0.007 *  s_q_shoulder_pan_joint_));
    (*this)(4,1) = (((((- 0.392 *  c_q_elbow_joint_) -  0.425) *  s_q_shoulder_lift_joint_) - (( 0.392 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_);
    (*this)(4,2) = ((((- 0.392 *  c_q_elbow_joint_) *  s_q_shoulder_lift_joint_) - (( 0.392 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_);
    (*this)(5,1) = ((( 0.392 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_) + (((- 0.392 *  c_q_elbow_joint_) -  0.425) *  c_q_shoulder_lift_joint_));
    (*this)(5,2) = ((( 0.392 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_) - (( 0.392 *  c_q_elbow_joint_) *  c_q_shoulder_lift_joint_));
    return *this;
}
template <typename TRAIT>
iit::ur5e::tpl::Jacobians<TRAIT>::Type_fr_base_link_ur5e_J_fr_wrist_2_link::Type_fr_base_link_ur5e_J_fr_wrist_2_link()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,3) = 0;
    (*this)(3,4) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
}

template <typename TRAIT>
const typename iit::ur5e::tpl::Jacobians<TRAIT>::Type_fr_base_link_ur5e_J_fr_wrist_2_link& iit::ur5e::tpl::Jacobians<TRAIT>::Type_fr_base_link_ur5e_J_fr_wrist_2_link::update(const JState& jState) {
    Scalar s_q_shoulder_pan_joint_;
    Scalar s_q_elbow_joint_;
    Scalar s_q_shoulder_lift_joint_;
    Scalar s_q_wrist_1_joint_;
    Scalar c_q_elbow_joint_;
    Scalar c_q_shoulder_lift_joint_;
    Scalar c_q_shoulder_pan_joint_;
    Scalar c_q_wrist_1_joint_;
    
    s_q_shoulder_pan_joint_ = TRAIT::sin( jState(SHOULDER_PAN_JOINT));
    s_q_elbow_joint_ = TRAIT::sin( jState(ELBOW_JOINT));
    s_q_shoulder_lift_joint_ = TRAIT::sin( jState(SHOULDER_LIFT_JOINT));
    s_q_wrist_1_joint_ = TRAIT::sin( jState(WRIST_1_JOINT));
    c_q_elbow_joint_ = TRAIT::cos( jState(ELBOW_JOINT));
    c_q_shoulder_lift_joint_ = TRAIT::cos( jState(SHOULDER_LIFT_JOINT));
    c_q_shoulder_pan_joint_ = TRAIT::cos( jState(SHOULDER_PAN_JOINT));
    c_q_wrist_1_joint_ = TRAIT::cos( jState(WRIST_1_JOINT));
    
    (*this)(0,1) = - s_q_shoulder_pan_joint_;
    (*this)(0,2) = - s_q_shoulder_pan_joint_;
    (*this)(0,3) = - s_q_shoulder_pan_joint_;
    (*this)(0,4) = ((((( s_q_elbow_joint_ *  s_q_shoulder_lift_joint_) - ( c_q_elbow_joint_ *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_) *  s_q_wrist_1_joint_) + ((((- c_q_elbow_joint_ *  s_q_shoulder_lift_joint_) - ( s_q_elbow_joint_ *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_) *  c_q_wrist_1_joint_));
    (*this)(1,1) =  c_q_shoulder_pan_joint_;
    (*this)(1,2) =  c_q_shoulder_pan_joint_;
    (*this)(1,3) =  c_q_shoulder_pan_joint_;
    (*this)(1,4) = ((((( s_q_elbow_joint_ *  s_q_shoulder_lift_joint_) - ( c_q_elbow_joint_ *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_) *  s_q_wrist_1_joint_) + ((((- c_q_elbow_joint_ *  s_q_shoulder_lift_joint_) - ( s_q_elbow_joint_ *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_) *  c_q_wrist_1_joint_));
    (*this)(2,4) = (((( c_q_elbow_joint_ *  s_q_shoulder_lift_joint_) + ( s_q_elbow_joint_ *  c_q_shoulder_lift_joint_)) *  s_q_wrist_1_joint_) + ((( s_q_elbow_joint_ *  s_q_shoulder_lift_joint_) - ( c_q_elbow_joint_ *  c_q_shoulder_lift_joint_)) *  c_q_wrist_1_joint_));
    (*this)(3,0) = ((((( 0.392 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_) + (((- 0.392 *  c_q_elbow_joint_) -  0.425) *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_) - ( 0.1333 *  c_q_shoulder_pan_joint_));
    (*this)(3,1) = (((((- 0.392 *  c_q_elbow_joint_) -  0.425) *  s_q_shoulder_lift_joint_) - (( 0.392 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_);
    (*this)(3,2) = ((((- 0.392 *  c_q_elbow_joint_) *  s_q_shoulder_lift_joint_) - (( 0.392 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_);
    (*this)(4,0) = (((((( 0.392 *  c_q_elbow_joint_) +  0.425) *  c_q_shoulder_lift_joint_) - (( 0.392 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_) - ( 0.1333 *  s_q_shoulder_pan_joint_));
    (*this)(4,1) = (((((- 0.392 *  c_q_elbow_joint_) -  0.425) *  s_q_shoulder_lift_joint_) - (( 0.392 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_);
    (*this)(4,2) = ((((- 0.392 *  c_q_elbow_joint_) *  s_q_shoulder_lift_joint_) - (( 0.392 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_);
    (*this)(5,1) = ((( 0.392 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_) + (((- 0.392 *  c_q_elbow_joint_) -  0.425) *  c_q_shoulder_lift_joint_));
    (*this)(5,2) = ((( 0.392 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_) - (( 0.392 *  c_q_elbow_joint_) *  c_q_shoulder_lift_joint_));
    return *this;
}
template <typename TRAIT>
iit::ur5e::tpl::Jacobians<TRAIT>::Type_fr_base_link_ur5e_J_fr_wrist_3_link::Type_fr_base_link_ur5e_J_fr_wrist_3_link()
{
    (*this)(0,0) = 0;
    (*this)(1,0) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 0;
}

template <typename TRAIT>
const typename iit::ur5e::tpl::Jacobians<TRAIT>::Type_fr_base_link_ur5e_J_fr_wrist_3_link& iit::ur5e::tpl::Jacobians<TRAIT>::Type_fr_base_link_ur5e_J_fr_wrist_3_link::update(const JState& jState) {
    Scalar s_q_shoulder_pan_joint_;
    Scalar s_q_elbow_joint_;
    Scalar s_q_shoulder_lift_joint_;
    Scalar s_q_wrist_1_joint_;
    Scalar s_q_wrist_2_joint_;
    Scalar c_q_elbow_joint_;
    Scalar c_q_shoulder_lift_joint_;
    Scalar c_q_shoulder_pan_joint_;
    Scalar c_q_wrist_1_joint_;
    Scalar c_q_wrist_2_joint_;
    
    s_q_shoulder_pan_joint_ = TRAIT::sin( jState(SHOULDER_PAN_JOINT));
    s_q_elbow_joint_ = TRAIT::sin( jState(ELBOW_JOINT));
    s_q_shoulder_lift_joint_ = TRAIT::sin( jState(SHOULDER_LIFT_JOINT));
    s_q_wrist_1_joint_ = TRAIT::sin( jState(WRIST_1_JOINT));
    s_q_wrist_2_joint_ = TRAIT::sin( jState(WRIST_2_JOINT));
    c_q_elbow_joint_ = TRAIT::cos( jState(ELBOW_JOINT));
    c_q_shoulder_lift_joint_ = TRAIT::cos( jState(SHOULDER_LIFT_JOINT));
    c_q_shoulder_pan_joint_ = TRAIT::cos( jState(SHOULDER_PAN_JOINT));
    c_q_wrist_1_joint_ = TRAIT::cos( jState(WRIST_1_JOINT));
    c_q_wrist_2_joint_ = TRAIT::cos( jState(WRIST_2_JOINT));
    
    (*this)(0,1) = - s_q_shoulder_pan_joint_;
    (*this)(0,2) = - s_q_shoulder_pan_joint_;
    (*this)(0,3) = - s_q_shoulder_pan_joint_;
    (*this)(0,4) = ((((( s_q_elbow_joint_ *  s_q_shoulder_lift_joint_) - ( c_q_elbow_joint_ *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_) *  s_q_wrist_1_joint_) + ((((- c_q_elbow_joint_ *  s_q_shoulder_lift_joint_) - ( s_q_elbow_joint_ *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_) *  c_q_wrist_1_joint_));
    (*this)(0,5) = (((((((- c_q_elbow_joint_ *  s_q_shoulder_lift_joint_) - ( s_q_elbow_joint_ *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_) *  s_q_wrist_1_joint_) + (((( c_q_elbow_joint_ *  c_q_shoulder_lift_joint_) - ( s_q_elbow_joint_ *  s_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_) *  c_q_wrist_1_joint_)) *  s_q_wrist_2_joint_) - ( s_q_shoulder_pan_joint_ *  c_q_wrist_2_joint_));
    (*this)(1,1) =  c_q_shoulder_pan_joint_;
    (*this)(1,2) =  c_q_shoulder_pan_joint_;
    (*this)(1,3) =  c_q_shoulder_pan_joint_;
    (*this)(1,4) = ((((( s_q_elbow_joint_ *  s_q_shoulder_lift_joint_) - ( c_q_elbow_joint_ *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_) *  s_q_wrist_1_joint_) + ((((- c_q_elbow_joint_ *  s_q_shoulder_lift_joint_) - ( s_q_elbow_joint_ *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_) *  c_q_wrist_1_joint_));
    (*this)(1,5) = (((((((- c_q_elbow_joint_ *  s_q_shoulder_lift_joint_) - ( s_q_elbow_joint_ *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_) *  s_q_wrist_1_joint_) + (((( c_q_elbow_joint_ *  c_q_shoulder_lift_joint_) - ( s_q_elbow_joint_ *  s_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_) *  c_q_wrist_1_joint_)) *  s_q_wrist_2_joint_) + ( c_q_shoulder_pan_joint_ *  c_q_wrist_2_joint_));
    (*this)(2,4) = (((( c_q_elbow_joint_ *  s_q_shoulder_lift_joint_) + ( s_q_elbow_joint_ *  c_q_shoulder_lift_joint_)) *  s_q_wrist_1_joint_) + ((( s_q_elbow_joint_ *  s_q_shoulder_lift_joint_) - ( c_q_elbow_joint_ *  c_q_shoulder_lift_joint_)) *  c_q_wrist_1_joint_));
    (*this)(2,5) = ((((( s_q_elbow_joint_ *  s_q_shoulder_lift_joint_) - ( c_q_elbow_joint_ *  c_q_shoulder_lift_joint_)) *  s_q_wrist_1_joint_) + (((- c_q_elbow_joint_ *  s_q_shoulder_lift_joint_) - ( s_q_elbow_joint_ *  c_q_shoulder_lift_joint_)) *  c_q_wrist_1_joint_)) *  s_q_wrist_2_joint_);
    (*this)(3,0) = (((((((( 0.0997 *  c_q_elbow_joint_) *  c_q_shoulder_lift_joint_) - (( 0.0997 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_) *  s_q_wrist_1_joint_) + ((((( 0.0997 *  c_q_elbow_joint_) *  s_q_shoulder_lift_joint_) + (( 0.0997 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_) *  c_q_wrist_1_joint_)) + (((( 0.392 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_) + (((- 0.392 *  c_q_elbow_joint_) -  0.425) *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_)) - ( 0.1333 *  c_q_shoulder_pan_joint_));
    (*this)(3,1) = ((((((( 0.0997 *  c_q_elbow_joint_) *  s_q_shoulder_lift_joint_) + (( 0.0997 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_) *  s_q_wrist_1_joint_) + ((((( 0.0997 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_) - (( 0.0997 *  c_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_) *  c_q_wrist_1_joint_)) + (((((- 0.392 *  c_q_elbow_joint_) -  0.425) *  s_q_shoulder_lift_joint_) - (( 0.392 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_));
    (*this)(3,2) = ((((((( 0.0997 *  c_q_elbow_joint_) *  s_q_shoulder_lift_joint_) + (( 0.0997 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_) *  s_q_wrist_1_joint_) + ((((( 0.0997 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_) - (( 0.0997 *  c_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_) *  c_q_wrist_1_joint_)) + ((((- 0.392 *  c_q_elbow_joint_) *  s_q_shoulder_lift_joint_) - (( 0.392 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_));
    (*this)(3,3) = (((((( 0.0997 *  c_q_elbow_joint_) *  s_q_shoulder_lift_joint_) + (( 0.0997 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_) *  s_q_wrist_1_joint_) + ((((( 0.0997 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_) - (( 0.0997 *  c_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_) *  c_q_wrist_1_joint_));
    (*this)(4,0) = (((((((( 0.0997 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_) - (( 0.0997 *  c_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_) *  s_q_wrist_1_joint_) + (((((- 0.0997 *  c_q_elbow_joint_) *  s_q_shoulder_lift_joint_) - (( 0.0997 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_) *  c_q_wrist_1_joint_)) - ( 0.1333 *  s_q_shoulder_pan_joint_)) + ((((( 0.392 *  c_q_elbow_joint_) +  0.425) *  c_q_shoulder_lift_joint_) - (( 0.392 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_)) *  c_q_shoulder_pan_joint_));
    (*this)(4,1) = ((((((( 0.0997 *  c_q_elbow_joint_) *  s_q_shoulder_lift_joint_) + (( 0.0997 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_) *  s_q_wrist_1_joint_) + ((((( 0.0997 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_) - (( 0.0997 *  c_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_) *  c_q_wrist_1_joint_)) + (((((- 0.392 *  c_q_elbow_joint_) -  0.425) *  s_q_shoulder_lift_joint_) - (( 0.392 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_));
    (*this)(4,2) = ((((((( 0.0997 *  c_q_elbow_joint_) *  s_q_shoulder_lift_joint_) + (( 0.0997 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_) *  s_q_wrist_1_joint_) + ((((( 0.0997 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_) - (( 0.0997 *  c_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_) *  c_q_wrist_1_joint_)) + ((((- 0.392 *  c_q_elbow_joint_) *  s_q_shoulder_lift_joint_) - (( 0.392 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_));
    (*this)(4,3) = (((((( 0.0997 *  c_q_elbow_joint_) *  s_q_shoulder_lift_joint_) + (( 0.0997 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_) *  s_q_wrist_1_joint_) + ((((( 0.0997 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_) - (( 0.0997 *  c_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  s_q_shoulder_pan_joint_) *  c_q_wrist_1_joint_));
    (*this)(5,1) = ((((((( 0.0997 *  c_q_elbow_joint_) *  c_q_shoulder_lift_joint_) - (( 0.0997 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_)) *  s_q_wrist_1_joint_) + (((( 0.0997 *  c_q_elbow_joint_) *  s_q_shoulder_lift_joint_) + (( 0.0997 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  c_q_wrist_1_joint_)) + (( 0.392 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_)) + (((- 0.392 *  c_q_elbow_joint_) -  0.425) *  c_q_shoulder_lift_joint_));
    (*this)(5,2) = ((((((( 0.0997 *  c_q_elbow_joint_) *  c_q_shoulder_lift_joint_) - (( 0.0997 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_)) *  s_q_wrist_1_joint_) + (((( 0.0997 *  c_q_elbow_joint_) *  s_q_shoulder_lift_joint_) + (( 0.0997 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  c_q_wrist_1_joint_)) + (( 0.392 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_)) - (( 0.392 *  c_q_elbow_joint_) *  c_q_shoulder_lift_joint_));
    (*this)(5,3) = ((((( 0.0997 *  c_q_elbow_joint_) *  c_q_shoulder_lift_joint_) - (( 0.0997 *  s_q_elbow_joint_) *  s_q_shoulder_lift_joint_)) *  s_q_wrist_1_joint_) + (((( 0.0997 *  c_q_elbow_joint_) *  s_q_shoulder_lift_joint_) + (( 0.0997 *  s_q_elbow_joint_) *  c_q_shoulder_lift_joint_)) *  c_q_wrist_1_joint_));
    return *this;
}
