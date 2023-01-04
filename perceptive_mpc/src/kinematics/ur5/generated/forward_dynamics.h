#ifndef IIT_ROBOT_UR5E_FORWARD_DYNAMICS_H_
#define IIT_ROBOT_UR5E_FORWARD_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/robcogen_commons.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace iit {
namespace ur5e {
namespace dyn {

/**
 * The Forward Dynamics routine for the robot ur5e.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */

namespace tpl {

template <typename TRAIT>
class ForwardDynamics {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Convenient type aliases:

    typedef typename TRAIT::Scalar Scalar;

    typedef iit::rbd::Core<Scalar> CoreS;

    typedef typename CoreS::ForceVector Force;
    typedef typename CoreS::VelocityVector Velocity;
    typedef typename CoreS::VelocityVector Acceleration;
    typedef typename CoreS::Column6D Column6DS;
    typedef typename CoreS::Matrix66 Matrix66S;
    typedef LinkDataMap<Force> ExtForces;
    typedef typename iit::ur5e::tpl::JointState<Scalar> JointState;
    typedef iit::ur5e::tpl::MotionTransforms<TRAIT> MTransforms;

public:
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot ur5e, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(InertiaProperties<TRAIT>& in, MTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param base_link_ur5e_a
     * \param base_link_ur5e_v
     * \param g the gravity acceleration vector, expressed in the
     *          base coordinates
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
       JointState& qdd, Acceleration& base_link_ur5e_a, // output parameters,
       const Velocity& base_link_ur5e_v, const Acceleration& g,
       const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, Acceleration& base_link_ur5e_a, // output parameters,
        const Velocity& base_link_ur5e_v, const Acceleration& g,
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    InertiaProperties<TRAIT>* inertiaProps;
    MTransforms* motionTransforms;

    Matrix66S vcross; // support variable
    Matrix66S Ia_r;   // support variable, articulated inertia in the case of a revolute joint
    // Link 'base_link_ur5e'
    Matrix66S base_link_ur5e_AI;
    Force base_link_ur5e_p;

    // Link 'shoulder_link' :
    Matrix66S shoulder_link_AI;
    Velocity shoulder_link_a;
    Velocity shoulder_link_v;
    Velocity shoulder_link_c;
    Force    shoulder_link_p;

    Column6DS shoulder_link_U;
    Scalar shoulder_link_D;
    Scalar shoulder_link_u;
    // Link 'upper_arm_link' :
    Matrix66S upper_arm_link_AI;
    Velocity upper_arm_link_a;
    Velocity upper_arm_link_v;
    Velocity upper_arm_link_c;
    Force    upper_arm_link_p;

    Column6DS upper_arm_link_U;
    Scalar upper_arm_link_D;
    Scalar upper_arm_link_u;
    // Link 'forearm_link' :
    Matrix66S forearm_link_AI;
    Velocity forearm_link_a;
    Velocity forearm_link_v;
    Velocity forearm_link_c;
    Force    forearm_link_p;

    Column6DS forearm_link_U;
    Scalar forearm_link_D;
    Scalar forearm_link_u;
    // Link 'wrist_1_link' :
    Matrix66S wrist_1_link_AI;
    Velocity wrist_1_link_a;
    Velocity wrist_1_link_v;
    Velocity wrist_1_link_c;
    Force    wrist_1_link_p;

    Column6DS wrist_1_link_U;
    Scalar wrist_1_link_D;
    Scalar wrist_1_link_u;
    // Link 'wrist_2_link' :
    Matrix66S wrist_2_link_AI;
    Velocity wrist_2_link_a;
    Velocity wrist_2_link_v;
    Velocity wrist_2_link_c;
    Force    wrist_2_link_p;

    Column6DS wrist_2_link_U;
    Scalar wrist_2_link_D;
    Scalar wrist_2_link_u;
    // Link 'wrist_3_link' :
    Matrix66S wrist_3_link_AI;
    Velocity wrist_3_link_a;
    Velocity wrist_3_link_v;
    Velocity wrist_3_link_c;
    Force    wrist_3_link_p;

    Column6DS wrist_3_link_U;
    Scalar wrist_3_link_D;
    Scalar wrist_3_link_u;
private:
    static const ExtForces zeroExtForces;
};

template <typename TRAIT>
inline void ForwardDynamics<TRAIT>::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_shoulder_link_X_fr_base_link_ur5e)(q);
    (motionTransforms-> fr_upper_arm_link_X_fr_shoulder_link)(q);
    (motionTransforms-> fr_forearm_link_X_fr_upper_arm_link)(q);
    (motionTransforms-> fr_wrist_1_link_X_fr_forearm_link)(q);
    (motionTransforms-> fr_wrist_2_link_X_fr_wrist_1_link)(q);
    (motionTransforms-> fr_wrist_3_link_X_fr_wrist_2_link)(q);
}

template <typename TRAIT>
inline void ForwardDynamics<TRAIT>::fd(
    JointState& qdd, Acceleration& base_link_ur5e_a, // output parameters,
    const Velocity& base_link_ur5e_v, const Acceleration& g,
    const JointState& q,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    setJointStatus(q);
    fd(qdd, base_link_ur5e_a, base_link_ur5e_v, g, qd, tau, fext);
}

}

typedef tpl::ForwardDynamics<iit::rbd::DoubleTrait> ForwardDynamics;

}
}
}

#include "forward_dynamics.impl.h"

#endif
