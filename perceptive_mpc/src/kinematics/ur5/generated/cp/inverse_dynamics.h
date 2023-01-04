#ifndef IIT_UR5E_INVERSE_DYNAMICS_H_
#define IIT_UR5E_INVERSE_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/robcogen_commons.h>
#include <iit/rbd/traits/DoubleTrait.h>

#include "declarations.h"
#include "inertia_properties.h"
#include "transforms.h"
#include "link_data_map.h"

namespace iit {
namespace ur5e {
namespace dyn {

/**
 * The Inverse Dynamics routine for the robot ur5e.
 *
 * In addition to the full Newton-Euler algorithm, specialized versions to compute
 * only certain terms are provided.
 * The parameters common to most of the methods are the joint status vector \c q, the
 * joint velocity vector \c qd and the acceleration vector \c qdd.
 *
 * Additional overloaded methods are provided without the \c q parameter. These
 * methods use the current configuration of the robot; they are provided for the
 * sake of efficiency, in case the motion transforms of the robot have already
 * been updated elsewhere with the most recent configuration (eg by a call to
 * setJointStatus()), so that it is useless to compute them again.
 *
 * Whenever present, the external forces parameter is a set of external
 * wrenches acting on the robot links. Each wrench must be expressed in
 * the reference frame of the link it is excerted on.
 */

namespace tpl {

template <typename TRAIT>
class InverseDynamics {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename TRAIT::Scalar Scalar;

    typedef iit::rbd::Core<Scalar> CoreS;

    typedef typename CoreS::ForceVector Force;
    typedef typename CoreS::VelocityVector Velocity;
    typedef typename CoreS::VelocityVector Acceleration;
    typedef typename CoreS::Matrix66 Matrix66s;
    typedef iit::rbd::tpl::InertiaMatrixDense<Scalar> InertiaMatrix;
    typedef iit::ur5e::tpl::JointState<Scalar> JointState;
    typedef LinkDataMap<Force> ExtForces;
    typedef iit::ur5e::tpl::MotionTransforms<TRAIT> MTransforms;
    typedef InertiaProperties<TRAIT> IProperties;

public:
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot ur5e, which will be used by this instance
     *     to compute inverse-dynamics.
     */
    InverseDynamics(IProperties& in, MTransforms& tr);

    /** \name Inverse dynamics
     * The full algorithm for the inverse dynamics of this robot.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[out] baseAccel the spatial acceleration of the robot base
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] base_link_ur5e_v the spatial velocity of the base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id(
        JointState& jForces, Acceleration& base_link_ur5e_a,
        const Acceleration& g, const Velocity& base_link_ur5e_v,
        const JointState& q, const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    void id(
        JointState& jForces, Acceleration& base_link_ur5e_a,
        const Acceleration& g, const Velocity& base_link_ur5e_v,
        const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Inverse dynamics, fully actuated base
     * The inverse dynamics algorithm for the floating base robot,
     * in the assumption of a fully actuated base.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] baseWrench the spatial force to be applied to the robot base to achieve
     *             the desired accelerations
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] base_link_ur5e_v the spatial velocity of the base
     * \param[in] baseAccel the desired spatial acceleration of the robot base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_link_ur5e_v, const Acceleration& baseAccel,
        const JointState& q, const JointState& qd, const JointState& qdd, const ExtForces& fext = zeroExtForces);
    void id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_link_ur5e_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Gravity terms, fully actuated base
     */
    ///@{
    void G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const JointState& q);
    void G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g);
    ///@}
    /** \name Centrifugal and Coriolis terms, fully actuated base
     *
     * These functions take only velocity inputs, that is, they assume
     * a zero spatial acceleration of the base (in addition to zero acceleration
     * at the actuated joints).
     * Note that this is NOT the same as imposing zero acceleration
     * at the virtual 6-dof-floting-base joint, which would result, in general,
     * in a non-zero spatial acceleration of the base, due to velocity
     * product terms.
     */
    ///@{
    void C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& base_link_ur5e_v, const JointState& q, const JointState& qd);
    void C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& base_link_ur5e_v, const JointState& qd);
    ///@}
    /** Updates all the kinematics transforms used by the inverse dynamics routine. */
    void setJointStatus(const JointState& q) const;

public:
    /** \name Getters
     * These functions return various spatial quantities used internally
     * by the inverse dynamics routines, like the spatial acceleration
     * of the links.
     *
     * The getters can be useful to retrieve the additional data that is not
     * returned explicitly by the inverse dynamics routines even though it
     * is computed. For example, after a call to the inverse dynamics,
     * the spatial velocity of all the links has been determined and
     * can be accessed.
     *
     * However, beware that certain routines might not use some of the
     * spatial quantities, which therefore would retain their last value
     * without being updated nor reset (for example, the spatial velocity
     * of the links is unaffected by the computation of the gravity terms).
     */
    ///@{
    const Force& getForce_base_link_ur5e() const { return base_link_ur5e_f; }
    const Velocity& getVelocity_shoulder_link() const { return shoulder_link_v; }
    const Acceleration& getAcceleration_shoulder_link() const { return shoulder_link_a; }
    const Force& getForce_shoulder_link() const { return shoulder_link_f; }
    const Velocity& getVelocity_upper_arm_link() const { return upper_arm_link_v; }
    const Acceleration& getAcceleration_upper_arm_link() const { return upper_arm_link_a; }
    const Force& getForce_upper_arm_link() const { return upper_arm_link_f; }
    const Velocity& getVelocity_forearm_link() const { return forearm_link_v; }
    const Acceleration& getAcceleration_forearm_link() const { return forearm_link_a; }
    const Force& getForce_forearm_link() const { return forearm_link_f; }
    const Velocity& getVelocity_wrist_1_link() const { return wrist_1_link_v; }
    const Acceleration& getAcceleration_wrist_1_link() const { return wrist_1_link_a; }
    const Force& getForce_wrist_1_link() const { return wrist_1_link_f; }
    const Velocity& getVelocity_wrist_2_link() const { return wrist_2_link_v; }
    const Acceleration& getAcceleration_wrist_2_link() const { return wrist_2_link_a; }
    const Force& getForce_wrist_2_link() const { return wrist_2_link_f; }
    const Velocity& getVelocity_wrist_3_link() const { return wrist_3_link_v; }
    const Acceleration& getAcceleration_wrist_3_link() const { return wrist_3_link_a; }
    const Force& getForce_wrist_3_link() const { return wrist_3_link_f; }
    ///@}
protected:
    void secondPass_fullyActuated(JointState& jForces);

private:
    IProperties* inertiaProps;
    MTransforms* xm;
private:
    Matrix66s vcross; // support variable
    // Link 'shoulder_link' :
    const InertiaMatrix& shoulder_link_I;
    Velocity      shoulder_link_v;
    Acceleration  shoulder_link_a;
    Force         shoulder_link_f;
    // Link 'upper_arm_link' :
    const InertiaMatrix& upper_arm_link_I;
    Velocity      upper_arm_link_v;
    Acceleration  upper_arm_link_a;
    Force         upper_arm_link_f;
    // Link 'forearm_link' :
    const InertiaMatrix& forearm_link_I;
    Velocity      forearm_link_v;
    Acceleration  forearm_link_a;
    Force         forearm_link_f;
    // Link 'wrist_1_link' :
    const InertiaMatrix& wrist_1_link_I;
    Velocity      wrist_1_link_v;
    Acceleration  wrist_1_link_a;
    Force         wrist_1_link_f;
    // Link 'wrist_2_link' :
    const InertiaMatrix& wrist_2_link_I;
    Velocity      wrist_2_link_v;
    Acceleration  wrist_2_link_a;
    Force         wrist_2_link_f;
    // Link 'wrist_3_link' :
    const InertiaMatrix& wrist_3_link_I;
    Velocity      wrist_3_link_v;
    Acceleration  wrist_3_link_a;
    Force         wrist_3_link_f;

    // The robot base
    const InertiaMatrix& base_link_ur5e_I;
    InertiaMatrix base_link_ur5e_Ic;
    Force         base_link_ur5e_f;
    // The composite inertia tensors
    InertiaMatrix shoulder_link_Ic;
    InertiaMatrix upper_arm_link_Ic;
    InertiaMatrix forearm_link_Ic;
    InertiaMatrix wrist_1_link_Ic;
    InertiaMatrix wrist_2_link_Ic;
    const InertiaMatrix& wrist_3_link_Ic;

private:
    static const ExtForces zeroExtForces;
};

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::setJointStatus(const JointState& q) const
{
    (xm->fr_shoulder_link_X_fr_base_link_ur5e)(q);
    (xm->fr_upper_arm_link_X_fr_shoulder_link)(q);
    (xm->fr_forearm_link_X_fr_upper_arm_link)(q);
    (xm->fr_wrist_1_link_X_fr_forearm_link)(q);
    (xm->fr_wrist_2_link_X_fr_wrist_1_link)(q);
    (xm->fr_wrist_3_link_X_fr_wrist_2_link)(q);
}

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::id(
    JointState& jForces, Acceleration& base_link_ur5e_a,
    const Acceleration& g, const Velocity& base_link_ur5e_v,
    const JointState& q, const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    setJointStatus(q);
    id(jForces, base_link_ur5e_a, g, base_link_ur5e_v,
       qd, qdd, fext);
}

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g, const JointState& q)
{
    setJointStatus(q);
    G_terms_fully_actuated(baseWrench, jForces, g);
}

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& base_link_ur5e_v, const JointState& q, const JointState& qd)
{
    setJointStatus(q);
    C_terms_fully_actuated(baseWrench, jForces, base_link_ur5e_v, qd);
}

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_link_ur5e_v, const Acceleration& baseAccel,
        const JointState& q, const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    setJointStatus(q);
    id_fully_actuated(baseWrench, jForces, g, base_link_ur5e_v,
        baseAccel, qd, qdd, fext);
}

}

typedef tpl::InverseDynamics<rbd::DoubleTrait> InverseDynamics;

}
}

}

#include "inverse_dynamics.impl.h"

#endif
