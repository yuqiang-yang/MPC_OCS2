#ifndef UR5E_TRANSFORMS_H_
#define UR5E_TRANSFORMS_H_

#include <Eigen/Dense>
#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include <iit/rbd/traits/DoubleTrait.h>
#include "kinematics_parameters.h"

namespace iit {
namespace ur5e {

template<typename SCALAR, class M>
class TransformMotion : public iit::rbd::SpatialTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename SCALAR, class M>
class TransformForce : public iit::rbd::SpatialTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename SCALAR, class M>
class TransformHomogeneous : public iit::rbd::HomogeneousTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

namespace tpl {


/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
template <typename TRAIT>
class MotionTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformMotion<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_base_link_ur5e_X_fr_forearm_link : public TransformMotion<Scalar, Type_fr_base_link_ur5e_X_fr_forearm_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_forearm_link();
        const Type_fr_base_link_ur5e_X_fr_forearm_link& update(const JState&);
    protected:
    };
    
    class Type_fr_forearm_link_X_fr_base_link_ur5e : public TransformMotion<Scalar, Type_fr_forearm_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_forearm_link_X_fr_base_link_ur5e();
        const Type_fr_forearm_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_shoulder_link : public TransformMotion<Scalar, Type_fr_base_link_ur5e_X_fr_shoulder_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_shoulder_link();
        const Type_fr_base_link_ur5e_X_fr_shoulder_link& update(const JState&);
    protected:
    };
    
    class Type_fr_shoulder_link_X_fr_base_link_ur5e : public TransformMotion<Scalar, Type_fr_shoulder_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_shoulder_link_X_fr_base_link_ur5e();
        const Type_fr_shoulder_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_upper_arm_link : public TransformMotion<Scalar, Type_fr_base_link_ur5e_X_fr_upper_arm_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_upper_arm_link();
        const Type_fr_base_link_ur5e_X_fr_upper_arm_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_arm_link_X_fr_base_link_ur5e : public TransformMotion<Scalar, Type_fr_upper_arm_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_arm_link_X_fr_base_link_ur5e();
        const Type_fr_upper_arm_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_wrist_1_link : public TransformMotion<Scalar, Type_fr_base_link_ur5e_X_fr_wrist_1_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_wrist_1_link();
        const Type_fr_base_link_ur5e_X_fr_wrist_1_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_1_link_X_fr_base_link_ur5e : public TransformMotion<Scalar, Type_fr_wrist_1_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_1_link_X_fr_base_link_ur5e();
        const Type_fr_wrist_1_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_wrist_2_link : public TransformMotion<Scalar, Type_fr_base_link_ur5e_X_fr_wrist_2_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_wrist_2_link();
        const Type_fr_base_link_ur5e_X_fr_wrist_2_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_2_link_X_fr_base_link_ur5e : public TransformMotion<Scalar, Type_fr_wrist_2_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_2_link_X_fr_base_link_ur5e();
        const Type_fr_wrist_2_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_wrist_3_link : public TransformMotion<Scalar, Type_fr_base_link_ur5e_X_fr_wrist_3_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_wrist_3_link();
        const Type_fr_base_link_ur5e_X_fr_wrist_3_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_3_link_X_fr_base_link_ur5e : public TransformMotion<Scalar, Type_fr_wrist_3_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_3_link_X_fr_base_link_ur5e();
        const Type_fr_wrist_3_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_base_link_ur5e_COM : public TransformMotion<Scalar, Type_fr_base_link_ur5e_X_fr_base_link_ur5e_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_base_link_ur5e_COM();
        const Type_fr_base_link_ur5e_X_fr_base_link_ur5e_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_COM_X_fr_base_link_ur5e : public TransformMotion<Scalar, Type_fr_base_link_ur5e_COM_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_COM_X_fr_base_link_ur5e();
        const Type_fr_base_link_ur5e_COM_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_ee_link : public TransformMotion<Scalar, Type_fr_base_link_ur5e_X_fr_ee_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_ee_link();
        const Type_fr_base_link_ur5e_X_fr_ee_link& update(const JState&);
    protected:
    };
    
    class Type_fr_ee_link_X_fr_base_link_ur5e : public TransformMotion<Scalar, Type_fr_ee_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ee_link_X_fr_base_link_ur5e();
        const Type_fr_ee_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_shoulder_pan_joint : public TransformMotion<Scalar, Type_fr_base_link_ur5e_X_fr_shoulder_pan_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_shoulder_pan_joint();
        const Type_fr_base_link_ur5e_X_fr_shoulder_pan_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_shoulder_lift_joint : public TransformMotion<Scalar, Type_fr_base_link_ur5e_X_fr_shoulder_lift_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_shoulder_lift_joint();
        const Type_fr_base_link_ur5e_X_fr_shoulder_lift_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_elbow_joint : public TransformMotion<Scalar, Type_fr_base_link_ur5e_X_fr_elbow_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_elbow_joint();
        const Type_fr_base_link_ur5e_X_fr_elbow_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_wrist_1_joint : public TransformMotion<Scalar, Type_fr_base_link_ur5e_X_fr_wrist_1_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_wrist_1_joint();
        const Type_fr_base_link_ur5e_X_fr_wrist_1_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_wrist_2_joint : public TransformMotion<Scalar, Type_fr_base_link_ur5e_X_fr_wrist_2_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_wrist_2_joint();
        const Type_fr_base_link_ur5e_X_fr_wrist_2_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_wrist_3_joint : public TransformMotion<Scalar, Type_fr_base_link_ur5e_X_fr_wrist_3_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_wrist_3_joint();
        const Type_fr_base_link_ur5e_X_fr_wrist_3_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_arm_link_X_fr_shoulder_link : public TransformMotion<Scalar, Type_fr_upper_arm_link_X_fr_shoulder_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_arm_link_X_fr_shoulder_link();
        const Type_fr_upper_arm_link_X_fr_shoulder_link& update(const JState&);
    protected:
    };
    
    class Type_fr_shoulder_link_X_fr_upper_arm_link : public TransformMotion<Scalar, Type_fr_shoulder_link_X_fr_upper_arm_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_shoulder_link_X_fr_upper_arm_link();
        const Type_fr_shoulder_link_X_fr_upper_arm_link& update(const JState&);
    protected:
    };
    
    class Type_fr_forearm_link_X_fr_upper_arm_link : public TransformMotion<Scalar, Type_fr_forearm_link_X_fr_upper_arm_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_forearm_link_X_fr_upper_arm_link();
        const Type_fr_forearm_link_X_fr_upper_arm_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_arm_link_X_fr_forearm_link : public TransformMotion<Scalar, Type_fr_upper_arm_link_X_fr_forearm_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_arm_link_X_fr_forearm_link();
        const Type_fr_upper_arm_link_X_fr_forearm_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_1_link_X_fr_forearm_link : public TransformMotion<Scalar, Type_fr_wrist_1_link_X_fr_forearm_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_1_link_X_fr_forearm_link();
        const Type_fr_wrist_1_link_X_fr_forearm_link& update(const JState&);
    protected:
    };
    
    class Type_fr_forearm_link_X_fr_wrist_1_link : public TransformMotion<Scalar, Type_fr_forearm_link_X_fr_wrist_1_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_forearm_link_X_fr_wrist_1_link();
        const Type_fr_forearm_link_X_fr_wrist_1_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_2_link_X_fr_wrist_1_link : public TransformMotion<Scalar, Type_fr_wrist_2_link_X_fr_wrist_1_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_2_link_X_fr_wrist_1_link();
        const Type_fr_wrist_2_link_X_fr_wrist_1_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_1_link_X_fr_wrist_2_link : public TransformMotion<Scalar, Type_fr_wrist_1_link_X_fr_wrist_2_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_1_link_X_fr_wrist_2_link();
        const Type_fr_wrist_1_link_X_fr_wrist_2_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_3_link_X_fr_wrist_2_link : public TransformMotion<Scalar, Type_fr_wrist_3_link_X_fr_wrist_2_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_3_link_X_fr_wrist_2_link();
        const Type_fr_wrist_3_link_X_fr_wrist_2_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_2_link_X_fr_wrist_3_link : public TransformMotion<Scalar, Type_fr_wrist_2_link_X_fr_wrist_3_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_2_link_X_fr_wrist_3_link();
        const Type_fr_wrist_2_link_X_fr_wrist_3_link& update(const JState&);
    protected:
    };
    
public:
    MotionTransforms();
    void updateParameters();
    Type_fr_base_link_ur5e_X_fr_forearm_link fr_base_link_ur5e_X_fr_forearm_link;
    Type_fr_forearm_link_X_fr_base_link_ur5e fr_forearm_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_shoulder_link fr_base_link_ur5e_X_fr_shoulder_link;
    Type_fr_shoulder_link_X_fr_base_link_ur5e fr_shoulder_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_upper_arm_link fr_base_link_ur5e_X_fr_upper_arm_link;
    Type_fr_upper_arm_link_X_fr_base_link_ur5e fr_upper_arm_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_wrist_1_link fr_base_link_ur5e_X_fr_wrist_1_link;
    Type_fr_wrist_1_link_X_fr_base_link_ur5e fr_wrist_1_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_wrist_2_link fr_base_link_ur5e_X_fr_wrist_2_link;
    Type_fr_wrist_2_link_X_fr_base_link_ur5e fr_wrist_2_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_wrist_3_link fr_base_link_ur5e_X_fr_wrist_3_link;
    Type_fr_wrist_3_link_X_fr_base_link_ur5e fr_wrist_3_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_base_link_ur5e_COM fr_base_link_ur5e_X_fr_base_link_ur5e_COM;
    Type_fr_base_link_ur5e_COM_X_fr_base_link_ur5e fr_base_link_ur5e_COM_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_ee_link fr_base_link_ur5e_X_fr_ee_link;
    Type_fr_ee_link_X_fr_base_link_ur5e fr_ee_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_shoulder_pan_joint fr_base_link_ur5e_X_fr_shoulder_pan_joint;
    Type_fr_base_link_ur5e_X_fr_shoulder_lift_joint fr_base_link_ur5e_X_fr_shoulder_lift_joint;
    Type_fr_base_link_ur5e_X_fr_elbow_joint fr_base_link_ur5e_X_fr_elbow_joint;
    Type_fr_base_link_ur5e_X_fr_wrist_1_joint fr_base_link_ur5e_X_fr_wrist_1_joint;
    Type_fr_base_link_ur5e_X_fr_wrist_2_joint fr_base_link_ur5e_X_fr_wrist_2_joint;
    Type_fr_base_link_ur5e_X_fr_wrist_3_joint fr_base_link_ur5e_X_fr_wrist_3_joint;
    Type_fr_upper_arm_link_X_fr_shoulder_link fr_upper_arm_link_X_fr_shoulder_link;
    Type_fr_shoulder_link_X_fr_upper_arm_link fr_shoulder_link_X_fr_upper_arm_link;
    Type_fr_forearm_link_X_fr_upper_arm_link fr_forearm_link_X_fr_upper_arm_link;
    Type_fr_upper_arm_link_X_fr_forearm_link fr_upper_arm_link_X_fr_forearm_link;
    Type_fr_wrist_1_link_X_fr_forearm_link fr_wrist_1_link_X_fr_forearm_link;
    Type_fr_forearm_link_X_fr_wrist_1_link fr_forearm_link_X_fr_wrist_1_link;
    Type_fr_wrist_2_link_X_fr_wrist_1_link fr_wrist_2_link_X_fr_wrist_1_link;
    Type_fr_wrist_1_link_X_fr_wrist_2_link fr_wrist_1_link_X_fr_wrist_2_link;
    Type_fr_wrist_3_link_X_fr_wrist_2_link fr_wrist_3_link_X_fr_wrist_2_link;
    Type_fr_wrist_2_link_X_fr_wrist_3_link fr_wrist_2_link_X_fr_wrist_3_link;

protected:

}; //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
template <typename TRAIT>
class ForceTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformForce<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_base_link_ur5e_X_fr_forearm_link : public TransformForce<Scalar, Type_fr_base_link_ur5e_X_fr_forearm_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_forearm_link();
        const Type_fr_base_link_ur5e_X_fr_forearm_link& update(const JState&);
    protected:
    };
    
    class Type_fr_forearm_link_X_fr_base_link_ur5e : public TransformForce<Scalar, Type_fr_forearm_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_forearm_link_X_fr_base_link_ur5e();
        const Type_fr_forearm_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_shoulder_link : public TransformForce<Scalar, Type_fr_base_link_ur5e_X_fr_shoulder_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_shoulder_link();
        const Type_fr_base_link_ur5e_X_fr_shoulder_link& update(const JState&);
    protected:
    };
    
    class Type_fr_shoulder_link_X_fr_base_link_ur5e : public TransformForce<Scalar, Type_fr_shoulder_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_shoulder_link_X_fr_base_link_ur5e();
        const Type_fr_shoulder_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_upper_arm_link : public TransformForce<Scalar, Type_fr_base_link_ur5e_X_fr_upper_arm_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_upper_arm_link();
        const Type_fr_base_link_ur5e_X_fr_upper_arm_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_arm_link_X_fr_base_link_ur5e : public TransformForce<Scalar, Type_fr_upper_arm_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_arm_link_X_fr_base_link_ur5e();
        const Type_fr_upper_arm_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_wrist_1_link : public TransformForce<Scalar, Type_fr_base_link_ur5e_X_fr_wrist_1_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_wrist_1_link();
        const Type_fr_base_link_ur5e_X_fr_wrist_1_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_1_link_X_fr_base_link_ur5e : public TransformForce<Scalar, Type_fr_wrist_1_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_1_link_X_fr_base_link_ur5e();
        const Type_fr_wrist_1_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_wrist_2_link : public TransformForce<Scalar, Type_fr_base_link_ur5e_X_fr_wrist_2_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_wrist_2_link();
        const Type_fr_base_link_ur5e_X_fr_wrist_2_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_2_link_X_fr_base_link_ur5e : public TransformForce<Scalar, Type_fr_wrist_2_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_2_link_X_fr_base_link_ur5e();
        const Type_fr_wrist_2_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_wrist_3_link : public TransformForce<Scalar, Type_fr_base_link_ur5e_X_fr_wrist_3_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_wrist_3_link();
        const Type_fr_base_link_ur5e_X_fr_wrist_3_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_3_link_X_fr_base_link_ur5e : public TransformForce<Scalar, Type_fr_wrist_3_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_3_link_X_fr_base_link_ur5e();
        const Type_fr_wrist_3_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_base_link_ur5e_COM : public TransformForce<Scalar, Type_fr_base_link_ur5e_X_fr_base_link_ur5e_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_base_link_ur5e_COM();
        const Type_fr_base_link_ur5e_X_fr_base_link_ur5e_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_COM_X_fr_base_link_ur5e : public TransformForce<Scalar, Type_fr_base_link_ur5e_COM_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_COM_X_fr_base_link_ur5e();
        const Type_fr_base_link_ur5e_COM_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_ee_link : public TransformForce<Scalar, Type_fr_base_link_ur5e_X_fr_ee_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_ee_link();
        const Type_fr_base_link_ur5e_X_fr_ee_link& update(const JState&);
    protected:
    };
    
    class Type_fr_ee_link_X_fr_base_link_ur5e : public TransformForce<Scalar, Type_fr_ee_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ee_link_X_fr_base_link_ur5e();
        const Type_fr_ee_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_shoulder_pan_joint : public TransformForce<Scalar, Type_fr_base_link_ur5e_X_fr_shoulder_pan_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_shoulder_pan_joint();
        const Type_fr_base_link_ur5e_X_fr_shoulder_pan_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_shoulder_lift_joint : public TransformForce<Scalar, Type_fr_base_link_ur5e_X_fr_shoulder_lift_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_shoulder_lift_joint();
        const Type_fr_base_link_ur5e_X_fr_shoulder_lift_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_elbow_joint : public TransformForce<Scalar, Type_fr_base_link_ur5e_X_fr_elbow_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_elbow_joint();
        const Type_fr_base_link_ur5e_X_fr_elbow_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_wrist_1_joint : public TransformForce<Scalar, Type_fr_base_link_ur5e_X_fr_wrist_1_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_wrist_1_joint();
        const Type_fr_base_link_ur5e_X_fr_wrist_1_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_wrist_2_joint : public TransformForce<Scalar, Type_fr_base_link_ur5e_X_fr_wrist_2_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_wrist_2_joint();
        const Type_fr_base_link_ur5e_X_fr_wrist_2_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_wrist_3_joint : public TransformForce<Scalar, Type_fr_base_link_ur5e_X_fr_wrist_3_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_wrist_3_joint();
        const Type_fr_base_link_ur5e_X_fr_wrist_3_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_arm_link_X_fr_shoulder_link : public TransformForce<Scalar, Type_fr_upper_arm_link_X_fr_shoulder_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_arm_link_X_fr_shoulder_link();
        const Type_fr_upper_arm_link_X_fr_shoulder_link& update(const JState&);
    protected:
    };
    
    class Type_fr_shoulder_link_X_fr_upper_arm_link : public TransformForce<Scalar, Type_fr_shoulder_link_X_fr_upper_arm_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_shoulder_link_X_fr_upper_arm_link();
        const Type_fr_shoulder_link_X_fr_upper_arm_link& update(const JState&);
    protected:
    };
    
    class Type_fr_forearm_link_X_fr_upper_arm_link : public TransformForce<Scalar, Type_fr_forearm_link_X_fr_upper_arm_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_forearm_link_X_fr_upper_arm_link();
        const Type_fr_forearm_link_X_fr_upper_arm_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_arm_link_X_fr_forearm_link : public TransformForce<Scalar, Type_fr_upper_arm_link_X_fr_forearm_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_arm_link_X_fr_forearm_link();
        const Type_fr_upper_arm_link_X_fr_forearm_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_1_link_X_fr_forearm_link : public TransformForce<Scalar, Type_fr_wrist_1_link_X_fr_forearm_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_1_link_X_fr_forearm_link();
        const Type_fr_wrist_1_link_X_fr_forearm_link& update(const JState&);
    protected:
    };
    
    class Type_fr_forearm_link_X_fr_wrist_1_link : public TransformForce<Scalar, Type_fr_forearm_link_X_fr_wrist_1_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_forearm_link_X_fr_wrist_1_link();
        const Type_fr_forearm_link_X_fr_wrist_1_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_2_link_X_fr_wrist_1_link : public TransformForce<Scalar, Type_fr_wrist_2_link_X_fr_wrist_1_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_2_link_X_fr_wrist_1_link();
        const Type_fr_wrist_2_link_X_fr_wrist_1_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_1_link_X_fr_wrist_2_link : public TransformForce<Scalar, Type_fr_wrist_1_link_X_fr_wrist_2_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_1_link_X_fr_wrist_2_link();
        const Type_fr_wrist_1_link_X_fr_wrist_2_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_3_link_X_fr_wrist_2_link : public TransformForce<Scalar, Type_fr_wrist_3_link_X_fr_wrist_2_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_3_link_X_fr_wrist_2_link();
        const Type_fr_wrist_3_link_X_fr_wrist_2_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_2_link_X_fr_wrist_3_link : public TransformForce<Scalar, Type_fr_wrist_2_link_X_fr_wrist_3_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_2_link_X_fr_wrist_3_link();
        const Type_fr_wrist_2_link_X_fr_wrist_3_link& update(const JState&);
    protected:
    };
    
public:
    ForceTransforms();
    void updateParameters();
    Type_fr_base_link_ur5e_X_fr_forearm_link fr_base_link_ur5e_X_fr_forearm_link;
    Type_fr_forearm_link_X_fr_base_link_ur5e fr_forearm_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_shoulder_link fr_base_link_ur5e_X_fr_shoulder_link;
    Type_fr_shoulder_link_X_fr_base_link_ur5e fr_shoulder_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_upper_arm_link fr_base_link_ur5e_X_fr_upper_arm_link;
    Type_fr_upper_arm_link_X_fr_base_link_ur5e fr_upper_arm_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_wrist_1_link fr_base_link_ur5e_X_fr_wrist_1_link;
    Type_fr_wrist_1_link_X_fr_base_link_ur5e fr_wrist_1_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_wrist_2_link fr_base_link_ur5e_X_fr_wrist_2_link;
    Type_fr_wrist_2_link_X_fr_base_link_ur5e fr_wrist_2_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_wrist_3_link fr_base_link_ur5e_X_fr_wrist_3_link;
    Type_fr_wrist_3_link_X_fr_base_link_ur5e fr_wrist_3_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_base_link_ur5e_COM fr_base_link_ur5e_X_fr_base_link_ur5e_COM;
    Type_fr_base_link_ur5e_COM_X_fr_base_link_ur5e fr_base_link_ur5e_COM_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_ee_link fr_base_link_ur5e_X_fr_ee_link;
    Type_fr_ee_link_X_fr_base_link_ur5e fr_ee_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_shoulder_pan_joint fr_base_link_ur5e_X_fr_shoulder_pan_joint;
    Type_fr_base_link_ur5e_X_fr_shoulder_lift_joint fr_base_link_ur5e_X_fr_shoulder_lift_joint;
    Type_fr_base_link_ur5e_X_fr_elbow_joint fr_base_link_ur5e_X_fr_elbow_joint;
    Type_fr_base_link_ur5e_X_fr_wrist_1_joint fr_base_link_ur5e_X_fr_wrist_1_joint;
    Type_fr_base_link_ur5e_X_fr_wrist_2_joint fr_base_link_ur5e_X_fr_wrist_2_joint;
    Type_fr_base_link_ur5e_X_fr_wrist_3_joint fr_base_link_ur5e_X_fr_wrist_3_joint;
    Type_fr_upper_arm_link_X_fr_shoulder_link fr_upper_arm_link_X_fr_shoulder_link;
    Type_fr_shoulder_link_X_fr_upper_arm_link fr_shoulder_link_X_fr_upper_arm_link;
    Type_fr_forearm_link_X_fr_upper_arm_link fr_forearm_link_X_fr_upper_arm_link;
    Type_fr_upper_arm_link_X_fr_forearm_link fr_upper_arm_link_X_fr_forearm_link;
    Type_fr_wrist_1_link_X_fr_forearm_link fr_wrist_1_link_X_fr_forearm_link;
    Type_fr_forearm_link_X_fr_wrist_1_link fr_forearm_link_X_fr_wrist_1_link;
    Type_fr_wrist_2_link_X_fr_wrist_1_link fr_wrist_2_link_X_fr_wrist_1_link;
    Type_fr_wrist_1_link_X_fr_wrist_2_link fr_wrist_1_link_X_fr_wrist_2_link;
    Type_fr_wrist_3_link_X_fr_wrist_2_link fr_wrist_3_link_X_fr_wrist_2_link;
    Type_fr_wrist_2_link_X_fr_wrist_3_link fr_wrist_2_link_X_fr_wrist_3_link;

protected:

}; //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
template <typename TRAIT>
class HomogeneousTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformHomogeneous<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_base_link_ur5e_X_fr_forearm_link : public TransformHomogeneous<Scalar, Type_fr_base_link_ur5e_X_fr_forearm_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_forearm_link();
        const Type_fr_base_link_ur5e_X_fr_forearm_link& update(const JState&);
    protected:
    };
    
    class Type_fr_forearm_link_X_fr_base_link_ur5e : public TransformHomogeneous<Scalar, Type_fr_forearm_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_forearm_link_X_fr_base_link_ur5e();
        const Type_fr_forearm_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_shoulder_link : public TransformHomogeneous<Scalar, Type_fr_base_link_ur5e_X_fr_shoulder_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_shoulder_link();
        const Type_fr_base_link_ur5e_X_fr_shoulder_link& update(const JState&);
    protected:
    };
    
    class Type_fr_shoulder_link_X_fr_base_link_ur5e : public TransformHomogeneous<Scalar, Type_fr_shoulder_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_shoulder_link_X_fr_base_link_ur5e();
        const Type_fr_shoulder_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_upper_arm_link : public TransformHomogeneous<Scalar, Type_fr_base_link_ur5e_X_fr_upper_arm_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_upper_arm_link();
        const Type_fr_base_link_ur5e_X_fr_upper_arm_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_arm_link_X_fr_base_link_ur5e : public TransformHomogeneous<Scalar, Type_fr_upper_arm_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_arm_link_X_fr_base_link_ur5e();
        const Type_fr_upper_arm_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_wrist_1_link : public TransformHomogeneous<Scalar, Type_fr_base_link_ur5e_X_fr_wrist_1_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_wrist_1_link();
        const Type_fr_base_link_ur5e_X_fr_wrist_1_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_1_link_X_fr_base_link_ur5e : public TransformHomogeneous<Scalar, Type_fr_wrist_1_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_1_link_X_fr_base_link_ur5e();
        const Type_fr_wrist_1_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_wrist_2_link : public TransformHomogeneous<Scalar, Type_fr_base_link_ur5e_X_fr_wrist_2_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_wrist_2_link();
        const Type_fr_base_link_ur5e_X_fr_wrist_2_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_2_link_X_fr_base_link_ur5e : public TransformHomogeneous<Scalar, Type_fr_wrist_2_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_2_link_X_fr_base_link_ur5e();
        const Type_fr_wrist_2_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_wrist_3_link : public TransformHomogeneous<Scalar, Type_fr_base_link_ur5e_X_fr_wrist_3_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_wrist_3_link();
        const Type_fr_base_link_ur5e_X_fr_wrist_3_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_3_link_X_fr_base_link_ur5e : public TransformHomogeneous<Scalar, Type_fr_wrist_3_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_3_link_X_fr_base_link_ur5e();
        const Type_fr_wrist_3_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_base_link_ur5e_COM : public TransformHomogeneous<Scalar, Type_fr_base_link_ur5e_X_fr_base_link_ur5e_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_base_link_ur5e_COM();
        const Type_fr_base_link_ur5e_X_fr_base_link_ur5e_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_COM_X_fr_base_link_ur5e : public TransformHomogeneous<Scalar, Type_fr_base_link_ur5e_COM_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_COM_X_fr_base_link_ur5e();
        const Type_fr_base_link_ur5e_COM_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_ee_link : public TransformHomogeneous<Scalar, Type_fr_base_link_ur5e_X_fr_ee_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_ee_link();
        const Type_fr_base_link_ur5e_X_fr_ee_link& update(const JState&);
    protected:
    };
    
    class Type_fr_ee_link_X_fr_base_link_ur5e : public TransformHomogeneous<Scalar, Type_fr_ee_link_X_fr_base_link_ur5e>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ee_link_X_fr_base_link_ur5e();
        const Type_fr_ee_link_X_fr_base_link_ur5e& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_shoulder_pan_joint : public TransformHomogeneous<Scalar, Type_fr_base_link_ur5e_X_fr_shoulder_pan_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_shoulder_pan_joint();
        const Type_fr_base_link_ur5e_X_fr_shoulder_pan_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_shoulder_lift_joint : public TransformHomogeneous<Scalar, Type_fr_base_link_ur5e_X_fr_shoulder_lift_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_shoulder_lift_joint();
        const Type_fr_base_link_ur5e_X_fr_shoulder_lift_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_elbow_joint : public TransformHomogeneous<Scalar, Type_fr_base_link_ur5e_X_fr_elbow_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_elbow_joint();
        const Type_fr_base_link_ur5e_X_fr_elbow_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_wrist_1_joint : public TransformHomogeneous<Scalar, Type_fr_base_link_ur5e_X_fr_wrist_1_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_wrist_1_joint();
        const Type_fr_base_link_ur5e_X_fr_wrist_1_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_wrist_2_joint : public TransformHomogeneous<Scalar, Type_fr_base_link_ur5e_X_fr_wrist_2_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_wrist_2_joint();
        const Type_fr_base_link_ur5e_X_fr_wrist_2_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_ur5e_X_fr_wrist_3_joint : public TransformHomogeneous<Scalar, Type_fr_base_link_ur5e_X_fr_wrist_3_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_ur5e_X_fr_wrist_3_joint();
        const Type_fr_base_link_ur5e_X_fr_wrist_3_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_arm_link_X_fr_shoulder_link : public TransformHomogeneous<Scalar, Type_fr_upper_arm_link_X_fr_shoulder_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_arm_link_X_fr_shoulder_link();
        const Type_fr_upper_arm_link_X_fr_shoulder_link& update(const JState&);
    protected:
    };
    
    class Type_fr_shoulder_link_X_fr_upper_arm_link : public TransformHomogeneous<Scalar, Type_fr_shoulder_link_X_fr_upper_arm_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_shoulder_link_X_fr_upper_arm_link();
        const Type_fr_shoulder_link_X_fr_upper_arm_link& update(const JState&);
    protected:
    };
    
    class Type_fr_forearm_link_X_fr_upper_arm_link : public TransformHomogeneous<Scalar, Type_fr_forearm_link_X_fr_upper_arm_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_forearm_link_X_fr_upper_arm_link();
        const Type_fr_forearm_link_X_fr_upper_arm_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_arm_link_X_fr_forearm_link : public TransformHomogeneous<Scalar, Type_fr_upper_arm_link_X_fr_forearm_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_arm_link_X_fr_forearm_link();
        const Type_fr_upper_arm_link_X_fr_forearm_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_1_link_X_fr_forearm_link : public TransformHomogeneous<Scalar, Type_fr_wrist_1_link_X_fr_forearm_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_1_link_X_fr_forearm_link();
        const Type_fr_wrist_1_link_X_fr_forearm_link& update(const JState&);
    protected:
    };
    
    class Type_fr_forearm_link_X_fr_wrist_1_link : public TransformHomogeneous<Scalar, Type_fr_forearm_link_X_fr_wrist_1_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_forearm_link_X_fr_wrist_1_link();
        const Type_fr_forearm_link_X_fr_wrist_1_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_2_link_X_fr_wrist_1_link : public TransformHomogeneous<Scalar, Type_fr_wrist_2_link_X_fr_wrist_1_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_2_link_X_fr_wrist_1_link();
        const Type_fr_wrist_2_link_X_fr_wrist_1_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_1_link_X_fr_wrist_2_link : public TransformHomogeneous<Scalar, Type_fr_wrist_1_link_X_fr_wrist_2_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_1_link_X_fr_wrist_2_link();
        const Type_fr_wrist_1_link_X_fr_wrist_2_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_3_link_X_fr_wrist_2_link : public TransformHomogeneous<Scalar, Type_fr_wrist_3_link_X_fr_wrist_2_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_3_link_X_fr_wrist_2_link();
        const Type_fr_wrist_3_link_X_fr_wrist_2_link& update(const JState&);
    protected:
    };
    
    class Type_fr_wrist_2_link_X_fr_wrist_3_link : public TransformHomogeneous<Scalar, Type_fr_wrist_2_link_X_fr_wrist_3_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_wrist_2_link_X_fr_wrist_3_link();
        const Type_fr_wrist_2_link_X_fr_wrist_3_link& update(const JState&);
    protected:
    };
    
public:
    HomogeneousTransforms();
    void updateParameters();
    Type_fr_base_link_ur5e_X_fr_forearm_link fr_base_link_ur5e_X_fr_forearm_link;
    Type_fr_forearm_link_X_fr_base_link_ur5e fr_forearm_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_shoulder_link fr_base_link_ur5e_X_fr_shoulder_link;
    Type_fr_shoulder_link_X_fr_base_link_ur5e fr_shoulder_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_upper_arm_link fr_base_link_ur5e_X_fr_upper_arm_link;
    Type_fr_upper_arm_link_X_fr_base_link_ur5e fr_upper_arm_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_wrist_1_link fr_base_link_ur5e_X_fr_wrist_1_link;
    Type_fr_wrist_1_link_X_fr_base_link_ur5e fr_wrist_1_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_wrist_2_link fr_base_link_ur5e_X_fr_wrist_2_link;
    Type_fr_wrist_2_link_X_fr_base_link_ur5e fr_wrist_2_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_wrist_3_link fr_base_link_ur5e_X_fr_wrist_3_link;
    Type_fr_wrist_3_link_X_fr_base_link_ur5e fr_wrist_3_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_base_link_ur5e_COM fr_base_link_ur5e_X_fr_base_link_ur5e_COM;
    Type_fr_base_link_ur5e_COM_X_fr_base_link_ur5e fr_base_link_ur5e_COM_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_ee_link fr_base_link_ur5e_X_fr_ee_link;
    Type_fr_ee_link_X_fr_base_link_ur5e fr_ee_link_X_fr_base_link_ur5e;
    Type_fr_base_link_ur5e_X_fr_shoulder_pan_joint fr_base_link_ur5e_X_fr_shoulder_pan_joint;
    Type_fr_base_link_ur5e_X_fr_shoulder_lift_joint fr_base_link_ur5e_X_fr_shoulder_lift_joint;
    Type_fr_base_link_ur5e_X_fr_elbow_joint fr_base_link_ur5e_X_fr_elbow_joint;
    Type_fr_base_link_ur5e_X_fr_wrist_1_joint fr_base_link_ur5e_X_fr_wrist_1_joint;
    Type_fr_base_link_ur5e_X_fr_wrist_2_joint fr_base_link_ur5e_X_fr_wrist_2_joint;
    Type_fr_base_link_ur5e_X_fr_wrist_3_joint fr_base_link_ur5e_X_fr_wrist_3_joint;
    Type_fr_upper_arm_link_X_fr_shoulder_link fr_upper_arm_link_X_fr_shoulder_link;
    Type_fr_shoulder_link_X_fr_upper_arm_link fr_shoulder_link_X_fr_upper_arm_link;
    Type_fr_forearm_link_X_fr_upper_arm_link fr_forearm_link_X_fr_upper_arm_link;
    Type_fr_upper_arm_link_X_fr_forearm_link fr_upper_arm_link_X_fr_forearm_link;
    Type_fr_wrist_1_link_X_fr_forearm_link fr_wrist_1_link_X_fr_forearm_link;
    Type_fr_forearm_link_X_fr_wrist_1_link fr_forearm_link_X_fr_wrist_1_link;
    Type_fr_wrist_2_link_X_fr_wrist_1_link fr_wrist_2_link_X_fr_wrist_1_link;
    Type_fr_wrist_1_link_X_fr_wrist_2_link fr_wrist_1_link_X_fr_wrist_2_link;
    Type_fr_wrist_3_link_X_fr_wrist_2_link fr_wrist_3_link_X_fr_wrist_2_link;
    Type_fr_wrist_2_link_X_fr_wrist_3_link fr_wrist_2_link_X_fr_wrist_3_link;

protected:

}; //class 'HomogeneousTransforms'

}

using MotionTransforms = tpl::MotionTransforms<rbd::DoubleTrait>;
using ForceTransforms = tpl::ForceTransforms<rbd::DoubleTrait>;
using HomogeneousTransforms = tpl::HomogeneousTransforms<rbd::DoubleTrait>;

}
}

#include "transforms.impl.h"

#endif
