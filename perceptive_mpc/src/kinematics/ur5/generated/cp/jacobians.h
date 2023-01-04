#ifndef UR5E_JACOBIANS_H_
#define UR5E_JACOBIANS_H_

		#include <iit/rbd/rbd.h>
#include <iit/rbd/TransformsBase.h>
#include <iit/rbd/traits/DoubleTrait.h>
#include "declarations.h"
#include "kinematics_parameters.h"

namespace iit {
namespace ur5e {

template<typename SCALAR, int COLS, class M>
class JacobianT : public iit::rbd::JacobianBase<tpl::JointState<SCALAR>, COLS, M>
{};

namespace tpl {

/**
 *
 */
template <typename TRAIT>
class Jacobians {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef typename TRAIT::Scalar Scalar;
        typedef iit::rbd::Core<Scalar> CoreS;

        typedef JointState<Scalar> JState;

        class Type_fr_base_link_ur5e_J_fr_forearm_link : public JacobianT<Scalar, 3, Type_fr_base_link_ur5e_J_fr_forearm_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_link_ur5e_J_fr_forearm_link();
            const Type_fr_base_link_ur5e_J_fr_forearm_link& update(const JState&);
        protected:
        };
        
        class Type_fr_base_link_ur5e_J_fr_shoulder_link : public JacobianT<Scalar, 1, Type_fr_base_link_ur5e_J_fr_shoulder_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_link_ur5e_J_fr_shoulder_link();
            const Type_fr_base_link_ur5e_J_fr_shoulder_link& update(const JState&);
        protected:
        };
        
        class Type_fr_base_link_ur5e_J_fr_upper_arm_link : public JacobianT<Scalar, 2, Type_fr_base_link_ur5e_J_fr_upper_arm_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_link_ur5e_J_fr_upper_arm_link();
            const Type_fr_base_link_ur5e_J_fr_upper_arm_link& update(const JState&);
        protected:
        };
        
        class Type_fr_base_link_ur5e_J_fr_wrist_1_link : public JacobianT<Scalar, 4, Type_fr_base_link_ur5e_J_fr_wrist_1_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_link_ur5e_J_fr_wrist_1_link();
            const Type_fr_base_link_ur5e_J_fr_wrist_1_link& update(const JState&);
        protected:
        };
        
        class Type_fr_base_link_ur5e_J_fr_wrist_2_link : public JacobianT<Scalar, 5, Type_fr_base_link_ur5e_J_fr_wrist_2_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_link_ur5e_J_fr_wrist_2_link();
            const Type_fr_base_link_ur5e_J_fr_wrist_2_link& update(const JState&);
        protected:
        };
        
        class Type_fr_base_link_ur5e_J_fr_wrist_3_link : public JacobianT<Scalar, 6, Type_fr_base_link_ur5e_J_fr_wrist_3_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_link_ur5e_J_fr_wrist_3_link();
            const Type_fr_base_link_ur5e_J_fr_wrist_3_link& update(const JState&);
        protected:
        };
        
    public:
        Jacobians();
        void updateParameters();
    public:
        Type_fr_base_link_ur5e_J_fr_forearm_link fr_base_link_ur5e_J_fr_forearm_link;
        Type_fr_base_link_ur5e_J_fr_shoulder_link fr_base_link_ur5e_J_fr_shoulder_link;
        Type_fr_base_link_ur5e_J_fr_upper_arm_link fr_base_link_ur5e_J_fr_upper_arm_link;
        Type_fr_base_link_ur5e_J_fr_wrist_1_link fr_base_link_ur5e_J_fr_wrist_1_link;
        Type_fr_base_link_ur5e_J_fr_wrist_2_link fr_base_link_ur5e_J_fr_wrist_2_link;
        Type_fr_base_link_ur5e_J_fr_wrist_3_link fr_base_link_ur5e_J_fr_wrist_3_link;

    protected:

};

} //namespace tpl

using Jacobians = tpl::Jacobians<rbd::DoubleTrait>;

#include "jacobians.impl.h"


}
}

#endif
