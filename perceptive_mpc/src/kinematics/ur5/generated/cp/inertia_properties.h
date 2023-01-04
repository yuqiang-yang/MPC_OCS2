#ifndef IIT_ROBOT_UR5E_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_UR5E_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/traits/DoubleTrait.h>

#include "declarations.h"

namespace iit {
namespace ur5e {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot ur5e.
 */
namespace dyn {

using InertiaMatrix = iit::rbd::InertiaMatrixDense;

namespace tpl {

template <typename TRAIT>
class InertiaProperties {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef typename TRAIT::Scalar Scalar;
        typedef iit::rbd::Core<Scalar> CoreS;
        typedef iit::rbd::tpl::InertiaMatrixDense<Scalar> IMatrix;
        typedef typename CoreS::Vector3 Vec3d;

        InertiaProperties();
        ~InertiaProperties();
        const IMatrix& getTensor_base_link_ur5e() const;
        const IMatrix& getTensor_shoulder_link() const;
        const IMatrix& getTensor_upper_arm_link() const;
        const IMatrix& getTensor_forearm_link() const;
        const IMatrix& getTensor_wrist_1_link() const;
        const IMatrix& getTensor_wrist_2_link() const;
        const IMatrix& getTensor_wrist_3_link() const;
        Scalar getMass_base_link_ur5e() const;
        Scalar getMass_shoulder_link() const;
        Scalar getMass_upper_arm_link() const;
        Scalar getMass_forearm_link() const;
        Scalar getMass_wrist_1_link() const;
        Scalar getMass_wrist_2_link() const;
        Scalar getMass_wrist_3_link() const;
        const Vec3d& getCOM_base_link_ur5e() const;
        const Vec3d& getCOM_shoulder_link() const;
        const Vec3d& getCOM_upper_arm_link() const;
        const Vec3d& getCOM_forearm_link() const;
        const Vec3d& getCOM_wrist_1_link() const;
        const Vec3d& getCOM_wrist_2_link() const;
        const Vec3d& getCOM_wrist_3_link() const;
        Scalar getTotalMass() const;

    private:

        IMatrix tensor_base_link_ur5e;
        IMatrix tensor_shoulder_link;
        IMatrix tensor_upper_arm_link;
        IMatrix tensor_forearm_link;
        IMatrix tensor_wrist_1_link;
        IMatrix tensor_wrist_2_link;
        IMatrix tensor_wrist_3_link;
        Vec3d com_base_link_ur5e;
        Vec3d com_shoulder_link;
        Vec3d com_upper_arm_link;
        Vec3d com_forearm_link;
        Vec3d com_wrist_1_link;
        Vec3d com_wrist_2_link;
        Vec3d com_wrist_3_link;
};

template <typename TRAIT>
inline InertiaProperties<TRAIT>::~InertiaProperties() {}

template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_base_link_ur5e() const {
    return this->tensor_base_link_ur5e;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_shoulder_link() const {
    return this->tensor_shoulder_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_upper_arm_link() const {
    return this->tensor_upper_arm_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_forearm_link() const {
    return this->tensor_forearm_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_wrist_1_link() const {
    return this->tensor_wrist_1_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_wrist_2_link() const {
    return this->tensor_wrist_2_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_wrist_3_link() const {
    return this->tensor_wrist_3_link;
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_base_link_ur5e() const {
    return this->tensor_base_link_ur5e.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_shoulder_link() const {
    return this->tensor_shoulder_link.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_upper_arm_link() const {
    return this->tensor_upper_arm_link.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_forearm_link() const {
    return this->tensor_forearm_link.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_wrist_1_link() const {
    return this->tensor_wrist_1_link.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_wrist_2_link() const {
    return this->tensor_wrist_2_link.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_wrist_3_link() const {
    return this->tensor_wrist_3_link.getMass();
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_base_link_ur5e() const {
    return this->com_base_link_ur5e;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_shoulder_link() const {
    return this->com_shoulder_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_upper_arm_link() const {
    return this->com_upper_arm_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_forearm_link() const {
    return this->com_forearm_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_wrist_1_link() const {
    return this->com_wrist_1_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_wrist_2_link() const {
    return this->com_wrist_2_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_wrist_3_link() const {
    return this->com_wrist_3_link;
}

template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getTotalMass() const {
    return 4.0 + 3.7 + 8.393 + 2.275 + 1.219 + 1.219 + 0.1879;
}

}

using InertiaProperties = tpl::InertiaProperties<rbd::DoubleTrait>;

}
}
}

#include "inertia_properties.impl.h"

#endif
