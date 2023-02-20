#ifndef IIT_ROBOGEN__UR5E_TRAITS_H_
#define IIT_ROBOGEN__UR5E_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace ur5e {

namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename ur5e::JointIdentifiers JointID;
    typedef typename ur5e::LinkIdentifiers  LinkID;
    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename ur5e::tpl::JointState<SCALAR> JointState;



    typedef typename ur5e::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename ur5e::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename ur5e::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename ur5e::tpl::Jacobians<Trait> Jacobians;

    typedef typename iit::ur5e::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename iit::ur5e::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename iit::ur5e::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename iit::ur5e::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = ur5e::jointsCount;
    static const int links_count  = ur5e::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return ur5e::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return ur5e::orderedLinkIDs;
}

}

typedef tpl::Traits<double> Traits; // default instantiation - backward compatibility...

}
}

#endif
