template <typename TRAIT>
iit::ur5e::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_base_link_ur5e = iit::rbd::Vector3d(0.0,0.0,0.0).cast<Scalar>();
    tensor_base_link_ur5e.fill(
        Scalar(4.0),
        com_base_link_ur5e,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0044333315),
                Scalar(0.0044333315),
                Scalar(0.0072),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

    com_shoulder_link = iit::rbd::Vector3d(0.0,0.0,0.0).cast<Scalar>();
    tensor_shoulder_link.fill(
        Scalar(3.7),
        com_shoulder_link,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.010267496),
                Scalar(0.010267496),
                Scalar(0.00666),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

    com_upper_arm_link = iit::rbd::Vector3d(0.2125,-1.0409451E-12,5.293956E-23).cast<Scalar>();
    tensor_upper_arm_link.fill(
        Scalar(8.393),
        com_upper_arm_link,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0151074),
                Scalar(0.5128822),
                Scalar(0.5128822),
                Scalar(5.1901115E-9),
                Scalar(2.2694818E-16),
                Scalar(-6.457544E-23)) );

    com_forearm_link = iit::rbd::Vector3d(0.196,-9.601209E-13,2.646978E-23).cast<Scalar>();
    tensor_forearm_link.fill(
        Scalar(2.275),
        com_forearm_link,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.004095),
                Scalar(0.11857602),
                Scalar(0.11857602),
                Scalar(1.1834782E-9),
                Scalar(5.1750203E-17),
                Scalar(-1.4942117E-23)) );

    com_wrist_1_link = iit::rbd::Vector3d(0.0,0.0,0.1263).cast<Scalar>();
    tensor_wrist_1_link.fill(
        Scalar(1.219),
        com_wrist_1_link,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.02200501),
                Scalar(0.021639312),
                Scalar(0.002559899),
                Scalar(0.0),
                Scalar(1.9721523E-31),
                Scalar(-1.5985213E-11)) );

    com_wrist_2_link = iit::rbd::Vector3d(-9.76E-13,0.0,0.0997).cast<Scalar>();
    tensor_wrist_2_link.fill(
        Scalar(1.219),
        com_wrist_2_link,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.014676869),
                Scalar(0.014676869),
                Scalar(0.0021942),
                Scalar(0.0),
                Scalar(-1.1861746E-13),
                Scalar(0.0)) );

    com_wrist_3_link = iit::rbd::Vector3d(-5.293956E-23,0.0,0.0767).cast<Scalar>();
    tensor_wrist_3_link.fill(
        Scalar(0.1879),
        com_wrist_3_link,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0012042993),
                Scalar(0.0012042993),
                Scalar(1.3211719E-4),
                Scalar(7.3856944E-25),
                Scalar(0.0),
                Scalar(-2.9035802E-12)) );

}

