#ifndef RCG_UR5E_MODEL_CONSTANTS_H_
#define RCG_UR5E_MODEL_CONSTANTS_H_

#include "rbd_types.h"

/**
 * \file
 * This file contains the definitions of all the non-zero numerical
 * constants of the robot model (i.e. the numbers appearing in the
 * .kindsl file).
 *
 * Varying these values (and recompiling) is a quick & dirty
 * way to vary the kinematics/dynamics model. For a much more
 * flexible way of exploring variations of the model, consider
 * using the parametrization feature of RobCoGen (see the wiki).
 *
 * Beware of inconsistencies when changing any of the inertia
 * properties.
 */

namespace ur5e {
namespace rcg {

// Do not use 'constexpr' to allow for non-literal scalar types

const Scalar tz_shoulder_pan_joint = 0.16249999403953552;
const Scalar ty_shoulder_lift_joint = 0.1379999965429306;
const Scalar tx_elbow_joint = 0.42500001192092896;
const Scalar ty_elbow_joint = -2.081000085793483E-12;
const Scalar tz_elbow_joint = -0.13099999725818634;
const Scalar tx_wrist_1_joint = 0.3919999897480011;
const Scalar ty_wrist_1_joint = -1.918999898223306E-12;
const Scalar tz_wrist_2_joint = 0.12630000710487366;
const Scalar tx_wrist_3_joint = -9.759999882938408E-13;
const Scalar tz_wrist_3_joint = 0.09969999641180038;
const Scalar tx_fr_upper_arm_link_COM = 0.21250000596046448;
const Scalar ty_fr_upper_arm_link_COM = -1.0409999685184745E-12;
const Scalar tx_fr_forearm_link_COM = 0.19599999487400055;
const Scalar ty_fr_forearm_link_COM = -9.599999831536032E-13;
const Scalar tz_fr_wrist_1_link_COM = 0.12630000710487366;
const Scalar tx_fr_wrist_2_link_COM = -9.759999882938408E-13;
const Scalar tz_fr_wrist_2_link_COM = 0.09969999641180038;
const Scalar tz_fr_ee_link = 0.09960000216960907;
const Scalar tz_fr_wrist_3_link_COM = 0.07670000195503235;
const Scalar m_base_link_ur5e = 4.0;
const Scalar ix_base_link_ur5e = 0.004433331545442343;
const Scalar iy_base_link_ur5e = 0.004433331545442343;
const Scalar iz_base_link_ur5e = 0.007199999876320362;
const Scalar m_shoulder_link = 3.700000047683716;
const Scalar ix_shoulder_link = 0.010267496109008789;
const Scalar iy_shoulder_link = 0.010267496109008789;
const Scalar iz_shoulder_link = 0.006659999955445528;
const Scalar m_upper_arm_link = 8.392999649047852;
const Scalar comx_upper_arm_link = 0.21250000596046448;
const Scalar comy_upper_arm_link = -1.0409451078885468E-12;
const Scalar comz_upper_arm_link = 5.293955920339377E-23;
const Scalar ix_upper_arm_link = 0.01510739978402853;
const Scalar ixy_upper_arm_link = 5.190111451724988E-9;
const Scalar ixz_upper_arm_link = 2.2694818453580486E-16;
const Scalar iy_upper_arm_link = 0.5128821730613708;
const Scalar iyz_upper_arm_link = -6.457544057113371E-23;
const Scalar iz_upper_arm_link = 0.5128821730613708;
const Scalar m_forearm_link = 2.2750000953674316;
const Scalar comx_forearm_link = 0.19599999487400055;
const Scalar comy_forearm_link = -9.601208716958354E-13;
const Scalar comz_forearm_link = 2.6469779601696886E-23;
const Scalar ix_forearm_link = 0.004095000214874744;
const Scalar ixy_forearm_link = 1.1834782043251835E-9;
const Scalar ixz_forearm_link = 5.1750202522718076E-17;
const Scalar iy_forearm_link = 0.11857602000236511;
const Scalar iyz_forearm_link = -1.494211732837879E-23;
const Scalar iz_forearm_link = 0.11857602000236511;
const Scalar m_wrist_1_link = 1.218999981880188;
const Scalar comz_wrist_1_link = 0.12630000710487366;
const Scalar ix_wrist_1_link = 0.02200501039624214;
const Scalar ixz_wrist_1_link = 1.9721522630525295E-31;
const Scalar iy_wrist_1_link = 0.02163931168615818;
const Scalar iyz_wrist_1_link = -1.598521315315793E-11;
const Scalar iz_wrist_1_link = 0.0025598988868296146;
const Scalar m_wrist_2_link = 1.218999981880188;
const Scalar comx_wrist_2_link = -9.759999882938408E-13;
const Scalar comz_wrist_2_link = 0.09969999641180038;
const Scalar ix_wrist_2_link = 0.014676868915557861;
const Scalar ixz_wrist_2_link = -1.1861746394142836E-13;
const Scalar iy_wrist_2_link = 0.014676868915557861;
const Scalar iz_wrist_2_link = 0.0021941999439150095;
const Scalar m_wrist_3_link = 0.18790000677108765;
const Scalar comx_wrist_3_link = -5.293955920339377E-23;
const Scalar comz_wrist_3_link = 0.07670000195503235;
const Scalar ix_wrist_3_link = 0.0012042992748320103;
const Scalar ixy_wrist_3_link = 7.385694447913619E-25;
const Scalar iy_wrist_3_link = 0.0012042992748320103;
const Scalar iyz_wrist_3_link = -2.9035801540899797E-12;
const Scalar iz_wrist_3_link = 1.3211718760430813E-4;

}
}
#endif
