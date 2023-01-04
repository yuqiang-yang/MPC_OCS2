#include "perceptive_mpc/costs/FrontOrientationCost.h"

using namespace perceptive_mpc;

void FrontOrientationCost::intermediateCostFunction(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& input,
                                const ad_dynamic_vector_t& parameters, ad_scalar_t& costValue) const{
    costValue = kinematics_->getEEOrientationAtan(state.tail<6>());
    costValue = costValue * costValue * static_cast<ad_scalar_t>(weight_);
    std::cerr << "(debugging) FrontOrientationCost" << costValue << std::endl;

}

void FrontOrientationCost::terminalCostFunction(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& parameters,
                            ad_scalar_t& costValue) const{
    costValue = 0;
}

