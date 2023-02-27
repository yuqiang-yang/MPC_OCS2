#include "graceful_mpc/costs/FrontOrientationCost.h"

using namespace graceful_mpc;

void FrontOrientationCost::intermediateCostFunction(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& input,
                                const ad_dynamic_vector_t& parameters, ad_scalar_t& costValue) const{
    costValue = kinematics_->getEEOrientationAtan(state);
    costValue = costValue * costValue * static_cast<ad_scalar_t>(weight_);

}

void FrontOrientationCost::terminalCostFunction(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& parameters,
                            ad_scalar_t& costValue) const{
    costValue = kinematics_->getEEOrientationAtan(state);
    costValue = costValue * costValue * static_cast<ad_scalar_t>(weight_);
    
    
    }

