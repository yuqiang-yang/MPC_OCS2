#include "graceful_mpc/costs/ManipulabilityCost.h"

// #include <Eigen/Dense>
// #include <cppad/speed/det_by_minor.hpp>
using namespace graceful_mpc;

ad_scalar_t bruteforce_det3_helper(const Eigen::Matrix<ad_scalar_t,3,3>& matrix, int a, int b, int c)
{
  return matrix.coeff(0,a)
         * (matrix.coeff(1,b) * matrix.coeff(2,c) - matrix.coeff(1,c) * matrix.coeff(2,b));
}

ad_scalar_t bruteforce_det4_helper(const Eigen::Matrix<ad_scalar_t,4,4>& matrix, int j, int k, int m, int n)
{
  return (matrix.coeff(j,0) * matrix.coeff(k,1) - matrix.coeff(k,0) * matrix.coeff(j,1))
       * (matrix.coeff(m,2) * matrix.coeff(n,3) - matrix.coeff(n,2) * matrix.coeff(m,3));
}

void ManipulabilityCost::intermediateCostFunction(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& input,
                                const ad_dynamic_vector_t& parameters, ad_scalar_t& costValue) const{
    // Eigen::Matrix<ad_scalar_t,6,6> baseToWrist3Jacobian = kinematics_->getArmJacobian(state.head<Definitions::POSITION_STATE_DIM_>().tail<Definitions::ARM_STATE_DIM_>());

    // baseToWrist3Jacobian = baseToWrist3Jacobian.transpose()*baseToWrist3Jacobian;
    // costValue = calculateDeterminant(baseToWrist3Jacobian);
    // costValue = static_cast<ad_scalar_t>(-weight_)*CppAD::sqrt(costValue);

    // std::cerr << "(debugging) Manipulability cost:" << costValue << std::endl;

}

void ManipulabilityCost::terminalCostFunction(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& parameters,
                            ad_scalar_t& costValue) const{
    costValue = 0;
}

ad_scalar_t ManipulabilityCost::calculateDeterminant(const Eigen::Matrix<ad_scalar_t,6,6>& mat)const{
    ad_scalar_t determinantRes = ad_scalar_t(0);
    int cnt = 0;
    const int DEGREE = 6;
    for(int i = 0;i < DEGREE;i++){
        Eigen::Matrix<ad_scalar_t,DEGREE-1,DEGREE-1> mat1;
        cnt = 0;
        for(int j = 0;j < DEGREE;j++){
            if(j == i){
                continue;
            }
            mat1.col(cnt) = mat.col(j).tail<DEGREE-1>();
            cnt++;
        }
        determinantRes += CppAD::pow(-1,i) * mat(0,i) * calculateDeterminant(mat1);
        // std::cerr << "DEGREE:" << DEGREE << " i= " << i <<"CppAD::pow(-1,i):" << CppAD::pow(-1,i) << "mat="<<mat(0,i) <<" calculateDeterminant(mat1)" << calculateDeterminant(mat1) << std::endl;
    }
    return determinantRes;

}
ad_scalar_t ManipulabilityCost::calculateDeterminant(const Eigen::Matrix<ad_scalar_t,5,5>& mat)const{
    ad_scalar_t determinantRes = ad_scalar_t(0);
    int cnt = 0;
    const int DEGREE = 5;
    for(int i = 0;i < DEGREE;i++){
        Eigen::Matrix<ad_scalar_t,DEGREE-1,DEGREE-1> mat1;
        cnt = 0;
        for(int j = 0;j < DEGREE;j++){
            if(j == i){
                continue;
            }
            mat1.col(cnt) = mat.col(j).tail<DEGREE-1>();
            cnt++;
        }
        determinantRes += CppAD::pow(-1,i) * mat(0,i) *calculateDeterminant(mat1);
        // std::cerr << "DEGREE:" << DEGREE << " i= " << i <<"CppAD::pow(-1,i):" << CppAD::pow(-1,i) << "mat="<<mat(0,i) <<" calculateDeterminant(mat1)" << calculateDeterminant(mat1) << std::endl;

    }
    return determinantRes;
}
ad_scalar_t ManipulabilityCost::calculateDeterminant(const Eigen::Matrix<ad_scalar_t,4,4>& mat)const{
    // ad_scalar_t determinantRes = ad_scalar_t(0);
    // int cnt = 0;
    // const int DEGREE = 4;
    // for(int i = 0;i < DEGREE;i++){
    //     Eigen::Matrix<ad_scalar_t,DEGREE-1,DEGREE-1> mat1;
    //     cnt = 0;
    //     for(int j = 0;j < DEGREE;j++){
    //         if(j == i){
    //             continue;
    //         }
    //         mat1.col(cnt) = mat.col(j).tail<DEGREE-1>();
    //         cnt++;
    //     }
    //     determinantRes += CppAD::pow(-1,i) * mat(0,i) * calculateDeterminant(mat1);
    //     // std::cerr << "DEGREE:" << DEGREE << " i= " << i <<"CppAD::pow(-1,i):" << CppAD::pow(-1,i) << "mat="<<mat(0,i) <<" calculateDeterminant(mat1)" << calculateDeterminant(mat1) << std::endl;

    // }
    // return determinantRes;
    return bruteforce_det4_helper(mat,0,1,2,3)
          - bruteforce_det4_helper(mat,0,2,1,3)
          + bruteforce_det4_helper(mat,0,3,1,2)
          + bruteforce_det4_helper(mat,1,2,0,3)
          - bruteforce_det4_helper(mat,1,3,0,2)
          + bruteforce_det4_helper(mat,2,3,0,1);
}
ad_scalar_t ManipulabilityCost::calculateDeterminant(const Eigen::Matrix<ad_scalar_t,3,3>& mat)const{
    ad_scalar_t determinantRes = ad_scalar_t(0);
    int cnt = 0;
    const int DEGREE = 3;
    for(int i = 0;i < DEGREE;i++){
        Eigen::Matrix<ad_scalar_t,DEGREE-1,DEGREE-1> mat1;
        cnt = 0;
        for(int j = 0;j < DEGREE;j++){
            if(j == i){
                continue;
            }
            mat1.col(cnt) = mat.col(j).tail<DEGREE-1>();
            cnt++;
        }
        determinantRes += CppAD::pow(-1,i) * mat(0,i) * calculateDeterminant(mat1);
    }
    return determinantRes;
}
ad_scalar_t ManipulabilityCost::calculateDeterminant(const Eigen::Matrix<ad_scalar_t,2,2>& mat)const{
    return mat(0,0)*mat(1,1)-mat(0,1)*mat(1,0);
}
