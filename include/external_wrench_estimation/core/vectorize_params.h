
/******************************************************************************
 * Copyright 2023 YYHAN YIN. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/


#pragma once

#include <Eigen/Dense>
#include <external_wrench_estimation/types/params_t.hpp>

namespace external_wrench_estimation {

template<typename Scalar>
Eigen::Matrix<Scalar, 30, 1> VectorizeParams(const external_wrench_estimation::params_t &params_in) {
    Eigen::Matrix<Scalar, 30, 1> _params_out;

    _params_out(0, 0) = params_in.m;
    _params_out(1, 0) = params_in.J[0];
    _params_out(2, 0) = params_in.J[1];
    _params_out(3, 0) = params_in.J[2];
    _params_out(4, 0) = params_in.g;
    _params_out(5, 0) = params_in.W_sigma.sigma_a[0];
    _params_out(6, 0) = params_in.W_sigma.sigma_a[1];
    _params_out(7, 0) = params_in.W_sigma.sigma_a[2];
    _params_out(8, 0) = params_in.W_sigma.sigma_alpha[0];
    _params_out(9, 0) = params_in.W_sigma.sigma_alpha[1];
    _params_out(10, 0) = params_in.W_sigma.sigma_alpha[2];
    _params_out(11, 0) = params_in.W_sigma.sigma_F_ext_body[0];
    _params_out(12, 0) = params_in.W_sigma.sigma_F_ext_body[1];
    _params_out(13, 0) = params_in.W_sigma.sigma_F_ext_body[2];
    _params_out(14, 0) = params_in.W_sigma.sigma_M_ext_body[0];
    _params_out(15, 0) = params_in.W_sigma.sigma_M_ext_body[1];
    _params_out(16, 0) = params_in.W_sigma.sigma_M_ext_body[2];
    _params_out(17, 0) = params_in.N_sigma.sigma_v_m[0];
    _params_out(18, 0) = params_in.N_sigma.sigma_v_m[1];
    _params_out(19, 0) = params_in.N_sigma.sigma_v_m[2];
    _params_out(20, 0) = params_in.N_sigma.sigma_q_m[0];
    _params_out(21, 0) = params_in.N_sigma.sigma_q_m[1];
    _params_out(22, 0) = params_in.N_sigma.sigma_q_m[2];
    _params_out(23, 0) = params_in.N_sigma.sigma_q_m[3];
    _params_out(24, 0) = params_in.N_sigma.sigma_a_m[0];
    _params_out(25, 0) = params_in.N_sigma.sigma_a_m[1];
    _params_out(26, 0) = params_in.N_sigma.sigma_a_m[2];
    _params_out(27, 0) = params_in.N_sigma.sigma_omega_m[0];
    _params_out(28, 0) = params_in.N_sigma.sigma_omega_m[1];
    _params_out(29, 0) = params_in.N_sigma.sigma_omega_m[2];

    return _params_out;
}

}  // namespace external_wrench_estimation
