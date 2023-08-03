
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
#include <imu_mocap_fusion/types/params_t.hpp>

namespace imu_mocap_fusion {


template<typename Scalar>
imu_mocap_fusion::params_t DevectorizeParams(const Eigen::Matrix<Scalar, 20, 1>& params_in) {
    imu_mocap_fusion::params_t _params_out;


    _params_out.g = params_in(0, 0);
    _params_out.W_sigma.sigma_a_m[0] = params_in(1, 0);
    _params_out.W_sigma.sigma_a_m[1] = params_in(2, 0);
    _params_out.W_sigma.sigma_a_m[2] = params_in(3, 0);
    _params_out.W_sigma.sigma_omega_m[0] = params_in(4, 0);
    _params_out.W_sigma.sigma_omega_m[1] = params_in(5, 0);
    _params_out.W_sigma.sigma_omega_m[2] = params_in(6, 0);
    _params_out.W_sigma.sigma_b_a[0] = params_in(7, 0);
    _params_out.W_sigma.sigma_b_a[1] = params_in(8, 0);
    _params_out.W_sigma.sigma_b_a[2] = params_in(9, 0);
    _params_out.W_sigma.sigma_b_g[0] = params_in(10, 0);
    _params_out.W_sigma.sigma_b_g[1] = params_in(11, 0);
    _params_out.W_sigma.sigma_b_g[2] = params_in(12, 0);
    _params_out.N_sigma.sigma_mocap_p[0] = params_in(13, 0);
    _params_out.N_sigma.sigma_mocap_p[1] = params_in(14, 0);
    _params_out.N_sigma.sigma_mocap_p[2] = params_in(15, 0);
    _params_out.N_sigma.sigma_mocap_q[0] = params_in(16, 0);
    _params_out.N_sigma.sigma_mocap_q[1] = params_in(17, 0);
    _params_out.N_sigma.sigma_mocap_q[2] = params_in(18, 0);
    _params_out.N_sigma.sigma_mocap_q[3] = params_in(19, 0);

    return _params_out;
}


}  // namespace imu_mocap_fusion
