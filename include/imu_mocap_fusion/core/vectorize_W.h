
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
#include <imu_mocap_fusion/types/W_t.hpp>

namespace imu_mocap_fusion {


template<typename Scalar>
Eigen::Matrix<Scalar, 12, 1> VectorizeW(const imu_mocap_fusion::W_t& W_in) {
    Eigen::Matrix<Scalar, 12, 1> _W_out;


    _W_out(0, 0) = W_in.w_a_m[0];
    _W_out(1, 0) = W_in.w_a_m[1];
    _W_out(2, 0) = W_in.w_a_m[2];
    _W_out(3, 0) = W_in.w_omega_m[0];
    _W_out(4, 0) = W_in.w_omega_m[1];
    _W_out(5, 0) = W_in.w_omega_m[2];
    _W_out(6, 0) = W_in.w_b_a[0];
    _W_out(7, 0) = W_in.w_b_a[1];
    _W_out(8, 0) = W_in.w_b_a[2];
    _W_out(9, 0) = W_in.w_b_g[0];
    _W_out(10, 0) = W_in.w_b_g[1];
    _W_out(11, 0) = W_in.w_b_g[2];

    return _W_out;
}


}  // namespace imu_mocap_fusion
