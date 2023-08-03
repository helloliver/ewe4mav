
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
#include <external_wrench_estimation/types/Y_t.hpp>

namespace external_wrench_estimation {

template<typename Scalar>
Eigen::Matrix<Scalar, 13, 1> VectorizeY(const external_wrench_estimation::Y_t &Y_in) {
    Eigen::Matrix<Scalar, 13, 1> _Y_out;

    _Y_out(0, 0) = Y_in.v_m[0];
    _Y_out(1, 0) = Y_in.v_m[1];
    _Y_out(2, 0) = Y_in.v_m[2];
    _Y_out(3, 0) = Y_in.q_m[0];
    _Y_out(4, 0) = Y_in.q_m[1];
    _Y_out(5, 0) = Y_in.q_m[2];
    _Y_out(6, 0) = Y_in.q_m[3];
    _Y_out(7, 0) = Y_in.a_m[0];
    _Y_out(8, 0) = Y_in.a_m[1];
    _Y_out(9, 0) = Y_in.a_m[2];
    _Y_out(10, 0) = Y_in.omega_m[0];
    _Y_out(11, 0) = Y_in.omega_m[1];
    _Y_out(12, 0) = Y_in.omega_m[2];

    return _Y_out;
}

}  // namespace external_wrench_estimation
