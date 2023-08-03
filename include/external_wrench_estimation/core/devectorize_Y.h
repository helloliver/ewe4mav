
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
external_wrench_estimation::Y_t DevectorizeY(const Eigen::Matrix<Scalar, 13, 1> &Y_in) {
    external_wrench_estimation::Y_t _Y_out;

    _Y_out.v_m[0] = Y_in(0, 0);
    _Y_out.v_m[1] = Y_in(1, 0);
    _Y_out.v_m[2] = Y_in(2, 0);
    _Y_out.q_m[0] = Y_in(3, 0);
    _Y_out.q_m[1] = Y_in(4, 0);
    _Y_out.q_m[2] = Y_in(5, 0);
    _Y_out.q_m[3] = Y_in(6, 0);
    _Y_out.a_m[0] = Y_in(7, 0);
    _Y_out.a_m[1] = Y_in(8, 0);
    _Y_out.a_m[2] = Y_in(9, 0);
    _Y_out.omega_m[0] = Y_in(10, 0);
    _Y_out.omega_m[1] = Y_in(11, 0);
    _Y_out.omega_m[2] = Y_in(12, 0);

    return _Y_out;
}

}  // namespace external_wrench_estimation
