
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
#include <external_wrench_estimation/types/N_t.hpp>

namespace external_wrench_estimation {

template<typename Scalar>
external_wrench_estimation::N_t DevectorizeN(const Eigen::Matrix<Scalar, 13, 1> &N_in) {
    external_wrench_estimation::N_t _N_out;

    _N_out.n_v_m[0] = N_in(0, 0);
    _N_out.n_v_m[1] = N_in(1, 0);
    _N_out.n_v_m[2] = N_in(2, 0);
    _N_out.n_q_m[0] = N_in(3, 0);
    _N_out.n_q_m[1] = N_in(4, 0);
    _N_out.n_q_m[2] = N_in(5, 0);
    _N_out.n_q_m[3] = N_in(6, 0);
    _N_out.n_a_m[0] = N_in(7, 0);
    _N_out.n_a_m[1] = N_in(8, 0);
    _N_out.n_a_m[2] = N_in(9, 0);
    _N_out.n_omega_m[0] = N_in(10, 0);
    _N_out.n_omega_m[1] = N_in(11, 0);
    _N_out.n_omega_m[2] = N_in(12, 0);

    return _N_out;
}

}  // namespace external_wrench_estimation
