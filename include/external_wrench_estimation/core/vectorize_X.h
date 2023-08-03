
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
#include <external_wrench_estimation/types/X_t.hpp>

namespace external_wrench_estimation {

template<typename Scalar>
Eigen::Matrix<Scalar, 16, 1> VectorizeX(const external_wrench_estimation::X_t &X_in) {
    Eigen::Matrix<Scalar, 16, 1> _X_out;

    _X_out(0, 0) = X_in.v_world[0];
    _X_out(1, 0) = X_in.v_world[1];
    _X_out(2, 0) = X_in.v_world[2];
    _X_out(3, 0) = X_in.world_q_body[0];
    _X_out(4, 0) = X_in.world_q_body[1];
    _X_out(5, 0) = X_in.world_q_body[2];
    _X_out(6, 0) = X_in.world_q_body[3];
    _X_out(7, 0) = X_in.omega[0];
    _X_out(8, 0) = X_in.omega[1];
    _X_out(9, 0) = X_in.omega[2];
    _X_out(10, 0) = X_in.F_ext_body[0];
    _X_out(11, 0) = X_in.F_ext_body[1];
    _X_out(12, 0) = X_in.F_ext_body[2];
    _X_out(13, 0) = X_in.M_ext_body[0];
    _X_out(14, 0) = X_in.M_ext_body[1];
    _X_out(15, 0) = X_in.M_ext_body[2];

    return _X_out;
}

}  // namespace external_wrench_estimation
