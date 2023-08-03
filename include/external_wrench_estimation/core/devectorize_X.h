
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
external_wrench_estimation::X_t DevectorizeX(const Eigen::Matrix<Scalar, 16, 1> &X_in) {
    external_wrench_estimation::X_t _X_out;

    _X_out.v_world[0] = X_in(0, 0);
    _X_out.v_world[1] = X_in(1, 0);
    _X_out.v_world[2] = X_in(2, 0);
    _X_out.world_q_body[0] = X_in(3, 0);
    _X_out.world_q_body[1] = X_in(4, 0);
    _X_out.world_q_body[2] = X_in(5, 0);
    _X_out.world_q_body[3] = X_in(6, 0);
    _X_out.omega[0] = X_in(7, 0);
    _X_out.omega[1] = X_in(8, 0);
    _X_out.omega[2] = X_in(9, 0);
    _X_out.F_ext_body[0] = X_in(10, 0);
    _X_out.F_ext_body[1] = X_in(11, 0);
    _X_out.F_ext_body[2] = X_in(12, 0);
    _X_out.M_ext_body[0] = X_in(13, 0);
    _X_out.M_ext_body[1] = X_in(14, 0);
    _X_out.M_ext_body[2] = X_in(15, 0);

    return _X_out;
}

}  // namespace external_wrench_estimation
