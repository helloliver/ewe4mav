
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
#include <external_wrench_estimation/types/error_X_t.hpp>

namespace external_wrench_estimation {

template<typename Scalar>
external_wrench_estimation::error_X_t DevectorizeErrorX(const Eigen::Matrix<Scalar, 15, 1> &error_X_in) {
    external_wrench_estimation::error_X_t _error_X_out;

    _error_X_out.delta_v_world[0] = error_X_in(0, 0);
    _error_X_out.delta_v_world[1] = error_X_in(1, 0);
    _error_X_out.delta_v_world[2] = error_X_in(2, 0);
    _error_X_out.delta_theta[0] = error_X_in(3, 0);
    _error_X_out.delta_theta[1] = error_X_in(4, 0);
    _error_X_out.delta_theta[2] = error_X_in(5, 0);
    _error_X_out.delta_omega[0] = error_X_in(6, 0);
    _error_X_out.delta_omega[1] = error_X_in(7, 0);
    _error_X_out.delta_omega[2] = error_X_in(8, 0);
    _error_X_out.delta_F_ext_body[0] = error_X_in(9, 0);
    _error_X_out.delta_F_ext_body[1] = error_X_in(10, 0);
    _error_X_out.delta_F_ext_body[2] = error_X_in(11, 0);
    _error_X_out.delta_M_ext_body[0] = error_X_in(12, 0);
    _error_X_out.delta_M_ext_body[1] = error_X_in(13, 0);
    _error_X_out.delta_M_ext_body[2] = error_X_in(14, 0);

    return _error_X_out;
}

}  // namespace external_wrench_estimation
