
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
#include <external_wrench_estimation/types/U_t.hpp>

namespace external_wrench_estimation {

template<typename Scalar>
Eigen::Matrix<Scalar, 6, 1> VectorizeU(const external_wrench_estimation::U_t &U_in) {
    Eigen::Matrix<Scalar, 6, 1> _U_out;

    _U_out(0, 0) = U_in.F_c_body[0];
    _U_out(1, 0) = U_in.F_c_body[1];
    _U_out(2, 0) = U_in.F_c_body[2];
    _U_out(3, 0) = U_in.M_c_body[0];
    _U_out(4, 0) = U_in.M_c_body[1];
    _U_out(5, 0) = U_in.M_c_body[2];

    return _U_out;
}

}  // namespace external_wrench_estimation
