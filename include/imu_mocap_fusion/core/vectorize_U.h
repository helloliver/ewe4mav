
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
#include <imu_mocap_fusion/types/U_t.hpp>

namespace imu_mocap_fusion {


template<typename Scalar>
Eigen::Matrix<Scalar, 6, 1> VectorizeU(const imu_mocap_fusion::U_t& U_in) {
    Eigen::Matrix<Scalar, 6, 1> _U_out;


    _U_out(0, 0) = U_in.a_m[0];
    _U_out(1, 0) = U_in.a_m[1];
    _U_out(2, 0) = U_in.a_m[2];
    _U_out(3, 0) = U_in.omega_m[0];
    _U_out(4, 0) = U_in.omega_m[1];
    _U_out(5, 0) = U_in.omega_m[2];

    return _U_out;
}


}  // namespace imu_mocap_fusion
