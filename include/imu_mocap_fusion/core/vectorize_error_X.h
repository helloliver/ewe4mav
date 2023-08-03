
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
#include <imu_mocap_fusion/types/error_X_t.hpp>

namespace imu_mocap_fusion {


template<typename Scalar>
Eigen::Matrix<Scalar, 15, 1> VectorizeErrorX(const imu_mocap_fusion::error_X_t& error_X_in) {
    Eigen::Matrix<Scalar, 15, 1> _error_X_out;


    _error_X_out(0, 0) = error_X_in.delta_p_world[0];
    _error_X_out(1, 0) = error_X_in.delta_p_world[1];
    _error_X_out(2, 0) = error_X_in.delta_p_world[2];
    _error_X_out(3, 0) = error_X_in.delta_v_world[0];
    _error_X_out(4, 0) = error_X_in.delta_v_world[1];
    _error_X_out(5, 0) = error_X_in.delta_v_world[2];
    _error_X_out(6, 0) = error_X_in.delta_theta[0];
    _error_X_out(7, 0) = error_X_in.delta_theta[1];
    _error_X_out(8, 0) = error_X_in.delta_theta[2];
    _error_X_out(9, 0) = error_X_in.delta_b_a[0];
    _error_X_out(10, 0) = error_X_in.delta_b_a[1];
    _error_X_out(11, 0) = error_X_in.delta_b_a[2];
    _error_X_out(12, 0) = error_X_in.delta_b_g[0];
    _error_X_out(13, 0) = error_X_in.delta_b_g[1];
    _error_X_out(14, 0) = error_X_in.delta_b_g[2];

    return _error_X_out;
}


}  // namespace imu_mocap_fusion
