
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
#include <imu_mocap_fusion/types/Y_t.hpp>

namespace imu_mocap_fusion {


template<typename Scalar>
Eigen::Matrix<Scalar, 7, 1> VectorizeY(const imu_mocap_fusion::Y_t& Y_in) {
    Eigen::Matrix<Scalar, 7, 1> _Y_out;


    _Y_out(0, 0) = Y_in.mocap_p[0];
    _Y_out(1, 0) = Y_in.mocap_p[1];
    _Y_out(2, 0) = Y_in.mocap_p[2];
    _Y_out(3, 0) = Y_in.mocap_q[0];
    _Y_out(4, 0) = Y_in.mocap_q[1];
    _Y_out(5, 0) = Y_in.mocap_q[2];
    _Y_out(6, 0) = Y_in.mocap_q[3];

    return _Y_out;
}


}  // namespace imu_mocap_fusion
