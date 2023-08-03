
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
imu_mocap_fusion::Y_t DevectorizeY(const Eigen::Matrix<Scalar, 7, 1>& Y_in) {
    imu_mocap_fusion::Y_t _Y_out;


    _Y_out.mocap_p[0] = Y_in(0, 0);
    _Y_out.mocap_p[1] = Y_in(1, 0);
    _Y_out.mocap_p[2] = Y_in(2, 0);
    _Y_out.mocap_q[0] = Y_in(3, 0);
    _Y_out.mocap_q[1] = Y_in(4, 0);
    _Y_out.mocap_q[2] = Y_in(5, 0);
    _Y_out.mocap_q[3] = Y_in(6, 0);

    return _Y_out;
}


}  // namespace imu_mocap_fusion
