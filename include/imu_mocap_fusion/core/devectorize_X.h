
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
#include <imu_mocap_fusion/types/X_t.hpp>

namespace imu_mocap_fusion {


template<typename Scalar>
imu_mocap_fusion::X_t DevectorizeX(const Eigen::Matrix<Scalar, 16, 1>& X_in) {
    imu_mocap_fusion::X_t _X_out;


    _X_out.p_world[0] = X_in(0, 0);
    _X_out.p_world[1] = X_in(1, 0);
    _X_out.p_world[2] = X_in(2, 0);
    _X_out.v_world[0] = X_in(3, 0);
    _X_out.v_world[1] = X_in(4, 0);
    _X_out.v_world[2] = X_in(5, 0);
    _X_out.world_q_body[0] = X_in(6, 0);
    _X_out.world_q_body[1] = X_in(7, 0);
    _X_out.world_q_body[2] = X_in(8, 0);
    _X_out.world_q_body[3] = X_in(9, 0);
    _X_out.b_a[0] = X_in(10, 0);
    _X_out.b_a[1] = X_in(11, 0);
    _X_out.b_a[2] = X_in(12, 0);
    _X_out.b_g[0] = X_in(13, 0);
    _X_out.b_g[1] = X_in(14, 0);
    _X_out.b_g[2] = X_in(15, 0);

    return _X_out;
}


}  // namespace imu_mocap_fusion
