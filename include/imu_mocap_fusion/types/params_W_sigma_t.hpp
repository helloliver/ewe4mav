
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


#include <array>


namespace imu_mocap_fusion {

class params_W_sigma_t {
   public:
    std::array<double, 3> sigma_a_m;
    std::array<double, 3> sigma_omega_m;
    std::array<double, 3> sigma_b_a;
    std::array<double, 3> sigma_b_g;

   public:
    params_W_sigma_t() = default;
};

}  // namespace imu_mocap_fusion