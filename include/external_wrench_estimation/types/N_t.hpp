
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

namespace external_wrench_estimation {

class N_t {
   public:
    std::array<double, 3> n_v_m;
    std::array<double, 4> n_q_m;
    std::array<double, 3> n_a_m;
    std::array<double, 3> n_omega_m;

   public:
    N_t() = default;
};

}  // namespace external_wrench_estimation