
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

#include "external_wrench_estimation/types/params_N_sigma_t.hpp"
#include "external_wrench_estimation/types/params_W_sigma_t.hpp"

namespace external_wrench_estimation {

class params_t {
   public:
    double m;
    std::array<double, 3> J;
    double g;
    ::external_wrench_estimation::params_W_sigma_t W_sigma;
    ::external_wrench_estimation::params_N_sigma_t N_sigma;

   public:
    params_t() = default;
};

}  // namespace external_wrench_estimation