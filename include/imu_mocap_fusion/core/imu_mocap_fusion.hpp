
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

#include <imu_mocap_fusion/core/devectorize_N.h>
#include <imu_mocap_fusion/core/devectorize_U.h>
#include <imu_mocap_fusion/core/devectorize_W.h>
#include <imu_mocap_fusion/core/devectorize_X.h>
#include <imu_mocap_fusion/core/devectorize_Y.h>
#include <imu_mocap_fusion/core/devectorize_error_X.h>
#include <imu_mocap_fusion/core/devectorize_params.h>
#include <imu_mocap_fusion/core/error_state_extended_kalman_filter.h>
#include <imu_mocap_fusion/core/vectorize_N.h>
#include <imu_mocap_fusion/core/vectorize_U.h>
#include <imu_mocap_fusion/core/vectorize_W.h>
#include <imu_mocap_fusion/core/vectorize_X.h>
#include <imu_mocap_fusion/core/vectorize_Y.h>
#include <imu_mocap_fusion/core/vectorize_error_X.h>
#include <imu_mocap_fusion/core/vectorize_params.h>
#include <yaml-cpp/yaml.h>

#include <condition_variable>
#include <fstream>
#include <imu_mocap_fusion/types/U_t.hpp>
#include <imu_mocap_fusion/types/X_t.hpp>
#include <imu_mocap_fusion/types/Y_t.hpp>
#include <imu_mocap_fusion/types/error_X_t.hpp>
#include <imu_mocap_fusion/types/params_t.hpp>
#include <iostream>
#include <map>
#include <mutex>
#include <queue>
#include <thread>

namespace imu_mocap_fusion {

class ImuMocapFusion {
   public:
    typedef std::pair<double, U_t> InputMsg;
    typedef std::pair<double, Y_t> MeasureMsg;
    typedef std::shared_ptr<InputMsg const> InputConstPtr;
    typedef std::shared_ptr<MeasureMsg const> MeasureConstPtr;

    double header;
    std::shared_ptr<Eigen::Matrix<double, 16, 1>> X;
    std::shared_ptr<Eigen::Matrix<double, -1, -1>> Cov;

   public:
    ImuMocapFusion(const YAML::Node &cfg);

    ~ImuMocapFusion();

    void SetX(const Eigen::Matrix<double, 16, 1> X) { _pKF->X = X; }

    void SetCov(const Eigen::Matrix<double, -1, -1> P) {
        assert(P.rows() == _pKF->P.rows() && P.cols() == _pKF->P.cols());
        _pKF->P = P;
    }

    void SetParams(const Eigen::Matrix<double, 20, 1> params) { _pKF->params = params; }

    void Config(const YAML::Node &cfg = YAML::Node(YAML::NodeType::Null)) { _pKF->Config(cfg); }

    void AddInputMsg(const InputMsg &msg);
    void AddMeasureMsg(const MeasureMsg &msg);

   public:
    std::shared_ptr<ErrorStateExtendedKalmanFilter<double>> _pKF;

    std::queue<InputConstPtr> _inputBuf;
    std::queue<MeasureConstPtr> _measureBuf;

    std::condition_variable _con;
    std::mutex _bufMutex;
    std::mutex _estimatorMutex;

    std::ofstream _logFile;

    bool _stop = false;

    bool ClearOldMsg();
};

ImuMocapFusion::ImuMocapFusion(const YAML::Node &cfg) {
    header = 0;

    _pKF.reset(new ErrorStateExtendedKalmanFilter<double>());
    _pKF->Config(cfg);

    X = std::make_shared<Eigen::Matrix<double, 16, 1>>(_pKF->X);
    Cov = std::make_shared<Eigen::Matrix<double, -1, -1>>(_pKF->P);
}

ImuMocapFusion::~ImuMocapFusion() {
    header = 0;

    if (_logFile.is_open()) _logFile.close();

    _bufMutex.lock();
    while (!_inputBuf.empty()) _inputBuf.pop();
    while (!_measureBuf.empty()) _measureBuf.pop();
    _bufMutex.unlock();

    _estimatorMutex.lock();

    _estimatorMutex.unlock();
}

void ImuMocapFusion::AddInputMsg(const InputMsg &msg) {
    static double last_header = 0;
    if (msg.first <= last_header) {
        return;
    }
    last_header = msg.first;

    if (msg.first < header) {
        return;
    }

    _bufMutex.lock();
    _inputBuf.push(std::make_shared<imu_mocap_fusion::ImuMocapFusion::InputMsg>(msg));
    _bufMutex.unlock();
    _con.notify_one();
}

void ImuMocapFusion::AddMeasureMsg(const MeasureMsg &msg) {
    static double last_header = 0;
    if (msg.first <= last_header) {
        return;
    }
    last_header = msg.first;

    if (msg.first < header) {
        return;
    }

    _bufMutex.lock();
    _measureBuf.push(std::make_shared<imu_mocap_fusion::ImuMocapFusion::MeasureMsg>(msg));
    _bufMutex.unlock();
    _con.notify_one();
}

bool ImuMocapFusion::ClearOldMsg() {
    _bufMutex.lock();
    while (!_inputBuf.empty()) {
        if (_inputBuf.front()->first < header) {
            _inputBuf.pop();
        } else {
            break;
        }
    }

    while (!_measureBuf.empty()) {
        if (_measureBuf.front()->first < header) {
            _measureBuf.pop();
        } else {
            break;
        }
    }
    _bufMutex.unlock();
    return true;
}

Eigen::Matrix<double, 16, 1> LoadDefaultX(const YAML::Node &cfg) {
    imu_mocap_fusion::X_t X;

    X.p_world = std::array<double, 3>{};
    X.v_world = std::array<double, 3>{};
    X.world_q_body = std::array<double, 4>{0, 0, 0, 1};
    X.b_a = std::array<double, 3>{0.3, 0.1, -0.08};
    X.b_g = std::array<double, 3>{};

    return VectorizeX<double>(X);
}

template<typename T, size_t N>
std::array<T, N> Vector2Array(std::vector<T> vector) {
    std::array<T, N> array;
    for (size_t i = 0; i < N; ++i) {
        array[i] = vector[i];
    }
    return array;
}

Eigen::Matrix<double, 20, 1> LoadDefaultParams(const YAML::Node &cfg) {
    imu_mocap_fusion::params_t params;

    params.g = cfg["Global"]["g"].as<double>();

    params.W_sigma.sigma_omega_m = Vector2Array<double, 3>(cfg["Estimator"]["W_sigma"]["sigma_omega_m"].as<std::vector<double>>());
    params.W_sigma.sigma_b_g = Vector2Array<double, 3>(cfg["Estimator"]["W_sigma"]["sigma_b_g"].as<std::vector<double>>());
    params.W_sigma.sigma_a_m = Vector2Array<double, 3>(cfg["Estimator"]["W_sigma"]["sigma_a_m"].as<std::vector<double>>());
    params.W_sigma.sigma_b_a = Vector2Array<double, 3>(cfg["Estimator"]["W_sigma"]["sigma_b_a"].as<std::vector<double>>());

    params.N_sigma.sigma_mocap_p = Vector2Array<double, 3>(cfg["Estimator"]["N_sigma"]["sigma_mocap_p"].as<std::vector<double>>());
    params.N_sigma.sigma_mocap_q = Vector2Array<double, 4>(cfg["Estimator"]["N_sigma"]["sigma_mocap_q"].as<std::vector<double>>());

    return VectorizeParams<double>(params);
}

Eigen::Matrix<double, 15, 15> LoadDefaultErrorXcov() {
    Eigen::Matrix<double, 15, 15> P = 1e-1 * Eigen::Matrix<double, 15, 15>::Identity();

    return P;
}

}  // namespace imu_mocap_fusion