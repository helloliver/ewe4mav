
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

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

namespace external_wrench_estimation {

#ifndef DEF_external_wrench_estimation_INDEX_X
#define DEF_external_wrench_estimation_INDEX_X
enum INDEX_X {
    X_v_world = 0,
    X_world_q_body = 3,
    X_omega = 7,
    X_F_ext_body = 10,
    X_M_ext_body = 13,
};
#endif

template<typename Scalar>
class ErrorStateExtendedKalmanFilter {
   public:
    Eigen::Matrix<Scalar, 16, 1> X;
    Eigen::Matrix<Scalar, 15, 1> error_X;
    Eigen::Matrix<Scalar, 13, 1> Y;
    Eigen::Matrix<Scalar, 6, 1> U;
    Eigen::Matrix<Scalar, 30, 1> params;
    Eigen::Matrix<Scalar, -1, -1> P;

   private:
    Eigen::Matrix<Scalar, 15, 15> Q;
    Eigen::Matrix<Scalar, 13, 13> R;
    Eigen::Matrix<Scalar, 15, 15> F;
    Eigen::Matrix<Scalar, 13, 15> H;
    Eigen::Matrix<Scalar, 15, 15> G;
    Eigen::Matrix<Scalar, 15, 13> K;

   public:
    ErrorStateExtendedKalmanFilter() {
        this->P.resize(15, 15);
        error_X.setZero();
    };

    ~ErrorStateExtendedKalmanFilter() = default;

    void Config(const YAML::Node &cfg = YAML::Node(YAML::NodeType::Null)){};

    void Predict(const Scalar &deltaT) {
        if (deltaT == 0) {
            return;
        }

        Eigen::Matrix<Scalar, 16, 1> X_vee;
        Eigen::Matrix<Scalar, 15, 15> P_vee;

        Process(this->U, this->X, this->params, deltaT, &X_vee, &F, &Q);
        P_vee = F * this->P * F.transpose() + Q;

        this->X = X_vee;

        this->X = RestoreX(this->X);

        this->P = P_vee;
    }

    void Update(void) {
        Eigen::Matrix<Scalar, 16, 1> X_vee = this->X;
        Eigen::Matrix<Scalar, 13, 1> Y_vee;
        Eigen::Matrix<Scalar, 15, 15> P_vee = this->P;

        Measure(this->U, X_vee, this->params, &Y_vee, &H, &R);
        K = P_vee * H.transpose() * ((H * P_vee * H.transpose() + R).inverse());
        error_X = K * (this->Y - Y_vee);

        this->P = (Eigen::Matrix<Scalar, 15, 15>::Identity() - K * H) * P_vee * ((Eigen::Matrix<Scalar, 15, 15>::Identity() - K * H).transpose()) + K * R * K.transpose();

        this->X = ComposeX(X_vee, error_X);

        this->X = RestoreX(this->X);

        Reset(error_X, &G);
        error_X.setZero();

        this->P = G * this->P * G.transpose();
    }

   private:
    void Process(const Eigen::Matrix<Scalar, 6, 1> &U, const Eigen::Matrix<Scalar, 16, 1> &X, const Eigen::Matrix<Scalar, 30, 1> &params, const Scalar deltaT, Eigen::Matrix<Scalar, 16, 1> *const X_predict = nullptr, Eigen::Matrix<Scalar, 15, 15> *const F = nullptr, Eigen::Matrix<Scalar, 15, 15> *const Q = nullptr) {
        const Scalar _tmp0 = 2 * X(6, 0);
        const Scalar _tmp1 = X(5, 0) * _tmp0;
        const Scalar _tmp2 = 2 * X(4, 0);
        const Scalar _tmp3 = X(3, 0) * _tmp2;
        const Scalar _tmp4 = _tmp1 - _tmp3;
        const Scalar _tmp5 = -_tmp4;
        const Scalar _tmp6 = Scalar(1.0) / (params(0, 0));
        const Scalar _tmp7 = X(3, 0) * _tmp0;
        const Scalar _tmp8 = X(5, 0) * _tmp2;
        const Scalar _tmp9 = _tmp7 + _tmp8;
        const Scalar _tmp10 = params(0, 0) * params(4, 0);
        const Scalar _tmp11 = _tmp6 * (U(1, 0) + X(11, 0) + _tmp10 * _tmp9);
        const Scalar _tmp12 = X(4, 0) * _tmp0;
        const Scalar _tmp13 = 2 * X(3, 0) * X(5, 0);
        const Scalar _tmp14 = _tmp12 + _tmp13;
        const Scalar _tmp15 = std::pow(X(3, 0), Scalar(2));
        const Scalar _tmp16 = 2 * _tmp15;
        const Scalar _tmp17 = std::pow(X(4, 0), Scalar(2));
        const Scalar _tmp18 = 2 * _tmp17 - 1;
        const Scalar _tmp19 = -_tmp16 - _tmp18;
        const Scalar _tmp20 = _tmp6 * (U(2, 0) + X(12, 0) + _tmp10 * _tmp19);
        const Scalar _tmp21 = std::pow(X(5, 0), Scalar(2));
        const Scalar _tmp22 = 2 * _tmp21;
        const Scalar _tmp23 = -_tmp18 - _tmp22;
        const Scalar _tmp24 = _tmp12 - _tmp13;
        const Scalar _tmp25 = -_tmp24;
        const Scalar _tmp26 = _tmp6 * (U(0, 0) + X(10, 0) + _tmp10 * _tmp25);
        const Scalar _tmp27 = _tmp1 + _tmp3;
        const Scalar _tmp28 = _tmp7 - _tmp8;
        const Scalar _tmp29 = -_tmp28;
        const Scalar _tmp30 = -_tmp16 - _tmp22 + 1;
        const Scalar _tmp31 = (Scalar(1) / Scalar(2)) * X(7, 0);
        const Scalar _tmp32 = (Scalar(1) / Scalar(2)) * X(8, 0);
        const Scalar _tmp33 = (Scalar(1) / Scalar(2)) * X(9, 0);
        const Scalar _tmp34 = X(8, 0) * params(2, 0);
        const Scalar _tmp35 = X(9, 0) * params(3, 0);
        const Scalar _tmp36 = deltaT / params(1, 0);
        const Scalar _tmp37 = X(9, 0) * params(1, 0);
        const Scalar _tmp38 = deltaT / params(2, 0);
        const Scalar _tmp39 = X(8, 0) * params(1, 0);
        const Scalar _tmp40 = deltaT / params(3, 0);
        const Scalar _tmp41 = -_tmp9;
        const Scalar _tmp42 = _tmp41 * params(4, 0);
        const Scalar _tmp43 = -_tmp17;
        const Scalar _tmp44 = std::pow(X(6, 0), Scalar(2));
        const Scalar _tmp45 = -_tmp15 + _tmp44;
        const Scalar _tmp46 = _tmp21 + _tmp43 + _tmp45;
        const Scalar _tmp47 = _tmp46 * params(4, 0);
        const Scalar _tmp48 = -_tmp21;
        const Scalar _tmp49 = _tmp17 + _tmp45 + _tmp48;
        const Scalar _tmp50 = X(9, 0) * deltaT;
        const Scalar _tmp51 = X(8, 0) * deltaT;
        const Scalar _tmp52 = _tmp25 * params(4, 0);
        const Scalar _tmp53 = -_tmp46;
        const Scalar _tmp54 = _tmp53 * params(4, 0);
        const Scalar _tmp55 = _tmp15 + _tmp43 + _tmp44 + _tmp48;
        const Scalar _tmp56 = X(7, 0) * deltaT;
        const Scalar _tmp57 = _tmp24 * params(4, 0);
        const Scalar _tmp58 = _tmp9 * params(4, 0);
        const Scalar _tmp59 = -_tmp35;
        const Scalar _tmp60 = X(7, 0) * params(1, 0);
        const Scalar _tmp61 = _tmp6 * deltaT;
        const Scalar _tmp62 = std::pow(deltaT, Scalar(2));

        if (X_predict != nullptr) {
            Eigen::Matrix<Scalar, 16, 1> &_X_predict = (*X_predict);

            _X_predict(0, 0) = X(0, 0) + deltaT * (_tmp11 * _tmp5 + _tmp14 * _tmp20 + _tmp23 * _tmp26);
            _X_predict(1, 0) = X(1, 0) + deltaT * (_tmp11 * _tmp30 + _tmp20 * _tmp29 + _tmp26 * _tmp27);
            _X_predict(2, 0) = X(2, 0) + deltaT * (_tmp11 * _tmp9 + _tmp19 * _tmp20 + _tmp25 * _tmp26);
            _X_predict(3, 0) = X(3, 0) + deltaT * (X(4, 0) * _tmp33 - X(5, 0) * _tmp32 + X(6, 0) * _tmp31);
            _X_predict(4, 0) = X(4, 0) + deltaT * (-X(3, 0) * _tmp33 + X(5, 0) * _tmp31 + X(6, 0) * _tmp32);
            _X_predict(5, 0) = X(5, 0) + deltaT * (X(3, 0) * _tmp32 - X(4, 0) * _tmp31 + X(6, 0) * _tmp33);
            _X_predict(6, 0) = X(6, 0) + deltaT * (-X(3, 0) * _tmp31 - X(4, 0) * _tmp32 - X(5, 0) * _tmp33);
            _X_predict(7, 0) = X(7, 0) + _tmp36 * (U(3, 0) + X(13, 0) - X(8, 0) * _tmp35 + X(9, 0) * _tmp34);
            _X_predict(8, 0) = X(8, 0) + _tmp38 * (U(4, 0) + X(14, 0) + X(7, 0) * _tmp35 - X(7, 0) * _tmp37);
            _X_predict(9, 0) = X(9, 0) + _tmp40 * (U(5, 0) + X(15, 0) - X(7, 0) * _tmp34 + X(7, 0) * _tmp39);
            _X_predict(10, 0) = X(10, 0);
            _X_predict(11, 0) = X(11, 0);
            _X_predict(12, 0) = X(12, 0);
            _X_predict(13, 0) = X(13, 0);
            _X_predict(14, 0) = X(14, 0);
            _X_predict(15, 0) = X(15, 0);
        }

        if (F != nullptr) {
            Eigen::Matrix<Scalar, 15, 15> &_F = (*F);

            _F.setZero();

            _F(0, 0) = 1;
            _F(1, 1) = 1;
            _F(2, 2) = 1;
            _F(0, 3) = deltaT * (_tmp11 * _tmp14 + _tmp14 * _tmp42 + _tmp20 * _tmp4 + _tmp47 * _tmp5);
            _F(1, 3) = deltaT * (_tmp11 * _tmp29 - _tmp20 * _tmp49 + _tmp29 * _tmp42 + _tmp30 * _tmp47);
            _F(2, 3) = deltaT * (_tmp11 * _tmp46 + _tmp19 * _tmp42 + _tmp20 * _tmp41 + _tmp47 * _tmp9);
            _F(3, 3) = 1;
            _F(4, 3) = -_tmp50;
            _F(5, 3) = _tmp51;
            _F(0, 4) = deltaT * (-_tmp14 * _tmp26 + _tmp14 * _tmp52 + _tmp20 * _tmp55 + _tmp23 * _tmp54);
            _F(1, 4) = deltaT * (_tmp20 * _tmp27 + _tmp26 * _tmp28 + _tmp27 * _tmp54 + _tmp29 * _tmp52);
            _F(2, 4) = deltaT * (_tmp19 * _tmp52 + _tmp20 * _tmp25 + _tmp26 * _tmp53 + _tmp52 * _tmp53);
            _F(3, 4) = _tmp50;
            _F(4, 4) = 1;
            _F(5, 4) = -_tmp56;
            _F(0, 5) = deltaT * (-_tmp11 * _tmp55 + _tmp23 * _tmp58 + _tmp26 * _tmp5 + _tmp5 * _tmp57);
            _F(1, 5) = deltaT * (-_tmp11 * _tmp27 + _tmp26 * _tmp49 + _tmp27 * _tmp58 + _tmp30 * _tmp57);
            _F(2, 5) = deltaT * (_tmp11 * _tmp24 + _tmp26 * _tmp9 + _tmp52 * _tmp9 + _tmp57 * _tmp9);
            _F(3, 5) = -_tmp51;
            _F(4, 5) = _tmp56;
            _F(5, 5) = 1;
            _F(3, 6) = deltaT;
            _F(6, 6) = 1;
            _F(7, 6) = _tmp38 * (-_tmp37 - _tmp59);
            _F(8, 6) = _tmp40 * (-_tmp34 + _tmp39);
            _F(4, 7) = deltaT;
            _F(6, 7) = _tmp36 * (X(9, 0) * params(2, 0) + _tmp59);
            _F(7, 7) = 1;
            _F(8, 7) = _tmp40 * (-X(7, 0) * params(2, 0) + _tmp60);
            _F(5, 8) = deltaT;
            _F(6, 8) = _tmp36 * (-X(8, 0) * params(3, 0) + _tmp34);
            _F(7, 8) = _tmp38 * (X(7, 0) * params(3, 0) - _tmp60);
            _F(8, 8) = 1;
            _F(0, 9) = _tmp23 * _tmp61;
            _F(1, 9) = _tmp27 * _tmp61;
            _F(2, 9) = _tmp25 * _tmp61;
            _F(9, 9) = 1;
            _F(0, 10) = _tmp5 * _tmp61;
            _F(1, 10) = _tmp30 * _tmp61;
            _F(2, 10) = _tmp61 * _tmp9;
            _F(10, 10) = 1;
            _F(0, 11) = _tmp14 * _tmp61;
            _F(1, 11) = _tmp29 * _tmp61;
            _F(2, 11) = _tmp19 * _tmp61;
            _F(11, 11) = 1;
            _F(6, 12) = _tmp36;
            _F(12, 12) = 1;
            _F(7, 13) = _tmp38;
            _F(13, 13) = 1;
            _F(8, 14) = _tmp40;
            _F(14, 14) = 1;
        }

        if (Q != nullptr) {
            Eigen::Matrix<Scalar, 15, 15> &_Q = (*Q);

            _Q.setZero();

            _Q(0, 0) = _tmp62 * std::pow(params(5, 0), Scalar(2));
            _Q(1, 1) = _tmp62 * std::pow(params(6, 0), Scalar(2));
            _Q(2, 2) = _tmp62 * std::pow(params(7, 0), Scalar(2));
            _Q(6, 6) = _tmp62 * std::pow(params(8, 0), Scalar(2));
            _Q(7, 7) = _tmp62 * std::pow(params(9, 0), Scalar(2));
            _Q(8, 8) = _tmp62 * std::pow(params(10, 0), Scalar(2));
            _Q(9, 9) = _tmp62 * std::pow(params(11, 0), Scalar(2));
            _Q(10, 10) = _tmp62 * std::pow(params(12, 0), Scalar(2));
            _Q(11, 11) = _tmp62 * std::pow(params(13, 0), Scalar(2));
            _Q(12, 12) = _tmp62 * std::pow(params(14, 0), Scalar(2));
            _Q(13, 13) = _tmp62 * std::pow(params(15, 0), Scalar(2));
            _Q(14, 14) = _tmp62 * std::pow(params(16, 0), Scalar(2));
        }
    }

    void Measure(const Eigen::Matrix<Scalar, 6, 1> &U, const Eigen::Matrix<Scalar, 16, 1> &X, const Eigen::Matrix<Scalar, 30, 1> &params, Eigen::Matrix<Scalar, 13, 1> *const Y = nullptr, Eigen::Matrix<Scalar, 13, 15> *const H = nullptr, Eigen::Matrix<Scalar, 13, 13> *const R = nullptr) {
        const Scalar _tmp0 = Scalar(1.0) / (params(0, 0));
        const Scalar _tmp1 = (Scalar(1) / Scalar(2)) * X(6, 0);
        const Scalar _tmp2 = (Scalar(1) / Scalar(2)) * X(5, 0);
        const Scalar _tmp3 = (Scalar(1) / Scalar(2)) * X(4, 0);
        const Scalar _tmp4 = -_tmp3;
        const Scalar _tmp5 = (Scalar(1) / Scalar(2)) * X(3, 0);
        const Scalar _tmp6 = -_tmp5;
        const Scalar _tmp7 = -_tmp2;

        if (Y != nullptr) {
            Eigen::Matrix<Scalar, 13, 1> &_Y = (*Y);

            _Y(0, 0) = X(0, 0);
            _Y(1, 0) = X(1, 0);
            _Y(2, 0) = X(2, 0);
            _Y(3, 0) = X(3, 0);
            _Y(4, 0) = X(4, 0);
            _Y(5, 0) = X(5, 0);
            _Y(6, 0) = X(6, 0);
            _Y(7, 0) = _tmp0 * (U(0, 0) + X(10, 0));
            _Y(8, 0) = _tmp0 * (U(1, 0) + X(11, 0));
            _Y(9, 0) = _tmp0 * (U(2, 0) + X(12, 0));
            _Y(10, 0) = X(7, 0);
            _Y(11, 0) = X(8, 0);
            _Y(12, 0) = X(9, 0);
        }

        if (H != nullptr) {
            Eigen::Matrix<Scalar, 13, 15> &_H = (*H);

            _H.setZero();

            _H(0, 0) = 1;
            _H(1, 1) = 1;
            _H(2, 2) = 1;
            _H(3, 3) = _tmp1;
            _H(4, 3) = _tmp2;
            _H(5, 3) = _tmp4;
            _H(6, 3) = _tmp6;
            _H(3, 4) = _tmp7;
            _H(4, 4) = _tmp1;
            _H(5, 4) = _tmp5;
            _H(6, 4) = _tmp4;
            _H(3, 5) = _tmp3;
            _H(4, 5) = _tmp6;
            _H(5, 5) = _tmp1;
            _H(6, 5) = _tmp7;
            _H(10, 6) = 1;
            _H(11, 7) = 1;
            _H(12, 8) = 1;
            _H(7, 9) = _tmp0;
            _H(8, 10) = _tmp0;
            _H(9, 11) = _tmp0;
        }

        if (R != nullptr) {
            Eigen::Matrix<Scalar, 13, 13> &_R = (*R);

            _R.setZero();

            _R(0, 0) = std::pow(params(17, 0), Scalar(2));
            _R(1, 1) = std::pow(params(18, 0), Scalar(2));
            _R(2, 2) = std::pow(params(19, 0), Scalar(2));
            _R(3, 3) = std::pow(params(20, 0), Scalar(2));
            _R(4, 4) = std::pow(params(21, 0), Scalar(2));
            _R(5, 5) = std::pow(params(22, 0), Scalar(2));
            _R(6, 6) = std::pow(params(23, 0), Scalar(2));
            _R(7, 7) = std::pow(params(24, 0), Scalar(2));
            _R(8, 8) = std::pow(params(25, 0), Scalar(2));
            _R(9, 9) = std::pow(params(26, 0), Scalar(2));
            _R(10, 10) = std::pow(params(27, 0), Scalar(2));
            _R(11, 11) = std::pow(params(28, 0), Scalar(2));
            _R(12, 12) = std::pow(params(29, 0), Scalar(2));
        }
    }

    void Reset(const Eigen::Matrix<Scalar, 15, 1> &error_X, Eigen::Matrix<Scalar, 15, 15> *const G = nullptr) {
        const Scalar _tmp0 = (Scalar(1) / Scalar(2)) * error_X(5, 0);
        const Scalar _tmp1 = (Scalar(1) / Scalar(2)) * error_X(4, 0);
        const Scalar _tmp2 = (Scalar(1) / Scalar(2)) * error_X(3, 0);

        if (G != nullptr) {
            Eigen::Matrix<Scalar, 15, 15> &_G = (*G);

            _G.setZero();

            _G(0, 0) = 1;
            _G(1, 1) = 1;
            _G(2, 2) = 1;
            _G(3, 3) = 1;
            _G(4, 3) = -_tmp0;
            _G(5, 3) = _tmp1;
            _G(3, 4) = _tmp0;
            _G(4, 4) = 1;
            _G(5, 4) = -_tmp2;
            _G(3, 5) = -_tmp1;
            _G(4, 5) = _tmp2;
            _G(5, 5) = 1;
            _G(6, 6) = 1;
            _G(7, 7) = 1;
            _G(8, 8) = 1;
            _G(9, 9) = 1;
            _G(10, 10) = 1;
            _G(11, 11) = 1;
            _G(12, 12) = 1;
            _G(13, 13) = 1;
            _G(14, 14) = 1;
        }
    }

    Eigen::Matrix<Scalar, 16, 1> ComposeX(const Eigen::Matrix<Scalar, 16, 1> &X, const Eigen::Matrix<Scalar, 15, 1> &error_X) {
        const Scalar _tmp0 = (Scalar(1) / Scalar(2)) * error_X(3, 0);
        const Scalar _tmp1 = (Scalar(1) / Scalar(2)) * error_X(4, 0);
        const Scalar _tmp2 = (Scalar(1) / Scalar(2)) * error_X(5, 0);

        Eigen::Matrix<Scalar, 16, 1> _X_;

        _X_(0, 0) = X(0, 0) + error_X(0, 0);
        _X_(1, 0) = X(1, 0) + error_X(1, 0);
        _X_(2, 0) = X(2, 0) + error_X(2, 0);
        _X_(3, 0) = X(3, 0) + X(4, 0) * _tmp2 - X(5, 0) * _tmp1 + X(6, 0) * _tmp0;
        _X_(4, 0) = -X(3, 0) * _tmp2 + X(4, 0) + X(5, 0) * _tmp0 + X(6, 0) * _tmp1;
        _X_(5, 0) = X(3, 0) * _tmp1 - X(4, 0) * _tmp0 + X(5, 0) + X(6, 0) * _tmp2;
        _X_(6, 0) = -X(3, 0) * _tmp0 - X(4, 0) * _tmp1 - X(5, 0) * _tmp2 + X(6, 0);
        _X_(7, 0) = X(7, 0) + error_X(6, 0);
        _X_(8, 0) = X(8, 0) + error_X(7, 0);
        _X_(9, 0) = X(9, 0) + error_X(8, 0);
        _X_(10, 0) = X(10, 0) + error_X(9, 0);
        _X_(11, 0) = X(11, 0) + error_X(10, 0);
        _X_(12, 0) = X(12, 0) + error_X(11, 0);
        _X_(13, 0) = X(13, 0) + error_X(12, 0);
        _X_(14, 0) = X(14, 0) + error_X(13, 0);
        _X_(15, 0) = X(15, 0) + error_X(14, 0);

        return _X_;
    }

    Eigen::Matrix<Scalar, 16, 1> RestoreX(const Eigen::Matrix<Scalar, 16, 1> &X_in) {
        const Scalar _tmp0 = std::pow(Scalar(std::pow(X_in(3, 0), Scalar(2)) + std::pow(X_in(4, 0), Scalar(2)) + std::pow(X_in(5, 0), Scalar(2)) + std::pow(X_in(6, 0), Scalar(2))), Scalar(Scalar(-1) / Scalar(2)));

        Eigen::Matrix<Scalar, 16, 1> _X_out;

        _X_out(0, 0) = X_in(0, 0);
        _X_out(1, 0) = X_in(1, 0);
        _X_out(2, 0) = X_in(2, 0);
        _X_out(3, 0) = X_in(3, 0) * _tmp0;
        _X_out(4, 0) = X_in(4, 0) * _tmp0;
        _X_out(5, 0) = X_in(5, 0) * _tmp0;
        _X_out(6, 0) = X_in(6, 0) * _tmp0;
        _X_out(7, 0) = X_in(7, 0);
        _X_out(8, 0) = X_in(8, 0);
        _X_out(9, 0) = X_in(9, 0);
        _X_out(10, 0) = X_in(10, 0);
        _X_out(11, 0) = X_in(11, 0);
        _X_out(12, 0) = X_in(12, 0);
        _X_out(13, 0) = X_in(13, 0);
        _X_out(14, 0) = X_in(14, 0);
        _X_out(15, 0) = X_in(15, 0);

        return _X_out;
    }
};

}  // namespace external_wrench_estimation