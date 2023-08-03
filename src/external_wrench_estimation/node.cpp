
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

#include <ewe4mav/ImfOut.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <mavros_msgs/ESCStatus.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "external_wrench_estimation/core/external_wrench_estimation.hpp"

using namespace external_wrench_estimation;

std::shared_ptr<ExternalWrenchEstimation> pEWE;

void ImfCb(const ewe4mav::ImfOut::ConstPtr &msg) {
    ExternalWrenchEstimation::MeasureMsg _msg;

    _msg.first = msg->header.stamp.toSec();

    _msg.second.a_m[0] = msg->unbiased_linear_acceleration.x;
    _msg.second.a_m[1] = msg->unbiased_linear_acceleration.y;
    _msg.second.a_m[2] = msg->unbiased_linear_acceleration.z;

    _msg.second.omega_m[0] = msg->unbiased_angular_velocity.x;
    _msg.second.omega_m[1] = msg->unbiased_angular_velocity.y;
    _msg.second.omega_m[2] = msg->unbiased_angular_velocity.z;

    _msg.second.q_m[0] = msg->attitude.x;
    _msg.second.q_m[1] = msg->attitude.y;
    _msg.second.q_m[2] = msg->attitude.z;
    _msg.second.q_m[3] = msg->attitude.w;

    _msg.second.v_m[0] = msg->velocity.x;
    _msg.second.v_m[1] = msg->velocity.y;
    _msg.second.v_m[2] = msg->velocity.z;

    pEWE->AddMeasureMsg(_msg);

    if (pEWE->_stop) {
        pEWE->header = _msg.first;
        pEWE->ClearOldMsg();
        pEWE->_stop = false;
    }
}

double k_f;
double k_m;
double L;

void RmuCb(const mavros_msgs::ESCStatus::ConstPtr &msg) {
    ExternalWrenchEstimation::InputMsg _msg;

    _msg.first = msg->header.stamp.toSec();

    Eigen::Vector4d rpm;
    rpm[0] = msg->esc_status[2].rpm;
    rpm[1] = msg->esc_status[1].rpm;
    rpm[2] = msg->esc_status[3].rpm;
    rpm[3] = msg->esc_status[0].rpm;

    rpm /= 1e4;

    _msg.second.F_c_body[0] = 0;
    _msg.second.F_c_body[1] = 0;
    _msg.second.F_c_body[2] = k_f * rpm.squaredNorm();
    _msg.second.M_c_body[0] = std::sqrt(2.0) / 4.0 * k_f * L * (rpm[0] * rpm[0] + rpm[1] * rpm[1] - rpm[2] * rpm[2] - rpm[3] * rpm[3]);
    _msg.second.M_c_body[1] = std::sqrt(2.0) / 4.0 * k_f * L * (-rpm[0] * rpm[0] + rpm[1] * rpm[1] + rpm[2] * rpm[2] - rpm[3] * rpm[3]);
    _msg.second.M_c_body[2] = k_m * (-rpm[0] * rpm[0] + rpm[1] * rpm[1] - rpm[2] * rpm[2] + rpm[3] * rpm[3]);

    pEWE->AddInputMsg(_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "external_wrench_estimation");
    ros::NodeHandle nh("~");

    std::string cfgPath;
    nh.getParam("cfgPath", cfgPath);
    YAML::Node cfg = YAML::LoadFile(cfgPath);
    k_f = cfg["Quadrotor"]["k_f"].as<double>();
    k_m = cfg["Quadrotor"]["k_m"].as<double>();
    L = cfg["Quadrotor"]["L"].as<double>();

    pEWE.reset(new ExternalWrenchEstimation(cfg));
    pEWE->SetX(LoadDefaultX(cfg));
    pEWE->SetCov(LoadDefaultErrorXcov());
    pEWE->SetParams(LoadDefaultParams(cfg));

    ros::Subscriber imfSub = nh.subscribe<ewe4mav::ImfOut>("imf", 100, ImfCb);
    ros::Subscriber rmuSub = nh.subscribe<mavros_msgs::ESCStatus>("rmu", 100, RmuCb);
    ros::Publisher ewePub = nh.advertise<geometry_msgs::WrenchStamped>("ewe", 100);

    geometry_msgs::WrenchStamped msg;
    U_t U;
    Y_t Y;
    ros::Rate rate(100);
    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();

        if (pEWE->_stop) continue;

        pEWE->_estimatorMutex.lock();

        while (pEWE->ClearOldMsg() && (!pEWE->_inputBuf.empty() || !pEWE->_measureBuf.empty())) {
            double last_header = pEWE->header;
            bool update;
            if (!pEWE->_measureBuf.empty() && !pEWE->_inputBuf.empty() && pEWE->_inputBuf.front()->first == pEWE->_measureBuf.front()->first) {
                pEWE->header = pEWE->_inputBuf.front()->first;
                U = pEWE->_inputBuf.front()->second;
                Y = pEWE->_measureBuf.front()->second;
                pEWE->_inputBuf.pop();
                pEWE->_measureBuf.pop();
                update = true;
            } else if (pEWE->_measureBuf.empty() || (!pEWE->_inputBuf.empty() && pEWE->_inputBuf.front()->first < pEWE->_measureBuf.front()->first)) {
                pEWE->header = pEWE->_inputBuf.front()->first;
                U = pEWE->_inputBuf.front()->second;
                pEWE->_inputBuf.pop();
                update = false;
            } else if (pEWE->_inputBuf.empty() || (!pEWE->_measureBuf.empty() && pEWE->_measureBuf.front()->first < pEWE->_inputBuf.front()->first)) {
                pEWE->header = pEWE->_measureBuf.front()->first;
                Y = pEWE->_measureBuf.front()->second;
                pEWE->_measureBuf.pop();
                update = true;
            } else {
                std::cerr << "Undefined case" << std::endl;
                return -1;
            }

            pEWE->_pKF->U = VectorizeU<double>(U);
            pEWE->_pKF->Predict(pEWE->header - last_header);

            if (update) {
                pEWE->_pKF->Y = VectorizeY<double>(Y);
                pEWE->_pKF->Update();
            }

            msg.header.stamp = ros::Time(pEWE->header);
            msg.header.seq++;
            msg.header.frame_id = std::string("base_link");

            msg.wrench.force.x = pEWE->_pKF->X[INDEX_X::X_F_ext_body + 0];
            msg.wrench.force.y = pEWE->_pKF->X[INDEX_X::X_F_ext_body + 1];
            msg.wrench.force.z = pEWE->_pKF->X[INDEX_X::X_F_ext_body + 2];

            msg.wrench.torque.x = pEWE->_pKF->X[INDEX_X::X_M_ext_body + 0];
            msg.wrench.torque.y = pEWE->_pKF->X[INDEX_X::X_M_ext_body + 1];
            msg.wrench.torque.z = pEWE->_pKF->X[INDEX_X::X_M_ext_body + 2];

            ewePub.publish(msg);
        }

        pEWE->_estimatorMutex.unlock();
    }

    return 0;
}
