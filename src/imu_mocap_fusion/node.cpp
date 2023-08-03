
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
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "imu_mocap_fusion/core/imu_mocap_fusion.hpp"

using namespace imu_mocap_fusion;

std::shared_ptr<ImuMocapFusion> pIMF;

void ImuCb(const sensor_msgs::Imu::ConstPtr &msg) {
    ImuMocapFusion::InputMsg _msg;

    _msg.first = msg->header.stamp.toSec();

    _msg.second.a_m[0] = msg->linear_acceleration.x;
    _msg.second.a_m[1] = msg->linear_acceleration.y;
    _msg.second.a_m[2] = msg->linear_acceleration.z;

    _msg.second.omega_m[0] = msg->angular_velocity.x;
    _msg.second.omega_m[1] = msg->angular_velocity.y;
    _msg.second.omega_m[2] = msg->angular_velocity.z;

    pIMF->AddInputMsg(_msg);
}

void MocapPoseFix(Y_t *const Y = nullptr) {
    auto _p = Y->mocap_p;
    Y->mocap_p[0] = _p[2];
    Y->mocap_p[1] = _p[0];
    Y->mocap_p[2] = _p[1];

    auto _q = Y->mocap_q;
    Y->mocap_q[0] = _q[2];
    Y->mocap_q[1] = _q[0];
    Y->mocap_q[2] = _q[1];
    Y->mocap_q[3] = _q[3];

    if (Y->mocap_q[3] < 0) {
        Y->mocap_q[0] = -Y->mocap_q[0];
        Y->mocap_q[1] = -Y->mocap_q[1];
        Y->mocap_q[2] = -Y->mocap_q[2];
        Y->mocap_q[3] = -Y->mocap_q[3];
    }
}

void MocapCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    ImuMocapFusion::MeasureMsg _msg;

    _msg.first = msg->header.stamp.toSec();

    _msg.second.mocap_p[0] = msg->pose.position.x;
    _msg.second.mocap_p[1] = msg->pose.position.y;
    _msg.second.mocap_p[2] = msg->pose.position.z;

    _msg.second.mocap_q[0] = msg->pose.orientation.x;
    _msg.second.mocap_q[1] = msg->pose.orientation.y;
    _msg.second.mocap_q[2] = msg->pose.orientation.z;
    _msg.second.mocap_q[3] = msg->pose.orientation.w;

    MocapPoseFix(&_msg.second);

    pIMF->AddMeasureMsg(_msg);

    if (pIMF->_stop) {
        pIMF->header = _msg.first;

        pIMF->_pKF->X[INDEX_X::X_p_world + 0] = _msg.second.mocap_p[0];
        pIMF->_pKF->X[INDEX_X::X_p_world + 1] = _msg.second.mocap_p[1];
        pIMF->_pKF->X[INDEX_X::X_p_world + 2] = _msg.second.mocap_p[2];

        pIMF->ClearOldMsg();
        pIMF->_stop = false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_mocap_fusion");
    ros::NodeHandle nh("~");

    std::string cfgPath;
    nh.getParam("cfgPath", cfgPath);
    YAML::Node cfg = YAML::LoadFile(cfgPath);

    pIMF.reset(new ImuMocapFusion(cfg));
    pIMF->SetX(LoadDefaultX(cfg));
    pIMF->SetCov(LoadDefaultErrorXcov());
    pIMF->SetParams(LoadDefaultParams(cfg));

    pIMF->_stop = true;

    ros::Subscriber imuSub = nh.subscribe<sensor_msgs::Imu>("imu", 100, ImuCb);
    ros::Subscriber mocapSub = nh.subscribe<geometry_msgs::PoseStamped>("mocap", 100, MocapCb);
    ros::Publisher imfPub = nh.advertise<ewe4mav::ImfOut>("imf", 100);

    ewe4mav::ImfOut msg;
    U_t U;
    Y_t Y;
    ros::Rate rate(100);
    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();

        if (pIMF->_stop) continue;

        pIMF->_estimatorMutex.lock();

        while (pIMF->ClearOldMsg() && (!pIMF->_inputBuf.empty() || !pIMF->_measureBuf.empty())) {
            double last_header = pIMF->header;
            bool update;
            if (!pIMF->_measureBuf.empty() && !pIMF->_inputBuf.empty() && pIMF->_inputBuf.front()->first == pIMF->_measureBuf.front()->first) {
                pIMF->header = pIMF->_inputBuf.front()->first;
                U = pIMF->_inputBuf.front()->second;
                Y = pIMF->_measureBuf.front()->second;
                pIMF->_inputBuf.pop();
                pIMF->_measureBuf.pop();
                update = true;
            } else if (pIMF->_measureBuf.empty() || (!pIMF->_inputBuf.empty() && pIMF->_inputBuf.front()->first < pIMF->_measureBuf.front()->first)) {
                pIMF->header = pIMF->_inputBuf.front()->first;
                U = pIMF->_inputBuf.front()->second;
                pIMF->_inputBuf.pop();
                update = false;
            } else if (pIMF->_inputBuf.empty() || (!pIMF->_measureBuf.empty() && pIMF->_measureBuf.front()->first < pIMF->_inputBuf.front()->first)) {
                pIMF->header = pIMF->_measureBuf.front()->first;
                Y = pIMF->_measureBuf.front()->second;
                pIMF->_measureBuf.pop();
                update = true;
            } else {
                std::cerr << "Undefined case" << std::endl;
                return -1;
            }

            pIMF->_pKF->U = VectorizeU<double>(U);
            pIMF->_pKF->Predict(pIMF->header - last_header);

            if (update) {
                pIMF->_pKF->Y = VectorizeY<double>(Y);
                pIMF->_pKF->Update();
            }

            msg.header.stamp = ros::Time(pIMF->header);
            msg.header.seq++;
            msg.header.frame_id = std::string("world");

            msg.unbiased_linear_acceleration.x = U.a_m[0] - pIMF->_pKF->X[INDEX_X::X_b_a + 0];
            msg.unbiased_linear_acceleration.y = U.a_m[1] - pIMF->_pKF->X[INDEX_X::X_b_a + 1];
            msg.unbiased_linear_acceleration.z = U.a_m[2] - pIMF->_pKF->X[INDEX_X::X_b_a + 2];

            msg.unbiased_angular_velocity.x = U.omega_m[0] - pIMF->_pKF->X[INDEX_X::X_b_g + 0];
            msg.unbiased_angular_velocity.y = U.omega_m[1] - pIMF->_pKF->X[INDEX_X::X_b_g + 1];
            msg.unbiased_angular_velocity.z = U.omega_m[2] - pIMF->_pKF->X[INDEX_X::X_b_g + 2];

            msg.attitude.x = pIMF->_pKF->X[INDEX_X::X_world_q_body + 0];
            msg.attitude.y = pIMF->_pKF->X[INDEX_X::X_world_q_body + 1];
            msg.attitude.z = pIMF->_pKF->X[INDEX_X::X_world_q_body + 2];
            msg.attitude.w = pIMF->_pKF->X[INDEX_X::X_world_q_body + 3];

            msg.velocity.x = pIMF->_pKF->X[INDEX_X::X_v_world + 0];
            msg.velocity.y = pIMF->_pKF->X[INDEX_X::X_v_world + 1];
            msg.velocity.z = pIMF->_pKF->X[INDEX_X::X_v_world + 2];

            imfPub.publish(msg);
        }

        pIMF->_estimatorMutex.unlock();
    }

    return 0;
}
