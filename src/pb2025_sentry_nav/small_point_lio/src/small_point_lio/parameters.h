/**
 * This file is part of Small Point-LIO, an advanced Point-LIO algorithm implementation.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#pragma once

#include <pch.h>

namespace small_point_lio {

    class Parameters {
    public:
        int point_filter_num;
        float min_distance_squared;
        float max_distance_squared;
        bool space_downsample;
        float space_downsample_leaf_size;

        Eigen::Vector3d gravity;
        bool check_satu;
        bool fix_gravity_direction;
        double satu_acc;
        double satu_gyro;
        double acc_norm;

        double map_resolution;
        size_t init_map_size;

        bool extrinsic_est_en;
        Eigen::Vector3d extrinsic_T;
        Eigen::Matrix3d extrinsic_R;

        double laser_point_cov;
        double imu_meas_acc_cov;
        double imu_meas_omg_cov;
        double velocity_cov;
        double omg_cov;
        double acceleration_cov;
        double bg_cov;
        double ba_cov;
        double plane_threshold;
        double match_sqaured;

        bool publish_odometry_without_downsample = false;

        // 轮速观测
        bool enable_wheel_fusion{false};
        bool wheel_mask_angular{true};        // true = 屏蔽轮速对角速度的更新（论文推荐）
        double wheel_meas_vel_cov{0.1};       // 轮速线速度测量协方差 R (vel部分)
        double wheel_meas_omg_cov{10.0};      // 轮速角速度测量协方差 R (omg部分)
        double wheel_residual_rms_scale{1.0}; // RMS残差缩放因子 k

        void read_parameters(rclcpp::Node &node);
    };

}// namespace small_point_lio
