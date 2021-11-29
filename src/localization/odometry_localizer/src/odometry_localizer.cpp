/**
 * @file odometry_localizer.cpp
 * @author Haoru Xue (hxue@ucsd.edu)
 * @brief Localize with odometry and IMU
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021 [Triton AI]
 * 
 */

#include "odometry_localizer/odometry_localizer.hpp"
#include <stdexcept>
#include <iostream>
#include <math.h>

namespace tritonai
{
    namespace localization
    {

        OdometryLocalizer::OdometryLocalizer()
            : pos_(0.0, 0.0, 0.0),
              velocity_(0.0, 0.0, 0.0),
              orientation_(1.0, 0.0, 0.0, 0.0)
        {
        }

        void OdometryLocalizer::update_lin_acc(double x, double y, double z, int64_t time_ns)
        {
            static int64_t last_update_ns = 0;
            static bool initialized = false;
            if (!initialized)
            {
                initialized = true;
                last_update_ns = time_ns;
                return;
            }
            double delta_s = (time_ns - last_update_ns) * 1e-9;
            if (delta_s < 0)
            {
                throw std::logic_error("detected past timestamp");
            }

            Eigen::Vector3d lin_acc(x, y, z);
            lin_acc = orientation_ * lin_acc;       // Transform to odom frame
            pos_ += velocity_ * delta_s + 0.5 * lin_acc * pow(delta_s, 2); // x = x_0 + v_0 * t + 1/2 * a * t^2
            velocity_ += lin_acc * delta_s;                                // v = v_0 + a * t

            last_update_ns = time_ns;
        }
        void OdometryLocalizer::update_ang_vel(double x, double y, double z, int64_t time_ns)
        {
            static int64_t last_update_ns = 0;
            static bool initialized = false;
            if (!initialized)
            {
                initialized = true;
                last_update_ns = time_ns;
                return;
            }
            double delta_s = (time_ns - last_update_ns) * 1e-9;
            if (delta_s < 0)
            {
                throw std::logic_error("detected past timestamp");
            }

            Eigen::Vector3d ang_vel(x, y, z);
            auto ang_rad = orientation_.toRotationMatrix().eulerAngles(0, 1, 2);
            ang_rad += orientation_ * ang_vel * delta_s;
            orientation_ = Eigen::AngleAxisd(ang_rad.x(), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(ang_rad.y(), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(ang_rad.z(), Eigen::Vector3d::UnitZ());

            last_update_ns = time_ns;
        }
        void OdometryLocalizer::update_vel(double x, double y, double z, int64_t time_ns)
        {
            throw std::logic_error("not implemented");
        }
        void OdometryLocalizer::update_orientation(double x, double y, double z, double w)
        {
            orientation_ = Eigen::Quaterniond(w, x, y, z).normalized();
        }
        Eigen::Vector3d OdometryLocalizer::get_position()
        {
            return pos_;
        }
        Eigen::Vector3d OdometryLocalizer::get_velocity()
        {
            return orientation_.inverse().normalized() * velocity_;
        }
        Eigen::Vector3d OdometryLocalizer::get_absolute_velocity()
        {
            return velocity_;
        }
        Eigen::Quaterniond OdometryLocalizer::get_orientation()
        {
            return orientation_;
        }
    }
}