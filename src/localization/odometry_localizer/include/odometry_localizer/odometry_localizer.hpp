/**
 * @file odometry_localizer.hpp
 * @author Haoru Xue (hxue@ucsd.edu)
 * @brief Localize with odometry and IMU
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021 [Triton AI]
 * 
 */

#ifndef ODOMETRY_LOCALIZER__ODOMETRY_LOCALIZER_HPP_
#define ODOMETRY_LOCALIZER__ODOMETRY_LOCALIZER_HPP_
#include <cstdint>
#include <Eigen/Geometry>

namespace tritonai
{
    namespace localization
    {

        class OdometryLocalizer
        {
        public:
            explicit OdometryLocalizer();
            void update_lin_acc(double x, double y, double z, int64_t time_ns);
            void update_ang_vel(double x, double y, double z, int64_t time_ns);
            void update_vel(double x, double y, double z, int64_t time_ns);
            void update_orientation(double x, double y, double z, double w);

            Eigen::Vector3d get_position();
            Eigen::Vector3d get_velocity();
            Eigen::Vector3d get_absolute_velocity();
            Eigen::Quaterniond get_orientation();

            private:
            Eigen::Vector3d pos_;
            Eigen::Vector3d velocity_;
            Eigen::Quaterniond orientation_;
        };
    }
}

#endif // ODOMETRY_LOCALIZER__ODOMETRY_LOCALIZER_HPP_