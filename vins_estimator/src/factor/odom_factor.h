// why:2019-9-15
// 轮速计的雅克比和残差
#pragma once
#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "../utility/utility.h"
#include "../estimator/parameters.h"

#include <ceres/ceres.h>

class OdomFactor : public ceres::SizedCostFunction<3, 1, 1, 2, 2>
{
    public:
        InitialOdomFactor(const double &_theata, const Eigen::Vector2d & _P)
        {
            // TODO
            sqrt_info = 1.0 / (0.001) * Eigen::Matrix<double, 6, 6>::Identity();
        }

        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
        {
            // 待优化变量
            double theata_i;
            Eigen::Vector2d Pi(parameters[0][0], parameters[0][1]);

            double theata_j;
            Eigen::Vector2d Pj(parameters[1][0], parameters[1][1]);

            // 残差---- 暂定是否采用哈工大那篇论文残差方式
            Eigen::Map<Eigen::Matrix<double, 3, 1>> residual(residuals);
            // residual.block<1, 1>(0, 0) = 

        }


    Eigen::Matrix<double, 6, 6> sqrt_info;
}