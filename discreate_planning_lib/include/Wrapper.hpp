#pragma once

#include <vector>
#include <Eigen/Dense>
#include <iostream>

class Wrapper {
public:

    Wrapper(const std::vector<std::vector<std::vector<int>>>& rod,
            const std::vector<std::vector<int>>& environment);

    std::vector<Eigen::MatrixXd> calculateCSpace();

    ~Wrapper() = default;

private:

    std::vector<Eigen::MatrixXd> rodMats;
    Eigen::MatrixXd environmentMat;

    void normalizationMatrix(Eigen::MatrixXd &matrixNorm, double threshold = 0.1);

    int reflectIndex(int x, int max_x);

    // scipy.signal.convolve2d(..., boundary='symm', mode='same')
    void convolve2d(const Eigen::MatrixXd &environmentMat,
                    const Eigen::MatrixXd &kernelMat,
                    Eigen::MatrixXd &resultMat);

};
