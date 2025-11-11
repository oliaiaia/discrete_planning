#include "Wrapper.hpp"

    Wrapper::Wrapper(const std::vector<std::vector<std::vector<int>>>& rod,
            const std::vector<std::vector<int>>& environment)
    {
        if (rod.empty() || rod[0].empty() || rod[0][0].empty() || environment.empty()) {
            std::cout << "Got empty rod or environment matrix" << std::endl;
            return;
        }

        int rodRows = static_cast<int>(rod[0].size());
        int rodCols = static_cast<int>(rod[0][0].size());

        for (int t = 0; t < static_cast<int>(rod.size()); ++t) {
            Eigen::MatrixXd rodMat(rodRows, rodCols);
            for (int i = 0; i < rodRows; ++i)
                for (int j = 0; j < rodCols; ++j)
                    rodMat(i, j) = static_cast<double>(rod[t][i][j]);
            rodMats.push_back(rodMat);
        }

        int envRows = static_cast<int>(environment.size());
        int envCols = static_cast<int>(environment[0].size());
        environmentMat.resize(envRows, envCols);

        for (int i = 0; i < envRows; ++i)
            for (int j = 0; j < envCols; ++j)
                environmentMat(i, j) = static_cast<double>(environment[i][j]);

    }

    
    void Wrapper::normalizationMatrix(Eigen::MatrixXd &matrixNorm, double threshold) {
        for (int i = 0; i < matrixNorm.rows(); ++i)
            for (int j = 0; j < matrixNorm.cols(); ++j)
                matrixNorm(i, j) = (matrixNorm(i, j) < threshold ? 0.0 : 1.0);
    }

    int Wrapper::reflectIndex(int x, int max_x) {
        if (x < 0) return -x - 1;
        if (x >= max_x) return 2 * max_x - x - 1;
        return x;
    }

    void Wrapper::convolve2d(const Eigen::MatrixXd &environmentMat,
                    const Eigen::MatrixXd &kernelMat,
                    Eigen::MatrixXd &resultMat)
    {
        int H = environmentMat.rows();
        int W = environmentMat.cols();
        int kH = kernelMat.rows();
        int kW = kernelMat.cols();

        int kCenterY = kH / 2;
        int kCenterX = kW / 2;

        resultMat.resize(H, W);
        resultMat.setZero();

        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                double sum = 0.0;
                for (int m = 0; m < kH; ++m) {
                    for (int n = 0; n < kW; ++n) {
                        int yy = reflectIndex(y + (m - kCenterY), H);
                        int xx = reflectIndex(x + (n - kCenterX), W);
                        sum += environmentMat(yy, xx) * kernelMat(m, n);
                    }
                }
                resultMat(y, x) = sum;
            }
        }
    }

    std::vector<Eigen::MatrixXd> Wrapper::calculateCSpace() {
        
        std::vector<Eigen::MatrixXd> cSpaceMats;
        cSpaceMats.reserve(rodMats.size());
        
        for (const auto &rodPos : rodMats) {
            Eigen::MatrixXd resultMat;
            convolve2d(environmentMat, rodPos, resultMat);
            normalizationMatrix(resultMat);
            cSpaceMats.push_back(std::move(resultMat));
        }
        return cSpaceMats;
    }
