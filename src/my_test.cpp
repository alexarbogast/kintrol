#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>

int main()
{
    Eigen::MatrixXd m(6, 6);

    m <<      0, 0.0576279, -0.334086, 0.000841471,       0, 0,
        0.62812,         0,         0,  -0.0988728,       0, 0,
              0,  -0.60312, -0.373608, 0.000540302, -0.1175, 0,
              0,         0,         0,    0.540302,       0, 1,
              0,         1,         1,           0,       1, 0,
              1,         0,         0,   -0.841471,       0, 0;

    
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd sigma = svd.singularValues().asDiagonal();
    Eigen::MatrixXd psuedo_inverse = svd.matrixV() * sigma.inverse() * svd.matrixU().transpose();

    std::cout << psuedo_inverse << std::endl;
}