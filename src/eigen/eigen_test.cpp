#include <iostream>
#include "Eigen/Dense"

int main()
{

    Eigen::Matrix3d Matrixcomp;
    Matrixcomp << 1, 1, 1, 2, 2, 2, 3, 3, 3;
    std::cout << Matrixcomp << std::endl;
}