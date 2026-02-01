#include <visualize.h>
#include <iostream>
#include <world.h>
#include <unistd.h>
#include <robot.h>
#include <solver.h>
//               Z    X
//               |   /
//               |  /
//       Y       | /
//       ---------/

using namespace Engine;

int main(int argc, char *argv[])
{
    Matrix<double, 4, 4> A = {
        4, -1, 0, 0,
        -1, 4, -1, 0,
        0, -1, 4, -1,
        0, 0, -1, 4};

    Matrix<double, 4, 1> b = {
        -1,
        -2,
        -2,
        -1};

    Matrix<double, 4, 1> x = {0, 0, 0, 0};
    LCPsolver::solve(A.data, b.data, x.data, 4);
    for (int i = 0; i < 4; i++)
    {
        std::cout << x.data[i] << std::endl;
    }
    std::cout << "------------------------" << std::endl;
    std::cout << A * x + b << std::endl;
    std::cout << "------------------------" << std::endl;
    std::cout << x.T() * (A * x + b) << std::endl;
}