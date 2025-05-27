#include <visualize.h>
#include <iostream>
#include <world.h>
#include <unistd.h>
#include <robot.h>

//               Z    X
//               |   /
//               |  /
//       Y       | /
//       ---------/

using namespace Engine;

int main(int argc, char *argv[])
{
    DynamicMatrix<DynamicMatrix<double>> mat(2, 2);
    mat.data[0] = DynamicMatrix<double>(1,2,3,4,5,6).set_size(2,3);
    mat.data[1] = DynamicMatrix<double>(1,2,3,4,5,6).set_size(2,3);
    mat.data[2] = DynamicMatrix<double>(1,2,3).set_size(1,3);
    mat.data[3] = DynamicMatrix<double>(1,2,3).set_size(1,3);


    std::cout <<mat.get_size()<<std::endl;;
    for(int i =0;i<mat.get_size()[0];i++)
    {
        for(int j =0;j<mat.get_size()[1];j++)
        {
            std::cout<<mat.at(i,j)<<" ";
        }
        std::cout<<std::endl;
    }
    
    return 0;
}
