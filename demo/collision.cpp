#include <visualize.h>
#include <iostream>
#include <world.h>
#include <unistd.h>
#include <robot.h>
#include <obb.h>
//               Z    X
//               |   /
//               |  /
//       Y       | /
//       ---------/

using namespace Engine;

int main(int argc, char *argv[])
{
    obb_box box1{{0, 0, 0}, {1, 1, 1}, EYE(3)};
    obb_box box2{{0, 0, 1}, {1, 1, 1}, EYE(3)};
    std::vector<contact_results> outputs;
    int code = -1;
    std::cout << obb_Intersection(box1, box2, 4, outputs, code) << std::endl;
    for (auto res : outputs)
    {
        std::cout << "normal" << res.normal.T() << std::endl;
        std::cout << "points" << res.points.T() << std::endl;
        std::cout << "---------" << std::endl;
    }
    return 0;
}