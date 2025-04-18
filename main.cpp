#include <visualize.h>
#include <iostream>
#include <world.h>
#include <unistd.h>

//               Z    X
//               |   /
//               |  /
//       Y       | /
//       ---------/

//     O------O------
//     |
//     |
//     |
//     O
//   -----
using namespace Engine;

int main(int argc, char *argv[])
{
    Engine::GFrame frame(argc, argv);
    Cube base_link(Vector3d{0.1, 0.1, 0.05});
    Cube link0(Vector3d{0.05, 0.05, 0.2});
    Cube link1(Vector3d{0.05, 0.05, 0.2}, Vector3d{M_PI / 2, 0, 0}, Vector3d{0, -0.1, 0});
    Cube link2(Vector3d{0.05, 0.05, 0.2}, Vector3d{M_PI / 2, 0, 0}, Vector3d{0, -0.1, 0});

    Camera camera(Vector3d{-0.5, 0, 0}, _R{300, 0, 640, 0, 300, 512, 0, 0, 1});
    World w(camera);

    Joint j0(base_link, link0, Vector3d{0, 0, 0.125}, 0, new CONTINUOUS_INFO(AXIS_Z));
    Joint j1(link0, link1, Vector3d{0, 0, 0.125}, 1, new CONTINUOUS_INFO(AXIS_X));
    Joint j2(link1, link2, Vector3d{0, -0.2, 0}, 2, new CONTINUOUS_INFO(AXIS_X));

    w.parse_robot({j0, j1, j2});

    std::vector<Engine::Point2i> projs = w.project();

    frame.show();

    w.set_speed(0, M_PI/4);
    w.set_speed(1, M_PI/8);
    w.set_speed(2, M_PI/16);
    while (1)
    {
       
        std::vector<Engine::Point2i> projs = w.project();
        frame.updateData(projs);
    }
    return 0;
}