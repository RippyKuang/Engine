#include <visualize.h>
#include <iostream>
#include <world.h>
#include <unistd.h>

//               Z    X
//               |   /
//               |  /
//       Y       | /
//       ---------/

int main(int argc, char *argv[])
{
    Engine::AngleAxis aa_cube(G_PI / 60, Engine::Vector3d{0, 1, 0});
    Engine::AngleAxis bb_cube(G_PI / 60, Engine::Vector3d{0, 0, 1});
    Engine::Cube cube0(Engine::Vector3d{0.6, 1, 0.6},Engine::Vector3d{0.95, 0, 0.5});
    Engine::Cube cube1(Engine::Vector3d{0.5, 0.5, 0.5},Engine::Vector3d{-0.15, 0.1, 0.4});

    Engine::Camera camera(Engine::Vector3d{-1.7, -0.5, 1.2}, Engine::_R{300, 0, 640, 0, 300, 512, 0, 0, 1});
    Engine::World w(camera);

    w.emplace(cube0, 0);
    w.emplace(cube1, 1);
    w.act(0, aa_cube.toRotationMat(), 0);
    w.act(1, aa_cube.toRotationMat(), 1);

    std::vector<Engine::Point2i> projs = w.project();

    Engine::GFrame frame(argc, argv);
    frame.show();

    frame.updateData(projs);

    while (1)
    {
        usleep(50010);
        w.act(1, aa_cube.toRotationMat(), 1);
        w.act(0, bb_cube.toRotationMat(), 0);
        projs = w.project();
        frame.updateData(projs);
    }

    return 0;
}