#include <visualize.h>
#include <iostream>
#include <world.h>

int main(int argc, char *argv[])
{
    Engine::AngleAxis aa_cube(G_PI / 3, Engine::Vector3d{0, 1, 0});
    Engine::Cube cube0(Engine::Vector3d{0.5, 0, 0}, 0.5, 0.5, 0.5);
    Engine::Cube cube1(Engine::Vector3d{0, 0, 0}, 0.5, 0.5, 0.5);

    Engine::Camera camera(Engine::Vector3d{2, 0, 0}, Engine::_R{300, 0, 640, 0, 300, 512, 0, 0, 1});
    Engine::World w(camera);

    w.emplace(cube0, 0);
    w.emplace(cube1, 1);
    w.act(0, aa_cube.toRotationMat());
    w.act(1, aa_cube.toRotationMat());

    std::vector<Engine::Point2i> projs = w.project();

    Engine::GFrame frame(argc, argv);
    frame.show();

    frame.updateData(projs);
    while (1)
    {
    }

    return 0;
}