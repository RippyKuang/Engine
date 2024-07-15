#include <visualize.h>
#include <iostream>
#include <world.h>

int main(int argc, char *argv[])
{
    Engine::AngleAxis aa(G_PI / 3, Engine::Vector3d{1, 0, 0});
    Engine::Cube cube(Engine::Vector3d{0.5, 0, 0}, 0.5, 0.5, 0.5);
    Engine::Camera camera(Engine::Vector3d{-0.5, 0, 0}, Engine::_R{300, 0, 640, 0, 300, 512, 0, 0, 1});
    Engine::World w;

    w.emplace(cube, 0);
    w.emplace(camera, 1);

    w.act(0, aa.toRotationMat());
    Engine::Item cube_in_camera = w.getCoord(0, 1);

    Engine::Camera *cam = (Engine::Camera *)w.get(1);
    std::vector<Engine::Point2i> projs = cam->project(cube_in_camera);
    for (auto proj : projs)
        std::cout << proj << std::endl;

    Engine::GFrame frame(argc, argv);
    frame.show();
    frame.updateData(projs);
    while (1)
    {
    }

    return 0;
}