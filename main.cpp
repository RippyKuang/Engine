#include <visualize.h>
#include <iostream>
#include <world.h>

int main(int argc, char *argv[])
{
    Engine::AngleAxis aa_cube(G_PI / 3, Engine::Vector3d{0, 1, 0});
    Engine::AngleAxis aa_cam(G_PI / 3, Engine::Vector3d{0, 0, 1});
    Engine::Cube cube(Engine::Vector3d{1, 0, 0}, 0.5, 0.5, 0.5);
    Engine::Camera camera(Engine::Vector3d{2, 0, 0}, Engine::_R{300, 0, 640, 0, 300, 512, 0, 0, 1});
    Engine::World w;

    w.emplace(cube, 0);
    w.emplace(camera, 1);

    w.act(0, aa_cube.toRotationMat());
    w.act(1, aa_cam.toRotationMat(), 0);
    Engine::Item cube_in_camera = w.getCoord(0, 1);
    Engine::Camera cam = w.getCamHandle(1);
    std::vector<Engine::Point2i> projs = cam.project(cube_in_camera);

    Engine::GFrame frame(argc, argv);
    frame.show();
    frame.updateData(projs);
    while (1)
    {
    }

    return 0;
}