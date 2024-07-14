#include <visualize.h>
#include <iostream>
#include <world.h>

int main(int argc, char *argv[])
{
   

    Engine::Vector3d a{1, 0, 0};
    Engine::AngleAxis aa(G_PI / 2, a);
    Engine::Cube cube(Engine::Vector3d{0.5,0.5,0.5},1,1,1);
    Engine::Camera camera(Engine::Vector3d{1,1,1});
    Engine::World w;

    w.emplace(cube, 0);
    w.emplace(camera, 1);
    w.print(0);
    w.print(1);
    w.act(0, aa.toRotationMat());
    w.print(0);

    return 0;
}