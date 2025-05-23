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
    const int ww = 1280;
    const int hh = 1024;
    Engine::GFrame frame(argc, argv, ww, hh);
    Cube base_link(Vector3d{0.05, 0.05, 0.2});
    Cube link0(Vector3d{0.05, 0.2, 0.05}, Vector3d{0, 0, 0}, Vector3d{0, -0.1 + 0.025, 0});
    Cube link1(Vector3d{0.2, 0.05, 0.05}, Vector3d{0, 0, 0}, Vector3d{+0.1, 0, 0});
    Cube link2(Vector3d{0.05, 0.05, 0.05});
    Cube link3(Vector3d{0.025, 0.025, 0.15});

    base_link.set_name("base_link");
    link0.set_name("link0");
    link1.set_name("link1");
    link2.set_name("link2");
    link3.set_name("link3");

    Camera camera(Vector3d{-0.6, -0.3, 0.2}, _R{300, 0, 640, 0, 300, 512, 0, 0, 1});
    World w(camera, ww, hh);

    Part j0(base_link, link0, Vector3d{0, 0, 0.2 / 2 - 0.025}, new Revolute(AXIS_Z));
    Part j1(link0, link1, Vector3d{0, -0.2 + 0.05, +0.05}, new Revolute(AXIS_Z));
    Part j2(link1, link2, Vector3d{0.2, 0, -0.05}, new Revolute(AXIS_Z));
    Part j3(link2, link3, Vector3d{0, 0, -0.075}, new Revolute(AXIS_Z));

    const Robot *robot = w.parse_robot({j0, j1, j2, j3});
    frame.show();
    robot->summary();

    while (1)
    {
        std::vector<Point2i> frame_projs;
        std::vector<_T> t;
        robot->FK(t);

        w.project_frame(frame_projs, t);
        std::vector<Engine::pixel> projs;
        auto promise = w.project();
        usleep(50000);
        frame.updateFuture(std::move(promise), std::move(frame_projs));
    }
    return 0;
}