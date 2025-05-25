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
    Mesh *mesh = Mesh::load_from_file("/home/kuang/project/Engine/sphere.obj");
    Cube base_link(Vector3d{0.05, 0.05, 0.2});
    Link link0(std::move(*mesh));
    base_link.set_name("base_link");
    link0.set_name("link0");
    Camera camera(Vector3d{-0.6, -0.3, 0.2}, _R{300, 0, 640, 0, 300, 512, 0, 0, 1});
    World w(camera, ww, hh);

    Part j0(base_link, link0, Vector3d{0, 0, 0.2 / 2 - 0.025}, new Revolute(AXIS_Z));

    const Robot *robot = w.parse_robot({j0});
    frame.show(w.get_camera_handle());
    robot->summary();

    while (1)
    {
        std::vector<Point2i> frame_projs;
        std::vector<_T> t;
        std::vector<Vector6d> v;
        robot->FK(t, v);

        w.project_frame(frame_projs, t);
        std::vector<Engine::pixel> projs;
        auto promise = w.project();
        usleep(50000);
        frame.updateFuture(std::move(promise), std::move(frame_projs));
    }
    return 0;
}