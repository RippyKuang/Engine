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

    Camera camera(Vector3d{-0.6, -0.3, 0.2}, _R{300, 0, ww / 2, 0, 300, hh / 2, 0, 0, 1});
    World w(camera, ww, hh);

    Part j0(base_link, link0, Vector3d{0, 0, 0.2 / 2 - 0.025}, new Revolute(AXIS_Z));
    Part j1(link0, link1, Vector3d{0, -0.2 + 0.05, +0.05}, new Revolute(AXIS_Z));
    Part j2(link1, link2, Vector3d{0.2, 0, -0.05}, new Revolute(AXIS_Z));
    Part j3(link2, link3, Vector3d{0, 0, -0.075}, new Revolute(AXIS_Z));

    const Robot *robot = w.parse_robot({j0, j1});
    frame.show(w.get_camera_handle());
    robot->summary();

    while (1)
    {
        std::vector<Point2i> frame_projs;
        std::vector<_T> t;
        std::vector<Vector6d> v;
        robot->FK(t, v);
        std::vector<Vector6d> tau;
        std::vector<double> v_dot = {1, 1, 1, 1};
        robot->ID(tau, v_dot);
        std::cout << "v0: " << v[0].T() << std::endl;
        std::cout << "v1: " << v[1].T() << std::endl;

        std::cout << "tau0: " << tau[0].T() << std::endl;
        std::cout << "tau1: " << tau[1].T() << std::endl;

        w.project_frame(frame_projs, t);
        std::vector<pixel> projs;
        auto promise = w.project();
        usleep(50000);
        frame.updateFuture(std::move(promise), std::move(frame_projs));
    }
    return 0;
}