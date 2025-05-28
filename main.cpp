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
    Cube link1(Vector3d{0.05, 0.2, 0.05}, Vector3d{0, 0, 0}, Vector3d{0, -0.1 + 0.025, 0});

    base_link.set_name("base_link");
    link0.set_name("link0");
    link1.set_name("link1");

    Camera camera(Vector3d{-0.6, -0.3, 0.2}, _R{300, 0, ww / 2, 0, 300, hh / 2, 0, 0, 1});
    World w(camera, ww, hh);

    Part j0(base_link, link0, Vector3d{0, 0, 0.2 / 2 - 0.025}, new Revolute(AXIS_X));
    Part j1(link0, link1, Vector3d{0, -0.2, 0}, new Revolute(AXIS_X));

    Robot *robot = w.parse_robot({j0,j1});
    frame.show(w.get_camera_handle());
    robot->summary();
    robot->set_tau({0,0});
    while (1)
    {
        std::vector<Point2i> frame_projs;
        // std::vector<double> tau;
        // std::vector<double> v_dot{1,1};
        // std::vector<M66> X;
        // robot->ID(tau, v_dot, X);
        // robot->set_tau(tau);
    
        std::vector<pixel> projs;
        auto promise = w.project();
        usleep(50000);
        frame.updateFuture(std::move(promise));
    }
    return 0;

}