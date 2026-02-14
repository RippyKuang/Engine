#include <visualize.h>
#include <iostream>
#include <world.h>
#include <unistd.h>
#include <robot.h>
#include <engine.hpp>
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
    EngineApplication app;

  //  Engine::GFrame frame(argc, argv, ww, hh);
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

    Robot *robot = w.parse_robot({j0, j1});
    app.loadWorld(w.getAllLinks());
  //  frame.show(w.get_camera_handle());
    robot->summary();
    robot->set_tau({0, 5});
    app.run();
   

    while (1)
    {
         std::vector<Engine::Matrix<float, 4, 4>> pose;
        robot->FK_vulkan(pose);
        //std::cout<<pose[1]<<std::endl;
        app.update(pose);
        usleep(16);

        // frame.updateFuture(std::move(promise));
    }
    return 0;
}