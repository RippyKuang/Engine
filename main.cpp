#include <visualize.h>
#include <iostream>
#include <world.h>
#include <unistd.h>

//               Z    X
//               |   /
//               |  /
//       Y       | /
//       ---------/

using namespace Engine;

int main(int argc, char *argv[])
{
    const int _w = 1280;
    const int _h = 1024;
    Engine::GFrame frame(argc, argv, _w, _h);
    Cube base_link(Vector3d{0.05, 0.05, 0.2});
    Cube link0(Vector3d{0.05, 0.2, 0.05}, Vector3d{0, 0, 0}, Vector3d{0, -0.1 + 0.025, 0});
    Cube link1(Vector3d{0.2, 0.05, 0.05}, Vector3d{0, 0, 0}, Vector3d{+0.1, 0, 0});
    Cube link2(Vector3d{0.05, 0.05, 0.05});
    Cube link3(Vector3d{0.025, 0.025, 0.15});

    Camera camera(Vector3d{-0.6, -0.3, 0.2}, _R{300, 0, _w/2, 0, 300, _h/2, 0, 0, 1});
    World w(camera,_w,_h);

    Joint j0(base_link, link0, Vector3d{0, 0, 0.1 - 0.025}, 0, new CONTINUOUS_INFO(AXIS_Z));
    Joint j1(link0, link1, Vector3d{0, -0.2 + 0.05, 0.05}, 1, new CONTINUOUS_INFO(AXIS_Z));
    Joint j2(link1, link2, Vector3d{0.2, 0, -0.05}, 2, new CONTINUOUS_INFO(AXIS_Z));
    Joint j3(link2, link3, Vector3d{0, 0, -0.075}, 3, new CONTINUOUS_INFO(AXIS_Z));

    w.parse_robot({j0, j1, j2, j3});
   

    frame.show();

    w.set_speed(0, M_PI / 2);
    w.set_speed(1, M_PI / 2);
    w.set_speed(2, M_PI / 2);
    w.set_speed(3, M_PI / 2);

    while (1)
    {
        std::vector<_T> t = w.get_pose({0, 1, 2, 3});

        std::vector<Engine::Point2i> frame_projs;
        w.project_frame(frame_projs, t);

        std::vector<Twist> v;
        std::vector<Twist> dv;
        w.inverse_dynamics(v, dv);

        std::vector<Engine::pixel> projs;
        w.project(projs);
        frame.updateData(projs, std::move(frame_projs));
    }
    return 0;
}