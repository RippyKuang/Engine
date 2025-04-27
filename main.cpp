// #include <iostream>
// #include <unistd.h>
// #include <trajectory.h>
// #include <timer.h>

// using namespace Engine;


// double time_function(duration t)
// {
//     return t.count();
// }

// int main(int argc, char *argv[])
// {
//     Matrix<double, 1, 5> a{0, 0, 0, 0, 0};
//     Matrix<double, 1, 5> b{5, 5, 5, 5, 5};

//     auto gen = sequence(LinearGen(a, b, 5 _ms));
//     while (true)
//     {
//         auto x = gen.next();
//         std::cout << x << std::endl;
//     }
//     return 0;
// }

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
    Engine::GFrame frame(argc, argv);
    Cube base_link(Vector3d{0.05, 0.05, 0.2});
    Cube link0(Vector3d{0.05, 0.2, 0.05}, Vector3d{0, 0, 0}, Vector3d{0, -0.1 + 0.025, 0});
    Cube link1(Vector3d{0.2, 0.05, 0.05}, Vector3d{0, 0, 0}, Vector3d{+0.1, 0, 0});
    Cube link2(Vector3d{0.05, 0.05, 0.05});
    Cube link3(Vector3d{0.025, 0.025, 0.15});

    Camera camera(Vector3d{-0.6, -0.3, 0.2}, _R{300, 0, 640, 0, 300, 512, 0, 0, 1});
    World w(camera);

    Joint j0(base_link, link0, Vector3d{0, 0, 0.1 - 0.025}, 0, new CONTINUOUS_INFO(AXIS_Z));
    Joint j1(link0, link1, Vector3d{0, -0.2 + 0.05, 0.05}, 1, new CONTINUOUS_INFO(AXIS_Z));
    Joint j2(link1, link2, Vector3d{0.2, 0, -0.05}, 2, new CONTINUOUS_INFO(AXIS_Z));
    Joint j3(link2, link3, Vector3d{0, 0, -0.075}, 3, new PRISMATIC_INFO(AXIS_Z));

    w.parse_robot({j0, j1, j2, j3});
    std::vector<Engine::Point2i> projs;
    w.project(projs);

    frame.show();

    w.set_speed(0, M_PI / 4);
    w.set_speed(1, M_PI / 4);
    w.set_speed(2, M_PI / 8);

  
    while (1)
    {
    //    usleep(10 * 1e3);
   
     //   std::cout<<w.Jacobian<4>()<<std::endl;
        std::vector<Engine::Point2i> projs;
        w.project(projs);
        frame.updateData(projs);
    }
    return 0;
}