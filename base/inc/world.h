#include <camera.h>
#include <vector>
#include <robot.h>
#include <unordered_map>
#include <functional>
#include <type_traits>
#include <raster.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace Engine
{
    class World
    {

    private:
        std::mutex m;
        std::map<int, _T> pose;
        int num_joints;
        Timer timer;
        Camera cam;
        std::vector<Vector4d> getCoord(int id, int base = -1);
        void emplace(Cube &, int);
        std::vector<Robot *> robots;
        void act(int id, _T t);
        Rasterizer raster;

    public:
        std::map<int, Link *> links;
        World(Camera &_cam, const int w, const int h) : cam(_cam), raster(w, h)
        {
            pose.insert(std::pair<int, _T>(-1, EYE(4)));
            pose.insert(std::pair<int, _T>(-2, _cam.init_pose));
        }
       
        const Robot* parse_robot(std::initializer_list<Part>);
        void project_frame(std::vector<Point2i> &, std::vector<_T> &);
        std::future<std::vector<pixel>> project();
    };

}