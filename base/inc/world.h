#pragma once
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
        std::map<int, Link *> links;
        int num_joints;
        Timer timer;
        Camera cam;
        std::vector<Vector4d> getCoord(int id, int base = -1);
        void emplace(Link &, int);
        std::vector<Robot *> robots;
        std::vector<Link *> objs;
        Rasterizer raster;



    public:

        World(Camera &_cam, const int w, const int h) : cam(_cam), raster(w, h)
        {
            pose.insert(std::pair<int, _T>(-1, EYE(4)));
            pose.insert(std::pair<int, _T>(-2, _cam.init_pose));
        }

        std::function<void(_T)> get_camera_handle()
        {
            return [this](_T t)
            {
                std::lock_guard<std::mutex> lock(m);
                this->pose[-2] = t * this->pose[-2];
            };
        }
        void emplace(Link&);
        Robot *parse_robot(std::initializer_list<Part>);
        void project_frame(std::vector<Point2i> &, std::vector<_T> &);
        const std::vector<Link *> getAllLinks() const;
        std::future<std::vector<pixel>> project();
    };

}