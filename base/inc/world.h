#include <camera.h>
#include <vector>
#include <map>

namespace Engine
{


    class World
    {
    private:
        std::map<int, Item *> items;
        std::map<int, _T> pose;
        Camera cam;
        std::vector<Vector4d> getCoord(int id, int base = -1);

    public:
        World(Camera &_cam) : cam(_cam)
        {
            pose.insert(std::pair<int, _T>(-1, EYE(4)));
            pose.insert(std::pair<int, _T>(-2, _cam.init_pose));
        }
        void emplace(Cube &item, int id);
        void act(int id, _T t, int base = -1);
        void act(int id, _R t, int base = -1);
        std::vector<Point2i> project();
    };
}