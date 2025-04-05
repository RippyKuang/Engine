#include <camera.h>
#include <vector>
#include <robot.h>
#include <map>
#include <unordered_map>
namespace Engine
{
    class World
    {
        
    private:
        std::map<int, Link *> links;
        std::map<int, _T> pose;
        std::map<int, Joint *> joints;
        Joint_node graph;
        Camera cam;
        std::vector<Vector4d> getCoord(int id, int base = -1);
        std::vector<Vector3d> discrete(std::vector<Vector3d> &, std::vector<Point2i> &, std::vector<bool> &);

    public:
        World(Camera &_cam) : cam(_cam)
        {
            pose.insert(std::pair<int, _T>(-1, EYE(4)));
            pose.insert(std::pair<int, _T>(-2, _cam.init_pose));
        }
        void emplace(Cube &item, int id);
        void emplace(Joint &i);
        void act(int id, _T t, int base = -1);
        void act(int id, _R t, int base = -1);
        void parse_robot(std::initializer_list<Joint>);
        std::vector<Point2i> project();
    };

}