#include <camera.h>
#include <vector>
#include <robot.h>
#include <timer.h>
#include <map>
#include <unordered_map>
#include <functional>
#include <type_traits>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

#define PRE_ALLOC 450
namespace Engine
{
    class World
    {

    private:
        std::mutex m;

        std::map<int, _T> pose;
        int num_joints;
        Timer timer;
        Joint_node graph;
        Camera cam;
        std::vector<Vector4d> getCoord(int id, int base = -1);
        double drive(int id);
        void act(int id, _T t);
        void discrete(std::vector<Vector3d> &, std::vector<Vector3d> &, std::vector<Point2i> &, std::vector<bool> &);

        std::vector<Point2i> tprojs;
        std::vector<Point2i> pseudo_tprojs;
        std::vector<Vector3d> discreted_pw;
    public:
        std::map<int, Link *> links;
        World(Camera &_cam) : cam(_cam)
        {
            pose.insert(std::pair<int, _T>(-1, EYE(4)));
            pose.insert(std::pair<int, _T>(-2, _cam.init_pose));
            tprojs.reserve(PRE_ALLOC);
            discreted_pw.reserve(PRE_ALLOC);
            pseudo_tprojs.reserve(15);
        }
        void emplace(Cube &, int);
        void parse_robot(std::initializer_list<Joint>);
        double drive(int id, double inc);
        void set_speed(int id, double speed);
        void set_acc(int id, double acc);

        std::vector<_T> get_pose(std::initializer_list<int> ids);
        void project_frame(std::vector<Point2i> &, std::vector<_T> &);
        void project(std::vector<Point2i> &);
        void inverse_dynamics(std::vector<Twist> &, std::vector<Twist> &);

    
        std::vector<Twist> Jacobian()
        {
            std::lock_guard<std::mutex> lock(m);
            std::vector<Twist> v;
            this->graph.Jacobian(v);

            return std::move(v);
        }
    };

}