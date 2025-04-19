#include <camera.h>
#include <vector>
#include <robot.h>
#include <timer.h>
#include <map>
#include <unordered_map>
#include <functional>

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
        Joint_node graph;
        Camera cam;
        std::vector<Vector4d> getCoord(int id, int base = -1);
        std::vector<Vector3d> discrete(std::vector<Vector3d> &, std::vector<Point2i> &, std::vector<bool> &);

    public:
        std::map<int, Link *> links;
        World(Camera &_cam) : cam(_cam)
        {
            pose.insert(std::pair<int, _T>(-1, EYE(4)));
            pose.insert(std::pair<int, _T>(-2, _cam.init_pose));
        }
        void emplace(Cube &, int);
        void act(int id, _T t, int base = -1);
        void act(int id, _R t, int base = -1);
        void parse_robot(std::initializer_list<Joint>);
        double drive(int id);
        double drive(int id,double inc);
        void set_speed(int id, double speed);
        std::vector<Point2i> project();

        template <int num>
        Matrix<double, 6, num> Jacobian()
        {
            std::lock_guard<std::mutex> lock(m);
            std::vector<Matrix<double, 6, 1>> v;
            Matrix<double, 6, 0> jac;
            this->graph.Jacobian(v);
            assert(num == v.size());
            for (int i = 0; i < v.size(); i++)
                jac = catCol(jac, v[i]);
            return jac;
        }
    };

}