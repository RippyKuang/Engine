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
        void act(int id, _T t, int base = -1);
        void act(int id, _R t, int base = -1);
        void discrete(std::vector<Vector3d> &,std::vector<Vector3d>&, std::vector<Point2i> &, std::vector<bool> &);

    public:
        std::map<int, Link *> links;
        World(Camera &_cam) : cam(_cam)
        {
            pose.insert(std::pair<int, _T>(-1, EYE(4)));
            pose.insert(std::pair<int, _T>(-2, _cam.init_pose));
        }
        void emplace(Cube &, int);

        void parse_robot(std::initializer_list<Joint>);
        double drive(int id, double inc);
        void set_speed(int id, double speed);
        void set_acc(int id, double acc);
        _T  get_pose(int id);
        void project_frame(std::vector<Point2i>&, _T&);
        void project(std::vector<Point2i>&);
        void inverse_dynamics(std::vector<Twist>&, std::vector<Twist>&);

        template <int Index, int Num, typename Enable = void>
        struct BuildJacobian;

        template <int Index, int Num>
        struct BuildJacobian<Index, Num, typename std::enable_if<Index != 0, void>::type>
        {
            Matrix<double, 6, Index + 1> operator()(std::vector<Matrix<double, 6, 1>>& v)
            {
                auto j = BuildJacobian<Index - 1, Num>()(v);
                return catCol(j, v[Index]);
            }
        };

        template <int Index, int Num>
        struct BuildJacobian<Index, Num, typename std::enable_if<Index == 0, void>::type>
        {

            auto operator()(std::vector<Matrix<double, 6, 1>>& v)->decltype(v[0])
            {
                return v[0];
            }
        };

        template <int Num>
        Matrix<double, 6, Num> Jacobian()
        {
            std::lock_guard<std::mutex> lock(m);
            std::vector<Matrix<double, 6, 1>> v;
            this->graph.Jacobian(v);

            return BuildJacobian<Num - 1, Num>()(v);
        }
    };

}