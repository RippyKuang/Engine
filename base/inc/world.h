#include <geometry.h>
#include <vector>
#include <map>
namespace Engine
{
    class Item
    {
        friend class World;
        friend class Camera;

    protected:
        _T init_pose;
        void transform(_T);
        std::vector<Vector4d> corners;

    public:
        Item(std::vector<Vector4d> _corners) : corners(_corners) {};
        Item(std::vector<Vector3d> _corners);
        Item() {};

        friend std::ostream &operator<<(std::ostream &output,
                                        const Item &item)
        {
            for (auto corner : item.corners)
                output << corner.T();
            return output;
        }
    };

    class Cube : public Item
    {
    public:
        Cube(std::vector<Vector4d> _corners) : Item(_corners)
        {
            assert(_corners.size() == 8);
        }
        Cube(std::vector<Vector3d> _corners) : Item(_corners)
        {
            assert(_corners.size() == 8);
        }
        Cube(Vector3d center, double x, double y, double z);
    };
    class Camera : public Item
    {
    private:
        _R intrisics;

    public:
        Camera(Vector3d center, _R _intrinsics);
        std::vector<Point2i> project(const Item &pw);
    };

    class World
    {
    private:
        std::map<int, Item *> items;
        std::map<int, Camera *> cameras;
        std::map<int, _T> pose;

    public:
        World()
        {
            pose.insert(std::pair<int, _T>(-1, EYE(4)));
        }
        void emplace(Cube &item, int id);
        void emplace(Camera &item, int id);
        Item *get(int id);
        Camera &getCamHandle(int id);
        void act(int id, _T t, int base = -1);
        void act(int id, _R t, int base = -1);
        std::vector<Vector4d> getCoord(int id, int base = -1);
    };
}