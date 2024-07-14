#include <geometry.h>
#include <vector>
#include <map>
namespace Engine
{
    class Item
    {
        friend class World;

    protected:
        _T init_pose;
        std::vector<Vector4d> corners;
        void transform(_T);

    public:
        Item(std::vector<Vector4d> _corners) : corners(_corners){};
        Item(std::vector<Vector3d> _corners);
        Item(){};

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

        // std::vector<Point2i> draw();
    };
    class Camera : public Item
    {
    public:
        Camera(Vector3d center);
    };

    class World
    {
    private:
        std::map<int, Item> items;
        std::map<int, _T> pose;

    public:
        World()
        {
            pose.insert(std::pair<int, _T>(-1, EYE(4)));
        }
        void emplace(Item &item, int id);
        void print(int id);
        void act(int id, _T t);
        void act(int id, _R t);
        Item getCoord(int id, int base=-1);
    };
}