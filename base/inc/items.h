#include <geometry.h>
#include <vector>

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
        Cube(Vector3d box, Vector3d xyz = Vector3d{0, 0, 0}, Vector3d rpy = Vector3d{0, 0, 0});
    };
}