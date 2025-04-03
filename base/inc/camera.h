#include <items.h>

namespace Engine
{
    class Camera : public Item
    {
    private:
        _R intrisics;

    public:
        Camera(Vector3d center, _R _intrinsics);
        std::vector<Point2i> project(Item &pw);
    };
}