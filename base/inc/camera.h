#include <items.h>

namespace Engine
{
    class Camera : public Link
    {
    private:
        _R intrisics;

    public:
        Camera(Vector3d center, _R _intrinsics);
        std::vector<Point2i> project(std::vector<Vector3d> &, std::vector<bool> &, bool keep_all = false);
    };
}