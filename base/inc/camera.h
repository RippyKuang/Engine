#include <items.h>

namespace Engine
{
    class Camera : public Link
    {
    private:
        _R intrisics;

    public:
        Camera(Vector3d center, _R _intrinsics);
     
        void project(std::vector<Vector3d> &, std::vector<Point2i> &, std::vector<bool> &, bool keep_all = false);
        void project_all(std::vector<Vector3d> &, std::vector<Point2i> &);
    };
}