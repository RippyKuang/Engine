#include <camera.h>

namespace Engine
{
    Camera::Camera(Vector3d center, _R _intrinsics) : intrisics(_intrinsics)
    {
        corners.push_back(Vector4d(center, 1));
        init_pose = getTransformMat(EYE(3), center);
    }

   
    void Camera::project(std::vector<Vector3d> &pw, std::vector<Point2i> &corners, std::vector<bool> &visible, bool keep_all)
    {

        for (int i = 0; i < pw.size(); i++)
        {
            if (!visible[i] && !keep_all)
                continue;
            Vector3d temp = this->intrisics * Vector3d{pw[i][1] / pw[i][0], pw[i][2] / pw[i][0], 1};
            corners.push_back(Point2i(temp));
        }
    }
}
