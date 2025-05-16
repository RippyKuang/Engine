#include <camera.h>

namespace Engine
{
    Camera::Camera(Vector3d center, _R _intrinsics) : intrisics(_intrinsics)
    {
        center = Vector4d(center, 1);
        init_pose = getTransformMat(EYE(3), center);
    }


    void Camera::project(Link &lk)
    {

        _T view_pad = catRow(catCol(this->intrisics, Vector3d()), Vector4d{0, 0, 0, 1}.T());
        lk.transform(view_pad * _T{0, 1, 0, 0,
                                   0, 0, 1, 0,
                                   1, 0, 0, 0,
                                   0, 0, 0, 1});
    }

    void Camera::project_all(std::vector<Vector3d> &pw, std::vector<Point2i> &corners)
    {

        for (int i = 0; i < pw.size(); i++)
        {
            Vector3d temp = this->intrisics * Vector3d{pw[i][1] / pw[i][0], pw[i][2] / pw[i][0], 1};
            corners.emplace_back(int(temp[0]), int(temp[1]));
        }
    }
}
