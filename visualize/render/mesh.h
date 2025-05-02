#include <matrix.h>

namespace Engine
{
    using Index3i = Matrix<int, 3, 1>;

    struct Mesh
    {
        const int nv;
        const int nt;
        std::vector<Vector4d> vertices;
        std::vector<Index3i> tInd;
        Mesh(int nv, int nt) : nv(nv), nt(nt)
        {
            vertices.resize(nv);
            tInd.resize(nt);
        }
    };

    struct cube_mesh : public Mesh
    {
        static const int nv = 8;
        static const int nt = 12;

        cube_mesh(Vector3d &box) : Mesh(nv, nt)
        {
            double x = box[0];
            double y = box[1];
            double z = box[2];

            vertices.emplace_back(+x / 2, -y / 2, +z / 2, 1);
            vertices.emplace_back(+x / 2, -y / 2, -z / 2, 1);
            vertices.emplace_back(+x / 2, +y / 2, +z / 2, 1);
            vertices.emplace_back(+x / 2, +y / 2, -z / 2, 1);

            vertices.emplace_back(-x / 2, -y / 2, +z / 2, 1);
            vertices.emplace_back(-x / 2, -y / 2, -z / 2, 1);
            vertices.emplace_back(-x / 2, +y / 2, +z / 2, 1);
            vertices.emplace_back(-x / 2, +y / 2, -z / 2, 1);

            tInd.emplace_back(4, 6, 2);
            tInd.emplace_back(4, 2, 0);
            tInd.emplace_back(4, 0, 2);
            tInd.emplace_back(4, 2, 5);
            tInd.emplace_back(4, 5, 7);
            tInd.emplace_back(4, 7, 6);
            tInd.emplace_back(3, 5, 1);
            tInd.emplace_back(3, 7, 5);
            tInd.emplace_back(3, 6, 7);
            tInd.emplace_back(3, 2, 6);
            tInd.emplace_back(3, 0, 2);
            tInd.emplace_back(3, 1, 0);
        }
    };
}
