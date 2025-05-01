#include <matrix.h>

namespace Engine
{

    template <int nv, int nt>
    struct Mesh
    {
        std::vector<Vector4d> vertices;
        int tInd[nt][3];
        Mesh()
        {
            vertices.resize(nv);
        }
    };

    struct cube_mesh : public Mesh<8, 12>
    {
        static const int nv = 8;
        static const int nt = 12;

        cube_mesh(Vector3d &box) : Mesh<nv, nt>()
        {
            double x = box[0];
            double y = box[1];
            double z = box[2];

            vertices.push_back(Vector4d{+x / 2, -y / 2, +z / 2, 1});
            vertices.push_back(Vector4d{+x / 2, -y / 2, -z / 2, 1});
            vertices.push_back(Vector4d{+x / 2, +y / 2, +z / 2, 1});
            vertices.push_back(Vector4d{+x / 2, +y / 2, -z / 2, 1});

            vertices.push_back(Vector4d{-x / 2, -y / 2, +z / 2, 1});
            vertices.push_back(Vector4d{-x / 2, -y / 2, -z / 2, 1});
            vertices.push_back(Vector4d{-x / 2, +y / 2, +z / 2, 1});
            vertices.push_back(Vector4d{-x / 2, +y / 2, -z / 2, 1});
#define ASSIGN(v, i, j, k) \
    tInd[v][0] = i;        \
    tInd[v][1] = j;        \
    tInd[v][2] = k;
            ASSIGN(0, 4, 6, 2);
            ASSIGN(1, 4, 2, 0);
            ASSIGN(2, 4, 0, 2);
            ASSIGN(3, 4, 2, 5);
            ASSIGN(4, 4, 5, 7);
            ASSIGN(5, 4, 7, 6);
            ASSIGN(6, 3, 5, 1);
            ASSIGN(7, 3, 7, 5);
            ASSIGN(8, 3, 6, 7);
            ASSIGN(9, 3, 2, 6);
            ASSIGN(10, 3, 0, 2);
            ASSIGN(11, 3, 1, 0);
#undef ASSIGN
        }
    };
}
