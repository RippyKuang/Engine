#pragma once
#include <geometry.h>

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
            vertices.reserve(nv);
            tInd.reserve(nt);
        }
        Mesh(const Mesh &t) : nv(t.nv), nt(t.nt)
        {
            this->vertices = t.vertices;
            this->tInd = t.tInd;
        }
        template <typename T,typename = typename std::enable_if<std::is_base_of<Mesh,T>::value>::type>
        Mesh(T&& t) : nv(t.nv), nt(t.nt)
        {
            this->vertices = std::move(t.vertices);
            this->tInd= std::move(t.tInd);
        }
        void _transform(_T t)
        {
            for (auto it = this->vertices.begin(); it != this->vertices.end(); it++)
                *it = t * (*it);
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

            tInd.emplace_back(4, 2, 6);
            tInd.emplace_back(4, 0, 2);
            tInd.emplace_back(4, 1, 0);
            tInd.emplace_back(4, 5, 1);
            tInd.emplace_back(4, 7, 5);
            tInd.emplace_back(4, 6, 7);
            tInd.emplace_back(3, 1, 5);
            tInd.emplace_back(3, 5, 7);
            tInd.emplace_back(3, 7, 6);
            tInd.emplace_back(3, 6, 2);
            tInd.emplace_back(3, 2, 0);
            tInd.emplace_back(3, 0, 1);
        }
    };
}
