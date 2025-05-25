#pragma once
#include <geometry.h>
#include <fstream>

namespace Engine
{
    using Index3i = Matrix<int, 3, 1>;

    inline std::vector<std::string> stringSplit(const std::string &strIn, char delim)
    {
        char *str = const_cast<char *>(strIn.c_str());
        std::string s;
        s.append(1, delim);
        std::vector<std::string> elems;
        char *splitted = strtok(str, s.c_str());
        while (splitted != NULL)
        {
            elems.push_back(std::string(splitted));
            splitted = strtok(NULL, s.c_str());
        }
        return elems;
    }

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
        template <typename T, typename = typename std::enable_if<std::is_base_of<Mesh, T>::value>::type>
        Mesh(T &&t) : nv(t.nv), nt(t.nt)
        {
            this->vertices = std::move(t.vertices);
            this->tInd = std::move(t.tInd);
        }
        void _transform(_T t)
        {
            for (auto it = this->vertices.begin(); it != this->vertices.end(); it++)
                *it = t * (*it);
        }

        static Mesh *load_from_file(const std::string &filename)
        {
            std::ifstream file;
            std::vector<Vector4d> vertices;
            std::vector<Index3i> tInd;
            file.open(filename, std::ios::in);
            if (file.is_open())
            {

                std::string strLine;
                while (getline(file, strLine))
                {

                    if (strLine.empty())
                        continue;
                    std::vector<std::string> splited = stringSplit(strLine, ' ');
                    if (splited[0] == "v")
                    {

                        double x = std::stod(splited[1]);
                        double y = std::stod(splited[2]);
                        double z = std::stod(splited[3]);
                        vertices.emplace_back(x, y, z, 1);
                    }
                    else if (splited[0] == "f")
                    {
                        Index3i face;
                        for (int i = 1; i < 4; ++i)
                        {
                            int index = std::stoi(stringSplit(splited[i], '/')[0]) - 1;
                            face[i - 1] = index;
                        }
                        tInd.emplace_back(face);
                    }
                    else
                    {
                        continue;
                    }
                }
                file.close();
                Mesh *mesh = new Mesh(vertices.size(), tInd.size());
                mesh->vertices = std::move(vertices);
                mesh->tInd = std::move(tInd);
                return mesh;
            }
            else
            {
                std::cerr << "Error opening file: " << filename << std::endl;
                file.close();
                return nullptr;
            }
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
