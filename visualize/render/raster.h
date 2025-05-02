#include <mesh.h>
#include <float.h>
namespace Engine
{

    /*
    输入是一个vector<mesh>，mesh里的点是u,v,z 经过光栅化和z_buffer ,返回vector<pixel>
    */

    class Rasterizer
    {
    private:
        double *z_buffer;

    public:
        const int w;
        const int h;
        Rasterizer(const int w, const int h) : w(w), h(h)
        {
            this->z_buffer = (double *)malloc(w * h * sizeof(double));
            std::fill(this->z_buffer, this->z_buffer + w * h, 9999.0);
        }

        inline double f(double x, double y, Vector3d &p0, Vector3d &p1)
        {
            return (p0[1] - p1[1]) * x + (p1[0] - p0[0]) * y + p0[0] * p1[1] - p1[0] * p0[1];
        }


        void rasterize(std::vector<Mesh> &meshes, std::vector<pixel> &pixels)
        {
            std::fill(this->z_buffer, this->z_buffer + w * h, 9999.0);
            for (auto iter = meshes.begin(); iter != meshes.end(); iter++)
            {

                for (int i = 0; i < iter->nt; i++)
                {
                    auto t = iter->tInd[i];
                    Vector3d p0 =  iter->vertices[t[0]];
                    Vector3d p1 =  iter->vertices[t[1]];
                    Vector3d p2 =  iter->vertices[t[2]];
                    p0 = {p0[0]/p0[2], p0[1]/p0[2], p0[2]};
                    p1 = {p1[0]/p1[2], p1[1]/p1[2], p1[2]};
                    p2 = {p2[0]/p2[2], p2[1]/p2[2], p2[2]};

                    int min_x = std::min({p0[0], p1[0], p2[0]});
                    int max_x = std::max({p0[0], p1[0], p2[0]});
                    int min_y = std::min({p0[1], p1[1], p2[1]});
                    int max_y = std::max({p0[1], p1[1], p2[1]});

                    for (int x = min_x; x <= max_x; x++)
                    {
                        for (int y = min_y; y <= max_y; y++)
                        {

                            double aplha = f(x, y, p0, p1) / f(p0[0], p0[1], p1, p2);
                            double beta = f(x, y, p1, p2) / f(p1[0], p1[1], p2, p0);
                            double gamma = f(x, y, p2, p0) / f(p2[0], p2[1], p0, p1);
                            if (aplha >= 0 && beta >= 0 && gamma >= 0)
                            {
                                double z = (p0[2] * aplha + p1[2] * beta + p2[2] * gamma);

                                if (z < this->z_buffer[x + y * w])
                                {
                                    this->z_buffer[x + y * w] = z;
                                    pixels.push_back({Vector3d{z, z, z}, Point2i{x, y}});
                                }
                            }
                        }
                    }
                }
            }
        }
        ~Rasterizer()
        {
            free(this->z_buffer);
        }
    };
}