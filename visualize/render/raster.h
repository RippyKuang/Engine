#include <mesh.h>
#include <float.h>

namespace Engine
{

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

        void rasterize(std::vector<Mesh> &meshes, std::vector<pixel> &pixels, Vector3d light_dir);
        Vector3d shading(Vector3d dir,Vector3d& p0,Vector3d& p1,Vector3d& p2);
        Vector3d Blinn_Phong(Vector3d light_dir,Vector3d eye_dir,Vector3d& p0,Vector3d& p1,Vector3d& p2);
        
        ~Rasterizer()
        {
            free(this->z_buffer);
        }
    };
}