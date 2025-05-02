#include <raster.h>

namespace Engine
{

    void Rasterizer::rasterize(std::vector<Mesh> &meshes, std::vector<pixel> &pixels, Vector3d light_dir)
    {

        std::fill(this->z_buffer, this->z_buffer + w * h, 9999.0);
        for (auto iter = meshes.begin(); iter != meshes.end(); iter++)
        {

            for (int i = 0; i < iter->nt; i++)
            {
                auto t = iter->tInd[i];
                Vector3d _p0 = iter->vertices[t[0]];
                Vector3d _p1 = iter->vertices[t[1]];
                Vector3d _p2 = iter->vertices[t[2]];
                Vector3d p0 = {_p0[0] / _p0[2], _p0[1] / _p0[2], _p0[2]};
                Vector3d p1 = {_p1[0] / _p1[2], _p1[1] / _p1[2], _p1[2]};
                Vector3d p2 = {_p2[0] / _p2[2], _p2[1] / _p2[2], _p2[2]};

                int min_x = floor(std::min({p0[0], p1[0], p2[0]}));
                int max_x = ceil(std::max({p0[0], p1[0], p2[0]}));
                int min_y = floor(std::min({p0[1], p1[1], p2[1]}));
                int max_y = ceil(std::max({p0[1], p1[1], p2[1]}));

                for (int x = min_x; x <= max_x; x++)
                {
                    for (int y = min_y; y <= max_y; y++)
                    {

                        double aplha = f(x, y, p1, p2) / f(p0[0], p0[1], p1, p2);
                        double beta = f(x, y, p2, p0) / f(p1[0], p1[1], p2, p0);
                        double gamma = f(x, y, p0, p1) / f(p2[0], p2[1], p0, p1);
                        if (aplha >= 0 && beta >= 0 && gamma >= 0)
                        {
                            double z = (p0[2] * aplha + p1[2] * beta + p2[2] * gamma);

                            if (z < this->z_buffer[x + y * w])
                            {
                                this->z_buffer[x + y * w] = z;
                                Vector3d ray = Vector3d((x - 640.0) / 500, (y - 512.0) / 500, 1);
                                Vector3d __p0 = {(_p0[0] - _p0[2] * 640.0) / 500.0, (_p0[1] - _p0[2] * 512.0) / 500.0, p0[2]};
                                Vector3d __p1 = {(_p1[0] - _p1[2] * 640.0) / 500.0, (_p1[1] - _p1[2] * 512.0) / 500.0, p1[2]};
                                Vector3d __p2 = {(_p2[0] - _p2[2] * 640.0) / 500.0, (_p2[1] - _p2[2] * 512.0) / 500.0, p2[2]};
                                Vector3d intersect = intersectLinePlane(ray, __p0, __p1, __p2);
                                pixels.push_back({Blinn_Phong(light_dir - intersect, ray, __p0, __p1, __p2), Point2i{x, y}});
                            }
                        }
                    }
                }
            }
        }
    }
    Vector3d Rasterizer::Blinn_Phong(Vector3d light_dir, Vector3d eye_dir, Vector3d &p0, Vector3d &p1, Vector3d &p2)
    {
        Vector3d h = light_dir + eye_dir;
        Vector3d normal = cross(p2 - p1, p0 - p1);

        double nh = std::max(std::pow(dot(norm(normal), norm(h)), 10), 0.0);
        double nl = std::max(dot(normal, norm(light_dir)), 0.0);
        double intensity = std::min(0.7 * nl + 0.2 * nh, 1.0);

        return Vector3d{intensity, intensity, intensity};
    }

    Vector3d Rasterizer::shading(Vector3d dir, Vector3d &p0, Vector3d &p1, Vector3d &p2)
    {

        Vector3d normal = cross(p1 - p0, p2 - p1);
        double intensity = dot(norm(normal), norm(dir));
        if (intensity < 0)
            intensity = 0;
        return Vector3d{intensity, intensity, intensity};
    }
}
