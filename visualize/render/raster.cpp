#include <raster.h>

namespace Engine
{

    inline void intersectLinePlane(d3 &P, d3 &A, d3 &ret, d3 &N)
    {

        double denom = N[0] * P[0] + N[1] * P[1] + N[2] * P[2];
        double t = (N[0] * A[0] + N[1] * A[1] + N[2] * A[2]) / denom;
        ret[0] = P[0] * t;
        ret[1] = P[1] * t;
        ret[2] = P[2] * t;
    }

    std::vector<pixel> Rasterizer::_rasterize(std::vector<Mesh> &meshes, Vector3d light_dir)
    {
        std::vector<pixel> pixels;
        int buffer_id = this->alloc_z_buffer();

        for (auto iter = meshes.begin(); iter != meshes.end(); iter++)
        {

            for (int i = 0; i < iter->nt; i++)
            {

                auto t = iter->tInd[i];
                Vector4d &v1 = iter->vertices[t[0]];
                Vector4d &v2 = iter->vertices[t[1]];
                Vector4d &v3 = iter->vertices[t[2]];
                d3 _p0 = {v1[0], v1[1], v1[2]};
                d3 _p1 = {v2[0], v2[1], v2[2]};
                d3 _p2 = {v3[0], v3[1], v3[2]};

                d2 p0 = {_p0[0] / _p0[2], _p0[1] / _p0[2]};
                d2 p1 = {_p1[0] / _p1[2], _p1[1] / _p1[2]};
                d2 p2 = {_p2[0] / _p2[2], _p2[1] / _p2[2]};

                d3 __p0 = {(_p0[0] - _p0[2] * 640.0) / 500.0, (_p0[1] - _p0[2] * 512.0) / 500.0, _p0[2]};
                d3 __p1 = {(_p1[0] - _p1[2] * 640.0) / 500.0, (_p1[1] - _p1[2] * 512.0) / 500.0, _p1[2]};
                d3 __p2 = {(_p2[0] - _p2[2] * 640.0) / 500.0, (_p2[1] - _p2[2] * 512.0) / 500.0, _p2[2]};

                double x11 = __p1[0] - __p0[0];
                double x12 = __p1[1] - __p0[1];
                double x13 = __p1[2] - __p0[2];
                double x21 = __p2[0] - __p0[0];
                double x22 = __p2[1] - __p0[1];
                double x23 = __p2[2] - __p0[2];

                d3 N = {x12 * x23 - x13 * x22,
                        x13 * x21 - x11 * x23,
                        x11 * x22 - x12 * x21};

                float min_xf = p0[0], max_xf = p0[0];
                float min_yf = p0[1], max_yf = p0[1];

                if (p1[0] < min_xf)
                    min_xf = p1[0];
                else if (p1[0] > max_xf)
                    max_xf = p1[0];
                if (p2[0] < min_xf)
                    min_xf = p2[0];
                else if (p2[0] > max_xf)
                    max_xf = p2[0];

                if (p1[1] < min_yf)
                    min_yf = p1[1];
                else if (p1[1] > max_yf)
                    max_yf = p1[1];
                if (p2[1] < min_yf)
                    min_yf = p2[1];
                else if (p2[1] > max_yf)
                    max_yf = p2[1];

                int min_x = static_cast<int>(std::floor(min_xf));
                int max_x = static_cast<int>(std::ceil(max_xf));
                int min_y = static_cast<int>(std::floor(min_yf));
                int max_y = static_cast<int>(std::ceil(max_yf));

                double _a = f(p0[0], p0[1], p1, p2);
                double _b = f(p1[0], p1[1], p2, p0);
                double _c = f(p2[0], p2[1], p0, p1);

                double alpha = f(min_x, min_y, p1, p2) / _a;
                double beta = f(min_x, min_y, p2, p0) / _b;
                double gamma = f(min_x, min_y, p0, p1) / _c;
                double delta_xa = (p1[1] - p2[1]) / _a;
                double delta_ya = (p2[0] - p1[0]) / _a;
                double delta_xb = (p2[1] - p0[1]) / _b;
                double delta_yb = (p0[0] - p2[0]) / _b;
                double delta_xc = (p0[1] - p1[1]) / _c;
                double delta_yc = (p1[0] - p0[0]) / _c;

                for (int x = min_x; x <= max_x; x++)
                {
                    int x_ = x - min_x;
                    double alpha_row = alpha + x_ * delta_xa;
                    double beta_row = beta + x_ * delta_xb;
                    double gamma_row = gamma + x_ * delta_xc;

                    for (int y = min_y; y <= max_y; y++)
                    {
                        int y_ = y - min_y;
                        double alpha_xy = alpha_row + y_ * delta_ya;
                        double beta_xy = beta_row + y_ * delta_yb;
                        double gamma_xy = gamma_row + y_ * delta_yc;

                        if (alpha_xy >= -1e-6 && beta_xy >= -1e-6 && gamma_xy >= -1e-6)
                        {
                            double z = (_p0[2] * alpha_xy + _p1[2] * beta_xy + _p2[2] * gamma_xy);

                            float &z_val = this->z_buffer[buffer_id][x + y * w];
                            if (z < z_val - 1e-5)
                            {
                                z_val = z;
                                d3 ray = {(x - 640.0) / 500, (y - 512.0) / 500, 1};
                                d3 ret;
                                intersectLinePlane(ray, __p0, ret, N);
                                pixels.emplace_back(pixel{Blinn_Phong(light_dir, ret, ray, N), Point2i{x, y}});
                            }
                        }
                    }
                }
            }
        }
        this->release_z_buffer(buffer_id);
        return std::move(pixels);
    }

    inline Vector3d Rasterizer::Blinn_Phong(Vector3d &light_dir, d3 &intersect, d3 &eye_dir, d3 &N)
    {
        d3 h = {light_dir[0] - intersect[0] + eye_dir[0], light_dir[1] - intersect[1] + eye_dir[1], light_dir[2] - intersect[2] + eye_dir[2]};
        double norm_h = std::sqrt(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]);
        double norm_N = std::sqrt(N[0] * N[0] + N[1] * N[1] + N[2] * N[2]);
        double norm_L = std::sqrt(light_dir[0] * light_dir[0] + light_dir[1] * light_dir[1] + light_dir[2] * light_dir[2]);
        double nh = std::max(std::pow((N[0] * h[0] + N[1] * h[1] + N[2] * h[2]) / (norm_h * norm_N), 10), 0.0);
        double nl = std::max((N[0] * light_dir[0] + N[1] * light_dir[1] + N[2] * light_dir[2]) / (norm_N * norm_L), 0.0);
        double intensity = std::min(0.7 * nl + 0.3 * nh, 1.0);

        return Vector3d{intensity, intensity, intensity};
    }

}
