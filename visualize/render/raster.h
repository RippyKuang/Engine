#include <mesh.h>
#include <timer.h>
#include <float.h>

namespace Engine
{
    using d2 = double[2];
    using d3 = double[3];


    class Rasterizer
    {
    private:
        std::vector<float *> z_buffer;
        ThreadPool pool{8};
        std::mutex m;
        std::stack<int> free_ids;
        int alloc_z_buffer()
        {
            std::lock_guard<std::mutex> lock(m);
            if(free_ids.size() > 0)
            {
                int id = free_ids.top();
                free_ids.pop();
                return id;
            }
            else
            {
                float *z = (float *)malloc(w * h * sizeof(float));
                z_buffer.push_back(z);
                std::cout <<"z_buffer size: "<<z_buffer.size() << std::endl;
                return z_buffer.size() - 1;
            }
        }
        void release_z_buffer(int id)
        {
            pool.enqueue([this, id]()
            {
                std::fill(this->z_buffer[id], this->z_buffer[id] + w * h, std::numeric_limits<float>::infinity());
                std::lock_guard<std::mutex> lock(this->m);
                free_ids.push(id);
            });
          
        }

    public:
        const int w;
        const int h;
        Rasterizer(const int w, const int h) : w(w), h(h)
        {
          
        }

        inline double f(double x, double y, d2 &p0, d2 &p1)
        {
            return (p0[1] - p1[1]) * x + (p1[0] - p0[0]) * y + p0[0] * p1[1] - p1[0] * p0[1];
        }

        std::vector<pixel> _rasterize(std::vector<Mesh> &meshes, Vector3d light_dir);
      
        Vector3d Blinn_Phong(Vector3d& light_dir, d3& intersect,d3& eye_dir, d3 &N);

        std::future<std::vector<pixel>> parallel_rasterize(std::vector<Mesh> &meshes, Vector3d light_dir)
        {
            return this->pool.enqueue([this](std::vector<Mesh> meshes, Vector3d light_dir)
                                      { return this->_rasterize(meshes, light_dir); }, std::move(meshes), light_dir);
        }

        ~Rasterizer()
        {
            for (auto it = this->z_buffer.begin(); it != this->z_buffer.end(); it++)
            {
                free(*it);
            }
        }
    };
}