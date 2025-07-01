#include <world.h>
#include <robot.h>

namespace Engine
{

    void World::emplace(Link &i, int id)
    {
        links.insert(std::pair<int, Link *>(id, &i));
    }


    void World::project_frame(std::vector<Point2i> &projs, std::vector<_T> &t)
    {
        for (auto trans : t)
        {
            _T to_cam = inv(this->pose[-2]) * trans;
            Vector4d origin = to_cam * Vector4d{0, 0, 0, 1};
            Vector4d x = to_cam * Vector4d{0.05, 0, 0, 1};
            Vector4d y = to_cam * Vector4d{0, 0.05, 0, 1};
            Vector4d z = to_cam * Vector4d{0, 0, 0.05, 1};
            std::vector<Vector3d> cube_in_camera = to_3d(std::vector<Vector4d>{origin, x, y, z});
            projs.reserve(cube_in_camera.size());
            cam.project_all(cube_in_camera, projs);
        }
    }

    std::future<std::vector<pixel>> World::project()
    {
        std::vector<Mesh> cubes;
        const Vector4d light_dir = {-2, -2, 5, 1};
        {
            std::lock_guard<std::mutex> lock(m);
            for (auto robot_iter = robots.begin(); robot_iter != robots.end(); robot_iter++)
            {
                std::vector<_T> robot_pose;
                std::vector<Vector6d> robot_v;
                (*robot_iter)->FK(robot_pose, robot_v);
                for (int i = 0; i < (*robot_iter)->bo.size(); i++)
                {
                    Link it = *(*robot_iter)->bo[i];
                    it.transform(inv(pose[-2]) * robot_pose[i]);
                    cam.project(it);
                    cubes.push_back(it.mesh);
                }
            }

            for(auto obj_iter = objs.begin(); obj_iter != objs.end(); obj_iter++)
            {
                Link it = *(*obj_iter);
                it.transform(inv(pose[-2]));
                cam.project(it);
                cubes.push_back(it.mesh);
            }
        }
        Vector4d v = _T{0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1} * inv(pose[-2]) * light_dir;
        return this->raster.parallel_rasterize(cubes, v);
    }
    void World::emplace(Link &i)
    {
        this->objs.emplace_back(&i);
    }
    std::vector<Vector4d> World::getCoord(int id, int base)
    {
        Link it = *links.at(id);
        it.transform(inv(pose[base]));
        return std::move(it.get_corners());
    }

    Robot *World::parse_robot(std::initializer_list<Part> jo)
    {

        int body_num = 0;
        std::vector<Link *> links;
        std::vector<Joint *> joints;
        std::vector<int> vp;
        std::vector<int> vs;
        std::unordered_map<Link *, int> upart_map;
        for (auto iter = jo.begin(); iter != jo.end(); iter++)
        {
            Part base = *iter;
            if (upart_map.find(iter->parent_link) == upart_map.end())
            {
                upart_map.insert(std::pair<Link *, int>(iter->parent_link, body_num));
                this->emplace(*iter->parent_link, body_num);
                links.emplace_back(iter->parent_link);
                body_num++;
            }
            for (auto p = jo.begin(); p != jo.end(); p++)
            {
                if (p->parent_link == iter->parent_link)
                {
                    if (upart_map.find(p->child_link) == upart_map.end())
                    {
                        upart_map.insert(std::pair<Link *, int>(p->child_link, body_num));
                        this->emplace(*p->child_link, body_num);
                        links.emplace_back(p->child_link);
                        joints.emplace_back(new Joint(*p));
                        vp.push_back(upart_map[iter->parent_link]);
                        vs.push_back(upart_map[p->child_link]);
                        body_num++;
                    }
                }
            }
        }
        Robot *rp = new Robot(vp, vs, joints, links);
        this->robots.emplace_back(rp);
        return rp;
    }

}