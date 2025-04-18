#include <world.h>

namespace Engine
{

    void World::emplace(Cube &i, int id)
    {
        Link *j = (Link *)malloc(sizeof(Cube));
        memcpy(j, &i, sizeof(Cube));
        links.insert(std::pair<int, Link *>(id, j));
        pose.insert(std::pair<int, _T>(id, i.init_pose));
    }

    void World::act(int id, _T t, int base)
    {
        std::lock_guard<std::mutex> lock(m);
        if (id != base)
        {
            pose[id] = (pose[base] * t * inv(pose[base])) * pose[id];
            (*links.at(id)).transform((pose[base] * t * inv(pose[base])));
        }
        else
        {
            (*links.at(id)).transform((pose[base] * t * inv(pose[base])));
            pose[id] = pose[base] * t;
        }
    }
    void World::act(int id, _R r, int base)
    {
        std::lock_guard<std::mutex> lock(m);
        auto t = catRow(catCol(r, Vector3d()), catCol(Vector3d().T(), EYE(1)));
        if (id != base)
        {
            pose[id] = (pose[base] * t * inv(pose[base])) * pose[id];
            (*links.at(id)).transform((pose[base] * t * inv(pose[base])));
        }
        else
        {
            (*links.at(id)).transform((pose[base] * t * inv(pose[base])));
            pose[id] = pose[base] * t;
        }
    }
    double World::drive(int id, double inc)
    {

        Joint_node *tgt = graph.find(id);
        INFO *info = tgt->get_info();

        std::function<void(int, _T)> act_func = std::bind((void (World::*)(int, _T, int))&World::act, this, _1, _2, -1);

        if (info->type == FIXED)
            throw "Driving a fixed joint!";
        if (info->type == REVOLUTE)
        {
            REVOLUTE_INFO *pinfo = static_cast<REVOLUTE_INFO *>(info);
            if ((pinfo->pos + inc >= pinfo->min_rad) && (pinfo->pos + inc <= pinfo->max_rad))
                return Joint::forward(links, tgt, inc, act_func);
            else
                return _FALSE;
        }
        if (info->type == CONTINUOUS)
            return Joint::forward(links, tgt, inc, act_func);

        if (info->type == PRISMATIC)
            return Joint::forward(links, tgt, inc, act_func);

        throw "unknown joint!";
    }

    void World::set_speed(int id, double speed)
    {
        int a = timer.add([this, id, speed]
                          { this->drive(id, speed / 1e2); }, 1 * 1e4);
    }

    std::vector<Vector3d> World::discrete(std::vector<Vector3d> &pw, std::vector<Point2i> &tprojs, std::vector<bool> &vis)
    {

#define DISCRETE_LINE(a, b)                                                                                                      \
    do                                                                                                                           \
        if (vis[a] && vis[b])                                                                                                    \
        {                                                                                                                        \
            int CNT = std::ceil(std::sqrt(std::pow(tprojs[a][0] - tprojs[b][0], 2) + std::pow(tprojs[a][1] - tprojs[b][1], 2))); \
            for (int i = 0; i <= CNT; i++)                                                                                       \
                discreted_pw.push_back(pw[a] + ((pw[b] - pw[a]) / CNT) * i);                                                     \
        }                                                                                                                        \
    while (0)

#define PART_DISCRETE(a, b, c, d) \
    do                            \
    {                             \
        DISCRETE_LINE(a, b);      \
        DISCRETE_LINE(a, c);      \
        DISCRETE_LINE(a, d);      \
    } while (0)

        std::vector<Vector3d> discreted_pw;

        PART_DISCRETE(0, 1, 2, 4);
        PART_DISCRETE(6, 7, 2, 4);
        PART_DISCRETE(5, 4, 1, 7);
        PART_DISCRETE(3, 2, 1, 7);

#undef DISCRETE_LINE
#undef PART_DISCRETE

        return discreted_pw;
    }
    std::vector<Point2i> World::project()
    {
        std::map<int, Link *>::iterator _iter = links.begin();
        std::vector<std::vector<Vector3d>> cubes;
        std::vector<Point2i> projs;
        {
            std::lock_guard<std::mutex> lock(m);
            while (_iter != links.end())
            {
                cubes.push_back(to_3d(getCoord(_iter->first, -2)));
                _iter++;
            }
        }
        auto iter = cubes.begin();
        while (iter != cubes.end())
        {
            std::vector<Vector3d> cube_in_camera = *iter;
            std::vector<bool> visible(cube_in_camera.size(), true);
            remove_self_hidden(cube_in_camera, visible);
            std::vector<Point2i> pseudo_tprojs = cam.project(cube_in_camera, visible, true);
            std::vector<Vector3d> discreted_pw = discrete(cube_in_camera, pseudo_tprojs, visible);
            auto temp_iter = cubes.begin();
            visible.assign(discreted_pw.size(), true);
            while (temp_iter != cubes.end())
            {
                if (temp_iter != iter)
                    remove_inter_hidden(discreted_pw, *temp_iter, visible);
                temp_iter++;
            }
            std::vector<Point2i> tprojs = cam.project(discreted_pw, visible);
            projs.insert(projs.end(), tprojs.begin(), tprojs.end());
            iter++;
        }
        return projs;
    }
    std::vector<Vector4d> World::getCoord(int id, int base)
    {
        Link it = *links.at(id);
        it.transform(inv(pose[base]));
        return it.corners;
    }

    void World::parse_robot(std::initializer_list<Joint> jo)
    {

        const Joint *base_joint = jo.begin();
        int link_cnt = 0;

        Joint_node *curr_node = graph.add_child(base_joint);
        std::unordered_map<Cube *, int> link2parent_joint;
        std::unordered_map<Cube *, int> link2link_id;

        auto joint_iter = jo.begin();
        curr_node->set_parent_link_id(link_cnt);
        emplace(*joint_iter->parent_link, link_cnt);
        link2link_id.insert({joint_iter->parent_link, link_cnt++});
        link2parent_joint.insert({joint_iter->parent_link, base_joint->id});

        for (auto l : joint_iter->child_link)
        {
            emplace(*l, link_cnt);
            curr_node->append_child_link_id(link_cnt);
            link2link_id.insert({l, link_cnt++});
            link2parent_joint.insert({l, base_joint->id});
        }
        joint_iter++;

        while (joint_iter != jo.end())
        {
            int parent_id = link2parent_joint[joint_iter->parent_link];
            curr_node = graph.insert(parent_id, joint_iter);
            curr_node->set_parent_link_id(link2link_id[joint_iter->parent_link]);
            for (auto l : joint_iter->child_link)
            {
                emplace(*l, link_cnt);
                curr_node->append_child_link_id(link_cnt);
                link2link_id.insert({l, link_cnt});
                link2parent_joint.insert({l, joint_iter->id});
                act(link_cnt++, graph.find(parent_id)->get_pose());
            }
            joint_iter++;
        }
    }
}