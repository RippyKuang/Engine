#include <robot.h>

namespace Engine
{
    void Joint::add_child_link(Cube &link)
    {
        link.transform(getTransformMat(EYE(3), origin));
        child_link.push_back(&link);
    }

    Joint_node *Joint_node::add_child(const Joint *jo)
    {
        Vector3d this_origin = Vector3d{this->trans[3], this->trans[7], this->trans[11]};
        Joint_node *j = new Joint_node(jo->id, this_origin + jo->origin, jo->info);
        childs.push_back(j);
        return j;
    }
    void Joint_node::append_child_link_id(int id)
    {
        this->childs_link_id.push_back(id);
    }
    void Joint_node::set_parent_link_id(int id)
    {
        this->parent_link_id = id;
    }
    int Joint_node::get_parent_link_id()
    {
        return this->parent_link_id;
    }
    Joint_node *Joint_node::find(int id)
    {
        if (this->joint_id == id)
            return this;
        else
            for (auto child : childs)
            {
                Joint_node *node = child->find(id);
                if (node != NULL)
                    return node;
            }
        return NULL;
    }
    Joint_node *Joint_node::insert(int parent, const Joint *jo)
    {
        Joint_node *tgt = this->find(parent);
        return tgt->add_child(jo);
    }

    _T Joint_node::get_pose() const
    {
        return this->trans;
    }
    INFO *Joint_node::get_info() const
    {
        return this->info;
    }
    void Joint_node::transform_origin(_T t, _T base)
    {
        this->trans = base * t * inv(base) * this->trans;
    }
    void Joint_node::act(_T t, _T base, std::map<int, Link *> &links, std::function<void(int, _T)> func)
    {

        for (auto child_id : childs_link_id)
        {

            func(child_id, inv(base));
            func(child_id, t);
            func(child_id, base);
        }
        for (auto child : childs)
        {
            child->transform_origin(t, base);
            child->act(t, base, links, func);
        }
    }

    double Joint::forward(std::map<int, Link *> &links, Joint_node *tgt, double inc, std::function<void(int, _T)> func)
    {
        if (tgt->get_info()->type == CONTINUOUS || tgt->get_info()->type == REVOLUTE)
        {
            CONTINUOUS_INFO *pinfo = static_cast<CONTINUOUS_INFO *>(tgt->get_info());
            _T rot = getTransformMat(AngleAxis(inc, pinfo->axis), Vector3d());
            _T pose = tgt->get_pose();
            tgt->act(rot, pose, links, func);
        }
        else if (tgt->get_info()->type == PRISMATIC)
        {
            PRISMATIC_INFO *pinfo = static_cast<PRISMATIC_INFO *>(tgt->get_info());
            _T rot = getTransformMat(EYE(3), pinfo->axis * inc);
            _T pose = tgt->get_pose();
            tgt->act(rot, pose, links, func);
        }
        tgt->get_info()->pos = tgt->get_info()->pos + inc;
        return  tgt->get_info()->pos;
    }

}