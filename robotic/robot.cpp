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
        Joint_node *j = new Joint_node(jo->id, this->origin + jo->origin, jo->info);
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
    Vector3d Joint_node::get_origin() const
    {
        return this->origin;
    }
    _R Joint_node::get_pose() const
    {
        return this->pose;
    }
    INFO *Joint_node::get_info() const
    {
        return this->info;
    }
    void Joint_node::transform_origin(_R r, _R pose, Vector3d origin)
    {
        this->origin = pose * r * pose.T() *  (this->origin - origin) + origin;
        this->pose = pose * r * (pose.T() * this->pose);
    }
    void Joint_node::act(_R r, _R pose, Vector3d origin, std::map<int, Link *> &links, std::function<void(int, _T)> func)
    {
        auto t = catRow(catCol(r, Vector3d()), catCol(Vector3d().T(), EYE(1)));
        for (auto child_id : childs_link_id)
        {

            func(child_id, inv(getTransformMat(pose, origin)));
            func(child_id, t);
            func(child_id, getTransformMat(pose, origin));
        }
        for (auto child : childs)
        {
            child->transform_origin(r, pose, origin);
            child->act(r, pose, origin, links, func);
        }
    }

    double Joint::forward(std::map<int, Link *> &links, Joint_node *tgt, double inc, std::function<void(int, _T)> func)
    {
        CONTINUOUS_INFO *pinfo = static_cast<CONTINUOUS_INFO *>(tgt->get_info());
        _R rot = AngleAxis(inc, pinfo->axis).toRotationMat();
        Vector3d ori = tgt->get_origin();
        _R pose = tgt->get_pose();

        tgt->act(rot, pose, ori, links, func);
    }

}