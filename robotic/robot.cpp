#include <robot.h>

namespace Engine
{
    void Joint::add_child_link(Cube &link)
    {   
        _T pose = getTransformMat(EYE(3), origin);
        link.transform(pose);
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

    Twist Joint_node::get_twist()
    {

        return adjoint(this->trans) * (this->info->get_twist());
    }

    INFO *Joint_node::get_info() const
    {
        return this->info;
    }

    void Joint_node::transform_origin(_T t, _T base)
    {
        this->trans = base * t * inv(base) * this->trans;
        _R r;
        Vector3d tvec;
        getRT(this->trans, r, tvec);
        Quaternion q = rotationMatrixToQuaternion(r);
        q.norm();
        this->trans = getTransformMat(q.toRotationMat(), tvec);
        
    }
    void Joint_node::act(_T t, _T base, std::map<int, Link *> &links, std::function<void(int, _T)> func)
    {

        for (auto child_id : childs_link_id)
        {
            func(child_id, base * t * inv(base));
        }
        for (auto child : childs)
        {
            child->transform_origin(t, base);
            child->act(t, base, links, func);
        }
    }
    void Joint_node::_impl_ID(std::vector<Twist> &v, std::vector<Twist> &dv, _T &base, Twist last_t, Twist last_dt)
    {
        ad_se3 last2this = adjoint(inv(base) * this->trans);
        Twist this_t = this->info->get_twist() + last2this * last_t;
        Twist this_dt = this->info->get_dtwist() + last2this * last_dt + bracket(this_t, this->info->get_twist());
        v.push_back(this_t);
        dv.push_back(this_dt);
        for (auto child : childs)
        {
            child->_impl_ID(v, dv, this->trans, this_t, this_dt);
        }
    }

    void Joint_node::InvDynamics_forward(std::vector<Twist> &v, std::vector<Twist> &dv)
    {
        for(auto child : childs)
            child->_impl_ID(v, dv, this->trans, this->get_twist(), this->info->get_dtwist());
    }

    double Joint::forward(std::map<int, Link *> &links, Joint_node *tgt, double inc, std::function<void(int, _T)> func)
    {
        if (tgt->get_info()->type == CONTINUOUS || tgt->get_info()->type == REVOLUTE)
        {
            CONTINUOUS_INFO *pinfo = static_cast<CONTINUOUS_INFO *>(tgt->get_info());
            _T rot = getTransformMat(AngleAxis(inc, pinfo->axis), Vector3d());
            _T pose = tgt->get_pose();
            tgt->trans = pose * rot;
            tgt->act(rot,pose, links, func);

        }
        else if (tgt->get_info()->type == PRISMATIC)
        {
            PRISMATIC_INFO *pinfo = static_cast<PRISMATIC_INFO *>(tgt->get_info());
            _T rot = getTransformMat(EYE(3), pinfo->axis * inc);
            _T pose = tgt->get_pose();
            tgt->trans = pose * rot;
            tgt->act(rot, pose, links, func);
        }
        tgt->get_info()->pos = tgt->get_info()->pos + inc;
        return tgt->get_info()->pos;
    }

}