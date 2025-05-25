#pragma once
#include <joint_type.h>
#include <items.h>
#include <functional>
#include <twist.h>
#include <map>
#include <timer.h>
#include <dynamic_matrix.h>

namespace Engine
{

    class Part
    {
        friend class Robot;
        friend class World;
        friend class Joint;

    protected:
        Link *parent_link;
        Link *child_link;
        Vector3d origin;
        BaseJoint *type;
        int id;

    public:
        Part(Link &_pi, Link &_ci, Vector3d _origin, BaseJoint *_type) : parent_link(&_pi), origin(_origin),
                                                                         type(_type)
        {
            this->add_child_link(_ci);
        }
        void add_child_link(Link &);
    };

    struct Joint
    {
        BaseJoint *jtype;
        M66 parent2joint;
        Joint(const Part &part)
        {
            this->jtype = part.type;
            this->parent2joint = xlt(part.origin);
        }
    };

    class Robot
    {
        friend class World;

    private:
        std::vector<int> p;
        std::vector<int> s;
        std::vector<int> lambda;
        std::vector<Joint *> jo;
        Timer timer;

    protected:
        std::vector<Link *> bo;

    public:
        Robot(std::vector<int> &p,
              std::vector<int> &s,
              std::vector<Joint *> &jo,
              std::vector<Link *> &bo)
        {
            this->p = std::move(p);
            this->s = std::move(s);
            this->jo = std::move(jo);
            this->bo = std::move(bo);
            for (int i = 0; i < this->p.size(); i++)
            {
                this->lambda.emplace_back(std::min(this->p[i], this->s[i]));
            }
            timer.add([this]
                      {
                        for(int i = 0; i < this->jo.size(); i++)
                            this->jo[i]->jtype->step(1*1e-3); }, 1 _ms);
        }
        void FK(std::vector<_T> &T, std::vector<Vector6d> &v) const;
        void ID(std::vector<Vector6d> &tau, std::vector<double> &v_dot) const;
        void FD(std::vector<Vector6d> &tau)const;
        void summary() const;
    };

};