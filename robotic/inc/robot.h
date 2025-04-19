#pragma once
#include <items.h>
#include <functional>
#include <twist.h>
#include <map>

namespace Engine
{
    class Joint;
    class Joint_node;

    enum JOINT_TYPE
    {
        REVOLUTE,
        CONTINUOUS,
        FIXED,
        PRISMATIC
    };

    typedef struct _INFO
    {
        JOINT_TYPE type = FIXED;
        double speed = 0;
        double pos = 0;
        _INFO(double p = 0) : pos(p)
        {
        }
        virtual Twist get_twist()
        {
            return Twist();
        }
    } INFO;
    typedef INFO FIXED_INFO;
    typedef struct _PRISMATIC_INFO : INFO
    {
        Vector3d axis;

        _PRISMATIC_INFO(Vector3d _axis, double pos = 0) : INFO(pos), axis(_axis)
        {
            type = PRISMATIC;
        }
        Twist get_twist() override
        {
            return Twist(Vector3d(), axis * speed);
        }
    } PRISMATIC_INFO;

    typedef struct _CONTINUOUS_INFO : INFO
    {
        Vector3d axis;

        _CONTINUOUS_INFO(Vector3d _axis, double pos = 0) : INFO(pos), axis(_axis)
        {
            type = CONTINUOUS;
        }
        virtual Twist get_twist() override
        {
            std::cout << axis << " " << speed << std::endl;
            return Twist(axis * speed, Vector3d());
        }
    } CONTINUOUS_INFO;

    typedef struct _REVOLUTE_INFO : _CONTINUOUS_INFO
    {

        double max_rad;
        double min_rad;
        _REVOLUTE_INFO(Vector3d _axis, double max, double min, double pos = 0) : CONTINUOUS_INFO(_axis, pos),
                                                                                 max_rad(max), min_rad(min)
        {
            type = REVOLUTE;
        }
    } REVOLUTE_INFO;

    class Joint_node
    {
        friend class Joint;

    private:
        int parent_link_id;
        _T trans;
        INFO *info;
        std::vector<Joint_node *> childs;
        std::vector<int> childs_link_id;
        void transform_origin(_T, _T);

    protected:
        void act(_T, _T, std::map<int, Link *> &, std::function<void(int, _T)>);

    public:
        int joint_id;
        Joint_node()
        {
            joint_id = -1;
            parent_link_id = -1;
        }
        Joint_node(int _id, Vector3d _origin, INFO *_info) : joint_id(_id), info(_info)
        {
            trans = getTransformMat(EYE(3), _origin);
        }
        Joint_node *add_child(const Joint *);
        void append_child_link_id(int);
        void set_parent_link_id(int);
        int get_parent_link_id();
        Joint_node *find(int);
        Joint_node *insert(int, const Joint *);
        _T get_pose() const;
        Twist get_twist() const;
        INFO *get_info() const;

        void Jacobian(std::vector<Matrix<double, 6, 1>> &v)
        {
            if (this->parent_link_id != -1)
            {
                v.push_back(this->get_twist());
            }

            if (this->childs.size() == 0)
                return;
            else
                this->childs[0]->Jacobian(v);
        }
    };

    class Joint
    {
        friend class World;
        friend class Joint_node;

    protected:
        Cube *parent_link;
        Vector3d origin;
        INFO *info;
        int id;
        std::vector<Cube *> child_link;

    public:
        Joint(Cube &_pi, Vector3d _origin, int _id, INFO *_type) : parent_link(&_pi), origin(_origin),
                                                                   id(_id), info(_type)
        {
        }
        Joint(Cube &_pi, Cube &_ci, Vector3d _origin, int _id, INFO *_type) : parent_link(&_pi), origin(_origin),
                                                                              id(_id), info(_type)
        {
            this->add_child_link(_ci);
        }
        void add_child_link(Cube &);

        static double forward(std::map<int, Link *> &, Joint_node *tgt, double inc, std::function<void(int, _T)>);
    };

}
