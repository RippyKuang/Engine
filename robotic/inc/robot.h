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
        std::vector<double> tau;
        std::mutex dynamic_lock;
        std::thread daemon;
        bool daemon_running = true;

        std::vector<Vector6d> v;
        std::vector<Vector6d> a;
        std::vector<Vector6d> f;

        std::vector<M66> X;
        std::vector<M66> Ic;

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
                this->tau.emplace_back(0.0);
            }

            v.reserve(this->bo.size());
            a.reserve(this->bo.size());
            X.reserve(this->bo.size());
            f.reserve(this->bo.size()-1);
            Ic.reserve(this->bo.size()-1);


            daemon = std::thread(&Robot::daemon_run, this);
        }
        void FK(std::vector<_T> &T, std::vector<Vector6d> &v);
        void ID(std::vector<double> &tau, std::vector<double> &v_dot, std::vector<M66> &X);
        void FD(std::vector<double> &tau);
        void daemon_run()
        {
            using namespace std;
            using namespace chrono;

            while (daemon_running)
            {
                //auto start = system_clock::now();
                this->FD(this->tau);
                // auto end = system_clock::now();
                // auto duration = duration_cast<microseconds>(end - start);
                
                // cout << "花费了"
                //      << double(duration.count()) * microseconds::period::num / microseconds::period::den
                //      << "秒" << endl;

                for (int i = 0; i < this->jo.size(); i++)
                    this->jo[i]->jtype->step(1e-6);
            }
        }

        void set_tau(const std::vector<double> &tau)
        {
            this->tau = tau;
        }
        void summary() const;

        ~Robot()
        {
            daemon_running = false;
            if (daemon.joinable())
                daemon.join();
        }
    };

};