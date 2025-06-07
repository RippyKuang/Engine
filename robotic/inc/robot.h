#pragma once
#include <joint_type.h>
#include <items.h>
#include <functional>
#include <twist.h>
#include <map>
#include <timer.h>
#include <dynamic_matrix.h>
#include <obb.h>

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
        // robot description
        std::vector<int> p;
        std::vector<int> s;
        std::vector<int> lambda;
        std::vector<Joint *> jo;

        // simulation thread
        std::thread daemon;
        bool daemon_running = true;

        // control vector
        std::vector<double> tau;
        std::vector<Vector6d> ext_f;

        // used by Inverse Dynamic
        std::vector<double> zero;
        std::vector<M66> Xw2j;
        std::vector<Vector6d> v;
        std::vector<Vector6d> a;
        std::vector<Vector6d> f;

        // used by Forward Dynamic
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
                this->zero.emplace_back(0.0);
                this->ext_f.emplace_back(Vector6d());
            }

            v.reserve(this->bo.size());
            a.reserve(this->bo.size());
            X.reserve(this->bo.size());
            Xw2j.reserve(this->bo.size());
            f.reserve(this->bo.size() - 1);
            Ic.reserve(this->bo.size() - 1);

            daemon = std::thread(&Robot::daemon_run, this);
        }
        void FK(std::vector<_T> &T, std::vector<Vector6d> &v);
        void ID(std::vector<double> &tau, std::vector<double> &v_dot, std::vector<M66> &X, std::vector<Vector6d> &ext_f);
        void FD(std::vector<double> &tau, std::vector<Vector6d> &ext_f);
        void daemon_run()
        {
            using namespace std;
            using namespace chrono;
            this->FD(this->tau, this->ext_f);
            while (daemon_running)
            {
                // auto start = system_clock::now();
                for (int i = 1; i < this->bo.size(); i++)
                {
                    Vector3d pi;
                    _R roti;
                    Vector3d p;
                    _R rot;
                    Link *a = this->bo[i];
                    inv_plx(a->j2w * Xw2j[i], roti, pi);

                    obb_box box_a{roti.T() * pi * (-1), a->box, roti.T()};
                    for (int j = i + 1; j < this->bo.size(); j++)
                    {
                        Link *b = this->bo[j];
                        inv_plx(b->j2w * Xw2j[j], rot, p);

                        obb_box box_b{rot.T() * p * (-1), b->box, rot.T()};

                        std::vector<contact_results> outputs;
                        int code;

                        obb_Intersection(box_a, box_b, 4, outputs, code);

                        for (auto out:outputs)
                        {
                            Vector3d f_contact = out.normal * std::abs(out.depth);
                            Vector3d torque = cross(out.points - p, f_contact);
                            this->ext_f[i - 1] += catRow(torque, f_contact);
                            this->ext_f[j - 1] += catRow(torque*(-1), f_contact * (-1));
                        }
                    }
                }
                Xw2j.clear();
                this->FD(this->tau, this->ext_f);

                for (int i = 0; i < this->p.size(); i++)
                {

                    this->ext_f[i] = Vector6d();
                }
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