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
        std::vector<Link *> objs;

    public:
        Robot(std::vector<int> &p,
              std::vector<int> &s,
              std::vector<Joint *> &jo,
              std::vector<Link *> &bo,
              std::vector<Link *> &objs)
        {
            this->p = std::move(p);
            this->s = std::move(s);
            this->jo = std::move(jo);
            this->bo = std::move(bo);
            this->objs = objs;
            this->ext_f.reserve(this->bo.size());
            for (int i = 0; i < this->bo.size(); i++)
            {
                 this->ext_f.emplace_back(Vector6d());
            }
            for (int i = 0; i < this->p.size(); i++)
            {
                this->lambda.emplace_back(std::min(this->p[i], this->s[i]));
                this->tau.emplace_back(0.0);
                this->zero.emplace_back(0.0);
            }


            v.reserve(this->bo.size());
            a.reserve(this->bo.size());
            X.reserve(this->bo.size());
            Xw2j.reserve(this->bo.size());
            f.reserve(this->bo.size() - 1);
            Ic.reserve(this->bo.size() - 1);

            daemon = std::thread(&Robot::daemon_run, this);
        }
        static double calcHuntCrossleyContactForce(double z,
                                            double dz,
                                            double k = 2.5e5,
                                            double p = 2,
                                            double beta = 0.8);
        void FK(std::vector<_T> &T, std::vector<Vector6d> &v);
        void ID(std::vector<double> &tau, std::vector<double> &v_dot, std::vector<M66> &X, std::vector<Vector6d> &ext_f);
        void FD(std::vector<double> &tau, std::vector<Vector6d> &ext_f);
        void FK_vulkan(std::vector<Matrix<float, 4, 4>> &T);
        void daemon_run()
        {
            using namespace std;
            using namespace chrono;

            while (daemon_running)
            {
                // auto start = system_clock::now();
                std::vector<_T> pose;
                std::vector<Vector6d> v;
                
                _R r;
                Vector3d tvec;
                int code;
                this->FK(pose, v);
                for (int i = 0; i < objs.size(); i++)
                {
                    obb_box obj_box{objs[i]->center, objs[i]->box, EYE(3)};
                    for (int j = 0; j < bo.size(); j++)
                    {
                        getRT(pose[j], r, tvec);
                        obb_box robo_box{r * bo[j]->center + tvec, bo[j]->box, r};
                        std::vector<contact_results> cresults;
                        if (obb_Intersection(obj_box, robo_box, 32, cresults, code))
                        {
                            Vector3d w = {v[j][0], v[j][1], v[j][2]};
                            Vector3d v0 = {v[j][3], v[j][4], v[j][5]};
                            Vector6d spatialForce;
                         //   
                            for (contact_results cresult : cresults)
                            {
                                Vector3d cpoint = cresult.points;
                                Vector3d cnormal = cresult.normal*(-1);
                                double z = cresult.depth;
                               
                                double dz = dot(v0 + cross(w, cpoint), cnormal);
                              //  std::cout<<z<<std::endl;
                              
                                double force = Robot::calcHuntCrossleyContactForce(z,dz);
                              
                                spatialForce += catCol(cross(cnormal*force,cpoint),cnormal*force);
                                
                            }
                            
                            this->ext_f[j] = spatialForce;
                            
                        }
                    }
                }
               //  std::cout<<"aaa"<<std::endl;
                this->FD(this->tau, this->ext_f);

                // auto end = system_clock::now();
                // auto duration = duration_cast<microseconds>(end - start);

                // cout << "花费了"
                //      << double(duration.count()) * microseconds::period::num / microseconds::period::den
                //      << "秒" << endl;

                for (int i = 0; i < this->jo.size(); i++)
                    this->jo[i]->jtype->step(1e-6);

                this->Xw2j.clear();
                this->v.clear();
                this->f.clear();
                this->a.clear();
                this->Ic.clear();
                this->X.clear();
            }
        }

        void set_tau(const std::vector<double> &tau)
        {
            this->tau = tau;
        }
        void summary() const;
        void step()
        {
            for (int i = 0; i < this->jo.size(); i++)
                this->jo[i]->jtype->step(1e-2);
        }

        ~Robot()
        {
            daemon_running = false;
            if (daemon.joinable())
                daemon.join();
        }
    };

};