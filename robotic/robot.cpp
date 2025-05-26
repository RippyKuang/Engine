#include <robot.h>

namespace Engine
{
    void Part::add_child_link(Link &link)
    {
        child_link = &link;
    }

    void Robot::summary() const
    {
        std::cout << "Summary of Robot" << std::endl;
        for (int i = 0; i < this->p.size(); i++)
        {
            std::cout << "Joint " << i << ":  "
                      << this->bo[this->p[i]]->get_name() << "  "
                      << this->bo[this->s[i]]->get_name()
                      << std::endl;
        }
    }

    void Robot::FK(std::vector<_T> &T, std::vector<Vector6d> &v) const
    {
        std::vector<M66> X;
        T.emplace_back(EYE(4));
        X.emplace_back(EYE(6));
        v.emplace_back(Vector6d());
        for (int i = 0; i < this->bo.size() - 1; i++)
        {
            M66 Xj;
            Vector6d vj;
            jo[i]->jtype->jcalc(Xj, vj);

            M66 X0i = Xj * jo[i]->parent2joint * X[this->lambda[i]];
            X.emplace_back(X0i);
            _R E;
            Vector3d p;
            inv_plx(X0i, E, p);
            T.emplace_back(inv(getTransformMat(E, p)));
            v.emplace_back(v[this->lambda[i]] + plx(E.T(), E * p * (-1)) * vj);
        }
    }

    void Robot::ID(std::vector<Vector6d> &tau, std::vector<double> &v_dot) const
    {
        std::vector<Vector6d> v;
        std::vector<Vector6d> a;
        std::vector<Vector6d> f;
        std::vector<M66> X;
        v.emplace_back(Vector6d());
        X.emplace_back(EYE(6));
        a.emplace_back(Vector6d({0, 0, 0, 0, 0, -9.81}));
        for (int i = 0; i < this->bo.size() - 1; i++)
        {
            M66 Xj;
            Vector6d vj;
            BaseJoint *joint = jo[i]->jtype;

            if (joint->get_type() == JType::REVOLUTE)
            {
                Revolute *rjoint = reinterpret_cast<Revolute *>(joint);
                rjoint->jcalc(Xj, vj);
                M66 I = bo[i]->get_inertia();
                M66 Xip = Xj * jo[i]->parent2joint;

                Vector6d vi = Xip * v[this->lambda[i]] + vj;
                Vector6d ai = Xip * a[this->lambda[i]] + rjoint->get_motion_subspace() * v_dot[i] + crm(vi) * vj;
                Vector6d fi = I * ai + crf(vi) * I * vi;
                X.emplace_back(Xip);
                v.emplace_back(vi);
                a.emplace_back(ai);
                f.emplace_back(fi);
            }
        }

        for (int i = this->bo.size() - 2; i >= 0; i--)
        {
            BaseJoint *joint = jo[i]->jtype;
            if (joint->get_type() == JType::REVOLUTE)
            {
                Revolute *rjoint = reinterpret_cast<Revolute *>(joint);
                tau.emplace_back(rjoint->get_motion_subspace().T() * f[i]);
                f[this->lambda[i]] = f[this->lambda[i]] + X[i].T() * f[i];
            }
        }
    }

    void Robot::FD(std::vector<Vector6d> &tau)const
    {
        // std::vector<Vector6d> C;
        // DynamicMatrix<DynamicMatrix<double>> H(this->jo.size(), this->jo.size());
        // std::vector<double> zero(0,tau.size());
        // this->ID(C,zero);
        
    } 
}
