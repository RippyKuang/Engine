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

    void Robot::ID(std::vector<double> &tau, std::vector<double> &v_dot, std::vector<M66> &X) const
    {
        std::vector<Vector6d> v;
        std::vector<Vector6d> a;
        std::vector<Vector6d> f;
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
                M66 I = Xj.T() * bo[i + 1]->get_inertia() * Xj;
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
                tau.emplace_back((rjoint->get_motion_subspace().T() * f[i])[0]);

                if (this->lambda[i] != 0)
                    f[this->lambda[i] - 1] = f[this->lambda[i] - 1] + X[i + 1].T() * f[i];
            }
        }
    }

    void Robot::FD(std::vector<double> &tau) const
    {
        std::vector<double> C;
        std::vector<M66> X;
        DynamicMatrix<DynamicMatrix<double>> H(this->jo.size(), this->jo.size());
        std::vector<double> zero(tau.size(), 0.0);
        this->ID(C, zero, X);
        std::vector<M66> Ic;
        for (int i = 1; i < this->bo.size(); i++)
        {
            BaseJoint *joint = jo[i - 1]->jtype;
            M66 Xj;
            Vector6d vj;
            joint->jcalc(Xj, vj);
            Ic.emplace_back(Xj.T() * this->bo[i]->get_inertia() * Xj);
        }
        for (int i = bo.size() - 2; i >= 0; i--)
        {

            BaseJoint *joint = jo[i]->jtype;

            if (joint->get_type() == JType::REVOLUTE)
            {

                Revolute *rjoint = reinterpret_cast<Revolute *>(joint);

                if (this->lambda[i] != 0)
                    Ic[this->lambda[i] - 1] = Ic[this->lambda[i] - 1] + X[i + 1].T() * Ic[i] * X[i + 1];
                auto Si = rjoint->get_motion_subspace();
                auto F = Ic[i] * Si;
                H.data[i * this->jo.size() + i] = DynamicMatrix<double>(Si.T() * F);
                int j = i;
                while (this->lambda[j] != 0)
                {
                    F = X[j + 1].T() * F;
                    j = this->lambda[j] - 1;
                    H.data[i * this->jo.size() + j] = DynamicMatrix<double>(F.T() * Si);
                    H.data[j * this->jo.size() + i] = H.data[i * this->jo.size() + j].T();
                }
            }
        }

        DynamicMatrix<double> b(this->jo.size(), 1);
        for (int i = 0; i < this->jo.size(); i++)
        {
            b.data[i] = tau[i] - C[i];
        }
        DynamicMatrix<double> x(this->jo.size(), 1);
        solve(H.dense(), std::move(b), x);
        for (int i = 0; i < this->jo.size(); i++)
        {
            auto joint = reinterpret_cast<Revolute *>(this->jo[i]->jtype);
            joint->set_v_dot(x.data[i]);
            std::cout << "Joint " << i << " v_dot: " << x.data[i] << " C:" << C[i] << std::endl;
        }
    }
}
