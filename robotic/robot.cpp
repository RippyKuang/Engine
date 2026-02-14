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

    void Robot::FK(std::vector<_T> &T, std::vector<Vector6d> &v)
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

            M66 Xi0 = rot_mul_xlt(Xj, jo[i]->parent2joint) * X[this->lambda[i]];
            X.emplace_back(Xi0);
            _R E;
            Vector3d p;
            inv_plx(Xi0, E, p);
            T.emplace_back(getTransformMat(E.T(), p));
            v.emplace_back(v[this->lambda[i]] + plx(E.T(), E * p * (-1)) * vj);
        }
    }

    void Robot::FK_vulkan(std::vector<Matrix<float,4,4>> &T)
    {
        std::vector<M66> X;
        T.emplace_back(EYE(4));
        X.emplace_back(EYE(6));

        for (int i = 0; i < this->bo.size() - 1; i++)
        {
            M66 Xj;
            Vector6d vj;
            jo[i]->jtype->jcalc(Xj, vj);

            M66 Xi0 = rot_mul_xlt(Xj, jo[i]->parent2joint) * X[this->lambda[i]];
            X.emplace_back(Xi0);
            _R E;
            Vector3d p;
            inv_plx(Xi0, E, p);
            T.emplace_back(getTransformMat(E.T(), p));
        }
    }

    void Robot::ID(std::vector<double> &tau, std::vector<double> &v_dot, std::vector<M66> &X, std::vector<Vector6d> &ext_f)
    {

        v.emplace_back(Vector6d());
        X.emplace_back(EYE(6));
        Xw2j.emplace_back(EYE(6));
        a.emplace_back(Vector6d({0, 0, 0, 0, 0, 9.81}));
        for (int i = 0; i < this->bo.size() - 1; i++)
        {
            M66 Xj;
            M66 Xi0;
            Vector6d vj;
            BaseJoint *joint = jo[i]->jtype;
            joint->jcalc(Xj, vj);
            M66 I = bo[i + 1]->get_inertia();
            M66 Xip = rot_mul_xlt(Xj, jo[i]->parent2joint);

            Xi0 = Xip * Xw2j[this->lambda[i]];
            Vector6d vi = Xip * v[this->lambda[i]] + vj;
            Vector6d ai;
            if (joint->get_type() == JType::REVOLUTE)
            {
                Revolute *rjoint = reinterpret_cast<Revolute *>(joint);
                ai = Xip * a[this->lambda[i]] + rjoint->get_motion_subspace() * v_dot[i] + crm(vi) * vj;
            }
            else if (joint->get_type() == JType::PRISMATIC)
            {
                Prismatic *pjoint = reinterpret_cast<Prismatic *>(joint);
                ai = Xip * a[this->lambda[i]] + pjoint->get_motion_subspace() * v_dot[i] + crm(vi) * vj;
            }

            X.emplace_back(std::move(Xip));
            f.emplace_back(I * ai + crf(vi) * (I * vi) + force_trans(Xi0, ext_f[i]));
            Xw2j.emplace_back(std::move(Xi0));
            v.emplace_back(std::move(vi));
            a.emplace_back(std::move(ai));
        }

        for (int i = this->bo.size() - 2; i >= 0; i--)
        {
            BaseJoint *joint = jo[i]->jtype;
            if (joint->get_type() == JType::REVOLUTE)
            {
                Revolute *rjoint = reinterpret_cast<Revolute *>(joint);
                tau.emplace_back((rjoint->get_motion_subspace().T() * f[i])[0]);
            }
            else if (joint->get_type() == JType::PRISMATIC)
            {
                Prismatic *rjoint = reinterpret_cast<Prismatic *>(joint);
                tau.emplace_back((rjoint->get_motion_subspace().T() * f[i])[0]);
            }
            if (this->lambda[i] != 0)
                f[this->lambda[i] - 1] = f[this->lambda[i] - 1] + (X[i + 1] ^ f[i]);
        }
    }

    void Robot::FD(std::vector<double> &tau, std::vector<Vector6d> &ext_f)
    {
        std::vector<double> C;

        DynamicMatrix<DynamicMatrix<double>> H(this->jo.size(), this->jo.size());

        this->ID(C, zero, X, ext_f);

        for (int i = 1; i < this->bo.size(); i++)
        {
            Ic.emplace_back(this->bo[i]->get_inertia());
        }
        for (int i = bo.size() - 2; i >= 0; i--)
        {

            BaseJoint *joint = jo[i]->jtype;

            if (joint->get_type() == JType::REVOLUTE)
            {

                Revolute *rjoint = reinterpret_cast<Revolute *>(joint);

                if (this->lambda[i] != 0)
                {
                    Ic[this->lambda[i] - 1] += XTIX(X[i + 1], Ic[i]);
                }
                auto Si = rjoint->get_motion_subspace();
                auto F = Ic[i] * Si;
                H.data[i * this->jo.size() + i] = DynamicMatrix<double>(Si * F);
                int j = i;
                while (this->lambda[j] != 0)
                {
                    F = X[i + 1] ^ F;
                    j = this->lambda[j] - 1;
                    H.data[i * this->jo.size() + j] = DynamicMatrix<double>(F * Si);
                    H.data[j * this->jo.size() + i] = H.data[i * this->jo.size() + j].T();
                }
            }
            else if (joint->get_type() == JType::PRISMATIC)
            {
                Prismatic *rjoint = reinterpret_cast<Prismatic *>(joint);

                if (this->lambda[i] != 0)
                {
                    Ic[this->lambda[i] - 1] += XTIX(X[i + 1], Ic[i]);
                }
                auto Si = rjoint->get_motion_subspace();
                auto F = Ic[i] * Si;
                H.data[i * this->jo.size() + i] = DynamicMatrix<double>(Si * F);
                int j = i;
                while (this->lambda[j] != 0)
                {
                    F = X[i + 1] ^ F;
                    j = this->lambda[j] - 1;
                    H.data[i * this->jo.size() + j] = DynamicMatrix<double>(F * Si);
                    H.data[j * this->jo.size() + i] = H.data[i * this->jo.size() + j].T();
                }
            }
        }
        // qdot vector
        DynamicMatrix<double> qdot(this->jo.size(), 1);
        for (int i = 0; i < this->jo.size(); i++)
        {
            qdot.data[i] = this->jo[i]->jtype->get_q_dot();
        }

        DynamicMatrix<double> b(this->jo.size(), 1);
        for (int i = 0; i < this->jo.size(); i++)
        {
            b.data[this->jo.size() - 1 - i] = tau[this->jo.size() - 1 - i] - C[i] - 0.5 * qdot.data[this->jo.size() - 1 - i];
        }
        DynamicMatrix<double> x(this->jo.size(), 1);
        solve(H.dense(), std::move(b), x);
        for (int i = 0; i < this->jo.size(); i++)
        {
            if (this->jo[i]->jtype->get_type() == JType::REVOLUTE)
            {
                auto joint = reinterpret_cast<Revolute *>(this->jo[i]->jtype);
                joint->set_v_dot(x.data[i]);
            }
            if (this->jo[i]->jtype->get_type() == JType::PRISMATIC)
            {
                auto joint = reinterpret_cast<Prismatic *>(this->jo[i]->jtype);
                joint->set_v_dot(x.data[i]);
            }
        }
    }

}
