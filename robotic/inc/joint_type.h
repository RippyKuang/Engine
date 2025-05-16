#pragma once
#include <rmath.h>

namespace Engine
{

    enum JType
    {
        DEFAULT,
        REVOLUTE
    };
    struct BaseJoint
    {
        JType type = DEFAULT;
        virtual void jcalc(M66 &) = 0;
        virtual void jcalc(_T &) = 0;
        virtual JType get_type() = 0;
        virtual void step(double) = 0;
    };

    class Revolute : public BaseJoint
    {
    private:
        Matrix<double, 6, 1> motion_subspace;
        Matrix<double, 6, 5> constraint_subspace;
        Vector3d axis;
        _R (*fE)(double);
        double q=0;
        double q_dot=1;

    public:
        JType get_type() override
        {
            return this->type;
        }
        void jcalc(M66 &X) override
        {
            X = rot(this->fE(q)) * xlt(Vector3d{0, 0, 0});
            //  ms = this->motion_subspace;
        }
        void jcalc(_T &X) override
        {
            X = catRow(catCol(this->fE(q), Vector3d()), catCol(Vector3d().T(), EYE(1)));
        }
        void step(double dt) override
        {
            q += q_dot * dt;
            q = fmod(q, 2 * M_PI);
        }
        Revolute(Vector3d &&axis) : axis(axis)
        {
            this->type = REVOLUTE;
            motion_subspace = Matrix<double, 1, 6>{axis[0], axis[1], axis[2], 0, 0, 0};
            if (axis[0] == 1)
            {
                this->fE = rx;
                constraint_subspace = Matrix<double, 6, 5>{0, 0, 0, 0, 0,
                                                           1, 0, 0, 0, 0,
                                                           0, 1, 0, 0, 0,
                                                           0, 0, 1, 0, 0,
                                                           0, 0, 0, 1, 0,
                                                           0, 0, 0, 0, 1};
            }
            else if (axis[1] == 1)
            {
                this->fE = ry;
                constraint_subspace = Matrix<double, 6, 5>{1, 0, 0, 0, 0,
                                                           0, 0, 0, 0, 0,
                                                           0, 1, 0, 0, 0,
                                                           0, 0, 1, 0, 0,
                                                           0, 0, 0, 1, 0,
                                                           0, 0, 0, 0, 1};
            }
            else
            {
                this->fE = rz;
                constraint_subspace = Matrix<double, 6, 5>{1, 0, 0, 0, 0,
                                                           0, 1, 0, 0, 0,
                                                           0, 0, 0, 0, 0,
                                                           0, 0, 1, 0, 0,
                                                           0, 0, 0, 1, 0,
                                                           0, 0, 0, 0, 1};
            }
        }
    };

}